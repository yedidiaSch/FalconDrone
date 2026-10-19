#!/usr/bin/env python3
"""FalconDrone step-test analyzer.

Reads a tuning log (the "[TUNE] ..." lines and 100 Hz "T,..." lines the
firmware prints during a $s step test), measures every step edge and
suggests the next gain change as a ready-to-type command.

    python tools/tune_analyze.py logs/tune_20261018_221500.log
    python tools/tune_analyze.py logs/tune_*.log --all       # every test in the file
    python tools/tune_analyze.py logs/tune_x.log --json      # machine-readable

T line (see Core/Inc/tuning.h):
    T,t_ms,seg,ref,sp,meas,ang,p,i,d,u,thr,sat

What is measured, per edge (every change of the reference, 4 per rep):
    rise      10 % -> 90 % of the step, ms
    overshoot peak beyond the target, % of the step
    settle    time until it stays within +-10 % of the step, ms
    sse       mean error over the last 30 % of the segment, % of the step
    osc       peak-to-peak wobble after the response first reached 90 %, %
    osc_hz    frequency of that wobble
Per test:
    sat       share of samples with a motor or the PID output at its limit
    u_noise   high-frequency content of the PID output during the holds, us
              (D-term noise / motor jitter)

Rate tests (RATE mode, or yaw) look at the gyro rate vs the rate setpoint.
ANGLE-mode tests look at the angle vs the angle setpoint and tune ANGLE_KP.

The rules are the usual manual-tuning rules (P for speed, D against
overshoot, I against steady error), one gain at a time, in moderate steps.
They are a starting point, not a substitute for watching the rig.
"""
import argparse
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass, field

LIFTOFF_US = 1150          # flight_cfg.h — I-terms held at 0 below this

# Thresholds (fractions of the step size unless noted)
SAT_MAX = 0.20             # more saturated samples than this → test invalid
OSC_SUSTAINED = 0.30       # wobble p-p after reaching the target
OSC_LOW_HZ = 3.0           # below: slow wobble (I or outer loop), above: P/D
OVERSHOOT_HIGH = 0.30
OVERSHOOT_OK = 0.15
SSE_MAX = 0.10
U_NOISE_HIGH_US = 12.0     # rms of the high-passed PID output during holds
RISE_SLOW_MS = {'rate': 80.0, 'angle': 250.0}

T_FIELDS = ('t', 'seg', 'ref', 'sp', 'meas', 'ang', 'p', 'i', 'd', 'u', 'thr', 'sat')
LOOP_NAMES = {'r': 'roll', 'p': 'pitch', 'y': 'yaw'}


@dataclass
class Test:
    params: dict
    rows: list = field(default_factory=list)
    end: str = ''           # 'done', 'aborted: <reason>' or '' (log cut off)


def parse_start(line):
    params = {}
    for key, val in re.findall(r'(\w+)=(\S+)', line):
        try:
            params[key] = float(val)
        except ValueError:
            params[key] = val
    return params


def parse_log(lines):
    """Split a log into Test objects. Text lines other than [TUNE] are ignored."""
    tests, cur = [], None
    for raw in lines:
        line = raw.strip()
        if line.startswith('[TUNE] start'):
            cur = Test(parse_start(line))
            tests.append(cur)
        elif line.startswith('[TUNE] done') and cur is not None:
            cur.end, cur = 'done', None
        elif line.startswith('[TUNE] aborted') and cur is not None:
            cur.end, cur = line[len('[TUNE] '):], None
        elif line.startswith('T,') and cur is not None:
            parts = line.split(',')[1:]
            if len(parts) != len(T_FIELDS):
                continue
            try:
                cur.rows.append(dict(zip(T_FIELDS, map(float, parts))))
            except ValueError:
                continue
    return tests


# ---- Measurements ------------------------------------------------------------

def edge_metrics(t, y, a, b, t0, t_end):
    """One step edge from a to b. t in ms, samples within [t0, t_end)."""
    step = b - a
    e = [(v - a) / step for v in y]          # 0 → 1
    n = len(e)
    m = {'from': a, 'to': b, 'duration_ms': t_end - t0}

    def first(cond):
        return next((k for k in range(n) if cond(e[k])), None)

    k10, k90 = first(lambda v: v >= 0.1), first(lambda v: v >= 0.9)
    m['reach'] = max(e) if e else 0.0
    m['rise_ms'] = (t[k90] - t[k10]) if (k10 is not None and k90 is not None) else None
    m['overshoot'] = max(0.0, max(e) - 1.0) if e else 0.0

    outside = [k for k in range(n) if abs(e[k] - 1.0) > 0.10]
    if not outside:
        m['settle_ms'] = 0.0
    elif outside[-1] < n - 1:
        m['settle_ms'] = t[outside[-1] + 1] - t0
    else:
        m['settle_ms'] = None                     # never settled in this segment

    tail = e[int(n * 0.7):] or e
    m['sse'] = 1.0 - statistics.fmean(tail)

    # Sustained wobble: the second half of the segment (after the first
    # overshoot has had time to decay), only if the response got close.
    m['osc'], m['osc_hz'] = 0.0, 0.0
    k_start = max(k90 if k90 is not None else n, n // 2)
    if n - k_start >= 6:
        w = e[k_start:]
        mean = statistics.fmean(w)
        crossings, side = 0, 0
        for v in w:
            d = v - mean
            s = 1 if d > 0.05 else (-1 if d < -0.05 else 0)
            if s and side and s != side:
                crossings += 1
            if s:
                side = s
        span_s = (t[-1] - t[k_start]) / 1000.0
        if crossings >= 2 and span_s > 0:    # at least one full cycle
            m['osc'] = max(w) - min(w)
            m['osc_hz'] = crossings / 2.0 / span_s
    return m


def hp_rms(values, width=5):
    """RMS of values minus their centered moving average."""
    if len(values) <= width:
        return 0.0
    h = width // 2
    out = []
    for k in range(h, len(values) - h):
        avg = sum(values[k - h:k + h + 1]) / width
        out.append(values[k] - avg)
    return math.sqrt(sum(v * v for v in out) / len(out))


def analyze(test):
    p = test.params
    axis = str(p.get('axis', 'r'))
    angle = p.get('mode') == 'ANGLE' and axis != 'y'
    kind = 'angle' if angle else 'rate'
    rows = [r for r in test.rows if r['seg'] > 0]

    res = {'axis': axis, 'kind': kind, 'end': test.end or 'log ended mid-test',
           'gains': {k: p.get(k) for k in ('kp', 'ki', 'kd', 'akp', 'lim')},
           'amp': p.get('amp'), 'edges': [], 'warnings': []}
    if len(rows) < 10:
        res['warnings'].append('too few samples in the test')
        res['advice'] = advise(res)
        return res

    # Group rows by segment, keep order.
    segs = []
    for r in rows:
        if not segs or segs[-1][0] != r['seg']:
            segs.append((r['seg'], []))
        segs[-1][1].append(r)

    prev_ref = 0.0
    hold_u, hold_p, hold_d = [], [], []
    for seg_no, srows in segs:
        target = statistics.median(r['ref'] for r in srows)
        t = [r['t'] for r in srows]
        y = [r['ang'] if angle else r['meas'] for r in srows]
        if abs(target - prev_ref) >= 1.0 and len(srows) >= 5:
            m = edge_metrics(t, y, prev_ref, target, t[0], t[-1] + 10.0)
            m['seg'] = int(seg_no)
            res['edges'].append(m)
        if target == 0.0:
            half = srows[len(srows) // 2:]
            hold_u.extend(r['u'] for r in half)
            hold_p.extend(r['p'] for r in half)
            hold_d.extend(r['d'] for r in half)
        prev_ref = target

    res['sat'] = sum(1 for r in rows if r['sat'] >= 1) / len(rows)
    res['u_noise'] = hp_rms(hold_u)
    # Which term drives the output while it should be quiet: D-dominated
    # jitter means too much D (or noise), P-dominated means too much P.
    rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v)) if v else 0.0  # noqa: E731
    p_rms, d_rms = rms(hold_p), rms(hold_d)
    res['d_share'] = d_rms / (p_rms + d_rms) if (p_rms + d_rms) > 0 else 0.0
    res['thr'] = statistics.median(r['thr'] for r in rows)
    if res['thr'] < LIFTOFF_US and not angle:
        res['warnings'].append(f'throttle {res["thr"]:.0f} us < LIFTOFF_US {LIFTOFF_US}: '
                               'I-term is held at 0, Ki has no effect')

    edges = res['edges']
    if edges:
        def med(key):
            vals = [e[key] for e in edges if e[key] is not None]
            return statistics.median(vals) if vals else None
        res['summary'] = {
            'rise_ms': med('rise_ms'), 'overshoot': med('overshoot'),
            # |sse|: a constant offset (off-center weight) has opposite signs
            # on up and down steps and would cancel in a signed median.
            'settle_ms': med('settle_ms'),
            'sse': statistics.median(abs(e['sse']) for e in edges),
            'osc': med('osc'), 'osc_hz': med('osc_hz'), 'reach': med('reach'),
            'unsettled': sum(1 for e in edges if e['settle_ms'] is None),
        }
    res['advice'] = advise(res)
    return res


# ---- Advice --------------------------------------------------------------------

def fmt_gain(v):
    return f'{v:.4g}'


def advise(res):
    """Return {'verdict', 'why', 'command', 'gains'}; command may be None."""
    g = res['gains']
    s = res.get('summary')
    axis = res['axis']
    out = {'verdict': '', 'why': '', 'command': None, 'gains': None}

    if not s or len(res['edges']) < 2:
        out.update(verdict='NO DATA', why='fewer than 2 usable step edges')
        return out
    if res['end'].startswith('aborted') and len(res['edges']) < 4:
        out.update(verdict='ABORTED', why=res['end'])
        return out
    if res['sat'] > SAT_MAX:
        out.update(verdict='INVALID',
                   why=f'{res["sat"]:.0%} of samples saturated — lower the amplitude or move '
                       'the throttle away from the ends; gains unchanged')
        return out

    noisy = res['u_noise'] > U_NOISE_HIGH_US
    osc, osc_hz = s['osc'] or 0.0, s['osc_hz'] or 0.0
    over = s['overshoot'] or 0.0
    sse = s['sse'] or 0.0
    rise = s['rise_ms']
    reach = s['reach'] or 0.0

    if res['kind'] == 'angle':
        akp = g['akp'] or 0.0
        loop = 'a' + axis
        new = None
        if osc > OSC_SUSTAINED:
            new, why = akp * 0.7, f'angle oscillates ({osc:.0%} p-p, {osc_hz:.1f} Hz)'
        elif over > 0.20:
            new, why = akp * 0.8, f'angle overshoot {over:.0%}'
        elif reach < 0.9 or (rise is not None and rise > RISE_SLOW_MS['angle']):
            new, why = akp * 1.25, f'slow to level (rise {rise} ms, reached {reach:.0%})'
        elif abs(sse) > SSE_MAX:
            out.update(verdict='CHECK', why=f'angle steady error {sse:.0%}: check level trim ($L) '
                                            'and the rate-loop Ki first')
            return out
        if new is None:
            out.update(verdict='GOOD', why=f'rise {rise} ms, overshoot {over:.0%}, error {sse:.0%}')
            return out
        out.update(verdict='ADJUST', why=why, gains={'akp': new},
                   command=f'g {loop} {fmt_gain(new)} 0 0')
        return out

    kp, ki, kd = g['kp'] or 0.0, g['ki'] or 0.0, g['kd'] or 0.0
    loop = 'r' + axis
    nkp, nki, nkd = kp, ki, kd
    if osc > OSC_SUSTAINED and osc_hz >= OSC_LOW_HZ:
        if kd > 0 and res['d_share'] > 0.6:
            nkd, why = kd * 0.7, f'fast oscillation {osc_hz:.1f} Hz driven by the D-term: less D'
        else:
            nkp, why = kp * 0.7, f'oscillates ({osc:.0%} p-p at {osc_hz:.1f} Hz): less P'
    elif osc > OSC_SUSTAINED and ki > 0:
        nki, why = ki * 0.7, f'slow wobble ({osc_hz:.1f} Hz): less I'
    elif over > OVERSHOOT_HIGH:
        if kd == 0:
            nkd, why = kp * 0.03, f'overshoot {over:.0%} and no D: add D'
        elif noisy:
            nkp, why = kp * 0.85, f'overshoot {over:.0%}, D already noisy: less P'
        else:
            nkd, why = kd * 1.3, f'overshoot {over:.0%}: more D'
    elif reach < 0.9 or (rise is not None and rise > RISE_SLOW_MS['rate']):
        nkp, why = kp * 1.25, f'sluggish (rise {rise} ms, reached {reach:.0%}): more P'
    elif over > OVERSHOOT_OK and not noisy:
        nkd, why = (kd * 1.15 if kd > 0 else kp * 0.03), f'overshoot {over:.0%}: a little more D'
    elif abs(sse) > SSE_MAX:
        if res['thr'] < LIFTOFF_US:
            out.update(verdict='CHECK', why=f'steady error {sse:.0%} but throttle is below '
                                            'LIFTOFF_US, so I is held at 0 — raise the throttle')
            return out
        nki, why = (ki * 1.3 if ki > 0 else kp * 0.5), f'steady error {sse:.0%}: more I'
    elif noisy and kd > 0:
        nkd, why = kd * 0.8, f'output noise {res["u_noise"]:.0f} us rms: less D'
    else:
        out.update(verdict='GOOD',
                   why=f'rise {rise} ms, overshoot {over:.0%}, error {sse:.0%}, '
                       f'noise {res["u_noise"]:.0f} us — keep, or probe margin with '
                       f'g {loop} {fmt_gain(kp * 1.15)} {fmt_gain(ki)} {fmt_gain(kd)}')
        return out

    out.update(verdict='ADJUST', why=why, gains={'kp': nkp, 'ki': nki, 'kd': nkd},
               command=f'g {loop} {fmt_gain(nkp)} {fmt_gain(nki)} {fmt_gain(nkd)}')
    return out


# ---- Report --------------------------------------------------------------------

def pct(v):
    return '  -  ' if v is None else f'{v * 100:4.0f}%'


def ms(v):
    return '   - ' if v is None else f'{v:5.0f}'


def report(res):
    g = res['gains']
    unit = 'deg' if res['kind'] == 'angle' else 'deg/s'
    head = (f'== {LOOP_NAMES.get(res["axis"], res["axis"])} {res["kind"]} test, '
            f'amp {res["amp"]} {unit} — {res["end"]}')
    lines = [head]
    if res['kind'] == 'angle':
        lines.append(f'   akp={g["akp"]}')
    else:
        lines.append(f'   kp={g["kp"]} ki={g["ki"]} kd={g["kd"]} lim={g["lim"]}')
    for w in res['warnings']:
        lines.append(f'   ! {w}')
    if res['edges']:
        lines.append('   seg  from->to     rise  settle  overshoot  sse   osc   osc_hz')
        for e in res['edges']:
            lines.append(f'   {e["seg"]:3d} {e["from"]:+6.1f}->{e["to"]:+6.1f}'
                         f' {ms(e["rise_ms"])} {ms(e["settle_ms"])}   {pct(e["overshoot"])}'
                         f'   {pct(e["sse"])} {pct(e["osc"])}  {e["osc_hz"]:5.1f}')
        s = res['summary']
        lines.append(f'   median: rise {ms(s["rise_ms"])} ms, overshoot {pct(s["overshoot"])}, '
                     f'sse {pct(s["sse"])}, osc {pct(s["osc"])}; '
                     f'sat {res["sat"]:.0%}, noise {res["u_noise"]:.1f} us, thr {res["thr"]:.0f}')
    a = res['advice']
    lines.append(f'   => {a["verdict"]}: {a["why"]}')
    if a['command']:
        lines.append(f'   => next: {a["command"]}    (then repeat the same $s test)')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('log')
    ap.add_argument('--all', action='store_true', help='analyze every test, not just the last')
    ap.add_argument('--json', action='store_true', help='print JSON instead of text')
    args = ap.parse_args()

    with open(args.log, errors='ignore') as fp:
        tests = parse_log(fp)
    if not tests:
        print('no "[TUNE] start" found in', args.log, file=sys.stderr)
        sys.exit(1)
    results = [analyze(t) for t in (tests if args.all else tests[-1:])]
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print('\n\n'.join(report(r) for r in results))


if __name__ == '__main__':
    main()
