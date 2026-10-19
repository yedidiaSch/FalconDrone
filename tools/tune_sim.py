#!/usr/bin/env python3
"""Single-axis rig simulator that speaks the firmware's tuning-log format.

Used to test tools/tune_analyze.py without hardware, and to rehearse a
tuning session:

    python tools/tune_sim.py --kp 0.4 > /tmp/run.log
    python tools/tune_analyze.py /tmp/run.log
    python tools/tune_sim.py --autotune          # follow the advice in a loop

Model (one axis on a rod, props on, 100 Hz loop like the firmware):
    motor:  u_f' = (u - u_f) / TAU_MOTOR        first-order thrust lag
    frame:  w'   = K_TORQUE * u_f - DAMPING * w + disturbance
    angle:  a'   = w
    gyro:   w + noise, read one tick late (sample → control → PWM)

The PID, the angle P loop, the stick-free step injection, the I-term hold
below LIFTOFF_US and the doublet timing mirror pid.c, flight_ctrl.c and
tune_seq.c. Numbers are plausible for a small quad, not measured —
real gains will differ; the point is that the analyzer's rules converge.
"""
import argparse
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tune_analyze  # noqa: E402

DT = 0.01
TAU_MOTOR = 0.035          # s
K_TORQUE = 40.0            # (deg/s^2) per us of differential command
DAMPING = 2.0              # 1/s
GYRO_NOISE = 0.4           # deg/s rms
LIFTOFF_US = 1150
I_LIMIT, OUT_LIMIT = 200.0, 300.0

LEAD_MS, HOLD_MIN_MS = 500, 500


class Pid:
    """Port of Core/Src/pid.c."""

    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.integral, self.prev, self.has_prev = 0.0, 0.0, False
        self.p = self.i = self.d = self.out = 0.0

    def update(self, sp, meas, dt):
        err = sp - meas
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral + err * dt))
        d_meas = (meas - self.prev) / dt if self.has_prev else 0.0
        self.prev, self.has_prev = meas, True
        self.p, self.i, self.d = self.kp * err, self.ki * self.integral, -self.kd * d_meas
        self.out = max(-self.out_limit, min(self.out_limit, self.p + self.i + self.d))
        return self.out


def doublet(t_ms, amp, step_ms, reps):
    """Port of tune_seq.c: returns (ref, segment) or None when done."""
    hold = max(step_ms, HOLD_MIN_MS)
    if t_ms >= LEAD_MS + reps * 2 * (step_ms + hold):
        return None
    if t_ms < LEAD_MS:
        return 0.0, 0
    rep, in_rep = divmod(t_ms - LEAD_MS, 2 * (step_ms + hold))
    if in_rep < step_ms:
        k, ref = 0, amp
    elif in_rep < step_ms + hold:
        k, ref = 1, 0.0
    elif in_rep < 2 * step_ms + hold:
        k, ref = 2, -amp
    else:
        k, ref = 3, 0.0
    return ref, rep * 4 + k + 1


def run_test(kp, ki, kd, akp=2.0, mode='RATE', amp=60.0, step_ms=None, reps=3,
             thr=1350, disturbance=0.0, seed=1):
    """Simulate one $s test; return the log lines the firmware would print."""
    if step_ms is None:                        # same defaults as $s
        step_ms = 800 if mode == 'ANGLE' else 300
    rnd = random.Random(seed)
    rate = Pid(kp, ki, kd, I_LIMIT, OUT_LIMIT)
    outer = Pid(akp, 0.0, 0.0, 0.0, 150.0)
    w = a = u_f = 0.0
    gyro_prev = 0.0
    lines = [f'[TUNE] start axis=r mode={mode} amp={amp:.1f} step={step_ms} reps={reps} '
             f'kp={kp:.3f} ki={ki:.3f} kd={kd:.4f} lim={OUT_LIMIT:.0f} akp={akp:.3f}']
    t_ms = 0
    while True:
        seq = doublet(t_ms, amp, step_ms, reps)
        if seq is None:
            break
        ref, seg = seq
        gyro = gyro_prev                       # one tick of latency
        if mode == 'ANGLE':
            sp = outer.update(ref, a, DT)
        else:
            sp = ref
        u = rate.update(sp, gyro, DT)
        if thr < LIFTOFF_US:
            rate.integral = 0.0
        sat = int(abs(u) >= OUT_LIMIT * 0.999)
        lines.append(f'T,{t_ms},{seg},{ref:.1f},{sp:.1f},{gyro:.1f},{a:.1f},'
                     f'{rate.p:.1f},{rate.i:.1f},{rate.d:.1f},{u:.1f},{thr},{sat}')
        for _ in range(10):                    # plant at 1 kHz
            h = DT / 10
            u_f += (u - u_f) / TAU_MOTOR * h
            w += (K_TORQUE * u_f - DAMPING * w + disturbance) * h
            a += w * h
        gyro_prev = w + rnd.gauss(0.0, GYRO_NOISE)
        t_ms += 10
    lines.append('[TUNE] done')
    return lines


def autotune(kp, ki, kd, iterations, disturbance, thr, probes=6):
    """Apply the analyzer's advice over and over, like a careful human would.

    Once a set is GOOD, probe the margin: Kp x1.15 and test again, keeping
    the last GOOD set once the advice says to back off (advice that pushes
    on — more P or D — is followed). That ends near the highest gain
    that still looks clean, not at the first acceptable one.
    """
    good, probed = None, 0
    for n in range(1, iterations + 1):
        log = run_test(kp, ki, kd, disturbance=disturbance, thr=thr, seed=n)
        res = tune_analyze.analyze(tune_analyze.parse_log(log)[0])
        adv = res['advice']
        print(f'#{n:2d} kp={kp:.3f} ki={ki:.3f} kd={kd:.4f}  ->  {adv["verdict"]}: {adv["why"]}')
        if adv['verdict'] == 'GOOD':
            good = (kp, ki, kd)
            if probed >= probes:
                return (*good, 'GOOD')
            probed += 1
            kp *= 1.15
        elif good is not None:
            # Advice that pushes on (more P, more D) is followed; the first
            # "back off" (oscillation, noise) marks the edge.
            g = adv['gains'] if adv['verdict'] == 'ADJUST' else None
            tol = 0.01   # gains come back rounded from the log text
            if g and g['kp'] >= kp * (1 - tol) and g['kd'] >= kd * (1 - tol) \
                    and g['ki'] >= ki * (1 - tol):
                kp, ki, kd = g['kp'], g['ki'], g['kd']
                continue
            return (*good, 'GOOD (margin found)')
        elif adv['verdict'] == 'ADJUST':
            kp, ki, kd = adv['gains']['kp'], adv['gains']['ki'], adv['gains']['kd']
        else:
            return kp, ki, kd, adv['verdict']
    return (*good, 'GOOD') if good else (kp, ki, kd, 'NOT CONVERGED')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--kp', type=float, default=0.6)
    ap.add_argument('--ki', type=float, default=0.0)
    ap.add_argument('--kd', type=float, default=0.02)
    ap.add_argument('--akp', type=float, default=2.0)
    ap.add_argument('--mode', choices=('RATE', 'ANGLE'), default='RATE')
    ap.add_argument('--amp', type=float, default=60.0)
    ap.add_argument('--thr', type=int, default=1350)
    ap.add_argument('--disturbance', type=float, default=0.0,
                    help='constant torque, deg/s^2 (e.g. 400 = off-center battery)')
    ap.add_argument('--autotune', action='store_true')
    ap.add_argument('--iterations', type=int, default=20)
    args = ap.parse_args()

    if args.autotune:
        kp, ki, kd, verdict = autotune(args.kp, args.ki, args.kd, args.iterations,
                                       args.disturbance, args.thr)
        print(f'final: kp={kp:.3f} ki={ki:.3f} kd={kd:.4f} ({verdict})')
        return
    print('\n'.join(run_test(args.kp, args.ki, args.kd, args.akp, args.mode, args.amp,
                             thr=args.thr, disturbance=args.disturbance)))


if __name__ == '__main__':
    main()
