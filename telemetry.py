#!/usr/bin/env python3
"""FalconDrone live telemetry dashboard.

Reads CSV telemetry from the flight controller over UART2 and renders a
4-panel live view:

    +-----------------------+---------------------------+
    | Artificial horizon    | Motor PWM bar gauges      |
    | (roll / pitch)        | (live values 1000-2000us) |
    +-----------------------+---------------------------+
    | Gyro rates time-series| Motor PWM history         |
    | (gx, gy, gz)          | (m1..m4)                  |
    +-----------------------+---------------------------+

Firmware emits one CSV record every 20 ms (50 Hz):
    t_ms, roll, pitch, gx, gy, gz, m1, m2, m3, m4

Usage:
    python telemetry.py --port /dev/ttyACM0
    python telemetry.py --port /dev/ttyACM0 --record session.csv
"""
import argparse
import sys
import time
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial

FIELD_COUNT = 10
ESC_MIN, ESC_MAX = 1000, 2000
NOMINAL_RATE_HZ = 50

MOTOR_LABELS = ['M1\nFR/CW', 'M2\nRL/CW', 'M3\nFL/CCW', 'M4\nRR/CCW']
MOTOR_COLORS = ['#ff6b6b', '#ffa94d', '#74c0fc', '#4dabf7']
GYRO_LABELS  = ['gx (roll-rate)', 'gy (pitch-rate)', 'gz (yaw-rate)']
GYRO_COLORS  = ['#ff6b6b', '#51cf66', '#4dabf7']


def parse_args():
    p = argparse.ArgumentParser(description="FalconDrone telemetry visualizer")
    p.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    p.add_argument('--baud', type=int, default=115200, help='Baud rate')
    p.add_argument('--window', type=float, default=10.0,
                   help='Time window (seconds) shown on time-series plots')
    p.add_argument('--record', default=None,
                   help='Optional path to append received CSV records to')
    return p.parse_args()


class Telemetry:
    """Ring-buffered store of received samples."""

    def __init__(self, max_samples):
        self.t      = deque(maxlen=max_samples)
        self.roll   = deque(maxlen=max_samples)
        self.pitch  = deque(maxlen=max_samples)
        self.gx     = deque(maxlen=max_samples)
        self.gy     = deque(maxlen=max_samples)
        self.gz     = deque(maxlen=max_samples)
        self.m      = [deque(maxlen=max_samples) for _ in range(4)]
        self.rx_count = 0
        self.last_rx  = time.time()

    def feed(self, line):
        parts = line.split(',')
        if len(parts) != FIELD_COUNT:
            return False
        try:
            t  = float(parts[0])
            r  = float(parts[1]); p  = float(parts[2])
            gx = float(parts[3]); gy = float(parts[4]); gz = float(parts[5])
            m  = [int(float(parts[6 + i])) for i in range(4)]
        except ValueError:
            return False
        self.t.append(t / 1000.0)
        self.roll.append(r);   self.pitch.append(p)
        self.gx.append(gx);    self.gy.append(gy);   self.gz.append(gz)
        for i in range(4):
            self.m[i].append(m[i])
        self.rx_count += 1
        self.last_rx = time.time()
        return True


def build_dashboard(window_s):
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(13, 8))
    try:
        fig.canvas.manager.set_window_title('FalconDrone Telemetry')
    except Exception:
        pass

    gs = fig.add_gridspec(
        2, 2, width_ratios=[1, 1.4], height_ratios=[1, 1],
        hspace=0.40, wspace=0.25,
        left=0.07, right=0.97, top=0.91, bottom=0.08,
    )

    # ---------- Artificial horizon ----------
    ax_h = fig.add_subplot(gs[0, 0], aspect='equal')
    ax_h.set_xlim(-1.4, 1.4); ax_h.set_ylim(-1.4, 1.4)
    ax_h.axis('off')
    ax_h.set_title('Attitude  (roll / pitch)', color='#dddddd')
    ax_h.add_patch(plt.Circle((0, 0), 1.0, fill=False, color='#555', lw=1.5))
    for deg in (-45, 0, 45):
        a = np.radians(deg)
        ax_h.plot([0.95 * np.sin(a), 1.05 * np.sin(a)],
                  [0.95 * np.cos(a), 1.05 * np.cos(a)],
                  color='#666', lw=1)
    horizon, = ax_h.plot([-1, 1], [0, 0], color='#51cf66', lw=4)
    ax_h.plot([0], [0], 'o', color='#ff6b6b', ms=8, zorder=5)
    ax_h.plot([-1.18, -0.95], [0, 0], color='#ff6b6b', lw=3)
    ax_h.plot([0.95, 1.18],   [0, 0], color='#ff6b6b', lw=3)
    rp_text = ax_h.text(0, -1.30, '', ha='center', color='#bbbbbb', fontsize=10)

    # ---------- Motor bar gauges ----------
    ax_m = fig.add_subplot(gs[0, 1])
    ax_m.set_title('Motor PWM  (µs)', color='#dddddd')
    bars = ax_m.bar(MOTOR_LABELS, [ESC_MIN] * 4,
                    color=MOTOR_COLORS, edgecolor='#222')
    ax_m.set_ylim(ESC_MIN - 50, ESC_MAX + 50)
    ax_m.axhline(ESC_MIN, color='#666', ls=':', lw=0.8)
    ax_m.axhline(1500,    color='#444', ls=':', lw=0.6)
    ax_m.axhline(ESC_MAX, color='#666', ls=':', lw=0.8)
    ax_m.set_ylabel('pulse width µs')
    bar_texts = [
        ax_m.text(b.get_x() + b.get_width() / 2, ESC_MIN, '',
                  ha='center', va='bottom', fontsize=10, color='white')
        for b in bars
    ]

    # ---------- Gyro time series ----------
    ax_g = fig.add_subplot(gs[1, 0])
    ax_g.set_title('Gyro rates  (deg/s)', color='#dddddd')
    ax_g.set_xlabel('time (s, relative to now)')
    ax_g.set_ylabel('deg/s')
    ax_g.set_xlim(-window_s, 0)
    ax_g.set_ylim(-300, 300)
    ax_g.grid(True, alpha=0.2)
    gyro_lines = []
    for label, color in zip(GYRO_LABELS, GYRO_COLORS):
        (ln,) = ax_g.plot([], [], color=color, lw=1.5, label=label)
        gyro_lines.append(ln)
    ax_g.legend(loc='upper right', fontsize=8, framealpha=0.3)

    # ---------- Motor history ----------
    ax_t = fig.add_subplot(gs[1, 1])
    ax_t.set_title('Motor PWM history  (µs)', color='#dddddd')
    ax_t.set_xlabel('time (s, relative to now)')
    ax_t.set_ylabel('µs')
    ax_t.set_xlim(-window_s, 0)
    ax_t.set_ylim(ESC_MIN - 50, ESC_MAX + 50)
    ax_t.grid(True, alpha=0.2)
    motor_lines = []
    for i in range(4):
        (ln,) = ax_t.plot(
            [], [], color=MOTOR_COLORS[i], lw=1.3,
            label=MOTOR_LABELS[i].split('\n')[0],
        )
        motor_lines.append(ln)
    ax_t.legend(loc='upper right', fontsize=8, ncol=4, framealpha=0.3)

    fig.suptitle('FalconDrone — live telemetry',
                 fontsize=14, color='#cccccc', y=0.97)
    status = fig.text(0.5, 0.012, '', ha='center', color='#888', fontsize=8)

    return fig, {
        'horizon': horizon, 'rp_text': rp_text,
        'bars': bars, 'bar_texts': bar_texts,
        'gyro_lines': gyro_lines,
        'motor_lines': motor_lines,
        'ax_g': ax_g, 'ax_t': ax_t,
        'status': status,
    }


def main():
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"Cannot open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    record_fp = open(args.record, 'a') if args.record else None

    # Hold ~60 s of history at nominal rate, generous headroom.
    tel = Telemetry(max_samples=NOMINAL_RATE_HZ * 60)

    fig, h = build_dashboard(args.window)
    start_time = time.time()

    def drain_serial():
        while ser.in_waiting > 0:
            try:
                raw = ser.readline()
            except Exception:
                break
            if not raw:
                break
            try:
                line = raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            if not line:
                continue
            if tel.feed(line) and record_fp is not None:
                record_fp.write(line + '\n')

    def update(_frame):
        drain_serial()

        if not tel.t:
            elapsed = time.time() - start_time
            h['status'].set_text(
                f'waiting for data on {args.port} @ {args.baud}  '
                f'(elapsed {elapsed:0.1f}s)'
            )
            return [h['status']]

        t_now = tel.t[-1]
        t_arr = np.fromiter(tel.t, dtype=float) - t_now
        mask = t_arr >= -args.window
        t_arr = t_arr[mask]

        def take(d):
            return np.fromiter(d, dtype=float)[mask]

        # Artificial horizon — last sample
        roll  = tel.roll[-1]
        pitch = tel.pitch[-1]
        a = np.radians(roll)
        offset = np.sin(np.radians(pitch)) * 0.5
        x = np.array([-1.0, 1.0])
        y = np.array([offset, offset])
        x_r = x * np.cos(a) - y * np.sin(a)
        y_r = x * np.sin(a) + y * np.cos(a)
        h['horizon'].set_data(x_r, y_r)
        h['rp_text'].set_text(f'roll {roll:+6.1f}°    pitch {pitch:+6.1f}°')

        # Motor bars + value labels
        for i, bar in enumerate(h['bars']):
            v = tel.m[i][-1]
            bar.set_height(v)
            h['bar_texts'][i].set_position(
                (bar.get_x() + bar.get_width() / 2, v + 15)
            )
            h['bar_texts'][i].set_text(str(v))

        # Gyro time series
        for ln, src in zip(h['gyro_lines'], (tel.gx, tel.gy, tel.gz)):
            ln.set_data(t_arr, take(src))

        # Motor history
        for i, ln in enumerate(h['motor_lines']):
            ln.set_data(t_arr, take(tel.m[i]))

        elapsed = max(time.time() - start_time, 1e-3)
        rate = tel.rx_count / elapsed
        age_ms = (time.time() - tel.last_rx) * 1000.0
        h['status'].set_text(
            f'{args.port} @ {args.baud}  |  '
            f'samples {tel.rx_count}  |  rate {rate:5.1f} Hz  |  '
            f'last sample {age_ms:5.0f} ms ago'
        )

        artists = [h['horizon'], h['rp_text'], h['status']]
        artists.extend(h['bars'])
        artists.extend(h['bar_texts'])
        artists.extend(h['gyro_lines'])
        artists.extend(h['motor_lines'])
        return artists

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False,
                                  cache_frame_data=False)
    try:
        plt.show()
    finally:
        if record_fp is not None:
            record_fp.close()
        ser.close()


if __name__ == '__main__':
    main()
