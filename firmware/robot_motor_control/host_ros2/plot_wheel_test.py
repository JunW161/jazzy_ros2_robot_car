#!/usr/bin/env python3
"""Live/offline ROS 2 wheel-speed plots for ESP32-S3 delivery robot (Jazzy).

Live (reads the existing /robot/wheel_debug Float32MultiArray):
  source /opt/ros/jazzy/setup.bash
  python3 plot_wheel_test.py --live --save ~/wheel_trial.csv

Offline (plots CSV from this tool OR the original record_wheel_test.py):
  python3 plot_wheel_test.py --file ~/wheel_trial.csv

This is a PC-side diagnostic tool. It does not publish /cmd_vel and does NOT
change the ESP32 firmware, motor PID, FF, or safety controls.
"""

import argparse
import csv
import math
import statistics
import sys
import time
from pathlib import Path

COLS = (
    'timestamp_s', 'target_rpm_L', 'target_rpm_R', 'actual_rpm_L',
    'actual_rpm_R', 'pwm_L', 'pwm_R', 'distance_L_m', 'distance_R_m',
    'heading_deg', 'state',
)


def make_parser():
    parser = argparse.ArgumentParser(description='Plot target vs measured wheel RPM (forward/reverse), PWM, errors, distance, yaw.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--live', action='store_true', help='Subscribe to /robot/wheel_debug and graph live (default).')
    group.add_argument('--file', type=Path, help='View a recorded CSV without ROS/ESP32.')
    parser.add_argument('--save', type=Path, default=None, help='Live-mode CSV output path (default: timestamped file).')
    parser.add_argument('--png', type=Path, default=None, help='PNG output path (default: beside CSV).')
    parser.add_argument('--window', type=float, default=45.0, help='Live plot window in seconds (default: 45).')
    parser.add_argument('--no-show', action='store_true', help='Save PNG without opening a GUI (offline only).')
    return parser


def read_csv(path):
    with path.expanduser().open('r', newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        missing = sorted(set(COLS) - set(reader.fieldnames or []))
        if missing:
            raise ValueError(f'Missing columns: {missing}. Expected record_wheel_test.py CSV format.')
        rows = []
        for line_number, raw in enumerate(reader, start=2):
            try:
                values = tuple(float(raw[key]) for key in COLS)
            except (ValueError, TypeError, KeyError) as exc:
                raise ValueError(f'Invalid value in CSV line {line_number}: {exc}') from exc
            if all(math.isfinite(v) for v in values):
                rows.append(values)
    return rows


def save_csv(path, rows):
    path = path.expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(COLS)
        writer.writerows(rows)
    return path


def summarize(rows):
    if not rows:
        print('No telemetry received.')
        return
    # Treat forward and reverse as separate tests. Averaging signed +RPM and
    # -RPM together would misleadingly cancel their errors and target speeds.
    groups = {1: {'active': [], 'steady': []}, -1: {'active': [], 'steady': []}}
    prev_sign = 0
    prev_time = None
    run_start = None
    for row in rows:
        same_forward = row[1] > 1 and row[2] > 1
        same_reverse = row[1] < -1 and row[2] < -1
        sign = (1 if same_forward else -1 if same_reverse else 0) if round(row[10]) == 3 else 0
        if sign:
            if sign != prev_sign or prev_time is None or row[0] - prev_time > 0.5:
                run_start = row[0]
            groups[sign]['active'].append(row)
            if row[0] - run_start >= 1.0:
                groups[sign]['steady'].append(row)
        prev_sign = sign
        prev_time = row[0]

    shown = False
    for sign, title in [(1, 'FORWARD'), (-1, 'REVERSE')]:
        group = groups[sign]
        if len(group['steady']) >= 8:
            selected = group['steady']
            qualifier = 'steady ACTIVE; first 1 s excluded'
        elif len(group['active']) >= 4:
            selected = group['active']
            qualifier = 'all ACTIVE; too few samples for steady-state isolation'
        else:
            continue
        shown = True

        def avg(i):
            return statistics.mean(r[i] for r in selected)

        print(f'\n--- {title}: {qualifier} ---')
        print(f'Rows: {len(selected)} | full recording: {rows[-1][0] - rows[0][0]:.2f} s')
        for wheel, tgt, measured, pwm in [('LEFT ', 1, 3, 5), ('RIGHT', 2, 4, 6)]:
            errors = [r[tgt] - r[measured] for r in selected]
            mae = statistics.mean(abs(e) for e in errors)
            rmse = math.sqrt(statistics.mean(e * e for e in errors))
            print(f'{wheel}: target={avg(tgt):+.2f} rpm | actual={avg(measured):+.2f} rpm '
                  f'| error={statistics.mean(errors):+.2f} | MAE={mae:.2f} | RMSE={rmse:.2f} '
                  f'| PWM={avg(pwm):+.1f}')
        print(f'RPM difference L-R: {avg(3)-avg(4):+.2f} RPM')
    if not shown:
        print(f'Recorded {len(rows)} rows; not enough moving ACTIVE samples to assess RPM.')
    print('Note: equal encoder RPM or near-zero encoder heading change does not prove zero floor drift.')


def configure_figure(plt):
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle('Delivery Robot: Target vs Actual RPM | FF + PID', fontsize=14)
    specs = (
        ('Left wheel RPM', 'RPM', [('target L', 1, '--'), ('actual L', 3, '-')]),
        ('Right wheel RPM', 'RPM', [('target R', 2, '--'), ('actual R', 4, '-')]),
        ('RPM tracking error', 'RPM', [('error L', 'errL', '-'), ('error R', 'errR', '-')]),
        ('Actual motor PWM', 'PWM (signed duty)', [('PWM L', 5, '-'), ('PWM R', 6, '-')]),
        ('Wheel distance mismatch', 'R-L displacement (mm)', [('relative R-L', 'dist', '-')]),
        ('Encoder estimated heading', 'Yaw change (deg)', [('heading change', 'yaw', '-')]),
    )
    handles = []
    for ax, (title, ylabel, curves) in zip(axes.flat, specs):
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        lines = []
        for name, field, linestyle in curves:
            line, = ax.plot([], [], linestyle, label=name, linewidth=1.8)
            lines.append((line, field))
        ax.legend(loc='best', fontsize=9)
        handles.append((ax, lines))
    for ax in axes[-1, :]:
        ax.set_xlabel('Elapsed time (s)')
    fig.tight_layout(rect=(0, 0, 1, .97))
    return fig, handles


def draw_rows(fig, handles, rows, window=None):
    if not rows:
        return
    if window is None:
        shown = rows
    else:
        threshold = rows[-1][0] - window
        shown = [row for row in rows if row[0] >= threshold]
    t0 = rows[0][0]
    times = [r[0] - t0 for r in shown]
    initial_delta = rows[0][8] - rows[0][7]
    initial_heading = rows[0][9]
    values = {
        'errL': [r[1] - r[3] for r in shown],
        'errR': [r[2] - r[4] for r in shown],
        'dist': [1000 * ((r[8] - r[7]) - initial_delta) for r in shown],
        'yaw': [r[9] - initial_heading for r in shown],
    }
    xmin, xmax = times[0], times[-1]
    if xmax - xmin < 1.0:
        xmax = xmin + 1.0
    for ax, lines in handles:
        for line, field in lines:
            y = values[field] if isinstance(field, str) else [r[field] for r in shown]
            line.set_data(times, y)
        ax.set_xlim(xmin, xmax)
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)
    fig.canvas.draw_idle()


def run_offline(args, plt):
    rows = read_csv(args.file)
    print(f'Loaded {len(rows)} rows from {args.file.expanduser()}')
    if not rows:
        print('The CSV has no valid rows.')
        return 2
    summarize(rows)
    fig, handles = configure_figure(plt)
    draw_rows(fig, handles, rows)
    png = (args.png or args.file.with_suffix('.png')).expanduser()
    png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png, dpi=140)
    print(f'Chart saved: {png}')
    if not args.no_show:
        plt.show()
    plt.close(fig)
    return 0


def run_live(args, plt):
    if args.no_show:
        print('--no-show is only supported in --file offline mode.', file=sys.stderr)
        return 2
    if plt.get_backend().lower() in {'agg', 'pdf', 'ps', 'svg', 'cairo', 'template'}:
        print(f'No graphical Matplotlib backend ({plt.get_backend()}). On Ubuntu with display, install '
              '`sudo apt install python3-matplotlib python3-tk`, then run from desktop Terminal.', file=sys.stderr)
        return 2
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    except ImportError as exc:
        print(f'ROS Python import error: {exc}. Run `source /opt/ros/jazzy/setup.bash` first.', file=sys.stderr)
        return 2

    class WheelListener(Node):
        def __init__(self):
            super().__init__('wheel_rpm_live_plotter')
            self.rows = []
            self.last_rx = None
            qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Float32MultiArray, '/robot/wheel_debug', self.recv, qos)

        def recv(self, msg):
            if len(msg.data) != 10:
                self.get_logger().warning('Expected /robot/wheel_debug data with 10 elements')
                return
            if not all(math.isfinite(v) for v in msg.data):
                return
            self.rows.append((time.time(), *[float(v) for v in msg.data]))
            self.last_rx = time.monotonic()

    rclpy.init()
    node = WheelListener()
    fig, handles = configure_figure(plt)
    save_path = args.save or Path(time.strftime('wheel_trial_%Y%m%d_%H%M%S.csv'))
    png = args.png or save_path.with_suffix('.png')
    plt.show(block=False)
    print('\nLive plotting /robot/wheel_debug. You can send /cmd_vel from a DIFFERENT Terminal.')
    print('Close the graph window or Ctrl+C here to save CSV/PNG. This tool never drives the motors.')
    last_update = 0.0
    try:
        while plt.fignum_exists(fig.number):
            rclpy.spin_once(node, timeout_sec=0.035)
            tick = time.monotonic()
            if node.rows and tick - last_update >= 0.18:
                draw_rows(fig, handles, node.rows, window=max(args.window, 5))
                last_update = tick
            plt.pause(0.015)
    except KeyboardInterrupt:
        pass
    finally:
        rows = list(node.rows)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        saved = save_csv(save_path, rows)
        print(f'\nCSV saved: {saved} ({len(rows)} rows)')
        if rows:
            draw_rows(fig, handles, rows)  # Full trial for final PNG.
            png = png.expanduser()
            png.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(png, dpi=140)
            print(f'PNG saved: {png}')
            summarize(rows)
        plt.close(fig)
    return 0


def main():
    args = make_parser().parse_args()
    if args.window <= 0:
        print('--window must be positive', file=sys.stderr)
        return 2
    try:
        import matplotlib
        if args.no_show:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('Matplotlib missing. On Ubuntu: sudo apt install python3-matplotlib python3-tk', file=sys.stderr)
        return 2
    try:
        return run_offline(args, plt) if args.file else run_live(args, plt)
    except (OSError, ValueError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
