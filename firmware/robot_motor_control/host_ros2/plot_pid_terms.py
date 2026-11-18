#!/usr/bin/env python3
"""ROS 2 Jazzy - record and plot ESP32 wheel FF/PID/PWM terms.

Live:
  /usr/bin/python3 host_ros2/plot_pid_terms.py --live --save ~/pid_terms.csv
Offline:
  /usr/bin/python3 host_ros2/plot_pid_terms.py --file ~/pid_terms.csv

Subscribes to /robot/control_debug (Float32MultiArray with 14 values, Best Effort).
Does not send /cmd_vel or modify firmware/PID.
"""

import argparse
import csv
import math
import time
from pathlib import Path

FIELDS = [
    'timestamp_s', 'refL', 'refR', 'ffL', 'ffR', 'pL', 'iL', 'dL',
    'pR', 'iR', 'dR', 'demandL', 'demandR', 'pwmL', 'pwmR',
]

PLOTS = [
    ('Left FF + PID (PWM)', [(3, 'FF left'), (5, 'P left'), (6, 'I left'), (7, 'D left')]),
    ('Right FF + PID (PWM)', [(4, 'FF right'), (8, 'P right'), (9, 'I right'), (10, 'D right')]),
    ('Wheel RPM references', [(1, 'Target left'), (2, 'Target right')]),
    ('Demand PWM (signed)', [(11, 'Demand left'), (12, 'Demand right')]),
    ('Applied PWM (signed)', [(13, 'Applied left'), (14, 'Applied right')]),
    ('Demand - applied PWM', [('deltaL', 'Left'), ('deltaR', 'Right')]),
]


def write_csv(path, rows):
    path = path.expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(FIELDS)
        writer.writerows(rows)
    return path


def read_csv(path):
    with path.expanduser().open(newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        missing = [c for c in FIELDS if c not in (reader.fieldnames or ())]
        if missing:
            raise ValueError('CSV is missing columns: ' + ', '.join(missing))
        rows = []
        for record in reader:
            row = [float(record[k]) for k in FIELDS]
            if all(math.isfinite(v) for v in row):
                rows.append(row)
        return rows


def new_figure(plt):
    fig, axs = plt.subplots(3, 2, figsize=(14, 9), sharex=True)
    fig.suptitle('Delivery Robot - FF / PID / PWM telemetry', fontsize=14)
    chart_lines = []
    for ax, (title, keys) in zip(axs.flat, PLOTS):
        ax.set_title(title)
        ax.grid(alpha=0.3)
        group = []
        for key, label in keys:
            line, = ax.plot([], [], label=label, linewidth=1.6)
            group.append((key, line))
        ax.legend(fontsize=8)
        chart_lines.append((ax, group))
    for ax in axs[-1, :]:
        ax.set_xlabel('Elapsed time (s)')
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig, chart_lines


def update_figure(fig, chart_lines, rows, window=None):
    if not rows:
        return
    subset = rows
    if window is not None:
        cutoff = rows[-1][0] - window
        subset = [r for r in rows if r[0] >= cutoff]
    t = [r[0] - rows[0][0] for r in subset]
    data = {
        'deltaL': [r[11] - r[13] for r in subset],
        'deltaR': [r[12] - r[14] for r in subset],
    }
    for ax, group in chart_lines:
        for key, line in group:
            vals = data[key] if isinstance(key, str) else [r[key] for r in subset]
            line.set_data(t, vals)
        ax.relim()
        ax.set_xlim(t[0], max(t[-1], t[0] + 0.25))
        ax.autoscale_view(scalex=False, scaley=True)
    fig.canvas.draw_idle()


def render_csv(rows, png_path, show):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if not rows:
        print('No samples received, so no PNG can be created.')
        return
    fig, charts = new_figure(plt)
    update_figure(fig, charts, rows)
    png_path = png_path.expanduser()
    png_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(png_path, dpi=150)
    print('PNG saved:', png_path)
    if show:
        plt.show()
    plt.close(fig)


def record(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32MultiArray
    from rclpy.executors import ExternalShutdownException

    class Listener(Node):
        def __init__(self):
            super().__init__('pid_terms_recorder')
            self.rows = []
            self.bad_length_reported = False
            self.create_subscription(
                Float32MultiArray, '/robot/control_debug', self.on_msg,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

        def on_msg(self, msg):
            if len(msg.data) != 14:
                if not self.bad_length_reported:
                    self.get_logger().warning(
                        f'Expected 14 values on /robot/control_debug, got {len(msg.data)}')
                    self.bad_length_reported = True
                return
            values = [float(v) for v in msg.data]
            if all(math.isfinite(v) for v in values):
                self.rows.append([time.time(), *values])

    fig = None
    charts = None
    plt = None
    if not args.no_show:
        import matplotlib.pyplot as plt
        fig, charts = new_figure(plt)

    rclpy.init()
    node = Listener()
    print('Recording /robot/control_debug (best_effort, 14 values).', flush=True)
    print('Ctrl+C or close the chart window to save CSV + PNG.', flush=True)
    last_draw = 0.0
    last_message = time.monotonic()
    rows = []
    try:
        if fig is not None:
            plt.show(block=False)
        while rclpy.ok():
            if fig is not None and not plt.fignum_exists(fig.number):
                break
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if node.rows and now - last_draw >= 0.25:
                if fig is not None:
                    update_figure(fig, charts, node.rows, window=args.window)
                last_draw = now
            if now - last_message >= 5.0:
                print(f'Received {len(node.rows)} samples', flush=True)
                last_message = now
            if plt is not None:
                plt.pause(0.005)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rows = list(node.rows)
        try:
            node.destroy_node()
        finally:
            # SIGINT on Jazzy may already have shut down the ROS context.
            # Never call shutdown unconditionally: that raises RCLError.
            if rclpy.ok():
                rclpy.try_shutdown()
        if fig is not None:
            plt.close(fig)
        saved = write_csv(args.save, rows)
        print(f'CSV saved: {saved} ({len(rows)} samples)', flush=True)
        if not rows:
            print('No PID messages received. Check: '
                  'ros2 topic info /robot/control_debug -v', flush=True)
        else:
            # Live chart has already been shown; do not reopen a blocking chart on Ctrl+C.
            render_csv(rows, args.png or saved.with_suffix('.png'), False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--live', action='store_true', help='Record ROS topic and show live graphs')
    group.add_argument('--file', type=Path, help='Plot an existing CSV file')
    parser.add_argument('--save', type=Path, default=Path('pid_terms.csv'))
    parser.add_argument('--png', type=Path)
    parser.add_argument('--window', type=float, default=40.0, help='Live graph window (seconds)')
    parser.add_argument('--no-show', action='store_true', help='Save files without opening a plot window')
    args = parser.parse_args()
    if args.window <= 0:
        parser.error('--window must be > 0')
    if args.file is not None:
        rows = read_csv(args.file)
        print(f'Loaded {len(rows)} samples')
        render_csv(rows, args.png or args.file.with_suffix('.png'), not args.no_show)
    else:
        record(args)


if __name__ == '__main__':
    main()
