"""Entry point: never starts motors, Agents or SLAM automatically."""
from __future__ import annotations
import argparse
import fcntl
import getpass
import os
import sys
from pathlib import Path
from .core import Store, MotionLease, UserError


def parser():
    p = argparse.ArgumentParser(description='Robot Delivery local web for ROS 2 Jazzy')
    p.add_argument('--data-dir', default=str(Path.home()/'.local/share/delivery_robot_web'))
    sub = p.add_subparsers(dest='command', required=True)
    admin = sub.add_parser('init-admin', help='Create/reset a local login. Invalidates all sessions.')
    admin.add_argument('--username', default='admin')
    serve = sub.add_parser('serve', help='Bind to 127.0.0.1 only')
    serve.add_argument('--port', type=int, default=8080)
    serve.add_argument('--offline', action='store_true', help='UI testing only; never controls ROS/hardware')
    sub.add_parser('doctor', help='Read-only check of the current project paths')
    return p


def main():
    args = parser().parse_args()
    store = Store(Path(args.data_dir))
    if args.command == 'doctor':
        import json
        from .project import inspect_project
        from .agent_service import AgentService
        report = inspect_project(store.config)
        report['agent_docker_service'] = AgentService(store.config).snapshot(fresh=True)
        from .motion_profile import limits_info
        report['manual_motion'] = limits_info(store.config)
        print(json.dumps(report,ensure_ascii=False,indent=2))
        return
    if args.command == 'init-admin':
        if store.has_admin():
            print('Tài khoản đã tồn tại. Thao tác này đặt lại mật khẩu và thu hồi các phiên đăng nhập.')
            if input('Nhập RESET để xác nhận: ').strip() != 'RESET':
                return
        p1 = getpass.getpass('Mật khẩu mới (ít nhất 12 ký tự): ')
        p2 = getpass.getpass('Nhập lại mật khẩu: ')
        if p1 != p2:
            raise SystemExit('Hai mật khẩu không giống nhau.')
        try:
            store.set_admin(args.username, p1)
        except UserError as exc:
            raise SystemExit(str(exc))
        print(f'Đã tạo tài khoản {args.username}. Không có mật khẩu mặc định.')
        return
    if not 1024 <= args.port <= 65535:
        raise SystemExit('Port phải trong khoảng 1024–65535.')
    if not store.has_admin():
        raise SystemExit('Chưa có tài khoản. Chạy: ros2 run delivery_robot_web web init-admin')
    lock = open(store.root/'server.lock', 'a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('Web server đã chạy với thư mục dữ liệu này. Không mở bản thứ hai.')
    motion = MotionLease(store.config)
    from .ros_adapter import RosAdapter, OfflineAdapter
    if args.offline:
        ros = OfflineAdapter(store.config, motion)
    else:
        try:
            ros = RosAdapter(store.config, motion)
        except ImportError as exc:
            raise SystemExit(f'Thiếu thư viện ROS/Python: {exc}. Dùng /usr/bin/python3 và source Jazzy. Không tự chuyển sang OFFLINE.')
    from .processes import ProcessManager
    from .app import create_app
    import uvicorn
    manager = ProcessManager(store, ros, offline=args.offline)
    if not args.offline:
        # Clean only exact stale processes from a previous Web/project session.
        # Opening the Web must never mean that Odom/EKF/Model/LiDAR are already ON.
        cleaned = manager.cleanup_stale_project_stack()
        if cleaned:
            import time
            print('Đã dọn tiến trình cũ của dự án trước khi mở Web: ' + ', '.join(cleaned))
            time.sleep(0.8)
    app = create_app(store, ros, motion, manager, port=args.port)
    print(f'Robot Delivery: http://127.0.0.1:{args.port}')
    print('Chỉ quan sát lúc khởi động. Chưa phát lệnh motor. Ctrl+C để dừng server và các tiến trình do web quản lý.')
    try:
        uvicorn.run(app, host='127.0.0.1', port=args.port, access_log=False, workers=1)
    finally:
        motion.stop('Server đang đóng.')
        import time
        time.sleep(0.2)
        app.state.usb.close()
        manager.close()
        ros.close()
        lock.close()


if __name__ == '__main__':
    main()
