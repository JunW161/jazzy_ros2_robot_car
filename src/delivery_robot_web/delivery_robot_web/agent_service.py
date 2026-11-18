"""Manage the exact micro-ROS Docker Agent command through a narrow root helper.

The operator's verified terminal command is:
    sudo docker run -it --rm --net=host \
      microros/micro-ros-agent:jazzy udp4 --port 8888 -v6

A browser backend has no interactive TTY, therefore the installed helper runs the
same image/network/transport/port/verbosity without ``-it``.  No systemd service,
no Docker socket access and no arbitrary sudo command are granted to the web app.
"""
from __future__ import annotations
import os
import subprocess
import threading
import time
from pathlib import Path
from .core import UserError

HELPER = Path('/usr/local/sbin/delivery-robot-agent-control')


class AgentService:
    def __init__(self, config, helper=HELPER, runner=subprocess.run):
        self.config = config
        self.helper = Path(helper)
        self.runner = runner
        self.lock = threading.RLock()
        self._cache = None
        self._at = 0.0

    def _run(self, action: str, timeout=7):
        args = ['/usr/bin/sudo', '-n', str(self.helper), action]
        try:
            return self.runner(
                args, capture_output=True, text=True, timeout=timeout,
                stdin=subprocess.DEVNULL,
                env={'PATH': '/usr/sbin:/usr/bin:/sbin:/bin', 'LANG': 'C'},
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise UserError('Không gọi được helper Agent: ' + str(exc)) from exc

    def snapshot(self, fresh=False):
        with self.lock:
            if not fresh and self._cache is not None and time.monotonic() - self._at < 0.8:
                return dict(self._cache)
            value = {
                'installed': self.helper.is_file(), 'running': False,
                'active_state': 'not-installed', 'sub_state': '', 'pid': 0,
                'port': int(self.config.get('agent_port', 8888)),
                'domain_id': int(os.environ.get('ROS_DOMAIN_ID', '0')),
                'message': '', 'status_known': False,
            }
            if not value['installed']:
                value['message'] = 'Chưa cài helper Agent. Chạy SETUP_AGENT_WEB_HELPER.sh một lần.'
            else:
                try:
                    p = self._run('status', timeout=4)
                    running = p.returncode == 0 and 'RUNNING' in p.stdout
                    value.update(
                        running=running,
                        status_known=p.returncode in (0, 3),
                        active_state='active' if running else 'inactive',
                        sub_state='running' if running else 'dead',
                        message=(
                            'Agent Docker đang chạy bằng đúng image/host-network/UDP 8888.'
                            if running else 'Agent Docker đang dừng.'
                        ),
                    )
                    if p.returncode not in (0, 3):
                        value.update(active_state='unknown', sub_state='unknown')
                        value['message'] = (p.stderr or p.stdout).strip()[-1200:]
                except UserError as exc:
                    value['message'] = str(exc)
            self._cache, self._at = value, time.monotonic()
            return dict(value)

    def _validate(self):
        info = self.snapshot(fresh=True)
        if not info['installed']:
            raise UserError(info['message'])
        if not info.get('status_known', True):
            raise UserError('Chưa xác định được trạng thái Agent. ' + info.get('message', ''))
        if int(self.config.get('agent_port', 8888)) != 8888:
            raise UserError('Web V2.5 Map V4 dùng đúng Agent đã kiểm chứng ở UDP 8888. Đặt agent_port về 8888.')
        if int(os.environ.get('ROS_DOMAIN_ID', '0')) != 0:
            raise UserError('Web V2.5 Map V4 đang khớp ROS_DOMAIN_ID=0 của hệ thống hiện tại.')
        if os.environ.get('ROS_LOCALHOST_ONLY') == '1':
            raise UserError('ROS_LOCALHOST_ONLY=1 sẽ chặn kết nối LAN của robot.')
        return info

    def start(self):
        info = self._validate()
        if info['running']:
            return 'Agent Docker đã chạy; không mở bản trùng.'
        p = self._run('start', timeout=8)
        self._at = 0
        if p.returncode:
            detail = (p.stderr or p.stdout).strip()[-1800:]
            raise UserError('Không khởi động được Agent Docker. ' + detail)
        time.sleep(0.5)
        info = self.snapshot(fresh=True)
        if not info['running']:
            raise UserError('Helper đã chạy lệnh Docker nhưng chưa thấy container Agent. Xem log Agent.')
        return 'Agent Docker đã chạy; chờ ESP32 #1 tạo /odom.'

    def stop(self):
        info = self.snapshot(fresh=True)
        if not info['installed']:
            raise UserError(info['message'])
        if info.get('status_known', True) and not info['running']:
            return 'Agent Docker đã dừng.'
        p = self._run('stop', timeout=12)
        self._at = 0
        if p.returncode:
            raise UserError('Không dừng được Agent Docker. ' + (p.stderr or p.stdout).strip()[-1500:])
        verified = self.snapshot(fresh=True)
        if not verified.get('status_known', True) or verified.get('running'):
            raise UserError('Đã gửi lệnh dừng nhưng chưa xác nhận Agent Docker đã dừng. ' +
                            verified.get('message', ''))
        return 'Đã dừng Agent Docker do web quản lý.'

    def log(self):
        if not self.helper.is_file():
            return 'Chưa cài helper Agent. Chạy SETUP_AGENT_WEB_HELPER.sh.'
        p = self._run('log', timeout=6)
        if p.returncode not in (0, 3):
            return 'Không đọc được log Agent. ' + (p.stderr or p.stdout).strip()[-1500:]
        return p.stdout[-18000:]
