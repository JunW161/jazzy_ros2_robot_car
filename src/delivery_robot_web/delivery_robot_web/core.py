"""Local persistence and motion lease. This module does not import ROS."""
from __future__ import annotations
import hashlib
import hmac
import json
import math
import os
import re
import secrets
import sqlite3
import threading
import time
import uuid
from pathlib import Path
from typing import Any
from .motion_profile import profile_updates, clamp_twist, wheel_rpms

DEFAULT_CONFIG = {
    'agent_port': 8888,
    'agent_backend': 'docker_exact_helper',
    'ros_workspace': '~/ros2_ws',
    'description_urdf': '~/ros2_ws/src/delivery_robot_description/urdf/delivery_car_robot.urdf',
    'lidar_project': '~/Downloads/LD14P_WIFI_BRIDGE_V1/esp32_idf',
    'idf_export': '~/esp/esp-idf/export.sh',
    'require_motor_telemetry': True,
    'lidar_ip': '192.168.1.153',
    'lidar_port': 7777,
    'scan_topic': '/scan',
    'odom_topic': '/odom',
    'cmd_topic': '/cmd_vel_manual',
    'odom_frame': 'odom',
    'base_frame': 'base_link',
    'laser_frame': 'base_laser',
    'map_frame': 'map',
    'motor_project': '~/micro_ros_ws/micro_ros_espidf_component/examples/robot_motor_control',
    'lidar_script': '~/Downloads/LD14P_WIFI_BRIDGE_V1/ubuntu/start_ld14p_wifi.sh',
    'import_root': '~/robot_maps',
    **profile_updates(),
    'max_odom_age': 0.30,
    'max_receive_age': 0.50,
    'lease_seconds': 0.30,
}

class UserError(Exception):
    """An expected error safe to display in the local UI."""


def password_hash(password: str, salt: str | None = None) -> str:
    if len(password) < 12 or len(password) > 256:
        raise UserError('Mật khẩu cần 12–256 ký tự.')
    salt = salt or secrets.token_hex(16)
    digest = hashlib.pbkdf2_hmac('sha256', password.encode(), bytes.fromhex(salt), 600_000)
    return f'pbkdf2_sha256$600000${salt}${digest.hex()}'


def password_verify(password: str, stored: str) -> bool:
    try:
        alg, count, salt, expected = stored.split('$')
        if alg != 'pbkdf2_sha256' or len(password) > 256:
            return False
        got = hashlib.pbkdf2_hmac('sha256', password.encode(), bytes.fromhex(salt), int(count)).hex()
        return hmac.compare_digest(got, expected)
    except (ValueError, TypeError):
        return False


def validate_name(value: str, maximum: int = 80) -> str:
    value = value.strip()
    if not value or len(value) > maximum or any(ord(c) < 32 for c in value):
        raise UserError(f'Tên cần 1–{maximum} ký tự, không có ký tự điều khiển.')
    return value


class Store:
    def __init__(self, root: Path):
        self.root = root.expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self.root, 0o700)
        for name in ('maps', 'logs', 'runtime'):
            (self.root / name).mkdir(exist_ok=True, mode=0o700)
        self.lock = threading.RLock()
        self.db = sqlite3.connect(str(self.root / 'app.sqlite3'), check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.executescript('''
            PRAGMA foreign_keys=ON;
            CREATE TABLE IF NOT EXISTS users(name TEXT PRIMARY KEY, password TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS sessions(token_hash TEXT PRIMARY KEY, username TEXT,
                csrf TEXT NOT NULL, expires REAL NOT NULL);
            CREATE TABLE IF NOT EXISTS maps(id TEXT PRIMARY KEY, name TEXT NOT NULL,
                created REAL NOT NULL, meta TEXT NOT NULL, checksum TEXT NOT NULL, source TEXT);
            CREATE TABLE IF NOT EXISTS points(id TEXT PRIMARY KEY, map_id TEXT NOT NULL
                REFERENCES maps(id), name TEXT NOT NULL, kind TEXT NOT NULL,
                x REAL NOT NULL, y REAL NOT NULL, yaw REAL NOT NULL, note TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS audit(id INTEGER PRIMARY KEY AUTOINCREMENT,
                stamp REAL NOT NULL, event TEXT NOT NULL, detail TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS delivery_history(
                mission_id TEXT PRIMARY KEY, map_id TEXT NOT NULL, map_name TEXT NOT NULL,
                pickup_id TEXT NOT NULL, dropoff_id TEXT NOT NULL,
                pickup_name TEXT NOT NULL, dropoff_name TEXT NOT NULL,
                state_name TEXT NOT NULL, created_at REAL NOT NULL,
                updated_at REAL NOT NULL, message TEXT NOT NULL);
        ''')
        self.db.commit()
        os.chmod(self.root / 'app.sqlite3', 0o600)
        self.config_path = self.root / 'config.json'
        self.config = dict(DEFAULT_CONFIG)
        profile = Path(__file__).parent / 'install_profile.json'
        if profile.exists():
            self.config.update(json.loads(profile.read_text()))
        if self.config_path.exists():
            self.config.update(json.loads(self.config_path.read_text()))
        else:
            self.save_config({})
        # Upgrade the old direct-motor Web route to the established command mux.
        # Preserve all user paths, network settings, geometry, maps and accounts.
        if self.config.get('cmd_topic') == '/cmd_vel':
            self.config['cmd_topic'] = '/cmd_vel_manual'
            self.save_config({})
            self.audit('configuration_migrated', 'Web manual output: /cmd_vel_manual')

    def save_config(self, updates: dict[str, Any]):
        # Only this allowlist is editable via HTTP; paths/frames are local-admin config.
        allowed = {'lidar_ip', 'lidar_port', 'agent_port'}
        if set(updates) - allowed:
            raise UserError('Chỉ sửa IP LiDAR và cổng kết nối trong giao diện này.')
        if 'lidar_ip' in updates:
            import ipaddress
            try:
                ip = ipaddress.IPv4Address(str(updates['lidar_ip']))
                if ip.is_multicast or ip.is_unspecified or ip.is_loopback:
                    raise ValueError()
                updates['lidar_ip'] = str(ip)
            except ValueError:
                raise UserError('Địa chỉ IPv4 LiDAR không hợp lệ.')
        for k in ('lidar_port', 'agent_port'):
            if k in updates and (type(updates[k]) is not int or not 1024 <= updates[k] <= 65535):
                raise UserError('Cổng cần là số nguyên từ 1024 đến 65535.')
        with self.lock:
            self.config.update(updates)
            tmp = self.config_path.with_suffix('.tmp')
            tmp.write_text(json.dumps(self.config, ensure_ascii=False, indent=2))
            os.chmod(tmp, 0o600)
            tmp.replace(self.config_path)

    def set_admin(self, name: str, password: str):
        name = validate_name(name, 40)
        hashed = password_hash(password)
        with self.lock:
            self.db.execute('INSERT OR REPLACE INTO users VALUES (?,?)', (name, hashed))
            self.db.execute('DELETE FROM sessions')
            self.db.commit()

    def has_admin(self) -> bool:
        with self.lock:
            return self.db.execute('SELECT COUNT(*) FROM users').fetchone()[0] > 0

    def login(self, name: str, password: str) -> tuple[str, str] | None:
        with self.lock:
            row = self.db.execute('SELECT password FROM users WHERE name=?', (name,)).fetchone()
        if row is None or not password_verify(password, row[0]):
            return None
        token, csrf = secrets.token_urlsafe(32), secrets.token_urlsafe(32)
        with self.lock:
            self.db.execute('DELETE FROM sessions WHERE expires<?', (time.time(),))
            self.db.execute('INSERT INTO sessions VALUES (?,?,?,?)',
                (hashlib.sha256(token.encode()).hexdigest(), name, csrf, time.time() + 8 * 3600))
            self.db.commit()
        return token, csrf

    def session(self, token: str | None) -> dict | None:
        if not token or len(token) > 128:
            return None
        digest = hashlib.sha256(token.encode()).hexdigest()
        with self.lock:
            row = self.db.execute('SELECT * FROM sessions WHERE token_hash=? AND expires>?',
                (digest, time.time())).fetchone()
        return dict(row) if row else None

    def logout(self, token: str):
        with self.lock:
            self.db.execute('DELETE FROM sessions WHERE token_hash=?',
                (hashlib.sha256(token.encode()).hexdigest(),))
            self.db.commit()

    def audit(self, event: str, detail: str = ''):
        with self.lock:
            self.db.execute('INSERT INTO audit(stamp,event,detail) VALUES (?,?,?)',
                (time.time(), event, detail[:500]))
            self.db.commit()

    def events(self):
        with self.lock:
            return [dict(r) for r in self.db.execute('SELECT * FROM audit ORDER BY id DESC LIMIT 80')]

    def list_maps(self):
        with self.lock:
            rows = self.db.execute('SELECT * FROM maps ORDER BY created DESC').fetchall()
        return [{**dict(r), 'meta': json.loads(r['meta'])} for r in rows]

    def map(self, ident: str):
        if not re.fullmatch(r'[a-f0-9]{32}', ident):
            raise UserError('Mã bản đồ không hợp lệ.')
        with self.lock:
            row = self.db.execute('SELECT * FROM maps WHERE id=?', (ident,)).fetchone()
        if not row:
            raise UserError('Không tìm thấy bản đồ.')
        return {**dict(row), 'meta': json.loads(row['meta'])}

    def add_map(self, ident, name, meta, checksum, source):
        with self.lock:
            self.db.execute('INSERT INTO maps VALUES (?,?,?,?,?,?)',
                (ident, validate_name(name), time.time(), json.dumps(meta), checksum, source))
            self.db.commit()

    def points(self, ident: str):
        self.map(ident)
        with self.lock:
            return [dict(r) for r in self.db.execute('SELECT * FROM points WHERE map_id=? ORDER BY name', (ident,))]

    def save_point(self, ident: str, data: dict):
        self.map(ident)
        name = validate_name(str(data.get('name', '')))
        kind = data.get('kind', 'both')
        if kind not in ('pickup', 'delivery', 'both'):
            raise UserError('Loại điểm không hợp lệ.')
        try:
            x, y, yaw = (float(data[k]) for k in ('x', 'y', 'yaw'))
        except (KeyError, TypeError, ValueError):
            raise UserError('Cần tọa độ x, y và góc yaw.')
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise UserError('Tọa độ phải là số hữu hạn.')
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        note = str(data.get('note', ''))[:500]
        point_id = data.get('id') or uuid.uuid4().hex
        if not re.fullmatch(r'[a-f0-9]{32}', point_id):
            raise UserError('Mã điểm không hợp lệ.')
        with self.lock:
            old = self.db.execute('SELECT map_id FROM points WHERE id=?', (point_id,)).fetchone()
            if old and old[0] != ident:
                raise UserError('Điểm thuộc một phiên bản bản đồ khác.')
            self.db.execute('INSERT OR REPLACE INTO points VALUES (?,?,?,?,?,?,?,?)',
                (point_id, ident, name, kind, x, y, yaw, note))
            self.db.commit()
        return point_id

    def delete_point(self, ident, point_id):
        self.map(ident)
        with self.lock:
            self.db.execute('DELETE FROM points WHERE id=? AND map_id=?', (point_id, ident))
            self.db.commit()

    def point(self, ident: str, point_id: str):
        self.map(ident)
        with self.lock:
            row = self.db.execute('SELECT * FROM points WHERE id=? AND map_id=?',
                                  (point_id, ident)).fetchone()
        if row is None:
            raise UserError('Không tìm thấy điểm trong bản đồ đã chọn.')
        return dict(row)

    def record_delivery(self, mission_id: str, record: dict, pickup: dict, dropoff: dict):
        now = time.time()
        with self.lock:
            self.db.execute('''INSERT INTO delivery_history VALUES (?,?,?,?,?,?,?,?,?,?,?)''',
                (mission_id, record['id'], record['name'], pickup['id'], dropoff['id'],
                 pickup['name'], dropoff['name'], 'GOING_TO_PICKUP', now, now,
                 'Mission Manager đã nhận nhiệm vụ.'))
            self.db.commit()

    def observe_delivery(self, mission: dict):
        """Persist real state transitions, never infer success from time or lost feedback."""
        ident = mission.get('id') or mission.get('mission_id')
        if not ident or not mission.get('available'):
            return
        age = mission.get('receive_age')
        if age is None or age > 3.0:
            return
        state = str(mission.get('state_name', 'UNKNOWN'))
        message = str(mission.get('message', ''))[:1000]
        with self.lock:
            row = self.db.execute('SELECT state_name,message FROM delivery_history WHERE mission_id=?',
                                  (ident,)).fetchone()
            if row and (row['state_name'] != state or row['message'] != message):
                self.db.execute('UPDATE delivery_history SET state_name=?,message=?,updated_at=? WHERE mission_id=?',
                                (state, message, time.time(), ident))
                self.db.commit()

    def interrupt_delivery(self, mission: dict):
        """Record a verified owned Mission Manager shutdown, without a fake heartbeat.

        Only ProcessManager calls this after its child process group has exited.
        Normal feedback still goes through observe_delivery's freshness checks.
        """
        ident = mission.get('id') or mission.get('mission_id')
        if not ident or mission.get('state_name') != 'ERROR':
            return
        message = str(mission.get('message') or 'Mission Manager đã đóng; nhiệm vụ bị gián đoạn.')[:1000]
        with self.lock:
            self.db.execute('''UPDATE delivery_history SET state_name='ERROR',message=?,updated_at=?
                WHERE mission_id=? AND state_name IN
                ('GOING_TO_PICKUP','WAITING_PICKUP','GOING_TO_DROPOFF','WAITING_DROPOFF')''',
                (message, time.time(), ident))
            self.db.commit()

    def delivery_history(self, limit: int = 50):
        limit = max(1, min(200, int(limit)))
        with self.lock:
            return [dict(r) for r in self.db.execute(
                'SELECT * FROM delivery_history ORDER BY created_at DESC LIMIT ?', (limit,))]


class MotionLease:
    """A short-lived, single-owner command. No stale command is latched forever."""
    def __init__(self, config, clock=time.monotonic):
        self.config = config
        self.clock = clock
        self.lock = threading.RLock()
        self.owner = None
        self.arm_key = None
        self.seq = -1
        self.deadline = 0.0
        self.v = self.w = 0.0
        self.reason = 'Chưa bật điều khiển.'
        self.stop_generation = 0

    def arm(self, owner: str, ready: bool):
        with self.lock:
            if not ready:
                raise UserError('Chưa đủ điều kiện điều khiển: kiểm tra odom, subscriber và nguồn lệnh khác.')
            if self.owner:
                raise UserError('Đã có một phiên giữ quyền điều khiển. Dừng trước khi nhận quyền mới.')
            self.owner, self.arm_key = owner, secrets.token_urlsafe(24)
            self.seq = -1
            self.deadline = self.clock() + 2.0  # first heartbeat must arrive promptly
            self.v = self.w = 0.0
            self.reason = 'Đã nhận quyền; giữ nút để chạy.'
            return self.arm_key

    def command(self, owner, key, seq, v, w, sent_at_ms, ready=True):
        with self.lock:
            if self.clock() > self.deadline and self.owner:
                self.stop('Hết hạn heartbeat.')
            if owner != self.owner or not self.arm_key or not hmac.compare_digest(str(key), self.arm_key):
                raise UserError('Không có quyền điều khiển hoặc quyền đã hết hạn.')
            if not ready:
                self.stop('Dữ liệu robot không còn hợp lệ.')
                raise UserError(self.reason)
            if type(seq) is not int or seq <= self.seq:
                raise UserError('Bỏ lệnh cũ/không đúng thứ tự.')
            if not all(math.isfinite(x) for x in (v, w, sent_at_ms)):
                self.stop('Lệnh không hợp lệ.')
                raise UserError(self.reason)
            if abs(time.time() * 1000 - sent_at_ms) > 300:
                self.stop('Lệnh trình duyệt quá cũ hoặc đồng hồ không khớp.')
                raise UserError(self.reason)
            self.seq = seq
            self.v, self.w = clamp_twist(v, w, self.config)
            self.deadline = self.clock() + self.config['lease_seconds']

    def stop(self, reason='Đã dừng bằng phần mềm.', force=False):
        with self.lock:
            was_owned = self.owner is not None
            self.owner = self.arm_key = None
            self.v = self.w = 0.0
            self.deadline = 0.0
            self.reason = reason
            if was_owned or force:
                self.stop_generation += 1

    def output(self, ready: bool):
        with self.lock:
            if self.owner and (self.clock() > self.deadline or not ready):
                self.stop('Dừng do hết heartbeat hoặc dữ liệu robot không còn hợp lệ.')
            return self.v, self.w, self.owner is not None

    def status(self):
        with self.lock:
            left, right = wheel_rpms(self.v, self.w, self.config)
            return {'armed': self.owner is not None, 'v': self.v, 'w': self.w, 'reason': self.reason,
                    'requested_rpm_l': left, 'requested_rpm_r': right}
