"""Network configuration for EXISTING compile-time firmware.

No invented serial provisioning protocol. Build a private sdkconfig copy from the
operator's project, then explicitly app-flash that artifact over the selected USB
port. Original motor source/sdkconfig, bootloader, partition table and NVS are not
written by this tool. Firmware itself remains responsible for its NVS behavior.
"""
from __future__ import annotations
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time
import uuid
from .core import UserError
from . import serial_tools

KEYS = {
    'esp32_1': {'ssid':'CONFIG_ESP_WIFI_SSID', 'password':'CONFIG_ESP_WIFI_PASSWORD',
                'host':'CONFIG_MICRO_ROS_AGENT_IP', 'port':'CONFIG_MICRO_ROS_AGENT_PORT'},
    'esp32_2': {'ssid':'CONFIG_LIDAR_WIFI_SSID', 'password':'CONFIG_LIDAR_WIFI_PASSWORD',
                'port':'CONFIG_LIDAR_TCP_PORT'},
}

def source_fingerprint(project: Path) -> str:
    paths = [project/'CMakeLists.txt', project/'sdkconfig', project/'sdkconfig.defaults']
    paths += sorted(p for p in (project/'main').rglob('*') if p.is_file() and p.suffix in ('.c','.cpp','.h','.hpp','.txt','.projbuild'))
    h = hashlib.sha256()
    for p in paths:
        if p.is_file():
            h.update(str(p.relative_to(project)).encode()); h.update(p.read_bytes())
    return h.hexdigest()

def config_values(data: dict, board: str) -> dict:
    if board not in KEYS:
        raise UserError('Chọn ESP32 #1 hoặc #2.')
    ssid, password = data.get('ssid'), data.get('password')
    if not isinstance(ssid,str) or not 1 <= len(ssid.encode('utf-8')) <= 32 or any(ord(x)<32 for x in ssid):
        raise UserError('SSID phải có 1–32 byte UTF-8, không có xuống dòng.')
    if not isinstance(password,str) or any(ord(x)<32 for x in password) or not (8 <= len(password.encode()) <= 63 or re.fullmatch(r'[0-9a-fA-F]{64}',password)):
        raise UserError('Mật khẩu WPA cần 8–63 byte hoặc 64 ký tự hex. Bản này không cấu hình Wi-Fi mở.')
    port = data.get('port')
    if type(port) is not int or not 1024 <= port <= 65535:
        raise UserError('Cổng cần từ 1024 đến 65535.')
    result = {KEYS[board]['ssid']:ssid, KEYS[board]['password']:password,
              KEYS[board]['port']:str(port) if board=='esp32_1' else port}
    if board=='esp32_1':
        try:
            host = ipaddress.IPv4Address(data.get('host',''))
            if host.is_unspecified or host.is_multicast or host.is_loopback: raise ValueError()
        except (ValueError,TypeError):
            raise UserError('Cần IPv4 của máy Ubuntu chạy micro-ROS Agent, không dùng 127.0.0.1.')
        result[KEYS[board]['host']] = str(host)
    return result

def private_config(original: str, updates: dict) -> str:
    if not re.search(r'^CONFIG_IDF_TARGET="esp32s3"$', original, re.M):
        raise UserError('sdkconfig không phải ESP32-S3. Không tự chạy set-target hoặc ghi đè cấu hình.')
    if re.search(r'^CONFIG_(?:SECURE_BOOT|SECURE_BOOT_V2_ENABLED|SECURE_FLASH_ENC_ENABLED)=y$',original,re.M):
        raise UserError('Không hỗ trợ flash qua web khi bật Secure Boot/Flash Encryption.')
    for key in updates:
        if len(re.findall(r'^'+re.escape(key)+r'=', original, re.M)) != 1:
            raise UserError(f'Thiếu/trùng {key}. Project không đúng cấu hình đã kiểm tra; không đoán firmware.')
    lines = []
    for line in original.splitlines():
        key = line.partition('=')[0]
        if key in updates:
            value = updates[key]
            line = key+'='+(json.dumps(value,ensure_ascii=False) if isinstance(value,str) else str(value))
        lines.append(line)
    return '\n'.join(lines)+'\n'

class UsbConfigurator:
    def __init__(self, store, ros, offline=False):
        self.store,self.ros,self.offline = store,ros,offline
        self.lock = threading.RLock()
        self.job = None
        self.thread = None
        self.secrets = []
        self.proc = None
        self.closed = False

    def project(self, board):
        if board not in KEYS: raise UserError('Board không hợp lệ.')
        key = 'motor_project' if board=='esp32_1' else 'lidar_project'
        return Path(self.store.config[key]).expanduser().resolve()

    def probe(self, board):
        project = self.project(board)
        sdk = project/'sdkconfig'
        symbols = KEYS[board].values()
        body = sdk.read_text(errors='replace') if sdk.is_file() else ''
        missing = [k for k in symbols if not re.search(r'^'+k+'=',body,re.M)]
        version = re.search(r'ESP-IDF\) ([\w.\-]+)', body)
        return {'board':board,'project':str(project),'sdkconfig_exists':sdk.is_file(),
                'source_exists':(project/'main').is_dir(), 'missing_symbols':missing,
                'idf_export_exists':Path(self.store.config['idf_export']).expanduser().is_file(),
                'idf_version':version.group(1) if version else None,
                'method':'build_app_flash',
                'note':'Đổi mạng bằng build + app-flash qua USB; không phải ghi Wi-Fi nóng qua serial.'}

    def status(self):
        with self.lock:
            if not self.job: return {'state':'idle', 'busy':False}
            return {k:v for k,v in self.job.items() if k not in ('dir','project','fingerprint','artifact_hash','sdk_hash')}

    def log(self):
        with self.lock:
            if not self.job: return 'Chưa có công việc USB.'
            path = Path(self.job['dir'])/'job.log'
            if not path.exists(): return ''
            with path.open('rb') as f:
                f.seek(0,2); f.seek(max(0,f.tell()-18000)); result=f.read().decode(errors='replace')
            for secret in self.secrets:
                if secret: result=result.replace(secret,'[REDACTED]')
            return result

    def _update(self, **kwargs):
        with self.lock:
            self.job.update(kwargs)

    def _assert_idle(self):
        if self.closed: raise UserError('Ứng dụng đang đóng.')
        if self.job and self.job.get('busy'): raise UserError('Đang có build/flash USB. Không chạy hai công việc cùng lúc.')
        if self.ros.motion.status()['armed']: raise UserError('Khóa điều khiển trước khi cấu hình USB.')
        if self.offline: raise UserError('OFFLINE: không build/nạp firmware từ giao diện thử.')

    def build(self, board, data):
        with self.lock:
            self._assert_idle()
            if data.get('motor_power_off') is not True:
                raise UserError('Xác nhận đã ngắt nguồn driver motor và robot đứng yên.')
            project = self.project(board)
            if not (project/'sdkconfig').is_file() or not (project/'CMakeLists.txt').is_file():
                raise UserError('Thiếu project/sdkconfig hiện tại. Xem Kiểm tra dự án; không dùng firmware khác để thay thế.')
            export = Path(self.store.config['idf_export']).expanduser().resolve()
            if not export.is_file(): raise UserError('Không tìm thấy ESP-IDF export.sh.')
            updates = config_values(data,board)
            text = private_config((project/'sdkconfig').read_text(),updates)
            ident = uuid.uuid4().hex
            directory = self.store.root/'runtime'/('usb_'+ident)
            directory.mkdir(mode=0o700)
            sdk = directory/'sdkconfig'
            sdk.write_text(text); os.chmod(sdk,0o600)
            self.secrets = [str(updates[KEYS[board]['password']]), str(updates[KEYS[board]['ssid']])]
            self.job = {'id':ident,'board':board,'state':'building','busy':True,'message':'Đang build bản cấu hình riêng; chưa nạp ESP32.',
                        'dir':str(directory),'project':str(project),'fingerprint':source_fingerprint(project),
                        'started':time.time(),'returncode':None}
            self.ros.maintenance=True
            self.store.audit('usb_build_start',board)
            self.thread=threading.Thread(target=self._build_worker,args=(export,),daemon=True)
            self.thread.start()
            return self.status()

    def _run(self, export, args):
        # Only fixed shell syntax; all paths/options passed as argv, never interpolated.
        command = ['bash','-c',
            'set -eo pipefail; unset VIRTUAL_ENV PYTHONHOME PYTHONPATH IDF_TARGET SDKCONFIG SDKCONFIG_DEFAULTS; export PATH="/usr/bin:/bin:/usr/sbin:/sbin:$PATH"; source "$1"; shift; exec python "$IDF_PATH/tools/idf.py" "$@"',
            'robot-web-idf',str(export),*args]
        directory=Path(self.job['dir'])
        with (directory/'job.log').open('a',encoding='utf-8') as log:
            self.proc=subprocess.Popen(command,cwd=self.job['project'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                       stdin=subprocess.DEVNULL,text=True,errors='replace',start_new_session=True)
            for line in self.proc.stdout:
                for secret in self.secrets:
                    if secret: line=line.replace(secret,'[REDACTED]')
                log.write(line);log.flush()
            return self.proc.wait()

    def _args(self):
        directory=Path(self.job['dir'])
        return ['-B',str(directory/'build'),'-D','IDF_TARGET=esp32s3','-D','SDKCONFIG='+str(directory/'sdkconfig')]

    def _build_worker(self, export):
        try:
            required=self.probe(self.job['board']).get('idf_version')
            if self._run(export,['--version']): raise UserError('Không khởi động được ESP-IDF. Xem log export.sh.')
            version_log=(Path(self.job['dir'])/'job.log').read_text(errors='replace')
            if required and not re.search(r'ESP-IDF v?'+re.escape(required)+r'(?:\b|[-+])',version_log):
                raise UserError(f'ESP-IDF không khớp sdkconfig ({required}); dừng trước build. Không tự nâng/hạ IDF.')
            rc=self._run(export,self._args()+['reconfigure','build'])
            if rc: raise UserError('Build thất bại. Chưa nạp board; xem log. Không tự đổi phiên bản ESP-IDF.')
            directory=Path(self.job['dir'])
            desc=json.loads((directory/'build/project_description.json').read_text())
            app=directory/'build'/desc['app_bin']
            if not app.is_file() or not app.resolve().is_relative_to((directory/'build').resolve()):
                raise UserError('Không tìm thấy app binary trong build; từ chối flash.')
            self._update(state='built',busy=False,returncode=0,message='Build xong. Chưa flash và chưa xác nhận Wi-Fi hoạt động.',
                         artifact_hash=hashlib.sha256(app.read_bytes()).hexdigest(),
                         sdk_hash=hashlib.sha256((directory/'sdkconfig').read_bytes()).hexdigest())
            self.store.audit('usb_build_ok',self.job['board'])
        except Exception as exc:
            self._update(state='failed',busy=False,message=str(exc),returncode=self.proc.poll() if self.proc else None)
        finally:
            self.ros.maintenance=False

    def flash(self, data):
        with self.lock:
            self._assert_idle()
            if not self.job or self.job['state']!='built': raise UserError('Cần build thành công trong phiên web này trước.')
            board=self.job['board']
            if data.get('confirmation') != ('NAP '+board) or data.get('motor_power_off') is not True or data.get('board_verified') is not True:
                raise UserError(f'Ngắt nguồn motor, kiểm tra đúng board/cổng, gõ NAP {board} để xác nhận.')
            device=str(data.get('device',''))
            found=next((p for p in serial_tools.ports() if p['device']==device),None)
            if not found: raise UserError('Cổng USB không tồn tại trong Ubuntu.')
            if self.ros.status().get('other_cmd_publishers'): raise UserError('Dừng nguồn điều khiển robot khác trước khi flash.')
            serial_tools.ensure_available(device)
            if source_fingerprint(Path(self.job['project']))!=self.job['fingerprint']:
                raise UserError('Mã nguồn/sdkconfig gốc đã đổi sau khi build. Build lại trước khi nạp.')
            directory=Path(self.job['dir'])
            desc=json.loads((directory/'build/project_description.json').read_text())
            app=directory/'build'/desc['app_bin']
            if hashlib.sha256(app.read_bytes()).hexdigest()!=self.job['artifact_hash'] or hashlib.sha256((directory/'sdkconfig').read_bytes()).hexdigest()!=self.job['sdk_hash']:
                raise UserError('Build artifact đã thay đổi. Không nạp; build lại.')
            self._update(state='flashing',busy=True,message='Đang app-flash. Không rút USB hoặc tắt web.',usb_device=device)
            self.ros.maintenance=True
            self.store.audit('usb_flash_start',board+' '+device)
            self.thread=threading.Thread(target=self._flash_worker,args=(device,),daemon=True);self.thread.start()
            return self.status()

    def _flash_worker(self, device):
        try:
            export=Path(self.store.config['idf_export']).expanduser().resolve()
            rc=self._run(export,self._args()+['-p',device,'app-flash'])
            if rc: raise UserError('app-flash lỗi. Xem log, không báo Wi-Fi đã kết nối.')
            self._update(state='flashed',busy=False,returncode=0,message='Nạp ứng dụng xong; đọc log USB và xác nhận Wi-Fi/IP, /odom hoặc /scan. Chưa xác nhận mạng mới hoạt động.')
            self.store.audit('usb_flash_ok',self.job['board'])
        except Exception as exc:
            self._update(state='failed',busy=False,message=str(exc),returncode=self.proc.poll() if self.proc else None)
        finally:
            self.ros.maintenance=False

    def close(self):
        # Never cut a flash halfway through automatically. Wait for completion on normal shutdown.
        self.closed=True
        if self.thread and self.thread.is_alive(): self.thread.join()
