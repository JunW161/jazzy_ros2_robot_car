"""USB monitoring only for legacy firmware. No hidden flash, reset or Wi-Fi writes."""
from __future__ import annotations
import subprocess
import time
from pathlib import Path
from .core import UserError


def ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        raise UserError('Thiếu python3-serial. Cài bằng apt theo README.')
    return [{'device': p.device, 'description': p.description, 'serial_number': p.serial_number,
             'vid': p.vid, 'pid': p.pid, 'location': p.location}
            for p in list_ports.comports()]


def read_log(device, baud, motor_power_off):
    if motor_power_off is not True:
        raise UserError('Phải xác nhận đã ngắt nguồn driver motor trước khi mở USB.')
    if device not in {p['device'] for p in ports()}:
        raise UserError('Cổng USB không nằm trong danh sách thiết bị hiện có.')
    if baud not in (115200, 230400):
        raise UserError('V1 chỉ đọc log ở 115200 hoặc 230400 baud.')
    ensure_available(device)
    import serial
    serial_port = serial.Serial(port=None, baudrate=baud, timeout=0.1,
                                write_timeout=1, exclusive=True)
    serial_port.dtr = False
    serial_port.rts = False
    serial_port.port = device
    chunks = bytearray()
    try:
        serial_port.open()
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and len(chunks) < 16000:
            chunks.extend(serial_port.read(min(1024, 16000 - len(chunks))))
    except serial.SerialException as exc:
        raise UserError(f'Không mở/đọc được cổng: {exc}')
    finally:
        serial_port.close()
    text = chunks.decode('utf-8', errors='replace')
    return {'text': text or 'Không có log mới trong 4 giây. Không tự RESET board.',
            'bytes': len(chunks), 'wifi_write_supported': False,
            'note': 'Đọc log không đổi Wi-Fi. Dùng mục build + app-flash nếu cần cấu hình mạng.'}


def ensure_available(device):
    try:
        busy = subprocess.run(['fuser', str(device)], capture_output=True, timeout=2)
        if busy.returncode == 0 and busy.stdout.strip():
            raise UserError('Cổng đang được chương trình khác dùng. Đóng Serial Monitor/driver cũ trước.')
    except FileNotFoundError:
        raise UserError('Thiếu fuser (gói psmisc); chưa thể kiểm tra cổng bận an toàn.')
