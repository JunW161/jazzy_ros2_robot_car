"""Integration test: dummy TCP sensor -> socat PTY and termios2 compatibility."""
import fcntl
import os
import pathlib
import socket
import subprocess
import tempfile
import threading
import time

# Exact Linux ioctl number equivalent to LDROBOT's asmtermios::termios2 size=44
TCGETS2 = 0x802C542A
TCSETS2 = 0x402C542B
PACKET = b"\x54\x2c" + bytes(range(45))

with socket.socket() as listener:
    listener.bind(('127.0.0.1', 0))
    listener.listen(1)
    port = listener.getsockname()[1]
    def server():
        client, _ = listener.accept()
        with client:
            for _ in range(10):
                client.sendall(PACKET)
                time.sleep(0.05)
    worker = threading.Thread(target=server, daemon=True)
    worker.start()
    with tempfile.TemporaryDirectory(prefix='ld14_wifi_test_') as tmp:
        link = str(pathlib.Path(tmp) / 'lidarpty')
        proc = subprocess.Popen(['socat', '-d', '-d', f'PTY,link={link},raw,echo=0,mode=660',
                                 f'TCP4:127.0.0.1:{port},connect-timeout=3'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            for _ in range(100):
                if os.path.exists(link):
                    break
                time.sleep(0.02)
            assert os.path.exists(link), 'socat failed to create PTY'
            fd = os.open(link, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                options = bytearray(44)
                fcntl.ioctl(fd, TCGETS2, options, True)
                fcntl.ioctl(fd, TCSETS2, options, True)
                recv = bytearray()
                deadline = time.time()+3
                while time.time() < deadline and len(recv)<len(PACKET)*2:
                    try:
                        recv.extend(os.read(fd, 4096))
                    except BlockingIOError:
                        time.sleep(0.02)
                assert PACKET in recv, f'packet not transferred; got len={len(recv)}'
                print('PASS: TCP -> socat PTY works; LDROBOT TCGETS2/TCSETS2 ioctls supported')
            finally:
                os.close(fd)
        finally:
            proc.terminate()
            proc.wait(timeout=5)
