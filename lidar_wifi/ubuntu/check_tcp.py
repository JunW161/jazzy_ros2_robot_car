#!/usr/bin/env python3
"""Read-only diagnostic of the raw TCP LD14P UART byte stream."""
import argparse
import socket
import time


def main():
    p = argparse.ArgumentParser()
    p.add_argument('ip', help='ESP32-S3 #2 IP as printed in USB serial monitor')
    p.add_argument('--port', type=int, default=7777)
    p.add_argument('--seconds', type=float, default=3.0)
    args = p.parse_args()
    raw = bytearray()
    print(f'Connecting to {args.ip}:{args.port} (do not run socat simultaneously)')
    with socket.create_connection((args.ip, args.port), timeout=5) as sock:
        sock.settimeout(0.5)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            try:
                data = sock.recv(8192)
            except socket.timeout:
                continue
            if not data:
                print('ESP32 disconnected before test finished')
                break
            raw.extend(data)
    headers = raw.count(b'\x54\x2c')
    print(f'Bytes received: {len(raw)}')
    print(f'Candidate LD14P headers 54 2c: {headers}')
    print(f'First 32 bytes: {raw[:32].hex(" ")}')
    if len(raw) == 0:
        raise SystemExit('No UART bytes received: check sensor power, GPIO16, UART baud')
    if headers == 0:
        raise SystemExit('No expected LD14P packet header: check UART baud and wiring')
    print('Raw byte stream looks plausible. CRC validation is performed by the ROS driver.')


if __name__ == '__main__':
    main()
