"""Loopback-only FastAPI application. No shell-command endpoint, no anonymous control."""
from __future__ import annotations
import asyncio
import hashlib
import hmac
import io
import json
import math
import secrets
import shutil
import threading
import time
import uuid
import zipfile
from pathlib import Path
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware
from .core import Store, MotionLease, UserError, validate_name
from .maps import import_candidates, import_existing, live_map_image, live_map_clean_image, live_meta, register_map, validate_point_cell, map_pixel, ensure_display_images
from .processes import ProcessManager
from . import serial_tools
from .motion_profile import limits_info
from .project import inspect_project
from .usb_config import UsbConfigurator


def create_app(store: Store, ros, motion: MotionLease, manager: ProcessManager, port=8080):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=['127.0.0.1', 'localhost'])
    app.state.store, app.state.ros, app.state.motion = store, ros, motion
    app.state.manager = manager
    usb = UsbConfigurator(store, ros, offline=manager.offline)
    app.state.usb = usb
    origins = {f'http://127.0.0.1:{port}', f'http://localhost:{port}'}
    failures, active_operations = {}, threading.RLock()
    map_cache = {'revision': -1, 'image': None}
    clean_map_cache = {'revision': -1, 'image': None}
    ros_map_cache = {'revision': -1, 'image': None}
    static = Path(__file__).parent / 'static'
    app.mount('/static', StaticFiles(directory=static), name='static')

    @app.exception_handler(UserError)
    async def user_error(_req, exc):
        return JSONResponse({'detail': str(exc)}, status_code=409)

    @app.middleware('http')
    async def guard(request: Request, call_next):
        path = request.url.path
        if path.startswith('/api/'):
            size = request.headers.get('content-length', '0')
            try:
                if int(size) > 20000:
                    return JSONResponse({'detail': 'Yêu cầu quá lớn.'}, status_code=413)
            except ValueError:
                return JSONResponse({'detail': 'Yêu cầu không hợp lệ.'}, status_code=400)
            session = store.session(request.cookies.get('robot_session'))
            request.state.session = session
            if path != '/api/login' and not session:
                return JSONResponse({'detail': 'Vui lòng đăng nhập.'}, status_code=401)
            if request.method not in ('GET', 'HEAD', 'OPTIONS'):
                origin = request.headers.get('origin')
                if origin not in origins:
                    return JSONResponse({'detail': 'Chỉ chấp nhận yêu cầu từ giao diện cục bộ.'}, status_code=403)
                if path != '/api/login' and not hmac.compare_digest(request.headers.get('x-csrf-token', ''), session['csrf']):
                    return JSONResponse({'detail': 'Phiên không hợp lệ; đăng nhập lại.'}, status_code=403)
        response = await call_next(request)
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['X-Frame-Options'] = 'DENY'
        response.headers['Referrer-Policy'] = 'no-referrer'
        response.headers['Content-Security-Policy'] = f"default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self' ws://127.0.0.1:{port} ws://localhost:{port}; frame-ancestors 'none'; base-uri 'none'"
        return response

    def owner(request):
        return request.state.session['token_hash']

    def ensure_stationary():
        if usb.status().get('busy'):
            raise UserError('Đang build/nạp USB. Không điều khiển/thay đổi hệ thống.')
        if motion.status()['armed']:
            raise UserError('Dừng và khóa điều khiển trước khi thao tác này.')

    def complete_status():
        s = ros.status()
        s['motion'] = motion.status()
        s['processes'] = manager.status()
        s['usb_job'] = usb.status()
        s['limits'] = limits_info(store.config)
        s['version'] = '5.2.0'
        return s

    @app.get('/')
    def home():
        return HTMLResponse((static/'index.html').read_text())

    @app.post('/api/login')
    def login(request: Request, data: dict):
        key = request.client.host if request.client else 'local'
        now = time.monotonic()
        attempts = [t for t in failures.get(key, []) if now-t < 60]
        if len(attempts) >= 5:
            raise HTTPException(429, 'Thử đăng nhập sai nhiều lần. Thử lại sau 60 giây.')
        result = store.login(str(data.get('username', ''))[:40], str(data.get('password', ''))[:257])
        if not result:
            failures[key] = attempts + [now]
            raise HTTPException(401, 'Tài khoản hoặc mật khẩu không đúng.')
        failures.pop(key, None)
        token, csrf = result
        response = JSONResponse({'csrf': csrf, 'username': data['username']})
        response.set_cookie('robot_session', token, httponly=True, samesite='strict',
                            secure=False, max_age=8*3600, path='/')
        store.audit('login', str(data['username'])[:40])
        return response

    @app.get('/api/session')
    def session(request: Request):
        return {'username': request.state.session['username'], 'csrf': request.state.session['csrf']}

    @app.post('/api/logout')
    def logout(request: Request):
        if motion.owner == owner(request):
            motion.stop('Đăng xuất.')
        store.logout(request.cookies.get('robot_session', ''))
        response = JSONResponse({'ok': True})
        response.delete_cookie('robot_session', path='/')
        return response

    @app.get('/api/status')
    def status(): return complete_status()

    @app.websocket('/ws/status')
    async def websocket(ws: WebSocket):
        token = ws.cookies.get('robot_session')
        if ws.headers.get('origin') not in origins or not store.session(token):
            await ws.close(code=1008)
            return
        await ws.accept()
        try:
            while True:
                if not store.session(token):
                    await ws.close(code=1008)
                    break
                # A service probe / process lock must not block the ASGI loop or Stop API.
                await ws.send_json(await asyncio.to_thread(complete_status))
                await asyncio.sleep(0.25)
        except (WebSocketDisconnect, RuntimeError, asyncio.CancelledError):
            pass
        # Do not disarm another tab: motion uses its own expiring, authenticated heartbeat.

    @app.get('/api/settings')
    def settings(): return store.config

    @app.post('/api/settings')
    def settings_write(data: dict):
        ensure_stationary()
        if any(v['running'] for v in manager.status().values()):
            raise UserError('Dừng các tiến trình do web quản lý trước khi đổi địa chỉ/cổng.')
        store.save_config(data)
        store.audit('local_settings_changed')
        return {'ok': True, 'note': 'Chỉ cập nhật địa chỉ trên Ubuntu; chưa đổi Wi-Fi trong ESP32.'}

    @app.get('/api/project')
    def project_info(): return inspect_project(store.config)

    @app.post('/api/system/connect')
    def connect_system():
        ensure_stationary()
        # Validate the entire base stack before starting the first component.
        # This guarantees that a LiDAR conflict cannot leave Agent/Odom/EKF/Model
        # half-started with no LiDAR log in the current Web session.
        # Fresh Connect: stop only stale exact project processes left by
        # previous Web sessions, then validate the graph before starting anything.
        manager.prepare_base_stack_for_connect()
        manager.preflight_base_stack()
        results = []
        started = []
        order = ('agent','odom_bridge','ekf_odom','robot_model','lidar')
        try:
            for name in order:
                out = manager.start(name)
                started.append(name)
                results.append({'component':name, 'ok':True, 'message':out['message']})
                time.sleep(0.35)
        except UserError as exc:
            failed = order[len(started)] if len(started) < len(order) else 'unknown'
            results.append({'component':failed, 'ok':False, 'message':str(exc)})
            # Roll back only components started by this Connect request.
            for name in reversed(started):
                try:
                    manager.stop(name)
                except Exception:
                    pass
            return {
                'results': results,
                'rolled_back': True,
                'note': 'Kết nối thất bại nên Web đã rollback các thành phần vừa mở; hệ thống không để ở trạng thái nửa chừng.'
            }
        return {
            'results':results,
            'rolled_back': False,
            'note':'Thứ tự: Agent → Odom(no TF) → EKF V14 → Robot Model → LiDAR. Tất cả đều do phiên Web hiện tại quản lý và có log riêng.'
        }

    @app.post('/api/system/disconnect')
    def disconnect_system():
        motion.stop('Dừng các thành phần do web quản lý.', force=True)
        return {'results': manager.stop_all()}

    @app.get('/api/usb/probe/{board}')
    def usb_probe(board: str): return usb.probe(board)

    @app.get('/api/usb/status')
    def usb_status(): return usb.status()

    @app.get('/api/usb/log')
    def usb_log(): return {'text':usb.log()}

    @app.post('/api/usb/build/{board}')
    def usb_build(board: str, data: dict):
        with active_operations:
            ensure_stationary()
            return usb.build(board,data)

    @app.post('/api/usb/flash')
    def usb_flash(data: dict):
        with active_operations:
            ensure_stationary()
            return usb.flash(data)

    @app.get('/api/audit')
    def audit(): return store.events()

    @app.post('/api/process/{name}/start')
    def process_start(name: str):
        ensure_stationary()
        return manager.start(name)

    @app.post('/api/process/{name}/stop')
    def process_stop(name: str): return manager.stop(name)

    @app.get('/api/process/{name}/log')
    def process_log(name: str): return {'text': manager.log(name)}

    @app.post('/api/slam/activate')
    def activate():
        ensure_stationary()
        if 'slam' not in manager.status() or not manager.status()['slam']['running']:
            raise UserError('Web chỉ kích hoạt phiên SLAM do chính web khởi động.')
        if ros.status()['slam_state'] != 'inactive':
            raise UserError('Chỉ activate khi SLAM ở trạng thái inactive.')
        ros.activate()
        return {'ok': True}

    @app.post('/api/control/arm')
    def arm(request: Request, data: dict):
        if data.get('watchdog_tested') is not True or data.get('clear_area') is not True:
            raise UserError('Phải xác nhận đã thử watchdog/dừng và khu vực an toàn.')
        if usb.status().get('busy'):
            raise UserError('Đang build/nạp USB; điều khiển khóa.')
        if ros.status().get('motor', {}).get('state') not in (None, 1):
            raise UserError('Chỉ nhận quyền khi firmware IDLE. Dừng robot trước.')
        with active_operations:
            key = motion.arm(owner(request), ros.motion_ready())
        store.audit('motion_armed', request.state.session['username'])
        return {'key': key}

    @app.post('/api/control/command')
    def command(request: Request, data: dict):
        try:
            v, w, sent = float(data['v']), float(data['w']), float(data['sent_at_ms'])
        except (KeyError, TypeError, ValueError):
            motion.stop('Dữ liệu điều khiển sai định dạng.')
            raise UserError('Lệnh vận tốc không hợp lệ.')
        if usb.status().get('busy'):
            motion.stop('Cấu hình USB đang hoạt động.')
            raise UserError('Đang cấu hình USB.')
        motion.command(owner(request), str(data.get('key', '')), data.get('seq'),
                       v, w, sent, ros.motion_ready())
        return {'ok': True}

    @app.post('/api/control/stop')
    def stop():
        motion.stop('Người vận hành nhấn Dừng.', force=True)
        store.audit('software_stop')
        return {'ok': True, 'note': 'Dừng phần mềm, không thay thế dừng khẩn vật lý.'}

    @app.get('/api/map/live')
    def map_live():
        rev, msg = ros.current_map()
        if msg is None:
            return {'available': False, 'revision': 0}
        return {'available': True, 'revision': rev, 'meta': live_meta(msg)}

    @app.get('/api/map/live.png')
    def map_image():
        rev, msg = ros.current_map()
        if msg is None:
            raise HTTPException(404, 'Chưa có dữ liệu map.')
        with active_operations:
            if map_cache['revision'] != rev:
                map_cache.update(revision=rev, image=live_map_image(msg))
            return Response(map_cache['image'], media_type='image/png')

    @app.get('/api/map/live-clean.png')
    def map_image_clean():
        rev, msg = ros.current_map()
        if msg is None:
            raise HTTPException(404, 'Chưa có dữ liệu map.')
        with active_operations:
            if clean_map_cache['revision'] != rev:
                clean_map_cache.update(revision=rev, image=live_map_clean_image(msg))
            return Response(clean_map_cache['image'], media_type='image/png')

    @app.get('/api/map/ros')
    def map_ros():
        rev, msg, age = ros.current_ros_map()
        if msg is None:
            return {'available': False, 'revision': 0, 'age': None}
        return {'available': True, 'revision': rev, 'age': age, 'meta': live_meta(msg)}

    @app.get('/api/map/ros.png')
    def map_ros_image():
        rev, msg, _age = ros.current_ros_map()
        if msg is None:
            raise HTTPException(404, 'ROS chưa có /map.')
        with active_operations:
            if ros_map_cache['revision'] != rev:
                ros_map_cache.update(revision=rev, image=live_map_image(msg))
            return Response(ros_map_cache['image'], media_type='image/png')

    @app.get('/api/maps')
    def maps(): return store.list_maps()

    @app.get('/api/maps/import-candidates')
    def candidates(): return import_candidates(Path(store.config['import_root']))

    @app.post('/api/maps/import')
    def import_map(data: dict):
        return import_existing(store, str(data.get('key', '')), str(data.get('name', '')))

    @app.post('/api/maps/save')
    def save_map(data: dict):
        with active_operations:
            ensure_stationary()
            name = validate_name(str(data.get('name', '')))
            s = ros.status()
            if s['slam_state'] != 'active' or ros.current_map()[1] is None:
                raise UserError('SLAM phải active và /map phải thực sự có dữ liệu.')
            if s.get('map_receive_age') is None or s['map_receive_age'] > 20:
                raise UserError('Dữ liệu map đã cũ quá 20 giây; chưa lưu phiên này.')
            ident = uuid.uuid4().hex
            directory = store.root / 'maps' / ident
            directory.mkdir(mode=0o700)
            # Never reuse a previous filename or report success based on an old file.
            try:
                ros.save_map(directory / 'map')
                if not (directory/'map.yaml').is_file():
                    raise UserError('Dịch vụ trả lời nhưng chưa có map.yaml. Không báo lưu thành công.')
                record = register_map(store, directory, name, 'slam_service', ident)
            except Exception:
                # Keep failed output for diagnosis; not registered as a valid map.
                raise
            warning = None
            if data.get('posegraph') is True:
                try:
                    ros.serialize(directory / 'graph')
                    if not (directory/'graph.posegraph').exists() or not (directory/'graph.data').exists():
                        raise UserError('Chưa xác nhận đủ graph.posegraph và graph.data.')
                except Exception as exc:
                    warning = 'Ảnh map đã lưu; pose graph chưa xác nhận: ' + str(exc)
            # Saving closes the web-owned mapping session. This intentionally
            # removes the old LIVE map so the next SLAM start is visibly fresh.
            stop_note = None
            try:
                stop_note = manager.stop('slam')['message']
            except Exception as exc:
                ros.end_map_session()
                extra = 'Map đã lưu nhưng SLAM chưa dừng sạch: ' + str(exc)
                warning = (warning + ' | ' + extra) if warning else extra
            store.audit('map_saved', ident)
            return {'map': record, 'warning': warning, 'slam_stopped': stop_note}

    @app.get('/api/maps/{ident}/preview.png')
    def preview(ident: str):
        _record, directory, _display = ensure_display_images(store, ident)
        return Response((directory/'map_clean.png').read_bytes(), media_type='image/png')

    @app.get('/api/maps/{ident}/view/{mode}.png')
    def map_view(ident: str, mode: str):
        _record, directory, _display = ensure_display_images(store, ident)
        if mode not in ('clean', 'technical'):
            raise HTTPException(404, 'Chế độ map không tồn tại.')
        return Response((directory/f'map_{mode}.png').read_bytes(), media_type='image/png')

    @app.get('/api/maps/{ident}/display-meta')
    def map_display_meta(ident: str):
        _record, _directory, display = ensure_display_images(store, ident)
        return display

    @app.get('/api/maps/{ident}/export')
    def export(ident: str):
        record = store.map(ident)
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as archive:
            for f in (store.root/'maps'/ident).iterdir():
                if f.is_file() and not f.is_symlink():
                    archive.write(f, f.name)
            archive.writestr('locations.json', json.dumps({'map_id': ident, 'checksum': record['checksum'],
                'frame_id': 'map', 'locations': store.points(ident)}, ensure_ascii=False, indent=2))
            archive.writestr('metadata.json', json.dumps(record, ensure_ascii=False, indent=2))
        return Response(stream.getvalue(), media_type='application/zip',
            headers={'Content-Disposition': f'attachment; filename="robot_map_{ident[:8]}.zip"'})

    @app.post('/api/localization/start')
    def localization_start(data: dict):
        ensure_stationary()
        ident = str(data.get('map_id', ''))
        record = store.map(ident)
        yaml_path = (store.root / 'maps' / ident / 'map.yaml').resolve()
        # Do not let a transient /map from the previous SLAM/localization
        # session masquerade as the map just selected. The new map_server will
        # repopulate this observation cache through transient-local QoS.
        ros.clear_ros_map_observation()
        result = manager.start_localization(yaml_path, ident, record['name'])
        store.audit('localization_start', f'{ident}:{record["name"]}')
        return {'ok': True, 'map': record, **result}

    @app.post('/api/localization/stop')
    def localization_stop():
        ensure_stationary()
        result = manager.stop('localization')
        store.audit('localization_stop')
        return {'ok': True, **result}

    @app.post('/api/localization/initial-pose')
    def localization_initial_pose(data: dict):
        ensure_stationary()
        ident = str(data.get('map_id', ''))
        record = store.map(ident)
        proc = manager.status().get('localization', {})
        ctx = proc.get('context', {})
        if not proc.get('running') or proc.get('external'):
            raise UserError('Hãy khởi động Localization bằng web trước khi gửi Initial Pose.')
        if ctx.get('map_id') != ident:
            raise UserError('Initial Pose phải thuộc đúng map đang chạy AMCL.')
        try:
            x, y, yaw = (float(data[k]) for k in ('x', 'y', 'yaw'))
        except (KeyError, TypeError, ValueError):
            raise UserError('Cần x, y và yaw cho Initial Pose.')
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise UserError('Initial Pose phải là số hữu hạn.')
        col, row = map_pixel(record['meta'], x, y)
        if not (0 <= col < record['meta']['width'] and 0 <= row < record['meta']['height']):
            raise UserError('Initial Pose nằm ngoài bản đồ đã chọn.')
        pose = ros.publish_initial_pose(x, y, yaw)
        store.audit('initial_pose', f'{ident}:{x:.3f},{y:.3f},{yaw:.3f}')
        return {'ok': True, 'pose': pose, 'note': 'Đã gửi Initial Pose cho AMCL. Theo dõi map→odom và vị trí robot trên bản đồ.'}

    @app.post('/api/rviz/start')
    def rviz_start():
        ensure_stationary()
        return manager.start('rviz')

    @app.post('/api/rviz/stop')
    def rviz_stop():
        return manager.stop('rviz')

    @app.get('/api/maps/{ident}/points')
    def points(ident: str): return store.points(ident)

    @app.post('/api/maps/{ident}/points')
    def point(ident: str, data: dict):
        try:
            x, y = float(data['x']), float(data['y'])
        except (KeyError, TypeError, ValueError):
            raise UserError('Tọa độ điểm không hợp lệ.')
        if not math.isfinite(x) or not math.isfinite(y):
            raise UserError('Tọa độ phải hữu hạn.')
        validate_point_cell(store, ident, x, y)
        pid = store.save_point(ident, data)
        return {'id': pid, 'note': 'Đã lưu ô trống; chưa kiểm tra footprint hoặc khả năng đi tới bằng Nav2.'}

    @app.delete('/api/maps/{ident}/points/{pid}')
    def delete_point(ident: str, pid: str):
        store.delete_point(ident, pid)
        return {'ok': True}

    @app.get('/api/serial/ports')
    def ports(): return serial_tools.ports()

    @app.post('/api/serial/read')
    def serial_read(data: dict):
        with active_operations:
            ensure_stationary()
            if manager.offline: raise UserError('OFFLINE: không mở thiết bị USB thật.')
            return serial_tools.read_log(str(data.get('device', '')), data.get('baud'), data.get('motor_power_off'))

    return app
