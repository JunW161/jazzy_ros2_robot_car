import json
import math
import time
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from PIL import Image
from fastapi.testclient import TestClient
from delivery_robot_web.core import Store, MotionLease, UserError, password_verify, password_hash, DEFAULT_CONFIG
from delivery_robot_web.ros_adapter import OfflineAdapter, RosAdapter
from delivery_robot_web.processes import ProcessManager
from delivery_robot_web.app import create_app
from delivery_robot_web.maps import import_candidates, import_existing, map_pixel, live_map_image, validate_point_cell

@pytest.fixture
def store(tmp_path):
    s=Store(tmp_path/'state')
    s.set_admin('admin','robot-test-password-2026')
    return s

@pytest.fixture
def client(store):
    motion=MotionLease(store.config)
    ros=OfflineAdapter(store.config,motion)
    manager=ProcessManager(store,ros,offline=True)
    app=create_app(store,ros,motion,manager)
    with TestClient(app,base_url='http://127.0.0.1:8080') as c:
        yield c

@pytest.fixture
def auth(client):
    h={'Origin':'http://127.0.0.1:8080'}
    r=client.post('/api/login',json={'username':'admin','password':'robot-test-password-2026'},headers=h)
    assert r.status_code==200
    return {**h,'X-CSRF-Token':r.json()['csrf']}


def sample_map(root):
    root.mkdir(parents=True,exist_ok=True)
    im=Image.new('L',(10,8),254)
    im.putpixel((0,0),0)
    im.putpixel((1,0),205)
    im.save(root/'floor.pgm')
    (root/'floor.yaml').write_text('image: floor.pgm\nresolution: 0.1\norigin: [-1.0, -2.0, 0.0]\nnegate: 0\nfree_thresh: 0.25\noccupied_thresh: 0.65\n')
    return root


def imported(store,tmp_path):
    root=sample_map(tmp_path/'oldmaps')
    store.config['import_root']=str(root)
    rows=import_candidates(root)
    assert len(rows)==1
    return import_existing(store,rows[0]['key'],'Phòng A')


def test_passwords():
    hashed=password_hash('valid-password-123')
    assert 'valid-password' not in hashed
    assert password_verify('valid-password-123',hashed)
    assert not password_verify('wrong',hashed)
    assert not password_verify('x','corrupt')
    with pytest.raises(UserError):password_hash('123')


def test_session_logout(store):
    token,csrf=store.login('admin','robot-test-password-2026')
    assert store.session(token)['csrf']==csrf
    store.logout(token)
    assert store.session(token) is None


def test_local_settings(store):
    store.save_config({'lidar_ip':'192.168.1.153','agent_port':8888})
    assert json.loads(store.config_path.read_text())['agent_port']==8888
    for data in ({'lidar_ip':'x; rm -rf /'},{'lidar_ip':'127.0.0.1'},{'lidar_port':1},{'motor_project':'/tmp/x'}):
        with pytest.raises(UserError):store.save_config(data)


def test_motion_lease_expiry():
    now=[100.0]
    m=MotionLease(DEFAULT_CONFIG.copy(),lambda:now[0])
    key=m.arm('one',True)
    m.command('one',key,1,.5,-.7,time.time()*1000)
    from delivery_robot_web.motion_profile import wheel_rpms
    v,w,armed=m.output(True)
    assert armed and v>0 and w<0
    assert max(abs(x) for x in wheel_rpms(v,w,m.config))==pytest.approx(40.0)
    now[0]+=.31
    assert m.output(True)==(0.,0.,False)
    with pytest.raises(UserError):m.command('one',key,2,.1,0,time.time()*1000)


def test_motion_exclusive_and_stop():
    m=MotionLease(DEFAULT_CONFIG.copy())
    with pytest.raises(UserError):m.arm('owner',False)
    key=m.arm('owner',True)
    with pytest.raises(UserError):m.arm('second',True)
    with pytest.raises(UserError):m.command('second',key,1,.1,0,time.time()*1000)
    m.command('owner',key,1,.1,0,time.time()*1000)
    with pytest.raises(UserError):m.command('owner',key,1,.1,0,time.time()*1000)
    assert m.output(False)==(0,0,False)


def test_stale_and_nan_command():
    m=MotionLease(DEFAULT_CONFIG.copy())
    key=m.arm('owner',True)
    with pytest.raises(UserError):m.command('owner',key,1,.1,0,(time.time()-2)*1000)
    assert not m.status()['armed']
    key=m.arm('owner',True)
    with pytest.raises(UserError):m.command('owner',key,1,float('nan'),0,time.time()*1000)
    assert not m.status()['armed']


def test_motion_health_decisions():
    r=RosAdapter.__new__(RosAdapter);r.config=DEFAULT_CONFIG.copy()
    s={'ros_ok':True,'odom_publishers':1,'cmd_subscribers':1,'other_cmd_publishers':[],
       'motor_cmd_publishers':['/delivery_robot_cmd_mux'],
       'motor_cmd_receivers':['/delivery_robot_motor'],
       'odom':{'stamp_age':.05,'receive_age':.06,'valid':True},
       'motor':{'safety_age':.05,'wheel_age':.05,'state':1,'rpm_l':0.,'interlock':1,'bench':False}}
    assert r.motion_ready(s)
    s['odom']['stamp_age']=.6
    assert not r.motion_ready(s)
    s['odom']['stamp_age']=.05;s['other_cmd_publishers']=['teleop_keyboard']
    assert not r.motion_ready(s)
    s['other_cmd_publishers']=[];s['odom_publishers']=0
    assert not r.motion_ready(s)


def test_map_import_identity_points(store,tmp_path):
    record=imported(store,tmp_path)
    assert record['meta']['width']==10
    assert len(record['checksum'])==64
    pid=store.save_point(record['id'],{'name':'Nhận A','kind':'pickup','x':-.5,'y':-1.5,'yaw':4*math.pi})
    point=store.points(record['id'])[0]
    assert point['id']==pid and abs(point['yaw'])<1e-10
    store.delete_point(record['id'],pid)
    assert store.points(record['id'])==[]
    assert (tmp_path/'oldmaps'/'floor.pgm').exists()


def test_free_unknown_occupied_cells(store,tmp_path):
    record=imported(store,tmp_path);ident=record['id']
    validate_point_cell(store,ident,-.55,-1.55)
    for x,y in [(-.95,-1.25),(-.85,-1.25),(30,30)]:
        with pytest.raises(UserError):validate_point_cell(store,ident,x,y)


def test_map_rotation():
    meta={'origin':[2,3,math.pi/2],'resolution':.1,'height':10}
    # local cell (2,3), center at (.25,.35) rotated +90: world (1.65,3.25)
    assert map_pixel(meta,1.65,3.25)==(2,6)


def test_live_png_orientation():
    from io import BytesIO
    msg=NS(info=NS(width=2,height=2),data=[0,100,-1,50])
    png=Image.open(BytesIO(live_map_image(msg)))
    assert [png.getpixel((x,y)) for y in range(2) for x in range(2)]==[205,127,254,0]


def test_import_path_escape(store,tmp_path):
    root=sample_map(tmp_path/'oldmaps')
    (tmp_path/'outside.pgm').write_bytes((root/'floor.pgm').read_bytes())
    (root/'bad.yaml').write_text('image: ../outside.pgm\nresolution: 0.1\norigin: [0,0,0]')
    assert len(import_candidates(root))==1
    with pytest.raises(UserError):store.map('../../etc/passwd')


def test_http_auth_and_csrf(client,auth):
    assert client.get('/').status_code==200
    assert client.get('/api/status').json()['mode']=='OFFLINE'
    assert client.post('/api/control/stop',json={},headers={'Origin':'https://evil.invalid'}).status_code==403
    assert client.post('/api/control/stop',json={},headers={'Origin':'http://127.0.0.1:8080'}).status_code==403
    assert client.post('/api/control/stop',json={},headers=auth).status_code==200
    assert client.post('/api/control/arm',json={'watchdog_tested':True,'clear_area':True},headers=auth).status_code==409
    assert client.post('/api/logout',json={},headers=auth).status_code==200
    assert client.get('/api/status').status_code==401


def test_http_no_unauthorized_control(client):
    assert client.get('/api/settings').status_code==401
    assert client.post('/api/process/agent/start',json={}).status_code==401
    assert client.post('/api/login',json={'username':'admin','password':'robot-test-password-2026'},headers={'Origin':'http://evil.test'}).status_code==403


def test_login_rate_limit(client):
    for _ in range(5):
        assert client.post('/api/login',json={'username':'missing','password':'wrong'},headers={'Origin':'http://127.0.0.1:8080'}).status_code==401
    assert client.post('/api/login',json={},headers={'Origin':'http://127.0.0.1:8080'}).status_code==429


def test_http_offline_actions(client,auth):
    for name in ('slam','agent','lidar','odom_bridge'):
        assert client.post(f'/api/process/{name}/start',json={},headers=auth).status_code==409
    assert client.post('/api/maps/save',json={'name':'test'},headers=auth).status_code==409
    assert client.get('/api/map/live').json()['available'] is False
    assert client.post('/api/serial/read',json={'device':'/etc/passwd','baud':115200},headers=auth).status_code==409


def test_http_import_point_export(store,client,auth,tmp_path):
    record=imported(store,tmp_path);ident=record['id']
    result=client.post(f'/api/maps/{ident}/points',headers=auth,json={'name':'Giao A','kind':'delivery','x':-.5,'y':-1.5,'yaw':1.57})
    assert result.status_code==200
    assert len(client.get(f'/api/maps/{ident}/points').json())==1
    assert client.get(f'/api/maps/{ident}/preview.png').headers['content-type']=='image/png'
    data=client.get(f'/api/maps/{ident}/export')
    from io import BytesIO
    import zipfile
    with zipfile.ZipFile(BytesIO(data.content)) as z:
        assert {'map.pgm','map.yaml','metadata.json','locations.json'} <= set(z.namelist())
        assert json.loads(z.read('locations.json'))['map_id']==ident


def test_websocket_session(client,auth):
    with client.websocket_connect('ws://127.0.0.1:8080/ws/status',headers={'Origin':'http://127.0.0.1:8080'}) as ws:
        data=ws.receive_json()
        assert data['mode']=='OFFLINE'
        assert data['motion']['armed'] is False


def test_process_ownership(store, monkeypatch):
    m=MotionLease(store.config);r=OfflineAdapter(store.config,m);pm=ProcessManager(store,r)
    monkeypatch.setattr(pm.agent, 'snapshot', lambda **kw: {'installed': False, 'running': False})
    monkeypatch.setattr(pm, 'udp_in_use', lambda port: False)
    monkeypatch.setattr(pm.agent, 'stop', lambda: pytest.fail('No helper stop for an absent Agent'))
    monkeypatch.setattr(pm, '_stop_owned_child', lambda process: pytest.fail('No owned process to signal'))
    result = pm.stop('agent', cascade=False)
    assert result['ok'] and result['results'][0]['already_stopped']
    with pytest.raises(UserError):pm.start('shell')
    assert pm.status()['agent']['running'] is False


def test_map_v4_http_display_derivatives(store, client, auth, tmp_path):
    record = imported(store, tmp_path); ident = record['id']
    for mode in ('clean', 'technical'):
        response = client.get(f'/api/maps/{ident}/view/{mode}.png')
        assert response.status_code == 200
        assert response.headers['content-type'] == 'image/png'
    meta = client.get(f'/api/maps/{ident}/display-meta').json()
    assert meta['version'] == 4
    assert meta['clean_visual_only'] is True
    assert meta['navigation_map_modified'] is False
    assert client.get('/api/map/ros').json()['available'] is False
