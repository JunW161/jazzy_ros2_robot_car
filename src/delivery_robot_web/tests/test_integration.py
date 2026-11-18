"""Unit/integration tests WITHOUT a ROS installation or physical robot.
USB workers use a fake idf runner; never open a serial port in these tests.
"""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS, ModuleType
import queue
import sys
import threading
import time
import pytest
from delivery_robot_web.core import DEFAULT_CONFIG, MotionLease, Store, UserError
from delivery_robot_web.ros_adapter import RosAdapter, OfflineAdapter
from delivery_robot_web.usb_config import config_values, private_config, source_fingerprint, UsbConfigurator
from delivery_robot_web.project import inspect_project
from delivery_robot_web.processes import ProcessManager

SDK='''# Espressif IoT Development Framework (ESP-IDF) 5.4.4 Project Configuration
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESP_WIFI_SSID="old"
CONFIG_ESP_WIFI_PASSWORD="old-password"
CONFIG_MICRO_ROS_AGENT_IP="192.168.1.144"
CONFIG_MICRO_ROS_AGENT_PORT="8888"
CONFIG_PWM_UNCHANGED=240
'''

def network():return {'ssid':'Robot Wifi','password':'private-password-123','host':'192.168.1.144','port':8888,'motor_power_off':True}
def healthy():
    return {'ros_ok':True,'odom_publishers':1,'cmd_subscribers':1,'other_cmd_publishers':[],
            'motor_cmd_publishers':['/delivery_robot_cmd_mux'],
            'motor_cmd_receivers':['/delivery_robot_motor'],
            'odom':{'stamp_age':.02,'receive_age':.02,'valid':True},
            'motor':{'safety_age':.05,'wheel_age':.05,'state':1,'rpm_l':0.,'interlock':1,'bench':False}}

@pytest.fixture
def project(tmp_path):
    p=tmp_path/'micro_ros_ws/micro_ros_espidf_component/examples/robot_motor_control'
    (p/'main').mkdir(parents=True);(p/'host_ros2').mkdir()
    (p/'main/main.cpp').write_text('Robot_Delivery_V12; geometry_msgs,msg,Twist); "/robot/safety_status"')
    (p/'main/robot_config.h').write_text('constexpr int CMD_TIMEOUT_MS=800; constexpr float RADIUS_M=0.044f, TRACK_M=0.195f; constexpr float LEFT_RADIUS_FACTOR=1.0f, RIGHT_RADIUS_FACTOR=1.0f; constexpr bool USE_NVS_GEOMETRY=false;')
    (p/'host_ros2/odom_tf_joint_bridge.py').write_text('# Original bridge')
    (p/'CMakeLists.txt').write_text('project(test)')
    (p/'sdkconfig').write_text(SDK)
    return p

@pytest.fixture
def usb(tmp_path,project):
    store=Store(tmp_path/'state');store.config['motor_project']=str(project)
    export=tmp_path/'export.sh';export.write_text('# fake')
    store.config['idf_export']=str(export)
    ros=OfflineAdapter(store.config,MotionLease(store.config))
    u=UsbConfigurator(store,ros,offline=False)
    yield u
    u.close()


def test_network_copy_changes_only_allowed_symbols():
    updates=config_values(network(),'esp32_1');edited=private_config(SDK,updates)
    old={x.partition('=')[0]:x for x in SDK.splitlines()};new={x.partition('=')[0]:x for x in edited.splitlines()}
    assert {k for k in old if old[k]!=new[k]}=={'CONFIG_ESP_WIFI_SSID','CONFIG_ESP_WIFI_PASSWORD'}
    assert 'CONFIG_PWM_UNCHANGED=240' in edited

@pytest.mark.parametrize('change',[{'ssid':''},{'ssid':'x\nfoo'},{'password':'123'},{'host':'127.0.0.1'},{'host':'x;echo hi'},{'port':80}])
def test_invalid_network(change):
    data=network();data.update(change)
    with pytest.raises(UserError):config_values(data,'esp32_1')

@pytest.mark.parametrize('original',[SDK.replace('esp32s3','esp32'),SDK+'CONFIG_SECURE_BOOT=y\n',SDK.replace('CONFIG_MICRO_ROS_AGENT_IP','CONFIG_OTHER_IP'),SDK+'CONFIG_ESP_WIFI_SSID="second"\n'])
def test_unsafe_or_wrong_config_refused(original):
    with pytest.raises(UserError):private_config(original,config_values(network(),'esp32_1'))


def test_lidar_network_symbols():
    x=config_values(network(),'esp32_2')
    assert x['CONFIG_LIDAR_TCP_PORT']==8888 and 'CONFIG_MICRO_ROS_AGENT_IP' not in x


def test_probe_does_not_expose_credentials(usb):
    text=json.dumps(usb.probe('esp32_1'))
    assert 'old-password' not in text
    assert usb.probe('esp32_1')['missing_symbols']==[]


def fake_build(u, export, args):
    if args==['--version']:
        (Path(u.job['dir'])/'job.log').write_text('ESP-IDF v5.4.4\n'); return 0
    d=Path(u.job['dir']);(d/'build').mkdir(exist_ok=True)
    (d/'build/project_description.json').write_text('{"app_bin":"test.bin"}')
    (d/'build/test.bin').write_bytes(b'fake-test-image-NOT-FIRMWARE')
    (d/'job.log').write_text('fake build only')
    return 0


def test_build_only_private_config_never_flashes(usb,project,monkeypatch):
    before=source_fingerprint(project);calls=[]
    def runner(export,args):calls.append(args);return fake_build(usb,export,args)
    monkeypatch.setattr(usb,'_run',runner)
    usb.build('esp32_1',network());usb.thread.join(timeout=5)
    assert usb.status()['state']=='built'
    assert all('app-flash' not in cmd for cmd in calls)
    assert source_fingerprint(project)==before
    assert 'private-password' not in json.dumps(usb.status())
    assert (Path(usb.job['dir'])/'sdkconfig').stat().st_mode & 0o777 == 0o600


def test_build_failure_does_not_report_success(usb,monkeypatch):
    monkeypatch.setattr(usb,'_run',lambda *args:1)
    usb.build('esp32_1',network());usb.thread.join(timeout=5)
    assert usb.status()['state']=='failed'
    assert not usb.ros.maintenance


def test_flash_requires_confirmation_and_unchanged_source(usb,project,monkeypatch):
    monkeypatch.setattr(usb,'_run',lambda ex,args:fake_build(usb,ex,args))
    usb.build('esp32_1',network());usb.thread.join(timeout=5)
    with pytest.raises(UserError):usb.flash({})
    monkeypatch.setattr('delivery_robot_web.serial_tools.ports',lambda:[{'device':'/dev/ttyACM0'}])
    monkeypatch.setattr('delivery_robot_web.serial_tools.ensure_available',lambda d:None)
    (project/'main/main.cpp').write_text('changed')
    with pytest.raises(UserError,match='nguồn'):usb.flash({'confirmation':'NAP esp32_1','motor_power_off':True,'board_verified':True,'device':'/dev/ttyACM0'})


def test_app_flash_only_not_erase(usb,monkeypatch):
    calls=[]
    def runner(ex,args):calls.append(args);return fake_build(usb,ex,args)
    monkeypatch.setattr(usb,'_run',runner)
    usb.build('esp32_1',network());usb.thread.join(timeout=5)
    monkeypatch.setattr('delivery_robot_web.serial_tools.ports',lambda:[{'device':'/dev/ttyACM0'}])
    monkeypatch.setattr('delivery_robot_web.serial_tools.ensure_available',lambda d:None)
    usb.flash({'confirmation':'NAP esp32_1','motor_power_off':True,'board_verified':True,'device':'/dev/ttyACM0'})
    usb.thread.join(timeout=5)
    assert usb.status()['state']=='flashed'
    assert calls[-1][-1]=='app-flash'
    assert all('erase-flash' not in cmd for cmd in calls)

@pytest.mark.parametrize('state',[0,4,99])
def test_motor_fault_blocks_control(state):
    r=RosAdapter.__new__(RosAdapter);r.config=DEFAULT_CONFIG.copy();s=healthy();s['motor']['state']=state
    assert not r.motion_ready(s)


def test_stale_safety_bench_and_maintenance_block_control():
    r=RosAdapter.__new__(RosAdapter);r.config=DEFAULT_CONFIG.copy()
    for change in ({'safety_age':1.0},{'wheel_age':1.0},{'bench':True},{'interlock':12}):
        s=healthy();s['motor'].update(change);assert not r.motion_ready(s)
    s=healthy();s['maintenance']=True;assert not r.motion_ready(s)
    assert r.motion_ready(healthy())


def test_gate_preserves_timestamp_waits_tf_and_bounds_buffer(monkeypatch):
    tm=ModuleType('rclpy.time');tm.Time=NS(from_msg=lambda s:s);monkeypatch.setitem(sys.modules,'rclpy.time',tm)
    r=RosAdapter.__new__(RosAdapter);r.config=DEFAULT_CONFIG.copy();r.motion=MotionLease(r.config)
    r.status=lambda:healthy();r.lock=threading.RLock();r.jobs=queue.Queue();r.last_stop=0;r.zeros_remaining=0;r.was_armed=False;r.cmd_pub=None
    r.last_graph=r.last_service=time.monotonic()+100
    from collections import deque
    old=NS(header=NS(stamp=100,frame_id='base_laser'));fresh=NS(header=NS(stamp=200,frame_id='base_laser'))
    r.pending_scans=deque([(time.monotonic()-2,old),(time.monotonic(),fresh)])
    r.gate_dropped=r.gate_forwarded=0;r.lookup=lambda *args:None
    output=[];r.gate_pub=NS(publish=output.append)
    r.tick();assert r.gate_dropped==1 and output==[]
    r.lookup=lambda *args:object();r.tick()
    assert output==[fresh] and output[0].header.stamp==200 and r.gate_forwarded==1


def test_project_report_paths_version_and_no_password(project,tmp_path):
    c=DEFAULT_CONFIG.copy();c['motor_project']=str(project)
    p=tmp_path/'ros2_ws/src/delivery_robot_description/urdf/delivery_car_robot.urdf';p.parent.mkdir(parents=True)
    p.write_text('<robot name="existing"><joint name="wheel_joint_l" type="continuous"/></robot>');c['description_urdf']=str(p)
    result=inspect_project(c)
    assert result['firmware']['idf_version_in_sdkconfig']=='5.4.4'
    assert result['urdf']['joint_names']==['wheel_joint_l']
    assert 'old-password' not in json.dumps(result)


def test_installer_only_adds_web_preserves_robot_source(tmp_path,project):
    release=Path(__file__).resolve().parents[3]
    tool=release/'tools/install_package.py'
    # Installed package tests need not have release tools.
    if not tool.is_file():pytest.skip('Legacy tools/install_package.py absent; Web 6 installer has separate tests/test_release_installer.py coverage.')
    spec=importlib.util.spec_from_file_location('installer',tool);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    home=tmp_path/'home';home.mkdir()
    ros=tmp_path/'ros2_ws';urdf=ros/'src/delivery_robot_description/urdf/delivery_car_robot.urdf';urdf.parent.mkdir(parents=True);urdf.write_text('<robot/>')
    micro=project.parents[2];before=source_fingerprint(project);old_urdf=urdf.read_bytes()
    mod.install(release,ros,micro,home)
    assert (ros/'src/delivery_robot_web/package.xml').is_file()
    assert source_fingerprint(project)==before and urdf.read_bytes()==old_urdf
    with pytest.raises(RuntimeError):mod.install(release,ros,micro,home)
    state=home/'.local/share/delivery_robot_web'
    db=Store(state);db.set_admin('admin','test-kept-password-2026');db.db.close()
    settings=json.loads((state/'config.json').read_text())
    settings.update(max_linear=0.1,max_angular=0.3,max_odom_age=.27,lidar_ip='192.168.1.153')
    (state/'config.json').write_text(json.dumps(settings))
    (state/'maps/saved-map.txt').write_text('unchanged map')
    before_db=(state/'app.sqlite3').read_bytes()
    mod.install(release,ros,micro,home,upgrade=True)
    assert list((home/'.local/share/delivery_robot_web_backups').glob('package_*'))
    cfg=json.loads((state/'config.json').read_text())
    assert cfg['manual_straight_rpm']==40 and cfg['max_linear']==pytest.approx(.184306769)
    assert cfg['max_odom_age']==.27 and cfg['lidar_ip']=='192.168.1.153'
    assert before_db==(state/'app.sqlite3').read_bytes()
    assert (state/'maps/saved-map.txt').read_text()=='unchanged map'
    assert source_fingerprint(project)==before and urdf.read_bytes()==old_urdf


def test_service_request_waits_for_discovery_without_blocking_executor():
    from concurrent.futures import Future
    r=RosAdapter.__new__(RosAdapter);r.jobs=queue.Queue();r.executor_available=lambda:True
    timers=[];ready=[False];destroyed=[]
    class Timer:
        def __init__(self,fn):self.fn=fn;self.cancelled=False
        def cancel(self):self.cancelled=True
    class Client:
        def service_is_ready(self):return ready[0]
        def call_async(self,request):
            f=Future();f.set_result('saved');return f
    client=Client()
    def timer(_,fn):
        x=Timer(fn);timers.append(x);return x
    r.node=NS(create_client=lambda *_:client,create_timer=timer,
              destroy_timer=lambda t:None,destroy_client=destroyed.append)
    output=[]
    t=threading.Thread(target=lambda:output.append(r.request(object,'/save',object(),1)))
    t.start();r.jobs.get(timeout=1)()
    assert not output and t.is_alive()
    ready[0]=True;timers[0].fn();t.join(timeout=1)
    assert output==['saved'] and destroyed==[client] and timers[0].cancelled


def test_service_unavailable_timeout_reports_failure():
    r=RosAdapter.__new__(RosAdapter);r.jobs=queue.Queue();r.executor_available=lambda:True;timers=[];errors=[]
    class Timer:
        def __init__(self,fn):self.fn=fn
        def cancel(self):pass
    client=NS(service_is_ready=lambda:False)
    def timer(_,fn):
        x=Timer(fn);timers.append(x);return x
    r.node=NS(create_client=lambda *_:client,create_timer=timer,
              destroy_timer=lambda t:None,destroy_client=lambda c:None)
    def run():
        try:r.request(object,'/missing',object(),.05)
        except UserError as e:errors.append(str(e))
    t=threading.Thread(target=run);t.start();r.jobs.get(timeout=1)()
    time.sleep(.07);timers[0].fn();t.join(timeout=1)
    assert len(errors)==1 and '/missing' in errors[0]
