"""Read-only inspection of the user's two workspaces; never exports network secrets."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inspect_project(config: dict) -> dict:
    project = Path(config['motor_project']).expanduser()
    urdf = Path(config['description_urdf']).expanduser()
    report = {'motor_project': str(project), 'description_urdf': str(urdf),
              'checks': [], 'firmware': {}, 'urdf': {}, 'warnings': []}
    def check(label, path):
        p = Path(path).expanduser()
        report['checks'].append({'name': label, 'ok': p.is_file(), 'path': str(p)})
        return p
    for label, path in [
        ('Firmware ESP32 #1', project/'main/main.cpp'),
        ('Cấu hình motor', project/'main/robot_config.h'),
        ('Bridge odom / khớp', project/'host_ros2/odom_tf_joint_bridge.py'),
        ('Mô hình robot hiện tại', urdf),
        ('LiDAR Wi-Fi launcher', config['lidar_script']),
        ('ESP-IDF export', config['idf_export']),
    ]:
        check(label, path)
    conf = project/'main/robot_config.h'
    main = project/'main/main.cpp'
    if conf.is_file():
        content = conf.read_text(errors='replace')
        report['firmware']['config_sha256'] = digest(conf)
        for symbol in ('CMD_TIMEOUT_MS','ODOM_MS','RADIUS_M','TRACK_M','HEADING_DEFAULT_ENABLED'):
            match = re.search(r'\b'+symbol+r'\s*=\s*([^;,]+)', content)
            if match:
                report['firmware'][symbol] = match.group(1).strip()
    if main.is_file():
        content = main.read_text(errors='replace')
        report['firmware']['cmd_twist'] = bool(re.search(r'geometry_msgs\s*,\s*msg\s*,\s*Twist\)', content))
        report['firmware']['safety_topic'] = '"/robot/safety_status"' in content
        report['firmware']['build_label'] = 'Robot_Delivery_V12' if 'Robot_Delivery_V12' in content else 'source khác / chưa xác định'
    sdk = project/'sdkconfig'
    if sdk.exists():
        content = sdk.read_text(errors='replace')
        match = re.search(r'ESP-IDF\) ([\w.\-]+)',content)
        report['firmware']['idf_version_in_sdkconfig'] = match.group(1) if match else 'unknown'
        # Deliberately never return SSID, password, or the complete sdkconfig.
    if urdf.is_file():
        try:
            root = ET.parse(urdf).getroot()
            report['urdf']['name'] = root.attrib.get('name')
            report['urdf']['joint_names'] = [j.attrib['name'] for j in root.findall('joint')]
            report['urdf']['sha256'] = digest(urdf)
            lidar = root.find("joint[@name='lidar_joint']")
            if lidar is not None:
                report['urdf']['lidar_joint'] = {key: lidar.find(key).attrib for key in ('parent','child','origin') if lidar.find(key) is not None}
        except (ET.ParseError, OSError) as exc:
            report['warnings'].append('Không đọc được URDF: '+str(exc))
    launch = urdf.parent.parent/'launch/display.launch.py'
    if launch.exists() and 'joint_state_publisher_gui' in launch.read_text(errors='replace'):
        report['warnings'].append('display.launch.py có joint_state_publisher_gui: chỉ dùng xem mô hình; không chạy GUI khớp cùng encoder thật.')
    report['warnings'].append('TF base_laser do LiDAR launcher cung cấp; lidar_1 trong URDF là frame khác. Không tự thay tọa độ lắp cảm biến.')
    contract_path = Path(__file__).parent/'source_contract.json'
    if contract_path.is_file():
        contract = json.loads(contract_path.read_text())
        comparisons=[]
        micro_root=project.parents[2]
        ros_root=Path(config['ros_workspace']).expanduser()
        for rel, ref in contract['files'].items():
            first, _, rest=rel.partition('/')
            local=(micro_root if first=='micro_ros_ws' else ros_root)/rest
            comparisons.append({'path':str(local),'exists':local.is_file(),
                                'matches_uploaded':local.is_file() and digest(local)==ref['sha256']})
        report['source_comparison']=comparisons
        report['source_matches']=sum(x['matches_uploaded'] for x in comparisons)
        report['source_total']=len(comparisons)
    return report
