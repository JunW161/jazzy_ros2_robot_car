from pathlib import Path
p=Path(__file__).resolve().parents[1]
main=(p/'main/main.cpp').read_text(); drv=(p/'main/robot_drive.cpp').read_text()
yaw=(p/'main/yaw_outer.h').read_text();cfg=(p/'main/robot_config.h').read_text()
meta=(p/'app-colcon.meta').read_text()
for token in ('L_RPWM=4','L_LPWM=5','L_EN=6','L_ENC_A=10','L_ENC_B=11',
              'R_RPWM=40','R_LPWM=41','R_EN=42','R_ENC_A=1','R_ENC_B=2',
              'TRACK_M=0.195f','RADIUS_M=0.044f'):
 assert token in cfg,token
assert 'state==State::ACTIVE &&' not in drv.split('// Independent of LAUNCH / ACTIVE:')[1].split('if(!benchMode')[0]
for topic in ('/robot/heading_enable','/robot/heading_pid_set','/robot/heading_status',
              '/robot/pid_set','/robot/pid_status','/robot/wheel_debug'):
 assert topic in main,topic
assert 'headingMsg.data.size=38' in main
assert 'wheelMsg.data.size=10' in main
assert 'pidStatus.data.size=8' in main
assert 'rclc_executor_init(&exec,&support.context,9,&alloc)' in main
assert '-DRMW_UXRCE_MAX_SUBSCRIPTIONS=10' in meta
assert 'headingApplyIfStopped' in drv
assert 'robot_drive_heading_enable_request' in drv
assert 'headingActive' in drv and 'launchSteerL' in drv
assert 'HEADING_TRIGGER_RAD=0.00872664625997f' in yaw
assert 'INTEGRAL_TERM_LIMIT_RADPS=0.045f' in yaw
assert 'forward_limit(' not in drv
print('HEADING PID source contracts PASS')
