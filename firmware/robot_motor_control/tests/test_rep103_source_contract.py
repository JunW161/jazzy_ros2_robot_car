from pathlib import Path
root=Path(__file__).resolve().parents[1]
cfg=(root/'main/robot_config.h').read_text()
main=(root/'main/main.cpp').read_text()
drv=(root/'main/robot_drive.cpp').read_text()
yaw=(root/'main/yaw_outer.h').read_text()
assert 'LOCAL_CROSSTRACK_ENABLED=false' in cfg
assert 'observeTravel(' in drv and 'r.pathCrossTrack=pathCte' in drv
assert '/robot/trajectory_status' in main and 'trajectoryMsg.data.size=14' in main
assert 'headingMsg.data.size=38' in main
assert 'pidStatus.data.size=8' in main and 'wheelMsg.data.size=10' in main
assert 'NEAR_FIELD_STOP_ENABLED' not in cfg and 'forward_limit(' not in drv
assert 'float path_corr' in yaw and 'sinf(new_heading)' in yaw
assert 'CMD_TIMEOUT_MS=800' in cfg
assert 'L_RPWM=4' in cfg and 'R_RPWM=40' in cfg
assert 'TRACK_M=0.195f' in cfg and 'RADIUS_M=0.044f' in cfg
assert '-DRMW_UXRCE_MAX_SUBSCRIPTIONS=10' in (root/'app-colcon.meta').read_text()
print('REP103 ROS contracts and hardware pins PASS')
