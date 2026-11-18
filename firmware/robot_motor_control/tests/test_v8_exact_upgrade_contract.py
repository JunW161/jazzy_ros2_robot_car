from pathlib import Path
r=Path(__file__).resolve().parents[1]
cfg=(r/'main/robot_config.h').read_text()
drive=(r/'main/robot_drive.cpp').read_text()
main=(r/'main/main.cpp').read_text()
assert 'LAUNCH_ELIGIBLE_MIN_RPM=8.0f' in cfg
assert 'fabsf(newL)>=LAUNCH_ELIGIBLE_MIN_RPM' in drive
assert 'fabsf(newR)>=LAUNCH_ELIGIBLE_MIN_RPM' in drive
assert 'forward_limit(' not in drive
assert 'yawCtl.reset();' not in drive
assert 'yawCtl.observe(' in drive
assert 'rclc_publisher_init_best_effort(&pubWheel' in main
assert 'rclc_publisher_init_best_effort(&pubPid' in main
assert 'float wheelData[10]' in main and 'float pidStatusData[8]' in main
assert 'headingMsg.data.size=38' in main
print('PASS: local V8 upgrade contract')
