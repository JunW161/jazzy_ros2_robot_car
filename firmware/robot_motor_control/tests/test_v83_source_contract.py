from pathlib import Path
base=Path(__file__).resolve().parents[1]
cfg=(base/'main/robot_config.h').read_text()
code=(base/'main/robot_drive.cpp').read_text()
main=(base/'main/main.cpp').read_text()
hdr=(base/'main/yaw_outer.h').read_text()
for token in ('I2C_SDA_GPIO=15','I2C_SCL_GPIO=16','IMU_INT_GPIO=17',
              'TOF_XSHUT_GPIO=47','TOF_INT_GPIO=21',
              'L_RPWM=4','L_LPWM=5','L_EN=6','R_RPWM=40','R_LPWM=41','R_EN=42',
              'TRACK_M=0.195f'):
    assert token in cfg,token
assert 'NearStopGate' not in code
assert 'should_stop_forward' not in code
assert 'NEAR_FIELD_STOP_ENABLED' not in cfg
assert 'VL6180X: OBSERVATION ONLY' in main
assert 'rclc_publisher_init_best_effort(&pubRange' in main
assert '/robot/heading_status' in main
assert 'headingMsg.data.size=38' in main
assert '11 publishers' in main
assert '/robot/trajectory_status' in main
assert 'yawCtl.observe(imu.gz,imu.time_us,imu.seq,imu_ok)' in code
assert 'yawCtl.inactive();' in code
assert 'yawCtl.reset();' not in code
assert 'yaw_+=0.5f*(gyro_+gz)*dt' in hdr
assert 'ControllerMath::differential_rpms' in code
assert 'ffDuty(' in code and 'pidDemand(' in code
print('V8.3 source contract PASS: fixed pins, ToF passive, gyro yaw persistent, motor FF/PID retained')
