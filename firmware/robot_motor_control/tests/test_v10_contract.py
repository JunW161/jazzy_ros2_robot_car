from pathlib import Path
r=Path(__file__).resolve().parents[1]
main=(r/'main/main.cpp').read_text()
drv=(r/'main/robot_drive.cpp').read_text()
meta=(r/'app-colcon.meta').read_text()
cfg=(r/'main/robot_config.h').read_text()
imu=(r/'main/robot_sensors.cpp').read_text()
for pin in ['I2C_SDA_GPIO=15','I2C_SCL_GPIO=16','IMU_INT_GPIO=17','L_RPWM=4','L_LPWM=5','L_EN=6','L_ENC_A=10','L_ENC_B=11','R_RPWM=40','R_LPWM=41','R_EN=42','R_ENC_A=1','R_ENC_B=2']:
 assert pin in cfg,pin
assert 'robot_drive_ff_point_request' in main and 'robot_drive_ff_scale_request' in main
assert 'robot_drive_ff_save_request' in main and 'robot_drive_ff_reset_request' in main
assert 'ffStatus.data.size=43' in main
assert 'rclc_executor_init(&exec,&support.context,9,&alloc)' in main
assert '-DRMW_UXRCE_MAX_SUBSCRIPTIONS=10' in meta
assert '-DRMW_UXRCE_MAX_PUBLISHERS=13' in meta
assert 'nvs_commit' in drv and 'ffSafeToEdit' in drv
assert 'ImuCalibrationV10::Window' in imu
assert 'fabsf(raw.gyro_counts[j]) > 150.0f' not in imu
assert 'rclc_publisher_init_best_effort(&pubWheel' in main
assert 'rclc_publisher_init_best_effort(&pubPid' in main
print('V10 integration/source contract PASS')
