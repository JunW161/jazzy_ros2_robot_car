from pathlib import Path
p=Path(__file__).resolve().parents[1]
cfg=(p/'main/robot_config.h').read_text()
drv=(p/'main/robot_drive.cpp').read_text()
sensors=(p/'main/robot_sensors.cpp').read_text()
assert 'HEADING_DEFAULT_ENABLED=false' in cfg
assert 'HeadingGuard::Monitor headingGuard' in drv
assert 'headingGuard.update(' in drv
assert 'headingGuard.blocked()' in drv
assert 'const uint64_t sensorNow=esp_timer_get_time()' in drv
assert 'sensorNow-imu.time_us<=80000ULL' in drv
assert 'const bool settledL=' in drv and 'const bool settledR=' in drv
assert 'fabsf(newL-rampL)<0.1f' not in drv
assert 'fabsf(newR-rampR)<0.1f' not in drv
assert 'variance<=25.0f*25.0f' in sensors
assert 'fabsf(raw.gyro_counts[j]) <= 150.0f' in sensors
print('Heading V2 source contract PASS')
