"""Static cross-module contract tests; these are not hardware tests."""
from pathlib import Path
base=Path(__file__).resolve().parents[1]
config=(base/'main/robot_config.h').read_text()
main=(base/'main/main.cpp').read_text()
drive=(base/'main/robot_drive.cpp').read_text()
for token in ('I2C_SDA_GPIO=15','I2C_SCL_GPIO=16','IMU_INT_GPIO=17',
              'TOF_XSHUT_GPIO=47','TOF_INT_GPIO=21',
              'L_RPWM=4','L_LPWM=5','L_EN=6','R_RPWM=40','R_LPWM=41','R_EN=42',
              'TRACK_M=0.195f'):
    assert token in config,token
assert '8 publishers' in main
assert 'executor_init(&exec,&support.context,3' in main
assert '/robot/bench_arm' in main
assert 'safetyMsg.data.size=12' in main
assert 'safetyData[8]=r.interlockReason' in main
assert 'cleanup();vTaskDelete(nullptr);return;' in main
assert 'benchJustEnded' in drive
assert 'BENCH_ARMED' in drive
assert 'if(v>0.001f && !benchMode)' in drive
assert 'FRONT_UNKNOWN' in drive
print('PASS: V8 source contracts and unchanged pin map')
