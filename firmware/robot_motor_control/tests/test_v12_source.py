#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1]
p=(root/'main/robot_drive.cpp').read_text()
h=(root/'main/robot_drive.h').read_text()
m=(root/'main/main.cpp').read_text()
c=(root/'main/robot_config.h').read_text()
y=(root/'main/yaw_outer.h').read_text()
assert len(p)>40000, 'motor source is empty/incomplete'
assert '#include "heading_enable_v12.h"' in p
assert 'HeadingEnableV12::step' in p
assert 'headingResetRequested=false' in p
assert 'steeringAuthority=y.active?1.0f:0.0f' in p
assert 'headingOwnsSteering?1.0f' not in p
assert 'previousSteeringAuthority!=steeringAuthority' in p
assert 'headingEnableApplyStatus=action.status' in p
assert 'r.headingEnableRxSeq=float(headingEnableRxSeq)' in p
assert 'headingEnableApplyStatus' in h
assert 'float headingData[42]={};' in m
assert 'headingMsg.data.size=42;headingMsg.data.capacity=42;' in m
assert 'headingData[41]=r.headingEnableApplyStatus' in m
assert 'headingData[19]=YawOuter::Controller::HEADING_TRIGGER_RAD' in m
assert 'constexpr int L_RPWM=4, L_LPWM=5, L_EN=6' in c
assert 'constexpr int R_RPWM=40, R_LPWM=41, R_EN=42' in c
assert 'constexpr int I2C_SDA_GPIO=15, I2C_SCL_GPIO=16' in c
assert 'FF_LEFT[FF_COUNT]={55,73,98,119,144,164,187,218,237}' in c
assert 'FF_RIGHT[FF_COUNT]={54,70,94,112,132,156,180,200,223}' in c
assert 'HEADING_TRIGGER_RAD' in y and '0.0209439510239f' in y
print('V12 source checks PASS: preserve user config/yaw + motor/ROS coordination')
