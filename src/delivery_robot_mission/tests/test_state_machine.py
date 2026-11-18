import pytest
from delivery_robot_mission.state_machine import *


def test_full_confirmed_mission():
    s=StateMachine(); s.start(); assert s.state==GOING_TO_PICKUP
    s.pickup_arrived(True); assert s.state==WAITING_PICKUP
    s.confirm_pickup(); assert s.state==GOING_TO_DROPOFF
    s.dropoff_arrived(True); assert s.state==WAITING_DROPOFF
    s.confirm_dropoff(); assert s.state==COMPLETED


def test_auto_mission():
    s=StateMachine(); s.start(); s.pickup_arrived(False); assert s.state==GOING_TO_DROPOFF
    s.dropoff_arrived(False); assert s.state==COMPLETED


def test_reject_double_start():
    s=StateMachine(); s.start()
    with pytest.raises(ValueError): s.start()
