from delivery_robot_cmd_mux.logic import Velocity, clamp_velocity, select_velocity


def test_clamp():
    assert clamp_velocity(Velocity(1.0, -5.0), 0.35, 1.2) == Velocity(0.35, -1.2)


def test_stop_on_stale():
    assert select_velocity('NAVIGATION', Velocity(1, 1), Velocity(.2, .3), True, False) == Velocity()


def test_modes():
    assert select_velocity('MANUAL', Velocity(.1, .2), Velocity(.3, .4), True, True) == Velocity(.1, .2)
    assert select_velocity('NAVIGATION', Velocity(.1, .2), Velocity(.3, .4), True, True) == Velocity(.3, .4)
