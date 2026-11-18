import math
import unittest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host_ros2'))
from pose_math import goal_errors, forward_arc_command, yaw_from_quat,wrap

class TestGoalMath(unittest.TestCase):
    def test_straight(self):
        d,head,yaw=goal_errors(0,0,0,0.50,0,0)
        self.assertAlmostEqual(d,.5)
        self.assertAlmostEqual(head,0)
        self.assertAlmostEqual(yaw,0)
        self.assertAlmostEqual(forward_arc_command(d,head)[1],0)

    def test_yaw_wrap(self):
        self.assertAlmostEqual(wrap(3*math.pi),math.pi,places=12)
        self.assertAlmostEqual(yaw_from_quat(0,0,math.sin(.25),math.cos(.25)),.5)

    def test_no_negative_wheel_in_forward_mode(self):
        for angle in (-math.pi, -.6,0,.6,math.pi):
            v,w=forward_arc_command(.5,angle)
            self.assertGreaterEqual(v-w*.195/2,0)
            self.assertGreaterEqual(v+w*.195/2,0)

    def test_bad_quaternion(self):
        with self.assertRaises(ValueError):yaw_from_quat(0,0,0,0)

if __name__=='__main__':unittest.main()
