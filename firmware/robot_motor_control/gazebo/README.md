# Gazebo + ROS 2 Jazzy integration preparation

The V2 source ZIP has no world, URDF diff-drive plugin or LiDAR sensor definition. This project **does not pretend to supply those**: use your existing `delivery_robot_description` URDF + Gazebo plugin setup, and verify actual topic names. It provides a **bridge template**, common topic contracts and practical test steps.

1. Start Gazebo, spawn the URDF model after verifying joint names and wheel/track dimensions. Your project previously used:
   `ros2 run ros_gz_sim create --world default --file ~/ros2_ws/src/delivery_robot_description/urdf/delivery_car_robot.urdf --name delivery_car_robot --z 0.10`
2. Check `gz topic -l`. Find the actual Gazebo model velocity and odometry topics; they are not guaranteed to be names in `bridge_template.yaml`. Gazebo requires the `DiffDrive` system/plugin or another drivetrain controller to drive wheel joints.
3. Edit the YAML accordingly. Then:
   `ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$(pwd)/gazebo/bridge_template.yaml`
4. Check `/sim/odom` with `ros2 topic echo /sim/odom --once`.
5. Sim-only command (real robot must not have a publisher on its `/cmd_vel`):
   `ros2 topic pub -r 20 /sim/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08}, angular: {z: 0.0}}"`
   Then publish zero to stop. Prefer a bounded-duration publisher on a clear simulation scene.
6. To reuse the Python goal controller with Gazebo, remap its interfaces:
   `python3 host_ros2/go_to_pose.py --ros-args -r /cmd_vel:=/sim/cmd_vel -r /odom:=/sim/odom -p enable_motion:=true -p allow_spin:=false`

Later, place real `/scan`, `/imu/data_raw`, `/odom`, `/cmd_vel` behind a common namespace interface, use robot_localization EKF as appropriate and add Nav2. Disable competing `odom -> base_link` TF publishers, e.g. do not run the RViz odom bridge simultaneously with EKF/Simulator TF.

**No collision avoidance** is included in `go_to_pose.py`: use supervised experiments only.
