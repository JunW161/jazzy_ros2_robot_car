# V7 extension
The ROS2 package remains the same EKF implementation unless installed as
`delivery_robot_sensor_fusion_v7` with INSTALL_V7_SAFE.py.

`/front_range` is a VL6180X short-range auxiliary observation in `front_range_link`.
Do not publish a fictitious front_range_link TF. Measure the optical center and
orientation relative to base_link, then publish it via URDF or a single static TF.

The LiDAR on ESP32-S3 #2 is **not** part of this firmware. Later integrate `/scan`
with SLAM/AMCL and Nav2 costmap/collision_monitor; LiDAR owns main obstacle
awareness, VL6180X supplements immediate frontal proximity.

Do not launch two EKF nodes or two sources of `odom -> base_link`.
