# c200_ros2

ROS2 node for Free-optics (FocusRay) C200 series Ethernet based LiDAR sensor.

Build :
-
```colcon build --symlink-install --packages-select c200_ros2```

Run :
-
```ros2 launch c200_ros2 c200.launch.py```

# TODO
- Fix the 0 meter dot ghost data.
- Only tested with a single C205 sensor, need to check other model (C206, C207)
