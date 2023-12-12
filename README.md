## DISCLAIMER
Hi this is a port of 3iRobotics lidar ros package to Ros 2(I'm gonna be honest I have no idea how I ported it either).

### How to build
```
cd YOUR_ROS_WS/src
git clone https://github.com/DCZhiRong/delta_lidar.git
cd ..
colcon build --symlink-install
```


### How to run
Default values are listed below
```
###Using ros run
ros2 run delta_lidar delta_lidar_node --ros-args -p baud_rate:=115200 -p frame_id:=laser_frame -p port:=/dev/ttyUSB0

###Using ros launch
ros2 launch delta_lidar lidar.launch.py

###Launch with rviz
ros2 launch delta_lidar view_lidar.launch.py
```



