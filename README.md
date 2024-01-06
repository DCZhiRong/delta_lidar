## DISCLAIMER
Hi this is a port of 3iRobotics'(Also commonly used in Xiaomi's vacuum robots) lidar ros package to Ros 2(I'm gonna be honest I have no idea how I ported it either).

### How to build
```
cd YOUR_ROS_WS/src
git clone https://github.com/DCZhiRong/delta_lidar.git
cd ..
colcon build --symlink-install
```

### Things to note
Default values
```
topic = /scan
frame_id = laser_frame
port = /dev/ttyUSB0
baud_rate = 115200
```


### How to run
```
###Using ros run
ros2 run delta_lidar delta_lidar_node --ros-args -p baud_rate:=115200 -p frame_id:=laser_frame -p port:=/dev/ttyUSB0

###Using ros launch
ros2 launch delta_lidar lidar.launch.py

###Launch with rviz
ros2 launch delta_lidar view_delta_lidar.launch.py
```



