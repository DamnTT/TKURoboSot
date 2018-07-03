# TKURoboSot
TKU Team participate in FIRA RoboSot league.

## Packages Required
```bash
$ sudo apt-get install ros-kinetic-prosilica-camera
# camera ip is 192.168.16.100
$ sudo apt-get install ros-kinetic-rosbridge-server
$ sudo apt-get install ros-kinetic-web-video-server

## Build

Need to build package `imu_3d` and `vision` first, because there are some `msg` need to build first.
After that, build all
```bash
$ cd fira_ws/
$ catkin_make --pkg imu_3d
$ catkin_make --pkg vision
$ catkin_make
```
