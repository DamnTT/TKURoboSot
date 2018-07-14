# TKURoboSot
TKU Team participate in FIRA RoboSot league.

## Packages Required
```bash
$ sudo apt-get install ros-kinetic-prosilica-camera
# camera ip is 192.168.16.100
$ sudo apt-get install ros-kinetic-rosbridge-server
$ sudo apt-get install ros-kinetic-web-video-server
```
## Build

Need to build package `imu_3d` and `vision` first, because there are some `msg` need to build first.
After that, build all
```bash
$ cd fira_ws/
$ catkin_make --pkg imu_3d
$ catkin_make --pkg vision
$ catkin_make
```

## Quickly start Robot 

```bash
$ roslaunch fira_launch main_launch.launch

Set robot's vision parameter. When you parameter setting has been completed,you might close this command on terminal.
$ rosrun vision interface_node

$ rosrun strategy FIRA_strategy
```
