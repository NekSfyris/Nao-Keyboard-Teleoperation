# Nao-Keyboard-Teleoperation

A ROS based package implementation for Nao v5 robot teleoperation using the keyboard in C++.
You will be able to control robot's base and head movement in space.

### Prerequisites

You need to have installed :
* OpenCV
* ROS ( with a catkin environment )
* naoqi_driver package

Has been tested for ros-kinetic and opencv 3.3.0 .

### Build

After you downloaded the package, move it in your catkin workspace and :
```
$ cd ~/catkin_ws
$ catkin_make
```

### Run

To run the package first run naoqi_driver in one terminal.
Then in another terminal : 
```
$ roslaunch nao_teleop_keyboard teleop_keyboard.launch 
```
