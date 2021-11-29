# ROS Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-Default.svg)](https://opensource.org/licenses/MIT)

# Overview
Project is a part of 808X coursework wherein the aim is to get familiarized with the basic concepts of ROS.
Task: Modify the TurtleBot simulation from the lecture and implement a simple walker algorithm much like a Roomba robot vacuum cleaner. The robot should move forward until it reaches an obstacle (but not colliding), then rotate in place until the way ahead is clear, then move forward again and repeat. Select/Modify a Gazebo World that can quickly and frequently test this behavior (add sufficient obstacles and enclose with walls such that robot cannot escape into open, endless space).


# Dependencies
- Ubuntu 20.04
- ROS Noetic

## Building package via command-line
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone https://github.com/viveksood97/dvd_robot.git
cd ../
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
catkin_make
```
## Run
1. Launch using launch fil
```
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash
roslaunch dvd_robot dvd_robot.launch
```

4. Launch using launch file with rosbag recording on
```
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash
roslaunch dvd_robot dvd_robot.launch recordbag:=true
```

## Run cppcheck and cpplint
Run cppcheck: Results are stored in `./results/cppcheck.txt` 
```
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp) > results/cppcheck.txt 2>&1
```

Run cpplint: Results are stored in `./results/cpplint.txt`
```
cpplint $( find . -name \*.hpp -or -name \*.cpp) > results/cpplint.txt 2>&1
```
