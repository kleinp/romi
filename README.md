# romi
ROS project for Pololu Romi robot, with Gazebo simulation capability. A work in progress since 8/15/2017. The goals of this project are:

1. Create a Gazebo model of the Romi robot
1. Write plugins to interface the model with ROS, specifically
    - Driving the wheels
    - Reading distance sensor measurements
    - Think about hooks for other sensors, such as touch, or downward facing light
1. Write ROS code to manually move the Gazebo model
1. Write ROS code to build a map from simulated sensor data
1. Write ROS code to have the robot explorer a simulated room and build a map by itself
1. Purchase actual hardware and repeat step 5, but with an actual room (this is a big step)
1. More advanced ROS...
    - Maze navigation
    - Integrating depth camera and making a 3D view of room

## Environment setup

1. I'm using Ubuntu 16.04 LTS
1. Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
1. Install [Gazebo 8.1.1](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
1. I'm using Visual Studio Code editor

## Setting up terminal
In your ~/.bashrc file, add the following. Note the first line may already be present after installing ROS. Adjust the second line as necessary for the path to your workspace folder.

```
source /opt/ros/kinetic/setup.bash
source /home/kleinp/romi/catkin_ws/devel/setup.bash
```

After adding this, whenever you open a terminal it will be set up and read to go with both Gazebo and ROS!