# Turtlesim Waypoints ROS Package

First assignment for ME 495: Embedded Systems in Robotics, Northwestern University, Fall 2020.

## Overview

This package contains nodes that cause a simulated turtle (from the package turtlesim) to move through a series of waypoints. The turtle uses feedback control to correct its position throughout the trajectory between waypoints. After reaching a waypoint, the turtle turns until it faces directly at the next waypoint. 

A second turtle is used to mark the locations of the waypoints prior to the main turtle moving.

The package contains a custom velocity message containing only the planar forward and planar rotational velocities of the turtle, which are the two components of a 6D twist that a differential drive robot can control.

## Demo

https://youtu.be/OozKJdEjlns 

## Usage Instructions

1. Create a new workspace and clone the demonstration code.
```Shell
# Create a new workspace
mkdir -p ws/src

# clone the demonstration code
cd ws/src
git clone https://github.com/ctsaitsao/turtlesim-waypoints.git turtlesim-waypoints

# return to ws root
cd ../
```

2. Build the workspace and activate it.
```Shell
catkin_make install
. devel/setup.bash
```

3. Launch the nodes.
```Shell
roslaunch turtlesim-waypoints waypoint_follow.launch
```

## Configuration Options

To change the locations of the waypoints, edit the `config/waypoint.yaml` file.

The `waypoint` node has configurable private parameters, which can be edited in the `launch/waypoint_follow.launch` file. See node docstring for parameter descriptions.
```
<node name="waypoint" pkg="turtlesim-waypoints" type="waypoint">
    <param name="~kp_linear" value="1.5"/>  
    <param name="~kp_rotational" value="6"/>  
    <param name="~dist_thresh" value="0.01"/>  
    <param name="~angle_thresh" value="0.05"/>  
</node>
```
