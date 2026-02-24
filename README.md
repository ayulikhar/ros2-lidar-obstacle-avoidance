# ROS2 Lidar-Based A* Navigation with Reactive Obstacle Avoidance
## Overview

This project implements a complete navigation stack in ROS2 (Humble) including:
- Custom A* global planner
- Path publishing using nav_msgs/Path
- Waypoint-based path following
- FSM-based reactive obstacle avoidance using sensor_msgs/LaserScan
- Simulation in Gazebo with TurtleBot3

## Architecture

Global Planner (A*)
- Path Publisher
- Path Follower (Odometry-based)
- Reactive Avoidance (FSM + hysteresis)

## Features
- Standalone A* grid planner
- Finite-state machine for obstacle handling
- Yaw-based avoidance commitment
- Oscillation prevention logic
- Gazebo-tested in dynamic scenarios

## How to Run
cd ~/ros2_ws
colcon build
source install/setup.bash

# Terminal 1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2
ros2 run astar_nav_cpp astar_planner_node

# Terminal 3
ros2 run astar_nav_cpp path_follower_node

## Simulation Environment
- ROS2 Humble
- Gazebo
- TurtleBot3 Burger

## Future Improvements
- Dynamic costmap integration
- Replanning during deviation
- Nav2 comparison benchmark
- RViz visualization markers
