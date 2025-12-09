# ROS2 Custom Path Planner (A* + Nav2 + Gazebo Sim)

This project implements a custom global planner for ROS2 Nav2 using a Python-based node.
The planner exposes a create_plan service used by a Nav2 plugin and supports replacing the default straight-line planner with an A*-based path planner with obstacle avoidance.

The project is tested with the Andino robot in Gazebo.

## Features

- Custom ROS2 node that provides a CreatePlan service

- Pluggable path planning logic

- Works with the full Nav2 stack

- Integration with Gazebo simulation

- Example launch workflow

- Modular code so you can extend or replace A*

## Requirements

- ROS2 Humble

- colcon build system

- Andino robot simulation (used via: `ros2 launch andino_gz andino_gz.launch.py nav2:=true`)

- Nav2 installed

# How to build

#### 1. Clone this repository

```
git clone git@github.com:Rassemus/andino_path_planner.git
```

# How to run simulation

If you get the error `ros2: command not found` check that the source is found
```
source /opt/ros/humble/setup.bash
```
Run each command in its own terminal.
#### 1.Start the andino robot simulation + Nav2
```
ros2 launch andino_gz andino_gz.launch.py nav2:=True
```

#### 2. Launch toolbox
```
ros2 launch andino_gz slam_toolbox_online_async.launch.py
```

#### 3. Build and run path_planner
#### 3.1 Build workspace
```
cd $HOME/andino_path_planner/
colcon build --symlink-install
```
##### 3.2 Source workspace
```
source install/setup.bash
```

##### 3.3 Run path planner
```
ros2 run path_planner_example path_planner_node --ros-args -p use_sim_time:=True
```