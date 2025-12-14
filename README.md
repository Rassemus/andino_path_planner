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

- Andino robot simulation

- Nav2 installed

# How to build

#### 1. Clone this repository

```
git clone git@github.com:Rassemus/andino_path_planner.git
```

# How to run simulation

> [!WARNING]
> If you get the error `ros2: command not found` check that the source is found
> ```
> source /opt/ros/humble/setup.bash
> ```

#### 1. Build and source workspace
```
cd $HOME/andino_path_planner/
colcon build --symlink-install
source install/setup.bash
```

#### 2. Launch the andino robot simulation + Nav2
```
ros2 launch path_planner_example my_andino_astar_launch.py nav2:=True
```


> [!NOTE]
> The following commands may require configuration changes to work.

#### 3. Test path_planner_node

```
ros2 run path_planner_example path_planner_node --ros-args -p use_sim_time:=True
```

##### 3.2 Call service
```
ros2 service call /create_plan create_plan_msgs/srv/CreatePlan "{start: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}}}, goal: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0}}}}"
```