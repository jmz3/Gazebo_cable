# Gazebo_cable

<img src="/doc/cable_demo07042024.gif" width="650">

A gazebo model of a cable with a hook at the end. Ideally used for simulating any kind of linear deformable object.

## Branches

- **main**: ROS 1 (Noetic) compatible version using catkin
- **dev_ros2**: ROS 2 compatible version using ament_cmake

## Usage

### For ROS 1 (main branch)
To use this model, clone this repository into your workspace and compile it using catkin_make or catkin build. 

### For ROS 2 (dev_ros2 branch)
To use the ROS 2 version, checkout the `dev_ros2` branch and compile it using colcon build.

```bash
git checkout dev_ros2
colcon build --packages-select cable_sim
```

The model has a fixed end and a free end. A velocity controller controls the free end and the position target can be defined using Cartesian coordinates.

### ROS 1 Launch
Use the launch file to spawn the model in Gazebo:
```bash
roslaunch cable_sim cable_standalone.launch
```

### ROS 2 Launch
Use the Python launch file to spawn the model in Gazebo:
```bash
ros2 launch cable_sim cable_standalone.launch.py
```

It contains the model spawner and a target publisher node that can move the end of the cable in the x, y, and z directions.

## Reasons for using Velocity controller
Gazebo is a physics engine. If you directly modify the position of any part of the cable, it may create an infinitely large force between the segments, causing a model collapse.

The velocity controller has a PID backbone. You can modify the kP, kI, and kD parameters in the launch file if you are not satisfied with the response time.
