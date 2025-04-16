# Gazebo_cable

<img src="/doc/cable_demo07042024.gif" width="650">

A gazebo model of a cable with a hook at the end. Ideally used for simulating any kind of linear deformable object.



## Usage
To use this model, clone this repository into your workspace and compile it using catkin_make or catkin build. 

The model has a fixed end and a free end. A velocity controller controls the free end and the position target can be defined using Cartesian coordinates.

Use the launch file to spawn the model in Gazebo. It contains the model spawner and a target publisher node that can move the end of the cable in the x, y, and z directions.

## Reasons for using Velocity controller
Gazebo is a physics engine. If you directly modify the position of any part of the cable, it may create an infinitely large force between the segments, causing a model collapse.

The velocity controller has a PID backbone. You can modify the kP, kI, and kD parameters in the launch file if you are not satisfied with the response time.
