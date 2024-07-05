# Gazebo_cable
A gazebo model of a cable with a hook at the end. Ideally used for simulating any kind of linear deformable object.

## Usage
To use this model, clone this repository into your workspace and compile it using catkin_make or catkin build. 

The model has a fixed end and a free end. The free end is controlled by a velocity controller and the position target can be defined using Cartesian coordinates.

Use the launch file to spawn the model in gazebo. It contains the model spawner and a target puublisher node that can move the end of the cable in the x, y and z directions.
