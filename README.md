## MPC Point Stablization
This repository contains MPC ROS package implemented in Python using CasADi toolbox. The package was tested and verified on Ubuntu 16.04 and ROS Kinetic.

# Description
- The current implementation takes a goal pose and tries to drive the ego vehicle to that goal. 
- There are no obstacles or any other vehicles implemented in this simulation.
- The initial goal pose **(x, y, theta)** is set to **(20, 5, 0)** and can be changed easily.

- The package contains a URDF ackerman vehicle model dynamic simulations using Gazebo. 
- The package contains two ROS nodes implemented in Python: 
    - **Ackerman_MPC_Point_Stabilization** node recieves current vehicle state from Gazebo, formulate the optimal control problem, and solves for the optimal control actions.
    - **Bicycle2Ackerman** node maps the obtained control actions into all-drive vehicle inputs and publishes them to Gazebo.

# Installation

Clone the repo:
``` bash
git clone https://github.com/MohamedDaoud1/mpc_go2goal
```

We also need to install the following three packages:

``` bash
sudo apt-get install ros-kinetic-ros-control
```

``` bash
sudo apt-get install ros-kinetic-ros-controllers
```

``` bash
sudo apt-get install ros-kinetic-gazebo-ros-control 
```

Then build using catkin make:
``` bash
catkin_make
```
Finally source the new package:

``` bash
source devel/setup.bash
```

# Running the simulation

To run the simulation:

``` bash
roslaunch mpc_sim Launch_Simulation.launch 
```

To change the goal pose (change x, y, & theta values)

``` bash
rostopic pub setGoal mpc_sim/Goal_Ackerman "x: 0.0
y: 5.0
theta: 0.0" 
```

This work is part my master's thesis available at https://uwspace.uwaterloo.ca/handle/10012/16436
