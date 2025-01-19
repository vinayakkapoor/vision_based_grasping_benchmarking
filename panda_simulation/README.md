# Franka Panda Simulator

Research under Prof Berk Calli (Manipulation and Environmental Robotics Lab, Worcester Polytechnic Institute)

This is a part of benchmarking vision based grasping algorithms project. The repository provides a simulation environment for Franka Panda robot build in Gazebo. The joints are effort controlled. The gripper uses [grasp plugin](https://github.com/JenniferBuehler/gazebo-pkgs "grasp plugin")  for simulating the grasps.

## Installation Instructions
```
sudo apt install python-is-python3 ros-noetic-moveit-commander
sudo apt install ros-<distro>-libfranka
mkdir -p panda_sim_ws/src
cd panda_sim_ws/src
git clone https://github.com/cdbharath/panda_simulation.git
cd ..
catkin build
source devel/setup.bash
```

## How to run:
```
roslaunch panda_simulation panda_simulation.launch  # eye in hand mode (used for our experiments)

# The simulator can also run in overhead camera mode
roslaunch panda_simulation panda_simulation.launch over_head:=true
```

## Note
There are some common packages between simulator repository and benchmarking repository. It is better to have them in separate ROS workspaces. If the manipulator is not able to grasp properly, try tuning the grasp plugin parameters (The instructions are in the references).

## References:
1. [Panda Simulator](https://github.com/erdalpekel/panda_simulation "Panda Simulator")
2. [Grasp plugin](https://github.com/JenniferBuehler/gazebo-pkgs "Grasp plugin")
