# Towards More Robust and Reliable Vision-Based Grasping: A Benchmarking Study
Research under Prof Berk Calli (Manipulation and Environmental Robotics Lab, Worcester Polytechnic Institute) [[YouTube]](https://youtu.be/hmgh5JGP-Ak "[YouTube]")

<!---
#### Video Demo of the benchmarking experiemnts
<a href="https://youtu.be/hmgh5JGP-Ak" target="_blank" rel="noopener noreferrer">
    <img src="https://img.youtube.com/vi/hmgh5JGP-Ak/0.jpg" alt="Video Demo" width="800" height="500">
-->
   
<a href="https://github.com/cdbharath/benchmarking_vision_based_grasping/assets/28064971/70457435-5f36-4c3d-a63c-441cc0bad9c4">
    <img src="https://github.com/cdbharath/benchmarking_vision_based_grasping/assets/28064971/70457435-5f36-4c3d-a63c-441cc0bad9c4" alt="benchmarking1" style="width:1000px; height:auto;" />
</a>
<br> <br>

This repository provides a pipeline for benchmarking vision based grasp detection algorithms. The pipeline performs manipulation (pick and place) of objects based on 3DOF/6DOF grasps detected by the grasp detection algorithms. The repository is tested with Franka Panda robot with Intel Realsense camera mounted on the end effector. 

It is assumed that the gripper approaches the object only from the top. The pointcloud/depth image is captured from the birdseye view. These constraints will be lifted eventually. 

Refer the [simulator repository](https://github.com/cdbharath/panda_simulation "simulator repository") to run the benchmarking procedures in the simulation environment. 
Refer the [grasp detection algorithms](https://github.com/cdbharath/grasp_synthesis "grasp detection algorithms") to find the algorithms that we tested with this pipeline

## Installation Instructions
```
mkdir -p benchmarking_ws/src
cd benchmarking_ws/src
git clone https://github.com/cdbharath/benchmarking_vision_based_grasping.git
cd ..
catkin build
source devel/setup.bash
```

## How to run (Perferably in order):
```
### Bringup controllers/planner for Franka Panda Robot (Check simulator repository to use simulator instead)
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<ip> load_gripper:=true
rosrun moveit_adapter moveit_adapter.py

### Run your grasp detection algorithm (Check grasp detection algorithms repository)
rosrun <grasp_det_pkg> <grasp_det_node>  

# Run any one of the commands as per the requirement to start benchmarking
roslaunch benchmarking_grasp run_benchmark.launch  # To run on real robot, depth input                                             
roslaunch benchmarking_grasp run_benchmark.launch sim_mode:=true  # To run in simulator, depth input
roslaunch benchmarking_grasp run_benchmark.launch point_cloud_input:=true align_depth:=false  # To run on real robot, point cloud input
roslaunch benchmarking_grasp run_benchmark.launch sim_mode:=true point_cloud_input:=true # To run in simulator, point cloud input
```

## Features 
1. Script to find depth ROI (```roslaunch benchmarking_grasp find_depth_roi.launch <args>:=<values>```)
2. Script to find point cloud ROI (```roslaunch benchmarking_grasp find_point_cloud_roi.launch <args>:=<values>```)
3. ```configuration.yaml``` exposes parameters to customize the setup/environment. ```example_configuration``` has sample configuration files that were used for the experimemnts.
4. ```benchmarking.yaml``` exposes parameters specific to the benchmarking protocols and objects to be simulated
5. The configuration file and launch file arguments can be used to turn on/off the preprocessing techiques such as depth completion, filtering ROI e.t.c.
6. Grasps can be visualized with ```visualize``` topic from ```rqt_image_view```
7. Enable video recording with external cameras by setting ```record_video:=true``` 
