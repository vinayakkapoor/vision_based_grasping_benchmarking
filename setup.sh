#!/bin/bash

mkdir ~/grasping_benchmarking
cd ~/grasping_benchmarking

echo "installing apt packages"
sudo apt install ros-noetic-combined-robot-hw ros-noetic-tf-conversions

echo "setting up the benchmarking resources"
mkdir benchmarking_ws
cd benchmarking_ws
mkdir src
cd src 
cp -r ~/mer_lab/ros_ws/src/projects/grasping_benchmarking/benchmarking_vision_based_grasping/ ./
pip install -r ./benchmarking_vision_based_grasping/requirements.txt
cd ..
catkin build
cd ..


echo "setting up the simulation resources"
mkdir panda_sim_ws
cd panda_sim_ws
mkdir src
cd src
cp -r ~/mer_lab/ros_ws/src/projects/grasping_benchmarking/panda_simulation/ ./
pip install -r ./panda_simulation/requirements.txt
cd ..
catkin build
cd ..


echo "setting up the grasp algorithm resources"
mkdir grasp_algo_ws
cd grasp_algo_ws
mkdir src
cd src
cp -r ~/mer_lab/ros_ws/src/projects/grasping_benchmarking/grasp_synthesis/ ./
pip install -r ./grasp_synthesis/requirements.txt
cd ..
catkin build
cd ..

cp ~/mer_lab/ros_ws/src/projects/grasping_benchmarking/benchmark_grasping.sh ~/grasping_benchmarking/
cd ~/grasping_benchmarking/

