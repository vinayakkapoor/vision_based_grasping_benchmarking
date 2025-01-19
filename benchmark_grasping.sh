#!/bin/bash

tmux new-session -d -s grasping_benchmarking

tmux rename-window -t grasping_benchmarking:0 'Simulation'
tmux select-window -t grasping_benchmarking:0
tmux send-keys 'source  "./venv/bin/activate"' C-m cd ~/grasping_benchmarking/panda_sim_ws/' C-m 'source devel/setup.bash' C-m 'roslaunch panda_simulation panda_simulation.launch' C-m

tmux new-window -t grasping_benchmarking:1 -n 'Grasp Algorithms'
tmux select-window -t grasping_benchmarking:1
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

tmux select-pane -t 0
tmux send-keys 'sleep 15' C-m 'source  "./venv/bin/activate"' C-m 'cd ~/grasping_benchmarking/grasp_algo_ws' C-m 'source devel/setup.bash' C-m 'rosrun mask_based_algo service_server.py' C-m 
tmux select-pane -t 1
tmux send-keys 'sleep 15' C-m 'source  "./venv/bin/activate"' C-m 'cd ~/grasping_benchmarking/grasp_algo_ws' C-m 'source devel/setup.bash' C-m 'rosrun ggcnn service_server.py' C-m
tmux select-pane -t 2
tmux send-keys 'sleep 15' C-m 'source  "./venv/bin/activate"' C-m 'cd ~/grasping_benchmarking/grasp_algo_ws' C-m 'source devel/setup.bash' C-m 'rosrun ros_deep_grasp service_server.py' C-m
tmux select-pane -t 3
tmux send-keys 'sleep 15' C-m 'source  "./venv/bin/activate"' C-m 'cd ~/grasping_benchmarking/grasp_algo_ws' C-m 'source devel/setup.bash' C-m 'roslaunch top_surface_algo top_surface.launch' C-m

tmux new-window -t grasping_benchmarking:2 -n 'Benchmarking'
tmux select-window -t grasping_benchmarking:2
tmux send-keys 'cd ~/grasping_benchmarking/benchmarking_ws' C-m 'source devel/setup.bash' C-m 'roslaunch benchmarking_grasp run_benchmark.launch sim_mode:=true point_cloud_input:=false'

tmux select-window -t grasping_benchmarking:0

tmux attach-session -t grasping_benchmarking
