#!/bin/bash

# Create a new tmux session in detached mode
tmux new-session -d -s grasping_benchmarking

# Window 0: roscore
tmux rename-window -t grasping_benchmarking:0 'roscore'
tmux send-keys -t grasping_benchmarking:0 'source "./venv/bin/activate"' C-m
tmux send-keys -t grasping_benchmarking:0 'source /opt/ros/$ROS_DISTRO/setup.bash' C-m
tmux send-keys -t grasping_benchmarking:0 'roscore' C-m

# Window 1: Simulation
tmux new-window -t grasping_benchmarking:1 -n 'Simulation'
tmux send-keys -t grasping_benchmarking:1 'sleep 5' C-m
# tmux send-keys -t grasping_benchmarking:1 'export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/' C-m
tmux send-keys -t grasping_benchmarking:1 'source "./venv/bin/activate"' C-m
tmux send-keys -t grasping_benchmarking:1 'cd ./panda_sim_ws' C-m
tmux send-keys -t grasping_benchmarking:1 'source devel/setup.bash' C-m
tmux send-keys -t grasping_benchmarking:1 'roslaunch panda_simulation panda_simulation.launch' C-m

# Window 2: Grasp Algorithms
tmux new-window -t grasping_benchmarking:2 -n 'Grasp Algorithms'
tmux split-window -h -t grasping_benchmarking:2
tmux split-window -v -t grasping_benchmarking:2
tmux select-pane -t 0
tmux split-window -v -t grasping_benchmarking:2

# Pane 0: Mask-based Algorithm
tmux select-pane -t grasping_benchmarking:2.0
tmux send-keys 'sleep 15' C-m 'source "./venv/bin/activate"' C-m
tmux send-keys 'cd ./grasp_algo_ws' C-m
tmux send-keys 'source devel/setup.bash' C-m
tmux send-keys 'rosrun mask_based_algo service_server.py' C-m

# Pane 1: GGCNN Algorithm
tmux select-pane -t grasping_benchmarking:2.1
tmux send-keys 'sleep 15' C-m 'source "./venv/bin/activate"' C-m
tmux send-keys 'cd ./grasp_algo_ws' C-m
tmux send-keys 'source devel/setup.bash' C-m
tmux send-keys 'rosrun ggcnn service_server.py' C-m

# Pane 2: ROS Deep Grasp
tmux select-pane -t grasping_benchmarking:2.2
tmux send-keys 'sleep 15' C-m 'source "./venv/bin/activate"' C-m
tmux send-keys 'cd ./grasp_algo_ws' C-m
tmux send-keys 'source devel/setup.bash' C-m
tmux send-keys 'rosrun ros_deep_grasp service_server.py' C-m

# # Pane 3: Top Surface Algorithm
# tmux select-pane -t grasping_benchmarking:2.3
# tmux send-keys 'sleep 15' C-m 'source "./venv/bin/activate"' C-m
# tmux send-keys 'cd ./grasp_algo_ws' C-m
# tmux send-keys 'source devel/setup.bash' C-m
# tmux send-keys 'roslaunch top_surface_algo top_surface.launch' C-m

# Pane 3: GPD Algorithm
tmux select-pane -t grasping_benchmarking:2.3
tmux send-keys 'sleep 15' C-m 'source "./venv/bin/activate"' C-m
tmux send-keys 'cd ./grasp_algo_ws' C-m
tmux send-keys 'source devel/setup.bash' C-m
tmux send-keys 'roslaunch gpd_ros ur5.launch' C-m

# Window 3: Benchmarking
tmux new-window -t grasping_benchmarking:3 -n 'Benchmarking'
tmux send-keys -t grasping_benchmarking:3 'sleep 15' C-m
tmux send-keys -t grasping_benchmarking:3 'export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/' C-m
tmux send-keys -t grasping_benchmarking:3 'source "./venv/bin/activate"' C-m
tmux send-keys -t grasping_benchmarking:3 'cd ./benchmarking_ws' C-m
tmux send-keys -t grasping_benchmarking:3 'source devel/setup.bash' C-m
tmux send-keys -t grasping_benchmarking:3 'roslaunch benchmarking_grasp run_benchmark.launch sim_mode:=true' C-m

# Focus back on the roscore window
tmux select-window -t grasping_benchmarking:0

# Attach to the session
tmux attach-session -t grasping_benchmarking