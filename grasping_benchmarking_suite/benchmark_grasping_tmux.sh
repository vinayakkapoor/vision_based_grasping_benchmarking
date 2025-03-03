#!/bin/bash

ROOT_DIR=$HOME/grasping_benchmarking_2
# Create a new tmux session in detached mode
tmux new-session -d -s grasping_benchmarking


# Window 1: Grasp Algorithms
tmux new-window -t grasping_benchmarking:2 -n 'Grasp Algorithms'
tmux split-window -h -t grasping_benchmarking:2
tmux split-window -v -t grasping_benchmarking:2
tmux select-pane -t 0
tmux split-window -v -t grasping_benchmarking:2

# Pane 0: Mask-based Algorithm
tmux select-pane -t grasping_benchmarking:2.0
tmux send-keys 'sleep 15' C-m 'source "./venv/bin/activate"' C-m
tmux send-keys 'cd ./grasp_algo_ws/' C-m
tmux send-keys 'source install/setup.bash' C-m
tmux send-keys 'ros2 run mask_based_algo service_server.py' C-m

# Pane 1: GGCNN Algorithm
tmux select-pane -t grasping_benchmarking:2.1
tmux send-keys 'sleep 15' C-m 'source "./venv/bin/activate"' C-m
tmux send-keys 'cd ./grasp_algo_ws/' C-m
tmux send-keys 'source install/setup.bash' C-m
tmux send-keys 'ros2 run ggcnn service_server.py' C-m

## Pane 2: ROS Deep Grasp
#tmux select-pane -t grasping_benchmarking:2.2
#tmux send-keys 'sleep 15' C-m 'source "./venv/bin/activate"' C-m
#tmux send-keys 'cd ~/$ROOT_DIR/grasp_algo_ws' C-m
#tmux send-keys 'source install/setup.bash' C-m
#tmux send-keys 'ros2 run ros_deep_grasp service_server.py' C-m

# Pane 3: Top Surface Algorithm
#tmux select-pane -t grasping_benchmarking:2.3
#tmux send-keys 'sleep 15' C-m 'source "./venv/bin/activate"' C-m
#tmux send-keys 'cd ~/$ROOT_DIR/grasp_algo_ws' C-m
#tmux send-keys 'source install/setup.bash' C-m
#tmux send-keys 'ros2 launch top_surface_algo top_surface.launch' C-m

# Window 3: Benchmarking
tmux new-window -t grasping_benchmarking:3 -n 'Benchmarking'
tmux send-keys -t grasping_benchmarking:3 'sleep 15' C-m
#tmux send-keys -t grasping_benchmarking:3 'export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages/' C-m
tmux send-keys -t grasping_benchmarking:3 'source "./venv/bin/activate"' C-m
tmux send-keys -t grasping_benchmarking:3 'cd ./benchmarking_ws' C-m
tmux send-keys -t grasping_benchmarking:3 'source install/setup.bash' C-m
# For the top surface algorithm, change point_cloud_input:=true
tmux send-keys -t grasping_benchmarking:3 'ros2 launch benchmarking_grasp launch_benchmarking_pipeline.xml' C-m

# Focus back on the roscore window
tmux select-window -t grasping_benchmarking:0

# Attach to the session
tmux attach-session -t grasping_benchmarking
