
  

# Grasping benchmarking

  

The grasing benchmarking project presents a pipeline to simulate and benchmark different grasping algorithms on the franka panda arm. Simulation, grasping algorithms and benchamarking code is combined to present a concise way to benchmark the algorithms in minimal steps.

  

## Getting started

Clone the merlab repo, change to the `grasping_benchmarking` branch and navigate to the root of this folder

    git clone git@github.com:berkcalli/mer_lab.git
    git checkout grasp_bench_enlearning
    cd ~/mer_lab/ros_ws/src/projects/grasping_benchmarking

Make the *setup.sh* and *benchmark_grasping* file executable 

    chmod +x setup.sh
    chmod +x benchmark_grasping.sh

Setup the simulation, benchmarking and grasping algorithm repos. Simply run the setup executable

    ./setup.sh

## What does setup.sh do?
The bash file appropriately installs all the necessary requirements and sets up the following structure -


    grasping_benchmarking
    ├── benchmark_grasping.sh
    ├── benchmarking_ws
    │   └── src
    │       └── benchmarking_vision_based_grasping
    │           ├── benchmarking_grasp
    │           ├── benchmarking_msgs
    │           ├── franka_ros
    │           ├── grasp_plugin
    │           ├── LICENSE
    │           ├── moveit_adapter
    │           ├── pick_and_place
    │           ├── README.md
    │           └── requirements.txt
    ├── grasp_algo_ws
    │   └── src
    │       └── grasp_synthesis
    │           ├── ggcnn
    │           ├── mask_based_algo
    │           ├── media
    │           ├── README.md
    │           ├── requirements.txt
    │           ├── ros_deep_grasp
    │           └── top_surface_algo
    └── panda_sim_ws
        └── src
            └── panda_simulation
                ├── franka_ros
                ├── grasp_plugin
                ├── LICENSE
                ├── moveit_adapter
                ├── panda_moveit_config
                ├── panda_simulation
                ├── README.md
                └── requirements.txt



## What does benchmark_grasping.sh do?
This bash file sets up 3 windows using tmux

- Simulation - launches the franka panda simulation
- Grasping algorithms - launches all the grasping algorithms in separate panes to easily see the outputs and switch algorithms on the fly
- Benchmarking - has the command autofilled which can be run by pressing enter to start the benchmarking

NOTE:
- Use *Ctrl+b* followed be *w* and then the arrows keys to select the window
- When in the second window, use *Ctrl+b* and then arrow keys to move between panes
- Edit the `~/grasping_benchmarking/benchmarking_ws/src/benchmarking_vision_based_grasping/benchmarking_grasp/config/configuration.yaml` file to change the algorithm being used for benchmarking

