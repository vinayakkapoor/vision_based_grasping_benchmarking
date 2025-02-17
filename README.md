
# Towards More Robust and Reliable Vision-Based Grasping: A Benchmarking Study

This is the official repository for **A Benchmarking Study of Vision-based Robotic Grasping Algorithms**

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/hmgh5JGP-Ak/0.jpg)](https://www.youtube.com/watch?v=hmgh5JGP-Ak)


<!---
#### Video Demo of the benchmarking experiemnts
<a href="https://youtu.be/hmgh5JGP-Ak" target="_blank" rel="noopener noreferrer">
    <img src="https://img.youtube.com/vi/hmgh5JGP-Ak/0.jpg" alt="Video Demo" width="800" height="500">
-->

## General Setup

<details>
  <summary> <b>With docker</b></summary>

Make sure you have *docker* installed on your system. Refer to the official docker setup [instructions](https://docs.docker.com/engine/install/) if you do not have docker installed.

Pull the docker image for this project using

    docker pull vinayakapoor/grasping_benchmarking_image:v0

#### Building your own docker image
To build your own docker image, clone the repo and use `docker build`

    git clone https://github.com/vinayakkapoor/vision_based_grasping_benchmarking.git
    cd vision_based_grasping_benchmarking
    docker build -t grasping_benchmarking_image:v0 .
You might need to add `sudo` depending on how your docker daemon is configured

    sudo docker build -t grasping_benchmarking_image:v0 .
</details>

### Without Docker
Clone the repo and run the setup script

    git clone https://github.com/vinayakkapoor/vision_based_grasping_benchmarking.git
    cd vision_based_grasping_benchmarking/grasping_benchmarking_suite/
    chmod +x benchmark_grasping.sh setup.sh
    # Setup and build the grasping_benchmarking directory
    ./setup.sh -r ~/grasping_benchmarking                 # Change the install directory if required

<details>
  <summary><b>But what does setup.sh do?</b></summary>
    The bash file appropriately installs all the necessary requirements and sets up the following structure -

```
grasping_benchmarking
├── benchmark_grasping.sh
├── benchmarking_ws
│   ├── build
│   ├── devel
│   ├── logs
│   └── src
├── grasp_algo_ws
│   ├── build
│   ├── devel
│   ├── logs
│   └── src
├── panda_sim_ws
│   ├── build
│   ├── devel
│   ├── logs
│   └── src
└── venv
    ├── bin
    ├── include
    ├── lib
    ├── lib64 -> lib
    ├── pyvenv.cfg
    └── share

```
</details>


## Running Benchmarking

<details>
  <summary><b>With Docker</b></summary>

  Run the image using

  ```sh
  xhost +
  sudo docker container run --rm -e DISPLAY=$DISPLAY --net host -v /tmp/.X11-unix:/tmp/.X11-unix -it grasping_benchmarking_image:v0
  ```

Then run the container using
```sh
./benchmark_grasping.sh
```

Run `xhost -` when you're done

</details>



### Without Docker

    cd ~/grasping_benchmarking                            # Change to the install directory
    ./benchmark_grasping.sh

## Usage
### Getting familiar with TMUX

`./benchmark_grasping.sh` autofills and spawns multiple terminals to approriately run nodes and launch files.

1. Switch between windows in tmux by pressing `Ctrl+b` then press `w`. You will be able to choose different windows. Navigate them using the `up` and `down` arrow keys.
2. Switch between different panes (in the grasp algorithms window) by pressing `Ctrl+b` and followed by the arrow key to the pane you want to navigate to.
3. Kill the entire session by pressing `Ctrl+b` then press `Shift+:` then type `kill-session` and press enter.

For a short intro to tmux, visit [here](https://hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/). | [Tmux cheatsheets](https://github.com/ctu-mrs/mrs_cheatsheet)


### Changing the configuration

All the rosparams are loaded from `configuration.yaml` file in `grasping_benchmarking_suite/benchmarking_vision_based_grasping/benchmarking_grasp/config`

Change the `grasp_in_image_frame:` to the provided values to switch the algorithm being used to generate grasps on the fly

## Architecture for grasp generation 



## Troubleshooting

1. **Problems with robot movement in Gazebo / MoveIt Commander errors in `computeCartesianPath()`:**

   The `computeCartesianPath()` function in MoveIt Commander was recently updated (see [moveit/pull/3618](https://github.com/moveit/moveit/pull/3618)). Please upgrade the package using:

   ```sh
   sudo apt install --only-upgrade ros-noetic-moveit-commander
   ```

   Consider upgrading other ROS packages if the issues persist.

   <details>
   <summary>What if I do not want to upgrade the packages?</summary>

   Navigate to:
   ```sh
   cd ./grasping_benchmarking_suite/panda_simulation/moveit_adapter/src/moveit_adapter_module/
   ```

   Change `False` to `0.0` in _eef_control.py_:

   ```python
   (plan, _) = move_group.compute_cartesian_path(
       cartesian_points,  # waypoints to follow
       0.01,  # eef_step
       0.0)  # Changed from False to 0.0
   ```
   </details>

2. **If the `catkin build` hangs, update this line in `setup.sh`**
```bash
catkin build -j6
```
3. **Problems with GUI when using docker**

    This [stackoverflow thread](https://stackoverflow.com/questions/40658095/how-to-open-ubuntu-gui-inside-a-docker-image) was incredibly helpful for troubleshooting



