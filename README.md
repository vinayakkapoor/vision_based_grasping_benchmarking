
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





## Usage

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

## Troubleshooting

1. Problems with robot movement in Gazebo / moveit commander errors in computeCartesianPath():

The computeCartesianPath() function in moveit commander was recently updated (see https://github.com/moveit/moveit/pull/3618).
Please upgrade the package using     sudo apt install --only-upgrade ros-noetic-moveit-commander




