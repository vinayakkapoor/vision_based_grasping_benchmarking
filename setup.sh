#!/bin/bash

set -e  # Exit on error
set -o pipefail  # Catch errors in pipes

PYTHON_VERSION="python3.8"  # Specify Python version
ROOT_DIR=~/grasping_benchmarking
SRC_DIR=~/vision_based_grasping_benchmarking

# Function to set up a workspace
setup_workspace() {
    local workspace_name=$1
    local package_name=$2
    local requirements_file=$3

    echo "Setting up $workspace_name..."
    mkdir -p "$ROOT_DIR/$workspace_name/src"
    cd "$ROOT_DIR/$workspace_name/src"
    cp -r "$SRC_DIR/$package_name" ./
    source "$ROOT_DIR/venv/bin/activate"  # Activate virtual environment
    pip install -r "./$package_name/$requirements_file"    
    # Configure Catkin to use the virtual environment's Python
    cd ..
    catkin config --extend /opt/ros/noetic \
                  --cmake-args -DPYTHON_EXECUTABLE="$ROOT_DIR/venv/bin/python3"
    source /opt/ros/noetic/setup.bash
    catkin build -j6
    deactivate
    cd "$ROOT_DIR"
}

# Install system dependencies
echo "Installing apt packages..."
sudo apt update
sudo apt install -y \
    ros-noetic-combined-robot-hw \
    ros-noetic-tf-conversions \
    $PYTHON_VERSION \
    $PYTHON_VERSION-venv \
    $PYTHON_VERSION-dev \
    python3-pip \
    libgirepository1.0-dev \
    libcups2-dev \
    python3-pykdl \
    liborocos-kdl-dev \
    tmux \
    ros-noetic-dynamic-reconfigure \
    python3-catkin-tools \
    ros-noetic-libfranka \
    python3-osrf-pycommon \
    libpcap-dev \
    libpng-dev \
    libusb-1.0-0-dev \
    ros-noetic-moveit \
    ros-noetic-effort-controllers \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-moveit-visual-tools \
    ros-noetic-rosbridge-library \
    ros-noetic-rosbridge-server \
    ros-noetic-tf2-web-republisher

# Create root directory
mkdir -p "$ROOT_DIR"
cd "$ROOT_DIR"

# Set up Python virtual environment
echo "Setting up Python virtual environment with $PYTHON_VERSION..."
$PYTHON_VERSION -m venv venv
source "$ROOT_DIR/venv/bin/activate"
pip install --upgrade pip setuptools wheel  # Upgrade essential tools
# pip install empy==3.3.4 defusedxml easydict shapely
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install tensorflow==2.10.0
deactivate

# Set up workspaces
setup_workspace "benchmarking_ws" "benchmarking_vision_based_grasping" "requirements.txt"
setup_workspace "panda_sim_ws" "panda_simulation" "requirements.txt"
setup_workspace "grasp_algo_ws" "grasp_synthesis" "requirements.txt"

# Copy additional scripts
cp "$SRC_DIR/benchmark_grasping.sh" "$ROOT_DIR/"
echo "All setups complete!"