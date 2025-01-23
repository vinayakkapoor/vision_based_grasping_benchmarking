#!/bin/bash

set -e  # Exit on error
set -o pipefail  # Catch errors in pipes

PYTHON_VERSION="python3.8"  # Specify Python version
ROOT_DIR=~/grasping_benchmarking
SRC_DIR=~/vision_based_grasping_benchmarking/grasping_benchmarking_suite

# Default value for USE_CACHE is 1 (use cache) for pip installations
USE_CACHE=1

# Function to set up a workspace
setup_workspace() {
    local workspace_name=$1
    local package_name=$2
    local requirements_file=$3

    echo "Setting up $workspace_name..."
    mkdir -p "$ROOT_DIR/$workspace_name/src"
    cd "$ROOT_DIR/$workspace_name/src"
    cp -r "$SRC_DIR/$package_name" ./
    
    # Activate virtual environment
    source "$ROOT_DIR/venv/bin/activate"  # Activate virtual environment

    # Source ROS setup file after activating virtual environment
    source /opt/ros/noetic/setup.bash  # Source ROS setup file

    # Install dependencies, use cache or not based on USE_CACHE
    if [ "$USE_CACHE" -eq 1 ]; then
        pip install -r "./$package_name/$requirements_file"  # Use cache
    else
        pip install --no-cache-dir -r "./$package_name/$requirements_file"  # Don't use cache
    fi

    # Configure Catkin to use the virtual environment's Python
    cd ..
    catkin config --extend /opt/ros/noetic \
                  --cmake-args -DPYTHON_EXECUTABLE="$ROOT_DIR/venv/bin/python3"
    
    catkin build -j6  # Build the workspace
    deactivate  # Deactivate virtual environment
    cd "$ROOT_DIR"  # Return to the root directory
}

get_cuda_version() {
    if command -v nvcc &> /dev/null; then
        cuda_version=$(nvcc --version | grep "release" | sed -E 's/.*release ([0-9]+.[0-9]+).*/\1/')
        echo "$cuda_version"
    elif [ -f /usr/local/cuda/version.txt ]; then
        cuda_version=$(cat /usr/local/cuda/version.txt | grep -oP '\d+\.\d+')
        echo "$cuda_version"
    else
        echo "none"
    fi
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
    ros-noetic-tf2-web-republisher \
    liborocos-kdl-dev

# Check if the directory exists
if [ ! -d "$ROOT_DIR" ]; then
    echo "Directory $ROOT_DIR does not exist. Creating..."
    mkdir -p "$ROOT_DIR"
else
    echo "Directory $ROOT_DIR already exists."
fi

cd "$ROOT_DIR"

# Check if the virtual environment exists
if [ ! -d "$ROOT_DIR/venv" ]; then
    echo "Virtual environment not found. Creating..."
    $PYTHON_VERSION -m venv venv
else
    echo "Virtual environment already exists. Using it."
fi

# Activate the virtual environment
source "$ROOT_DIR/venv/bin/activate"

# Upgrade essential tools
echo "Upgrading pip, setuptools, and wheel..."
pip install --upgrade pip setuptools wheel importlib_metadata packaging

# Detect CUDA version
# Check if PyTorch is already installed
if python -c "import torch; print(torch.__version__)" &> /dev/null; then
    echo "PyTorch is already installed. Skipping installation."
else
    echo "PyTorch not found. Proceeding with installation."

    # Detect CUDA version
    cuda_version=$(get_cuda_version)

    # Determine the appropriate PyTorch URL
    if [ "$cuda_version" == "12.1" ]; then
        url="https://download.pytorch.org/whl/cu121"
    elif [ "$cuda_version" == "12.0" ]; then
        url="https://download.pytorch.org/whl/cu120"
    elif [ "$cuda_version" == "11.8" ]; then
        url="https://download.pytorch.org/whl/cu118"
    elif [ "$cuda_version" == "11.7" ]; then
        url="https://download.pytorch.org/whl/cu117"
    elif [ "$cuda_version" == "11.6" ]; then
        url="https://download.pytorch.org/whl/cu116"
    elif [ "$cuda_version" == "11.3" ]; then
        url="https://download.pytorch.org/whl/cu113"
    elif [ "$cuda_version" == "10.2" ]; then
        url="https://download.pytorch.org/whl/cu102"
    else
        echo "CUDA not detected or unsupported version. Installing CPU-only PyTorch."
        url="https://download.pytorch.org/whl/cpu"
    fi

    # Install PyTorch
    echo "Installing PyTorch for CUDA version: $cuda_version (CPU, if CUDA version is none)."
    if [ "$USE_CACHE" -eq 1 ]; then
        pip install torch --index-url $url
    else
        pip install --no-cache-dir torch --index-url $url
    fi
    if [ $? -ne 0 ]; then
        echo "PyTorch installation failed!"
        exit 1
    fi
fi

# Install tensorflow
if [ "$USE_CACHE" -eq 1 ]; then
    pip install tensorflow
else
    pip install --no-cache-dir tensorflow
fi

# Deactivate the virtual environment
deactivate

# Set up workspaces
setup_workspace "benchmarking_ws" "benchmarking_vision_based_grasping" "requirements.txt"
setup_workspace "panda_sim_ws" "panda_simulation" "requirements.txt"
setup_workspace "grasp_algo_ws" "grasp_synthesis" "requirements.txt"

# Copy additional scripts
cp "$SRC_DIR/benchmark_grasping.sh" "$ROOT_DIR/"
echo "All setups complete!"
