#!/bin/bash

set -e  # Exit on error
set -o pipefail  # Catch errors in pipes

#PYTHON_VERSION="python3.12"  # Specify Python version
UBUNTU_VERSION=$(lsb_release -rs)

# Decide Python version based on Ubuntu version
if [[ "$UBUNTU_VERSION" == "20."* ]]; then
    PYTHON_VERSION="python3.8"
elif [[ "$UBUNTU_VERSION" == "22."* ]]; then
    PYTHON_VERSION="python3.10"
elif [[ "$UBUNTU_VERSION" == "24."* ]]; then
    PYTHON_VERSION="python3.12"
else
    echo "Unsupported Ubuntu version: $UBUNTU_VERSION"
    exit 1
fi

ROOT_DIR=$HOME/grasping_benchmarking
# SRC_DIR=~/vision_based_grasping_benchmarking/grasping_benchmarking_suite
SRC_DIR="$PWD"

# Default value for USE_CACHE is 1 (use cache) for pip installations
USE_CACHE=1

ROS_DISTRO=$(ls /opt/ros | tail -n 1)  # Gets the latest ROS version available

while getopts ":r:s:" opt; do
  case ${opt} in
    r )
      ROOT_DIR="${OPTARG/#\~/$HOME}"
      ;;
    s )
      SRC_DIR="${OPTARG/#\~/$HOME}"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" 1>&2
      exit 1
      ;;
    : )
      echo "Option -$OPTARG requires an argument." 1>&2
      exit 1
      ;;
  esac
done
shift $((OPTIND -1))


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

    # Source ROS2 setup file after activating virtual environment
    source /opt/ros/$ROS_DISTRO/setup.bash

    # Install dependencies, use cache or not based on USE_CACHE
    if [ "$USE_CACHE" -eq 1 ]; then
        pip install -r "./$package_name/$requirements_file"  # Use cache
    else
        pip install --no-cache-dir -r "./$package_name/$requirements_file"  # Don't use cache
    fi

    cd ..
    colcon build --symlink-install
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
    $PYTHON_VERSION \
    $PYTHON_VERSION-venv \
    $PYTHON_VERSION-dev \
    python3-pip \
    tmux 
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
        pip install numpy==1.26.2 torch==2.1 --index-url $url
    else
        pip install --no-cache-dir numpy==1.26.2 torch==2.1 --index-url $url
    fi
    if [ $? -ne 0 ]; then
        echo "PyTorch installation failed!"
        exit 1
    fi
fi

# Install tensorflow
if [ "$USE_CACHE" -eq 1 ]; then
    pip install numpy==1.24.1 tensorflow==2.10
else
    pip install --no-cache-dir numpy==1.26.2 tensorflow==2.10
fi

# Deactivate the virtual environment
deactivate

# Set up workspaces
setup_workspace "benchmarking_ws" "benchmarking_vision_based_grasping" "requirements.txt"
#setup_workspace "panda_sim_ws" "panda_simulation" "requirements.txt"
setup_workspace "grasp_algo_ws" "grasp_synthesis" "requirements.txt"

# Copy additional scripts
cp "$SRC_DIR/benchmark_grasping_tmux.sh" "$ROOT_DIR/"
cp "$SRC_DIR/benchmark_grasping_gnome_terminal.sh" "$ROOT_DIR/"
echo "All setups complete!"
