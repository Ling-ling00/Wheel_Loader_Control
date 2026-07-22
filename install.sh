#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

echo "========================================================"
echo "      Wheel Loader Simulation Installation Script"
echo "========================================================"

# Get the script and workspace directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "Workspace Directory:  $WORKSPACE_DIR"
echo "Repository Directory: $SCRIPT_DIR"
echo "--------------------------------------------------------"

# 1. Update and install ROS dependencies
echo "[1/3] Installing ROS Humble dependencies..."
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-tf-transformations

# 2. Setup Python virtual environment
echo "[2/3] Setting up Python virtual environment..."
cd "$WORKSPACE_DIR"
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
    echo "Virtual environment '.venv' created."
else
    echo "Virtual environment '.venv' already exists."
fi

# Activate environment and install dependencies
source .venv/bin/activate
echo "Upgrading pip..."
pip install --upgrade pip
echo "Installing requirements from requirements.txt..."
pip install -r "$SCRIPT_DIR/requirements.txt"

# 3. Build workspace
echo "[3/3] Building the workspace using colcon..."
colcon build

echo "--------------------------------------------------------"
echo "Installation complete!"
echo "To start working, run the following commands:"
echo "  cd $WORKSPACE_DIR"
echo "  source .venv/bin/activate"
echo "  source install/setup.bash"
echo "========================================================"
