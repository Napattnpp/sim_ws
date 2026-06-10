#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

echo "--- Step 1: Installing f1tenth_gym Python Pack ---"
# Navigate out of your workspace temporarily to clone the backend
cd ~
# Only clone if the directory doesn't already exist to prevent errors on re-runs
if [ ! -d "f1tenth_gym" ]; then
    git clone https://github.com/f1tenth/f1tenth_gym.git
else
    echo "Directory f1tenth_gym already exists. Skipping clone."
fi

# Move into the directory and install it via pip in editable mode
cd f1tenth_gym
pip3 install -e .

echo "--- Step 2: Downgrading NumPy ---"
# Ensure compatible version of NumPy
pip3 install "numpy<2.0"

echo "--- Step 3: Sourcing and Launching ---"
# Go back to your simulation workspace
cd ~/sim_ws

# Source the setup file
source install/setup.bash

# Spin up the simulation
echo "Launching f1tenth_gym_ros..."
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
