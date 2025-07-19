#!/bin/bash

set -e  # Exit on error
echo "[INFO] Starting ROS Noetic installation..."

# Step 1: Add ROS package sources to sources.list
echo "[STEP 1] Adding ROS package source..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Step 2: Set up keys
echo "[STEP 2] Setting up keys..."
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Step 3: Update package index
echo "[STEP 3] Updating package index..."
sudo apt update

# Step 4: Install ROS Noetic (Desktop Full)
echo "[STEP 4] Installing ROS Noetic Desktop-Full..."
sudo apt install -y ros-noetic-desktop-full

# Step 5: Environment setup
echo "[STEP 5] Setting up environment sourcing..."
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Step 6: Install dependencies for building packages
echo "[STEP 6] Installing build tools and Python ROS tools..."
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Step 7: Initialize rosdep
echo "[STEP 7] Initializing rosdep..."
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update

echo "[✅ DONE] ROS Noetic setup complete!"
