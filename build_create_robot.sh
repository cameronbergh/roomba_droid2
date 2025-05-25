#!/bin/bash

# Script to set up and build the AutonomyLab/create_robot driver for ROS 2 Humble
# This script is intended to be run inside a ROS 2 Humble Docker container.

echo "Starting create_robot build script..."

# 1. Prerequisites Check
# The ros:humble-ros-base Docker image should have these pre-installed.
# If not, uncomment and run the following lines:
# echo "Checking/installing prerequisites..."
# sudo apt-get update
# sudo apt-get install -y python3-rosdep python3-colcon-common-extensions git

# 2. Create a Colcon Workspace
echo "Creating Colcon workspace at ~/create_ws..."
mkdir -p ~/create_ws/src
cd ~/create_ws/src

# 3. Clone Repositories
echo "Cloning AutonomyLab/create_robot (humble branch)..."
git clone -b humble https://github.com/AutonomyLab/create_robot.git
echo "Cloning AutonomyLab/libcreate..."
git clone https://github.com/AutonomyLab/libcreate.git

# 4. Install Dependencies
echo "Installing dependencies using rosdep..."
cd ~/create_ws

# Initialize rosdep. In a fresh container, `rosdep update` is often sufficient.
# `sudo rosdep init` might be needed if it's the very first time rosdep is used on the system image.
# If `sudo rosdep init` fails, it might be because it's already initialized.
sudo rosdep init || echo "rosdep init already performed or failed, continuing with update."
rosdep update

# Install dependencies for the cloned packages
rosdep install --from-paths src -i -y --rosdistro humble

# 5. Build the Workspace
echo "Building the workspace with colcon..."
colcon build --symlink-install

# 6. USB Permissions Reminder (for the host system)
echo ""
echo "--------------------------------------------------------------------------------"
echo "IMPORTANT: USB Permissions for Serial Port (Roomba/Create)"
echo "--------------------------------------------------------------------------------"
echo "On your HOST Raspberry Pi system (NOT inside this Docker container), "
echo "you need to add your user to the 'dialout' group to access the Roomba's serial port."
echo "Run the following command on the HOST, then log out and log back in:"
echo ""
echo "  sudo usermod -a -G dialout \$USER"
echo ""
echo "This step is crucial for the driver to communicate with the robot."
echo "--------------------------------------------------------------------------------"

# 7. Sourcing the Workspace
echo ""
echo "Build complete!"
echo "To use the create_robot driver, source the workspace in your current terminal or add to .bashrc:"
echo ""
echo "  source ~/create_ws/install/setup.bash"
echo ""
echo "Script finished."
