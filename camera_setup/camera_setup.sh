#!/bin/bash
echo "Starting camera driver setup script (to be run inside Docker)..."
echo "Updating package lists (apt-get update)..."
sudo apt-get update -y

echo ""
echo "Installing ros-humble-usb-cam package..."
sudo apt-get install -y ros-humble-usb-cam

echo ""
echo "Script finished. The ros-humble-usb-cam package should now be installed."
echo "You can now try running the camera node, e.g.:"
echo "ros2 run usb_cam usb_cam_node --ros-args -r __ns:=/camera/usb_cam"
