# Using a USB Camera with ROS 2 Humble (usb_cam) in Docker

## 1. Overview

These instructions describe how to set up and use a USB camera (e.g., Mobius Action Cam, or any standard UVC webcam) with ROS 2 Humble running inside the Docker container environment previously configured for this project. We will use the `usb_cam` ROS 2 package to capture and publish images.

## 2. Identifying the Camera Device on Raspberry Pi (Host System)

When you plug your USB camera into the Raspberry Pi, the host operating system (Raspberry Pi OS) will assign it a device file in the `/dev/` directory. This is typically `/dev/video0`, `/dev/video1`, and so on.

To identify the correct device file for your camera:

1.  **Before plugging in the camera**, open a terminal on your Raspberry Pi (the host system, not inside Docker) and list existing video devices:
    ```bash
    ls /dev/video*
    ```
    Note down any devices listed, or if none are listed.

2.  **Plug in your USB camera** to the Raspberry Pi. Wait a few seconds for the system to recognize it.

3.  **List video devices again:**
    ```bash
    ls /dev/video*
    ```
    The new device that appears in the list (e.g., `/dev/video0`) is your USB camera. Note this device path, as you will need it for the Docker command.

## 3. Modifying Docker Run Command for Camera Access

To allow the ROS 2 Docker container to access the USB camera, you need to modify the `docker run` command using the `--device` flag. This command should also include any previous configurations for Roomba access and persistent workspace volumes.

Here's an example `docker run` command. **You must adapt this command to your specific setup.**

```bash
# On the Raspberry Pi HOST system:

# First, ensure any local directories you want to mount exist.
# For example, if you're mounting a workspace from your home directory:
# mkdir -p ~/roomba_droid2_ws # Or ~/create_ws, etc.

docker run -it --rm \
    --name humble_ros_container \
    -v ~/create_ws:/home/ros/create_ws \
    -v ~/roomba_droid2:/home/ros/roomba_droid2 \
    --device=/dev/ttyUSB0:/dev/ttyUSB0 \        # For Roomba (adjust /dev/ttyUSB0 if different)
    --device=/dev/video0:/dev/video0 \        # For Camera (CRITICAL: Replace /dev/video0 with YOUR camera device identified in step 2)
    # --net=host \                            # Uncomment if you need host networking (e.g., for X11 forwarding later)
    # Other X11 forwarding flags if attempting GUI tools (advanced):
    # -e DISPLAY=$DISPLAY \
    # -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros:humble-ros-base
```

**Important Considerations for the `docker run` command:**

*   **Replace `/dev/video0`:**  Crucially, change `/dev/video0` in the `--device` flag to the actual device path you identified in Step 2 (e.g., `/dev/video1`, `/dev/video2`, etc.).
*   **Roomba Device:** Ensure `--device=/dev/ttyUSB0:/dev/ttyUSB0` (or your Roomba's actual port) is still present if you intend to use the Roomba simultaneously.
*   **Persistent Volumes (`-v` flags):**
    *   The example uses `-v ~/create_ws:/home/ros/create_ws` for the Roomba driver workspace and `-v ~/roomba_droid2:/home/ros/roomba_droid2` for the main project files.
    *   Adjust these paths to match the actual locations of your ROS workspaces and project files on your Raspberry Pi host system and how you want them to appear inside the Docker container. **Using persistent volumes is highly recommended to save your work.**
*   **Container Name:** `--name humble_ros_container` is used for convenience. If a container with this name already exists (and wasn't removed with `--rm`), you might need to remove the old one (`docker rm humble_ros_container`) or choose a new name.
*   **Networking (`--net=host`):** If you plan to use GUI tools like `rqt_image_view` from within Docker, you might need `--net=host` and X11 forwarding setup. This is an advanced topic not fully covered here.

## 4. Installing `usb_cam` (Inside Docker)

1.  **Start your Docker container** using the modified `docker run` command from Step 3. You should now be at a shell prompt inside the container.

2.  **Run the setup script to install `usb_cam`:**
    Navigate to where your project files are mounted in Docker (e.g., `/home/ros/roomba_droid2` if you mounted `~/roomba_droid2` to that location). Then run the script:
    ```bash
    # Inside the Docker container:
    cd /path/to/your/project/camera_setup # Adjust this path to where camera_setup is located in your container
    bash camera_setup.sh
    ```
    This script will run `sudo apt-get update` and then `sudo apt-get install -y ros-humble-usb-cam`.

## 5. Running the Camera Node (Inside Docker)

1.  **Ensure your ROS 2 environment is sourced.** This is usually done automatically if you are using the `ros:humble-ros-base` image, but it's good practice to confirm or re-source:
    ```bash
    # Inside the Docker container:
    source /opt/ros/humble/setup.bash

    # If you have custom workspaces you need (e.g., for other nodes), source them too:
    # source ~/create_ws/install/setup.bash 
    # source ~/roomba_droid2/install/setup.bash # If you have a workspace for this project's nodes
    ```

2.  **Run the `usb_cam` node:**
    The following command runs the camera node with default settings. We use a namespace `/camera/usb_cam` to organize the topics.
    ```bash
    # Inside the Docker container:
    ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/camera/usb_cam
    ```
    *   **Note:** The executable might be `usb_cam_node_exe` or `usb_cam_node`. If one doesn't work, try the other. `ros2 pkg executables usb_cam` can show you the exact name.

    You should see output from the node, indicating it's trying to open the camera device (by default, it will try `/dev/video0` *within the container*, which we mapped from the host).

3.  **Parameterization (Optional):**
    The `usb_cam` node has several parameters you can configure, such as `video_device`, `image_width`, `image_height`, `framerate`, `pixel_format`, etc.
    *   **To specify the video device directly (if different from `/dev/video0` *inside the container*, though our Docker command maps it to `/dev/video0`):**
        ```bash
        # Example: if you mapped host /dev/video2 to container /dev/video2
        # and want to explicitly tell the node (though usually not needed with our mapping)
        # ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/camera/usb_cam -p video_device:=/dev/video0 
        ```
    *   For more complex configurations, it's better to use a YAML parameters file and load it with a launch file.
    *   Refer to the official `usb_cam` documentation for more details on parameters and launch files: [ROS Index - usb_cam (Humble)](https://index.ros.org/p/usb_cam/#humble)

4.  **Using the Launch File:**
    The `usb_cam` package also provides a launch file.
    ```bash
    # Inside the Docker container:
    ros2 launch usb_cam camera.launch.py
    ```
    This launch file might try to start `image_view` by default, which requires a GUI. If you don't have X11 forwarding set up for Docker, this part might fail or complain, but the camera node itself might still publish images.

## 6. Verifying Image Publication (Inside Docker)

To check if the camera node is publishing images, you'll need to open a **new terminal session attached to the same running Docker container.**

1.  **Open a new terminal on your Raspberry Pi host.**

2.  **Find your running container's ID or name:**
    ```bash
    # On the Raspberry Pi HOST system:
    docker ps
    ```
    Look for the container named `humble_ros_container` (or whatever name you used).

3.  **Attach a new shell to the running container:**
    Replace `humble_ros_container` if you used a different name.
    ```bash
    # On the Raspberry Pi HOST system:
    docker exec -it humble_ros_container bash
    ```
    You are now in a new shell session inside the *same* container where `usb_cam` is (or should be) running.

4.  **Inside the new container shell (Terminal 2):**
    a.  **Source the ROS 2 environment:**
        ```bash
        source /opt/ros/humble/setup.bash
        ```
    b.  **List available ROS 2 topics:**
        ```bash
        ros2 topic list
        ```
        You should see topics under the `/camera/usb_cam/` namespace, such as:
        *   `/camera/usb_cam/image_raw`
        *   `/camera/usb_cam/camera_info`
        *   (Potentially other compressed image topics depending on configuration)

    c.  **Echo the `image_raw` topic to see data flowing:**
        This will print a lot of numbers (the raw image data) to your terminal if images are being published.
        ```bash
        ros2 topic echo /camera/usb_cam/image_raw
        ```
        Press `Ctrl+C` to stop echoing the topic. If you see data, the camera is working!

    d.  **Viewing Images with `rqt_image_view` (Advanced):**
        If you have successfully configured X11 forwarding between your Docker container and your host Raspberry Pi (which allows GUI applications from the container to display on the Pi's desktop), you can use `rqt_image_view`:
        ```bash
        # First, install rqt_image_view if not already present
        # sudo apt-get install -y ros-humble-rqt-image-view
        
        # Then run it
        # rqt_image_view /camera/usb_cam/image_raw
        ```
        Setting up X11 forwarding for Docker is beyond these basic instructions and can be complex.

You should now have your USB camera publishing images through ROS 2! Remember to stop the `usb_cam_node` (Ctrl+C in its terminal) when you are done.
```
