# Testing the AutonomyLab/create_robot Driver for ROS 2 Humble

This guide provides instructions to test the `AutonomyLab/create_robot` driver with simple "mover" and "listener" ROS 2 nodes. It assumes you are working on a Raspberry Pi with Raspberry Pi OS (64-bit) and have followed previous setup instructions for Docker and building the `create_robot` driver.

## Prerequisites

*   **Roomba Connected:** Your iRobot Create (or Roomba with a serial interface) must be connected to the Raspberry Pi via a serial-to-USB cable. Ensure the Roomba is powered on.
*   **`create_robot` Driver Built:** The `AutonomyLab/create_robot` driver must be successfully built in the `~/create_ws` colcon workspace.
*   **Inside Docker Container:** You must be running these commands from within the ROS 2 Humble Docker container that was previously configured and used to build the `create_robot` driver.
*   **Host USB Permissions:** On your **host** Raspberry Pi system (not inside Docker), your user must be part of the `dialout` group to access the serial port. If you haven't done this:
    ```bash
    # On the HOST Raspberry Pi:
    sudo usermod -a -G dialout $USER 
    ```
    Then, log out and log back in on the host, or reboot the Raspberry Pi.

## Steps to Test

**Terminal 1: Launch `create_robot` Driver**

1.  **Open a new terminal inside your Docker container.**

2.  **Source ROS 2 Humble and your `create_robot` workspace:**
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/create_ws/install/setup.bash
    ```

3.  **Identify the Roomba's Serial Port:**
    *   Before plugging in the Roomba's USB cable to the Raspberry Pi, list available serial ports:
        ```bash
        ls /dev/ttyUSB*
        ```
        (It might show nothing if no other USB serial devices are connected).
    *   Plug in the Roomba's USB cable. Wait a few seconds.
    *   List the serial ports again:
        ```bash
        ls /dev/ttyUSB*
        ```
    *   The new `/dev/ttyUSBX` device (e.g., `/dev/ttyUSB0`) is your Roomba. Note this port.

4.  **Launch the `create_robot` Driver:**
    Replace `/dev/ttyUSB0` with the port you identified in the previous step if it's different. Adjust `robot_model` if you are not using an iRobot Create 2 (e.g., `ROOMBA_400`, `CREATE_1`).
    ```bash
    ros2 launch create_bringup create_2.launch.py robot_model:=CREATE_2 dev:=/dev/ttyUSB0
    ```
    You should see log messages indicating the driver is trying to connect to the robot.

**Terminal 2: Run the Mover Node**

1.  **Open a new terminal inside your Docker container.** (Do not close Terminal 1).

2.  **Source ROS 2 Humble and your `create_robot` workspace:**
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/create_ws/install/setup.bash
    ```

3.  **Navigate to the directory containing `mover.py`** (assuming it's in `~/roomba_test_nodes` relative to where you cloned/placed these files inside Docker):
    ```bash
    # Example: if your project is in /home/ros/my_project/roomba_test_nodes
    # cd /home/ros/my_project/roomba_test_nodes 
    cd <path_to_your_roomba_test_nodes_directory> 
    ```
    If you followed the previous steps, these files are in `roomba_test_nodes` in the current directory.
    ```bash
    cd roomba_test_nodes 
    ```


4.  **Run the `mover.py` script:**
    ```bash
    python3 mover.py
    ```

**Terminal 3: Run the Listener Node**

1.  **Open another new terminal inside your Docker container.** (Do not close Terminals 1 or 2).

2.  **Source ROS 2 Humble and your `create_robot` workspace:**
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/create_ws/install/setup.bash
    ```

3.  **Navigate to the directory containing `listener.py`:**
    ```bash
    # Example: if your project is in /home/ros/my_project/roomba_test_nodes
    # cd /home/ros/my_project/roomba_test_nodes
    cd <path_to_your_roomba_test_nodes_directory>
    ```
    If you followed the previous steps, these files are in `roomba_test_nodes` in the current directory.
     ```bash
    cd roomba_test_nodes
    ```

4.  **Run the `listener.py` script:**
    ```bash
    python3 listener.py
    ```

## What to Observe

*   **Roomba Movement:** When you run `mover.py` (Terminal 2), the Roomba should move forward a short distance and then stop. The `mover.py` script will print log messages and then exit.
*   **Sensor Data:** In Terminal 3, the `listener.py` script should start printing odometry messages received from the `/odom` topic. It will print a few messages and then exit.
*   **Driver Output:** Terminal 1 will show ongoing log messages from the `create_robot` driver.

## Troubleshooting Tips

*   **Roomba Doesn't Connect/Move:**
    *   **Serial Port:** Double-check that you are using the correct `/dev/ttyUSBX` port in the `ros2 launch` command.
    *   **USB Permissions (Host):** Ensure your user on the **host** Raspberry Pi is in the `dialout` group. After adding, you must log out/in or reboot the host.
    *   **Cable Connections:** Verify the serial-to-USB cable is securely connected to both the Pi and the Roomba. Check the Roomba's 7-pin DIN connector.
    *   **Roomba Power & Mode:** Ensure the Roomba is powered on. If it's a Create, it should be in a mode that accepts serial commands (usually on by default). If it's a Roomba, it might need to be in "Safe" or "Full" mode; the driver attempts to set this.
    *   **Driver Error Messages:** Look at the output in Terminal 1 for error messages from the `create_robot` driver.
*   **Nodes Don't Run (`mover.py` or `listener.py`):**
    *   **Python/ROS 2 Environment:** Make sure you have sourced `source /opt/ros/humble/setup.bash` in each terminal.
    *   **Workspace Sourcing:** Ensure you have sourced your `create_robot` workspace (`source ~/create_ws/install/setup.bash`) in each terminal.
    *   **File Not Found:** Verify you are in the correct directory (`roomba_test_nodes`) when trying to execute `python3 mover.py` or `python3 listener.py`.
    *   **Python Interpreter:** Ensure you are using `python3`.
*   **No Odometry Data:**
    *   Check that the `create_robot` driver is running correctly in Terminal 1 and reporting a connection.
    *   Ensure the Roomba is actually moving. Odometry is based on wheel encoders, so no movement means no new odometry.
*   **`rclpy.init()` errors:**
    *   This usually means ROS 2 is not sourced correctly or the ROS master (daemon) isn't running. Ensure you've sourced `setup.bash` files. Check `ros2 topic list` in a sourced terminal; if it hangs, there might be a deeper networking or ROS daemon issue.
```
