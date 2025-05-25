# roomba_droid2
cam's droid.

## Project Log

*   **2024-07-16:** Initial project discussion: Target a Python-based agent using Claude 4 with a JS interface, controlling a Roomba 655 via Raspberry Pi 5. Includes RAG with images, camera input, and potential for SLAM with LIDAR. User has multiple servers for offloading.
*   **2024-07-16:** Confirmed Raspberry Pi 5 runs 64-bit Raspberry Pi OS. User expressed interest in ROS (Robot Operating System), potentially for vision-based SLAM. Web interface to show camera image and Roomba sensor data. Agent should manage memories.
*   **2024-07-16:** Decided to use ROS 2 Humble as the specific ROS distribution.
*   **2024-07-16:** Planned to use Docker for deploying ROS 2 on the Raspberry Pi to simplify setup on Raspberry Pi OS.
*   **2024-07-16:** Researched and identified `AutonomyLab/create_robot` (Humble branch) as the most suitable ROS 2 driver for the Roomba 600 series.
*   **2024-07-16:** Generated `build_create_robot.sh` script to automate the compilation of `AutonomyLab/create_robot` and its dependency `libcreate` within the ROS 2 Docker container.
*   **2024-07-16:** Created `roomba_test_nodes` directory with `mover.py` and `listener.py` scripts, plus a `README.md`, for basic testing of Roomba control via ROS 2.
*   **2024-07-16:** Committed initial project files (root `README.md` for Docker/ROS setup, `build_create_robot.sh`, `roomba_test_nodes/`) to the `phase1-roomba-setup` branch. (Commit hash: [TODO - User will need to fill this in from git log if desired])
*   **2024-07-16:** User requested a running log of project activities be kept in this `README.md`. This log was initiated.
*   **2024-07-16:** User successfully tested Roomba control. This involved:
    *   Troubleshooting the `create_robot` build process (missing `.py` extensions on launch files, requiring `apt-get update` in Docker, and renaming files in the source).
    *   Successfully launching the `create_robot` driver in ROS 2 Humble.
    *   Confirming Roomba movement using the provided `mover.py` script.
    *   Confirming sensor data (odometry) reception using the `listener.py` script.
    *   Phase 1 (ROS 2 Setup and Basic Roomba Control) is now complete.
