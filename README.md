# TelloEDU-ROS2

TelloEDU-ROS2 is a ROS 2 package designed for controlling and managing Tello EDU drones. This project includes functionalities such as drone control, computer vision, and tracking using the ROS 2 Humble framework.

## Index
- [Project Overview](#project-overview)
- [Prerequisites](#prerequisites)
- [Project Setup](#project-setup)
- [Running the Project](#running-the-project)
- [Explanation of Important Scripts](#explanation-of-important-scripts)
  - [Aruco Detection and Tracking](#aruco-detection-and-tracking)
  - [Object Detection and Tracking](#object-detection-and-tracking)
  - [Custom Controller](#custom-controller)
- [Swarm Scripts](#swarm-scripts)
- [License](#license)

## Project Overview
TelloEDU-ROS2 provides a comprehensive solution for Tello EDU drones, offering features like:
- Aruco marker detection and tracking
- Object detection and tracking using YOLO models
- Manual control via a custom keyboard controller

## Prerequisites
### Tello EDU Setup
- Ensure you have a Tello EDU drone with the necessary firmware.

### ROS 2 Humble Installation
- Follow the official ROS 2 Humble installation guide for your operating system.

## Project Setup
1. Clone the repository:
    ```sh
    git clone https://github.com/your_username/TelloEDU-ROS2.git
    cd TelloEDU-ROS2
    ```
2. Build the workspace:
    ```sh
    colcon build
    ```
3. Source the setup file:
    ```sh
    source install/setup.bash
    ```

## Running the Project
To launch the various functionalities, use the provided launch files:
- Startup:
    ```sh
    ros2 launch tello tello_startup.launch.py
    ```

## Explanation of Important Scripts
### Aruco Detection and Tracking
- **aruco_detection.py**: Detects Aruco markers in the video stream.
- **aruco_tracking.py**: Tracks detected Aruco markers and provides position feedback.

### Object Detection and Tracking
- **yolo_detection.py**: Detects objects using a YOLO model.
- **object_tracking.py**: Tracks detected objects and provides position feedback.
- **compressed_image_viewer.py**: Utility to view compressed images from the drone's camera.

### Custom Controller
- **controller.py**: Allows manual control of the drone using keyboard inputs.

## Swarm Scripts
The `tello_swarm` directory contains scripts for swarm operations:
- **swarm.py**: Main swarm control script.
- **set_drone_ap.py**: Script to change the access point mode for the drone.
- **tello_formation_swarm.py**: Script for controlling multiple drones in a formation.

## License
This project is licensed under the MIT License - see the [LICENSE](tello/LICENSE) file for details.
