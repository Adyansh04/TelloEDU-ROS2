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
### Connect to Tello Drone WiFi
1. Connect your computer to the Tello drone's WiFi network.

### Launch the Startup File
2. Run the startup launch file which will also open a terminal for keyboard control:
    ```sh
    ros2 launch tello tello_startup.launch.py
    ```

### Object Tracking
To start object tracking, use the following command (replace \`target_class_id\` with the appropriate ID for the target object):
```sh
ros2 run tello object_tracking.py --ros-args -p target_class_id:=20
```

### Face Tracking
To start face tracking:
```sh
ros2 run tello face_tracking.py
```
### Pulling the Docker Image
To pull the latest version of the Docker image, use the following command:
```bash
docker pull adyansh04/crazyflie-ros2:latest
```

### Running the Docker Container
To run the container with access to your host's display and USB devices, including GPU support, use the following shell script. This script should be placed in the `docker/` folder of your project and can be named `run_container.sh`:
```bash
./docker/run_container.sh
```

Ensure that this script is executable:
```bash
chmod +x docker/run_container.sh
```
### Opening Another Terminal Session
To interact with the same running Docker container from another terminal session, use the `docker exec` command. Here's a shell script that can be named `open_terminal.sh` in the docker/ folder:
```bash
./docker/open_terminal.sh
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
1. **set_drone_ap.py**: Change the Tello drone's WiFi configuration to connect all drones to a single network. Use the following command:
    ```sh
    python set_drone_ap.py -s [SSID] -p [Password]
    ```
2. **tello_formation_swarm.py**: After connecting all drones to the same network, run this script to control them in formation:
    ```sh
    python tello_formation_swarm.py
    ```

## License
This project is licensed under the MIT License - see the [LICENSE](tello/LICENSE) file for details.
