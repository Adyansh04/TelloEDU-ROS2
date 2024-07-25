cmake_minimum_required(VERSION 3.8)
project(tello)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)  
find_package(std_msgs REQUIRED) 
find_package(tello_interfaces REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR} 
  ${OpenCV_INCLUDE_DIRS}  

)

add_executable(tello_control src/main.cpp)

ament_target_dependencies(tello_control
  rclcpp
  std_srvs
  std_msgs
  sensor_msgs
  OpenCV
  geometry_msgs
)

install(TARGETS
  tello_control
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(scripts)

# Install Python executables
install(PROGRAMS
  scripts/tello_server.py
  scripts/yolo_detection.py
  scripts/compressed_image_viewer.py
  scripts/controller.py
  scripts/object_tracking.py
  scripts/aruco_detection.py
  scripts/aruco_tracking.py
  scripts/face_tracking.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
