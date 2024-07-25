import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import yaml


def generate_launch_description():

    params_file = os.path.join(get_package_share_directory('tello'), 'config', 'tello_startup_params.yaml')
    
    tello_server = Node(
        package='tello',
        executable='tello_server.py',
        name='tello_server',
        output='screen',
        # parameters=[{'tello_ip': ''}]
    )

    tello_keyboard_control = ExecuteProcess(
                                 cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 run tello controller.py'],
                                 output='screen',

                              )

    yolo_detection_object = Node(
        package='tello',
        executable='yolo_detection.py',
        name='yolo_detection_object',
        output='screen',
        parameters=[{'model': 'object'}],   
        remappings=[
            ('yolo_results', 'yolo_results_object'),
            ('annotated_compressed_image', 'annotated_compressed_image/object')
            ]     
    )

    yolo_detection_face = Node(
        package='tello',
        executable='yolo_detection.py',
        name='yolo_detection_face',
        output='screen',
        parameters=[{'model': 'face'}],
        remappings=[
            ('yolo_results', 'yolo_results_face'),
            ('annotated_compressed_image', 'annotated_compressed_image/face')
            ]
    )

    compressed_image_viewer_object = Node(
        package='tello',
        executable='compressed_image_viewer.py',
        name='compressed_image_viewer_object',
        output='screen',
        remappings=[
            ('annotated_compressed_image', 'annotated_compressed_image/object'),
            ('aruco_annotations_image', 'aruco_annotations_object')
        ]
    )

    compressed_image_viewer_face = Node(
        package='tello',
        executable='compressed_image_viewer.py',
        name='compressed_image_viewer_face',
        output='screen',
        remappings=[
            ('annotated_compressed_image', 'annotated_compressed_image/face'),
            ('aruco_annotations_image', 'aruco_annotations_face')
        ]
    )

    aruco_detection = Node(
        package='tello',
        executable='aruco_detection.py',
        name='aruco_detection',
        output='screen'
    )

    aruco_tracking = Node(
        package='tello',
        executable='aruco_tracking.py',
        name='aruco_tracking',
        output='screen',
        parameters=[{'aruco_list': [0, 1, 2, 10]}]
    )

    return LaunchDescription([
        tello_server,
        aruco_detection,
        aruco_tracking,
        yolo_detection_object,
        yolo_detection_face,
        tello_keyboard_control,
        compressed_image_viewer_face,
        compressed_image_viewer_object
    ])
