import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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

    tello_keyboard_control = Node(
        package='tello',
        executable='controller.py',
        name='tello_keyboard_control',
        output='screen'
        remappings=[('control'), ('key_vel')]
    )

    # yolo_detection = Node(
    #     package='tello',
    #     executable='yolo_detection.py',
    #     name='yolo_detection',
    #     output='screen'
    # )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('cmd_vel_out', '/control')], 
        parameters=[params_file]
    )



    return LaunchDescription([
        tello_server,
        tello_keyboard_control,
        twist_mux
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d' + os.path.join(get_package_share_directory('skynet'), 'config', 'config.rviz')],
        # )
    ])
