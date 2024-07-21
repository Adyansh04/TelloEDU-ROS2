#!/usr/bin/env python3


import pprint
import math
import rclpy
import threading
import numpy as np
import time
import av
import tf2_ros
import cv2
import time
import yaml


from djitellopy import Tello

from rclpy.node import Node
from tello_interfaces.msg import TelloStatus, TelloID, TelloWifiConfig
from std_msgs.msg import Empty, UInt8, UInt8, Bool, String
from sensor_msgs.msg import Image, Imu, BatteryState, Temperature, CameraInfo, CompressedImage
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import ament_index_python
import tf_transformations

class TelloServerNode(Node):
    def __init__(self):
        super().__init__('tello_server')


        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('connect_timeout', '10.0'),
                ('tello_ip', '192.168.10.1'),
                ('tf_base', 'map'),
                ('tf_drone', 'drone'),
                ('tf_pub', True),
                # ('camera_info_file', '')
            ]
        )

        # Get parameters
        self.connect_timeout = self.get_parameter('connect_timeout').get_parameter_value().double_value
        self.tello_ip = self.get_parameter('tello_ip').get_parameter_value().string_value
        self.tf_base = self.get_parameter('tf_base').get_parameter_value().string_value
        self.tf_drone = self.get_parameter('tf_drone').get_parameter_value().string_value
        self.tf_pub = self.get_parameter('tf_pub').get_parameter_value().bool_value
        # self.camera_info_file = self.get_parameter('camera_info_file').get_parameter_value().string_value

        #Camera information loaded from calibration yaml
        self.camera_info = None

        # # Check if camera info file was received as argument
        # if len(self.camera_info_file) == 0:
        #     share_directory = ament_index_python.get_package_share_directory('tello')
        #     self.camera_info_file = share_directory + '/ost.yaml'

        # # Read camera info from YAML file
        # with open(self.camera_info_file, 'r') as file:
        #     self.camera_info = yaml.load(file, Loader=yaml.FullLoader)
        #     # self.node.get_logger().info('Tello: Camera information YAML' + self.camera_info.__str__())

        # Configure Drone Connection 
        Tello.TELLO_IP = self.tello_ip
        Tello.RESPONSE_TIMEOUT = int(self.connect_timeout)
        Tello.TAKEOFF_TIMEOUT = int(60)

        # Connecting to Drone
        self.get_logger().info('Connecting to Tello...')

        self.tello = Tello()
        self.tello.connect()

        self.get_logger().info('Connected to Tello')

        # Setup Publishers and Subscribers
        self.setup_publishers()
        self.setup_subscribers()

        #Processing Threads
        self.start_video_capture()
        self.start_tello_status()
        self.start_tello_odom()

        self.get_logger().info('Tello Server Node Started')

    def setup_publishers(self):
        """Setup publishers for the node"""
        self.pub_image_raw = self.create_publisher(Image, 'image_raw', 10)
        self.pub_image_compressed = self.create_publisher(CompressedImage, 'image_compressed', 10)
        self.pub_camera_info = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.pub_status = self.create_publisher(TelloStatus, 'status', 10)
        self.pub_id = self.create_publisher(TelloID, 'id', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu', 10)
        self.pub_battery = self.create_publisher(BatteryState, 'battery', 10)
        self.pub_temperature = self.create_publisher(Temperature, 'temperature', 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster
        if self.tf_pub:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def setup_subscribers(self):
        """ Setup subscribers for the node"""
        self.sub_emergency = self.create_subscription(Empty, 'emergency', self.cb_emergency, 10)
        self.sub_takeoff = self.create_subscription(Empty, 'takeoff', self.cb_takeoff, 10)
        self.sub_land = self.create_subscription(Empty, 'land', self.cb_land, 10)
        self.sub_control = self.create_subscription(Twist, 'control', self.cb_control, 10)
        self.sub_flip = self.create_subscription(String, 'flip', self.cb_flip, 10)
        self.seb_thow_takeoff = self.create_subscription(Empty, 'throw_takeoff', self.cb_throw_takeoff, 10)
        self.sub_wifi_config = self.create_subscription(TelloWifiConfig, 'wifi_config', self.cb_wifi_config, 10)

        
    # Get the orientation of the drone as a quaternion
    def get_orientation_quaternion(self):
        deg_to_rad = math.pi / 180.0
        return euler_to_quaternion([
            self.tello.get_yaw() * deg_to_rad,
            self.tello.get_pitch() * deg_to_rad,
            self.tello.get_roll() * deg_to_rad
        ])

    # Start drone info thread
    def start_tello_odom(self, rate=1.0/10.0):
        def status_odom():
            while True:
                # TF
                if self.tf_pub:
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = self.tf_base
                    t.child_frame_id = self.tf_drone
                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    t.transform.translation.z = (self.tello.get_barometer()) / 100.0
                    self.tf_broadcaster.sendTransform(t)
                
                # IMU
                if self.pub_imu.get_subscription_count() >= 0:
                    q = self.get_orientation_quaternion()

                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.tf_drone
                    msg.linear_acceleration.x = self.tello.get_acceleration_x() / 100.0
                    msg.linear_acceleration.y = self.tello.get_acceleration_y() / 100.0
                    msg.linear_acceleration.z = self.tello.get_acceleration_z() / 100.0
                    msg.orientation.x = q[0]
                    msg.orientation.y = q[1]
                    msg.orientation.z = q[2]
                    msg.orientation.w = q[3]
                    self.pub_imu.publish(msg)

                # Odometry
                if self.pub_odom.get_subscription_count() >= 0:
                    q = self.get_orientation_quaternion()

                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = self.tf_base
                    odom_msg.pose.pose.orientation.x = q[0]
                    odom_msg.pose.pose.orientation.y = q[1]
                    odom_msg.pose.pose.orientation.z = q[2]
                    odom_msg.pose.pose.orientation.w = q[3]
                    odom_msg.twist.twist.linear.x = float(self.tello.get_speed_x()) / 100.0
                    odom_msg.twist.twist.linear.y = float(self.tello.get_speed_y()) / 100.0
                    odom_msg.twist.twist.linear.z = float(self.tello.get_speed_z()) / 100.0
                    self.pub_odom.publish(odom_msg)
                
                time.sleep(rate)

        thread = threading.Thread(target=status_odom)
        thread.start()
        return thread

    # Start drone info thread
    def start_tello_status(self, rate=1.0/1.0):
        def status_loop():
            while True:
                # Battery
                if self.pub_battery.get_subscription_count() >= 0:
                    msg = BatteryState()
                    msg.header.frame_id = self.tf_drone
                    msg.percentage = float(self.tello.get_battery())
                    msg.voltage = 3.8
                    msg.design_capacity = 1.1
                    msg.present = True
                    msg.power_supply_technology = 2 # POWER_SUPPLY_TECHNOLOGY_LION
                    msg.power_supply_status = 2 # POWER_SUPPLY_STATUS_DISCHARGING
                    self.pub_battery.publish(msg)

                # Temperature
                if self.pub_temperature.get_subscription_count() >= 0:
                    msg = Temperature()
                    msg.header.frame_id = self.tf_drone
                    msg.temperature = self.tello.get_temperature()
                    msg.variance = 0.0
                    self.pub_temperature.publish(msg)

                # Tello Status
                if self.pub_status.get_subscription_count() >= 0:
                    msg = TelloStatus()
                    msg.acceleration.x = self.tello.get_acceleration_x()
                    msg.acceleration.y = self.tello.get_acceleration_y()
                    msg.acceleration.z = self.tello.get_acceleration_z()

                    msg.speed.x = float(self.tello.get_speed_x())
                    msg.speed.y = float(self.tello.get_speed_y())
                    msg.speed.z = float(self.tello.get_speed_z())

                    msg.pitch = self.tello.get_pitch()
                    msg.roll = self.tello.get_roll()
                    msg.yaw = self.tello.get_yaw()

                    msg.barometer = int(self.tello.get_barometer())
                    msg.distance_tof = self.tello.get_distance_tof()

                    msg.fligth_time = self.tello.get_flight_time()

                    msg.battery = self.tello.get_battery()

                    msg.highest_temperature = self.tello.get_highest_temperature()
                    msg.lowest_temperature = self.tello.get_lowest_temperature()
                    msg.temperature = self.tello.get_temperature()

                    # msg.wifi_snr = self.tello.query_wifi_signal_noise_ratio()

                    self.pub_status.publish(msg)

                # Tello ID
                if self.pub_id.get_subscription_count() > 0:
                    msg = TelloID()
                    msg.sdk_version = self.tello.query_sdk_version()
                    msg.serial_numbers = self.tello.query_serial_number()
                    self.pub_id.publish(msg)

                # # Camera info
                # if self.pub_camera_info.get_subscription_count() > 0:
                #     msg = CameraInfo()
                #     msg.height = self.camera_info.image_height
                #     msg.width = self.camera_info.image_width
                #     msg.distortion_model = self.camera_info.distortion_model
                #     msg.D = self.camera_info.distortion_coefficients
                #     msg.K = self.camera_info.camera_matrix
                #     msg.R = self.camera_info.rectification_matrix
                #     msg.P = self.camera_info.projection_matrix
                #     self.pub_camera_info.publish(msg)
                
                # Sleep
                time.sleep(rate)

        thread = threading.Thread(target=status_loop)
        thread.start()
        return thread


    # Start video capture thread.
    def start_video_capture(self, rate=1.0/30.0):
        self.tello.streamon()
        self.tello.set_video_fps(Tello.FPS_15)
        self.tello.set_video_resolution(Tello.RESOLUTION_720P)
        self.bridge = CvBridge()

        def video_capture_thread():
            frame_read = self.tello.get_frame_read()
            last_frame_id = -1

            while True:
                frame = frame_read.frame
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Debugging: Check if the frame ID (or timestamp) has changed
                current_frame_id = id(frame_bgr)  # Using the object's ID as a simple check
                if current_frame_id != last_frame_id:
                    # print("Frame has changed.")
                    last_frame_id = current_frame_id

                    msg = self.bridge.cv2_to_imgmsg(frame_bgr, 'bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.tf_drone
                    self.pub_image_raw.publish(msg)

                    # Compressed image
                    msg_compress = CompressedImage()
                    compressed_msg = self.bridge.cv2_to_compressed_imgmsg(frame_bgr, dst_format='jpeg')
                    compressed_msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp to match raw image
                    compressed_msg.header.frame_id = self.tf_drone
                    self.pub_image_compressed.publish(compressed_msg)

                    # Additional debugging to verify compression
                    # print(f"Compressed image size: {len(compressed_msg.data)} bytes")

                else:
                    print("Frame has not changed.")

                time.sleep(rate)

        thread = threading.Thread(target=video_capture_thread)
        thread.start()
        return thread

    # Terminate the code and shutdown node.
    def terminate(self, err):
        self.node.get_logger().error(str(err))
        self.tello.end()
        rclpy.shutdown()

    # Stop all movement in the drone
    def cb_emergency(self, msg):
        self.tello.emergency()
        return

    # Drone takeoff message control
    def cb_takeoff(self, msg):
        self.tello.takeoff()
        return

    # Land the drone message callback
    def cb_land(self, msg):
        self.tello.land()
        return

    def cb_throw_takeoff(self, msg):
        self.tello.initiate_throw_takeoff()
        return

    # Control messages received use to control the drone "analogically"
    #
    # This method of controls allow for more precision in the drone control.
    #
    # Receives the linear and angular velocities to be applied from -100 to 100.
    def cb_control(self, msg):
        self.tello.send_rc_control(-int(msg.linear.y), int(msg.linear.x), int(msg.linear.z), -int(msg.angular.z))
        return

    # Configure the wifi credential that should be used by the drone.
    #
    # The drone will be restarted after the credentials are changed.
    def cb_wifi_config(self, msg):
        self.tello.set_wifi_credentials(msg.ssid, msg.password)
    
    # Perform a drone flip in a direction specified.
    # 
    # Directions can be "r" for right, "l" for left, "f" for forward or "b" for backward.
    def cb_flip(self, msg):
        self.tello.flip(msg.data)

# Convert a rotation from euler to quaternion.
def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# Convert rotation from quaternion to euler.
def quaternion_to_euler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def main(args=None):

    rclpy.init(args=args)
    node = TelloServerNode()
    rclpy.spin(node)
    node.get_logger().info('Shutting down Tello Server Node')
    node.terminate()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()