#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tello_interfaces.msg import YoloResults
from simple_pid import PID
from numpy import clip

class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking')

        # Declare parameters for desired values and PID coefficients
        self.declare_parameter('desired_contour_area', 70000) #70000
        self.declare_parameter('desired_x_difference', 0)
        self.declare_parameter('desired_z_difference', 0)
        self.declare_parameter('kp_area', 0.0005)
        self.declare_parameter('ki_area', 0.0)
        self.declare_parameter('kd_area', 0.0)
        self.declare_parameter('kp_x', 0.1)
        self.declare_parameter('ki_x', 0.0001)
        self.declare_parameter('kd_x', 0.001)
        self.declare_parameter('kp_z', 0.1)
        self.declare_parameter('ki_z', 0.0001)
        self.declare_parameter('kd_z', 0.001)
        self.declare_parameter('threshold_area', 10000)
        self.declare_parameter('threshold_x', 10)
        self.declare_parameter('threshold_z', 10)
        self.declare_parameter('target_class_id', 39)  # Class ID to track
        self.declare_parameter('sweep_angular_z', 24)  # Angular z velocity for sweeping
        self.declare_parameter('max_velocity', 50.0)  # Maximum linear velocity (m/s

        # Get parameters
        self.desired_contour_area = self.get_parameter('desired_contour_area').get_parameter_value().integer_value
        self.desired_x_difference = self.get_parameter('desired_x_difference').get_parameter_value().integer_value
        self.desired_z_difference = self.get_parameter('desired_z_difference').get_parameter_value().integer_value

        kp_area = self.get_parameter('kp_area').get_parameter_value().double_value
        ki_area = self.get_parameter('ki_area').get_parameter_value().double_value
        kd_area = self.get_parameter('kd_area').get_parameter_value().double_value

        kp_x = self.get_parameter('kp_x').get_parameter_value().double_value
        ki_x = self.get_parameter('ki_x').get_parameter_value().double_value
        kd_x = self.get_parameter('kd_x').get_parameter_value().double_value

        kp_z = self.get_parameter('kp_z').get_parameter_value().double_value
        ki_z = self.get_parameter('ki_z').get_parameter_value().double_value
        kd_z = self.get_parameter('kd_z').get_parameter_value().double_value

        self.threshold_area = self.get_parameter('threshold_area').get_parameter_value().integer_value
        self.threshold_x = self.get_parameter('threshold_x').get_parameter_value().integer_value
        self.threshold_z = self.get_parameter('threshold_z').get_parameter_value().integer_value

        self.target_class_id = self.get_parameter('target_class_id').get_parameter_value().integer_value
        self.sweep_angular_z = self.get_parameter('sweep_angular_z').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value

        # Initialize PID controllers
        self.pid_area = PID(kp_area, ki_area, kd_area, setpoint=0)
        self.pid_x = PID(kp_x, ki_x, kd_x, setpoint=self.desired_x_difference)
        self.pid_z = PID(kp_z, ki_z, kd_z, setpoint=self.desired_z_difference)

        self.linearx = 0.0
        self.lineary = 0.0
        self.linearz = 0.0
        self.angularz = 0.0
        

        # Subscribe to YOLO results topic
        self.subscription = self.create_subscription(
            YoloResults,
            'yolo_results_object',
            self.yolo_callback,
            10
        )

        # Publisher for drone control commands
        self.publisher = self.create_publisher(Twist, 'control', 10)

        self.create_timer(0.1, self.vel_slow_pub)

        self.get_logger().info("Drone Controller Node Initialized")

    def vel_slow_pub(self):

        # print(self.linearx, self.lineary, self.linearz, self.angularz)

        # Limit linear velocity to maximum value
        self.linearx = clip(self.linearx, -self.max_velocity, self.max_velocity)
        self.lineary = clip(self.lineary, -self.max_velocity, self.max_velocity)
        self.linearz = clip(self.linearz, -self.max_velocity, self.max_velocity)

    
        twist_msg = Twist()

        twist_msg.linear.x = self.linearx
        twist_msg.linear.y = self.lineary
        twist_msg.linear.z = self.linearz
        twist_msg.angular.z = self.angularz


        print(self.linearx, self.lineary, self.linearz, self.angularz)

        # self.get_logger().info(f"Publishing Twist message: {twist_msg}")

        self.publisher.publish(twist_msg)

    def yolo_callback(self, msg):
        target_indices = [i for i, class_id in enumerate(msg.class_ids) if class_id == self.target_class_id]
        
        if not target_indices:
            # If no target is detected, sweep in angular z
            self.sweep()
            return

        # Sort indices based on contour area in descending order
        target_indices.sort(key=lambda i: msg.contour_area[i], reverse=True)

        # Select the object with the largest contour area
        target_index = target_indices[0]

        # Extract contour area and differences for the target object
        contour_area = msg.contour_area[target_index]
        x_difference = msg.x_differences[target_index]
        z_difference = msg.z_differences[target_index]

        # self.get_logger().error(f"Contour Area: {contour_area}, X Difference: {x_difference}, Z Difference: {z_difference}")

        # Calculate errors
        error_area =  self.desired_contour_area - contour_area 
        error_x = self.desired_x_difference - x_difference
        error_z = self.desired_z_difference - z_difference

        self.get_logger().info(f"Errors: Area: {error_area}",
                                # "X: {error_x}, Z: {error_z}"
                                )

        # Compute control signals using PID controllers
        control_area = self.pid_area(error_area)
        control_x = self.pid_x(error_x)
        control_z = self.pid_z(error_z)

        self.get_logger().warn(
                    f"Controls: Area: {control_area}, "
                    # f"X: {control_x}," 
                    # f"Z: {control_z}"
                    )

        # Check if all conditions are within thresholds
        within_threshold_area = abs(error_area) <= self.threshold_area
        within_threshold_x = abs(error_x) <= self.threshold_x
        within_threshold_z = abs(error_z) <= self.threshold_z

        # self.get_logger().info(f"Within Thresholds: Area: {within_threshold_area}, X: {within_threshold_x}, Z: {within_threshold_z}")


        if within_threshold_area and within_threshold_x and within_threshold_z:
            # Publish zero velocities if all conditions are within thresholds
            self.linearx = 0.0
            self.lineary = 0.0
            self.linearz = 0.0
            self.angularz = 0.0
            self.get_logger().info("All conditions met, stopping drone.")
            rclpy.shutdown()
        else:

            # Apply control signals to Twist message, with threshold checks
            self.linearx = -control_area if not within_threshold_area else 0.0
            self.lineary = 0.0  # Assuming no lateral movement control needed
            self.linearz = -control_z if not within_threshold_z else 0.0
            self.angularz = -control_x if not within_threshold_x else 0.0
            self.get_logger().info("Publishing control signals...")

            


    def sweep(self):    
        # Create Twist message for sweeping in angular z
        self.linearx = 0.0
        self.lineary = 0.0
        self.linearz = 0.0
        self.angularz = self.sweep_angular_z

        # Publish sweep command
        self.get_logger().info("Sweeping to find target object...")

def main(args=None):
    rclpy.init(args=args)
    object_tracking = ObjectTrackingNode()
    rclpy.spin(object_tracking)
    object_tracking.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
