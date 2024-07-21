#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/annotated_compressed_image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            CompressedImage,
            '/aruco_annotations_image',
            self.listener_callback_aruco,
            10)

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        cv2.imshow('Compressed Image', image_np)
        cv2.waitKey(1)

    def listener_callback_aruco(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        cv2.imshow('Aruco Annotations Image', image_np)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    compressed_image_subscriber = CompressedImageSubscriber()
    rclpy.spin(compressed_image_subscriber)
    compressed_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()