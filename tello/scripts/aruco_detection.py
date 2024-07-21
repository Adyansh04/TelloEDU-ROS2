#!/usr/bin/env python3

'''Script to detect ArUco markers in the image stream from the Tello drone'''

import rclpy
from rclpy.node import Node
import cv2
import cv_bridge
import numpy as np
import json

from sensor_msgs.msg import CompressedImage
from tello_interfaces.msg import ArucoID
class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection')

        self.subscription = self.create_subscription(
            CompressedImage,
            'image_compressed',
            # 'image_raw/compressed',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(
            CompressedImage,
            'aruco_annotations_image',
            10
        )

        self.aruco_pub = self.create_publisher(
            ArucoID,
            'aruco_ids',
            10
        )
        
        self.get_logger().info('Aruco Detection Node Initialized')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Load ArUco data from JSON file
        with open('/home/adyansh/uav_autonomous_ML/tello-ws/src/tello/scripts/aruco_data.json', 'r') as f:
            self.aruco_data = json.load(f)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        bridge = cv_bridge.CvBridge()
        img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect ArUco markers
        corners, ids, rejected = self.detector.detectMarkers(img)

        if ids is not None:

            aruco_ids_msg = ArucoID()
            aruco_ids_msg.aruco_ids = ids.flatten().tolist()
            self.aruco_pub.publish(aruco_ids_msg)

            for i in range(len(ids)):
                # Draw the marker outline
                cv2.aruco.drawDetectedMarkers(img, corners, ids)

                # Get the center of the marker
                c = corners[i][0]
                center = (int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4),
                          int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4))

                # Draw center point
                cv2.circle(img, center, 5, (0, 0, 255), -1)

                # Draw corner points
                for point in c:
                    cv2.circle(img, (int(point[0]), int(point[1])), 5, (255, 0, 0), -1)

                # Draw the orientation line
                vector = c[0] - c[3]
                angle = (np.arctan2(vector[1], vector[0]) * 180 / np.pi) + 90
                cv2.putText(img, f'ID: {ids[i][0]}', (int(c[0][0]), int(c[0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.line(img, center, (center[0] + int(vector[0] * 10), center[1] + int(vector[1] * 10)), (0, 255, 0), 2)
                cv2.putText(img, f'Angle: {int(angle)} deg', (center[0], center[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Display associated information from JSON
                aruco_id = str(ids[i][0])
                if aruco_id in self.aruco_data:
                    info = self.aruco_data[aruco_id]
                    cv2.putText(img, info, (int(c[0][0]), int(c[0][1]) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Convert OpenCV image back to ROS Image message and publish
        annotated_msg = bridge.cv2_to_compressed_imgmsg(img)
        self.publisher.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
