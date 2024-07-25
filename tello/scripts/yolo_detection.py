#!/usr/bin/env python3


from rclpy.node import Node
import rclpy
import cv_bridge
import cv2
import numpy as np

from ultralytics import YOLO

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from tello_interfaces.msg import Xywh, XyXy, YoloResults

class ObjectDetectionYOLO(Node):
    def __init__(self):
        super().__init__('object_detection_YOLO')

        self.subscription = self.create_subscription(
            CompressedImage,
            'image_compressed',
            self.image_callback,
            10)

        # Publish the annotated image
        self.publisher = self.create_publisher(
                                CompressedImage,
                                'annotated_compressed_image',
                                10
                            )
        # Publish the YOLO results
        self.yolo_result_publisher = self.create_publisher(
                                YoloResults,
                                'yolo_results',
                                10
                            )
        
        self.declare_parameter('model', 'object')
        self.model_type = self.get_parameter('model').get_parameter_value().string_value

        if self.model_type == 'face':
            self.model = YOLO(r'/home/adyansh/uav_autonomous_ML/tello-ws/src/tello/models/face_recognition.pt')
            self.get_logger().info('YOLO face model loaded successfully.')
        else: 
            self.model = YOLO(r'/home/adyansh/uav_autonomous_ML/tello-ws/src/tello/models/yolov8x.pt')
            self.get_logger().info('YOLO object model loaded successfully.')

        # Set the IOU and Confidence threshold
        self.iou = 0.3
        self.conf = 0.3


    def image_callback(self, msg):
        self.get_logger().info('Receiving image')

        # Convert ROS Image message to OpenCV image
        bridge = cv_bridge.CvBridge()
        img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.yolo_object_detect(img)

    def yolo_object_detect(self, img):
        
        # Perform object detection and stores the results
        results = self.model.track(img, persist=True, conf=self.conf, iou=self.iou)
        
        # Create a YoloResults message to store the results
        yolo_result_msg = YoloResults() 
        
        # List to store contour areas
        contour_areas = []

        # List to store differences
        differences = []

        # List to store z_differences
        z_differences = []

        # List to store center_y_to_track_points
        center_y_to_track_points = []

        # Calculate the  line's x-coordinate
        line_x = img.shape[1] // 2

        # Camera Center
        camera_center = (img.shape[1] // 2, img.shape[0] // 2)

        # Loop through the results and store the bounding box and stores the results in the YoloResults message
        for result in results:
            
            xywh_list = result.boxes.xywh.tolist() # center_x, center_y, width, height
            
            for xywh in xywh_list:
                xywh_msg = Xywh()
                xywh_msg.center_x = xywh[0]
                xywh_msg.center_y = xywh[1]
                xywh_msg.width = xywh[2]
                xywh_msg.height = xywh[3]
                
                # Calculate the center_y_to_track and store it in the list
                center_y_to_track = xywh[1] - (xywh[3] // 2)
                center_y_to_track_points.append(center_y_to_track)
                
                yolo_result_msg.xywh.append(xywh_msg)
                
                # Calculate contour area
                contour_area = xywh[2] ** 2
                contour_areas.append(contour_area)
                
                # Calculate  x - difference
                difference = (xywh[0] - line_x)
                differences.append(difference)

                # Calculate z - difference (height difference)
                z_difference = (camera_center[1] - center_y_to_track)
                z_differences.append(z_difference)

            xyxy_list = result.boxes.xyxy.tolist() # top_left_x, top_left_y, bottom_right_x, bottom_right_y
            
            for xyxy in xyxy_list:
                xyxy_msg = XyXy()
                xyxy_msg.tl_x = xyxy[0]
                xyxy_msg.tl_y = xyxy[1]
                xyxy_msg.br_x = xyxy[2]
                xyxy_msg.br_y = xyxy[3]
                
                yolo_result_msg.xyxy.append(xyxy_msg)
        
        # Class ids: 0- Blue-ball, 1- Purple-ball, 2- Red-Ball, 3- silo        
        cls_list = [int(cls) for cls in result.boxes.cls.tolist()]
        yolo_result_msg.class_ids.extend(cls_list)
        
        # Confidence values
        conf_list = result.boxes.conf.tolist()
        yolo_result_msg.confidence.extend(conf_list)
        
        # Tracking ids
        if result.boxes.id is not None:
            ids_list = result.boxes.id.tolist()
            yolo_result_msg.tracking_id.extend(ids_list)

        # Plot the annotated image
        annotated_frame = results[0].plot()

        # Draw the center points, contour areas, and differences
        for i, xywh in enumerate(yolo_result_msg.xywh):
            center_x, center_y, width, height = xywh.center_x, xywh.center_y, xywh.width, xywh.height
            contour_area = contour_areas[i]
            difference = differences[i]
            z_difference = z_differences[i]
            center_y_to_track = center_y_to_track_points[i]

            # Draw the center point
            cv2.circle(annotated_frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

            # Draw center_x_to_track point
            cv2.circle(annotated_frame, (int(center_x), int(center_y_to_track)), 5, (255, 255, 0), -1)


            # Put the contour area text
            cv2.putText(annotated_frame, f'Area: {int(contour_area)}', 
                        (int(center_x), int(center_y) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, 
                        (0, 255, 0), 
                        2)

            # Draw the difference line
            cv2.line(annotated_frame, (int(center_x), int(center_y_to_track)), (line_x, int(center_y_to_track)), (255, 0, 0), 2)

            # Put the difference text
            cv2.putText(annotated_frame, f'x_Diff: {int(difference)}', 
                        (int(center_x), int(center_y) + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, 
                        (255, 0, 0), 
                        2)
            
            # Put the z-difference text
            cv2.putText(annotated_frame, f'z_Diff: {int(z_difference)}', 
                        (int(center_x), int(center_y) + 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, 
                        (0, 255, 255), 
                        2)
            
        # Draw camera center point
        cv2.circle(annotated_frame, camera_center, 5, (0, 255, 255), -1)
        cv2.line(annotated_frame, (line_x, 0), (line_x, img.shape[0]), (0, 255, 0), 2) 

        # Add contour areas and differences to the YoloResults message
        yolo_result_msg.contour_area.extend(contour_areas)
        yolo_result_msg.x_differences.extend(differences)
        yolo_result_msg.center_y_to_track.extend(center_y_to_track_points)
        yolo_result_msg.z_differences.extend(z_differences)
        
        # cv2.imshow('YOLOv8 Tracking', annotated_frame)
        # cv2.waitKey(1)

        # Convert the annotated image to a ROS message
        bridge = cv_bridge.CvBridge()
        annotated_frame_msg = bridge.cv2_to_compressed_imgmsg(annotated_frame, dst_format='jpeg')
        
        # Publish the results        
        self.yolo_result_publisher.publish(yolo_result_msg)
        self.publisher.publish(annotated_frame_msg)

def main(args=None):
    rclpy.init(args=args)
    object_detection = ObjectDetectionYOLO()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()