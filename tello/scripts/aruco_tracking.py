#!/usr/bin/env python3

'''Script to check ArUco IDs from a topic and log the data comparing it with a JSON file, with a GUI display.'''

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Empty
from tello_interfaces.msg import ArucoID
from datetime import datetime
import json
import tkinter as tk
from tkinter import ttk

class ArucoTrackingNode(Node):

    def __init__(self):
        super().__init__('aruco_tracking')

        self.declare_parameter('aruco_list', [0, 1, 2])

        aruco_list_param = self.get_parameter('aruco_list').get_parameter_value().integer_array_value
        self.aruco_to_track_list = [int(i) for i in aruco_list_param]

        self.get_logger().info('Aruco IDs to track: {0}'.format(self.aruco_to_track_list))

        self.subscription = self.create_subscription(
            ArucoID,
            'aruco_ids',
            self.aruco_callback,
            10
        )

        self.start_subscription = self.create_subscription(
            Empty,
            'aruco_tracking_start',
            self.start_tracking_callback,
            10
        )

        self.stop_subscription = self.create_subscription(
            Empty,
            'aruco_tracking_stop',
            self.stop_tracking_callback,
            10
        )

        self.aruco_tracking = False
        self.unique_aruco_ids = set()

        # Load ArUco data from JSON file
        with open('/home/adyansh/uav_autonomous_ML/tello-ws/src/tello/scripts/aruco_data.json', 'r') as f:
            self.aruco_data = json.load(f)

        self.get_logger().info('Aruco Tracking Node Initialized')

        # Initialize GUI
        self.init_gui()

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("Aruco Tracking")

        self.tracked_label = tk.Label(self.root, text="Tracked ArUco IDs", font=('Arial', 14))
        self.tracked_label.pack()

        self.tracked_listbox = tk.Listbox(self.root, width=50, height=10)
        self.tracked_listbox.pack()

        self.missing_label = tk.Label(self.root, text="Missing ArUco IDs", font=('Arial', 14))
        self.missing_label.pack()

        self.missing_listbox = tk.Listbox(self.root, width=50, height=10)
        self.missing_listbox.pack()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def aruco_callback(self, msg):
        if self.aruco_tracking:
            self.get_logger().info('Aruco IDs: {0}'.format(msg.aruco_ids))
            for aruco_id in msg.aruco_ids:
                self.unique_aruco_ids.add(aruco_id)
            self.update_gui()

    def start_tracking_callback(self, msg):
        self.aruco_tracking = True
        self.unique_aruco_ids.clear()
        self.get_logger().info('Aruco Tracking Started')
        self.update_gui()

    def stop_tracking_callback(self, msg):
        self.aruco_tracking = False
        self.get_logger().info('Aruco Tracking Stopped')

        # Compare tracked IDs with the list of IDs to track
        missing_aruco_ids = [aruco_id for aruco_id in self.aruco_to_track_list if aruco_id not in self.unique_aruco_ids]

        # Prepare log data for tracked ArUco IDs
        log_data = {
            'timestamp': datetime.now().isoformat(),
            'tracked_aruco_ids': []
        }

        for aruco_id in self.unique_aruco_ids:
            aruco_id_str = str(aruco_id)
            if aruco_id_str in self.aruco_data:
                info = self.aruco_data[aruco_id_str]
                log_data['tracked_aruco_ids'].append({
                    'aruco_id': aruco_id,
                    'description': info,
                    'timestamp': datetime.now().isoformat()
                })

        # Store log data in a human-readable file
        with open('aruco_tracking_log.json', 'w') as f:
            json.dump(log_data, f, indent=4)

        # Update GUI
        self.update_gui(missing_aruco_ids)

        # Log the missing ArUco IDs
        print('\033[1;91mMissing ArUco IDs: {0}\033[0m'.format(missing_aruco_ids))

    def update_gui(self, missing_aruco_ids=None):
        self.tracked_listbox.delete(0, tk.END)
        for aruco_id in self.unique_aruco_ids:
            aruco_id_str = str(aruco_id)
            if aruco_id_str in self.aruco_data:
                info = self.aruco_data[aruco_id_str]
                self.tracked_listbox.insert(tk.END, f'ID: {aruco_id}, Description: {info}')

        self.missing_listbox.delete(0, tk.END)
        if missing_aruco_ids is not None:
            for aruco_id in missing_aruco_ids:
                self.missing_listbox.insert(tk.END, f'ID: {aruco_id}')

    def on_closing(self):
        self.root.quit()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    aruco_tracking_node = ArucoTrackingNode()

    # Run the ROS2 node in a non-blocking way
    executor = MultiThreadedExecutor()
    executor.add_node(aruco_tracking_node)

    def spin():
        try:
            executor.spin()
        finally:
            executor.shutdown()
            aruco_tracking_node.destroy_node()
            rclpy.shutdown()

    # Run the spin function in a separate thread
    import threading
    spin_thread = threading.Thread(target=spin)
    spin_thread.start()

    # Run the Tkinter main loop in the main thread
    aruco_tracking_node.root.mainloop()

    # Ensure the spin thread is joined before exiting
    spin_thread.join()

if __name__ == '__main__':
    main()
