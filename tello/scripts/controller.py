#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from pynput import keyboard

class KeyboardControllerNode(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        self.publisher = self.create_publisher(Twist, 'control', 10)
        self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, 'land', 10)
        self.flip_publisher = self.create_publisher(String, 'flip', 10)
        self.emergency_publisher = self.create_publisher(Empty, 'emergency', 10)
        self.throw_takeoff_publisher = self.create_publisher(Empty, 'throw_takeoff', 10)

        self.aruco_tracking_start = self.create_publisher(Empty, 'aruco_tracking_start', 10)
        self.aruco_tracking_stop = self.create_publisher(Empty, 'aruco_tracking_stop', 10)

        self.velocity_factor = 10.0
        self.current_twist = Twist()
        self.key_pressed = set()

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

        self.get_logger().info("Keyboard Controller Initialized")

        # Print the instructions to control the drone using the keyboard

        instructions = """
        \033[1;34mControl the drone using the following keys:\033[0m

        \033[3;32mPlanar Movement:\033[0m
             \033[1;33mW\033[0m            -> Move forward
        \033[1;33mA\033[0m    \033[1;33mS\033[0m    \033[1;33mD\033[0m       -> Move left, backward, right

        \033[3;32mAltitude:\033[0m
        \033[36mUP Arrow\033[0m -> Increase altitude
        \033[36mDOWN Arrow\033[0m -> Decrease altitude

        \033[3;32mRotation:\033[0m
        \033[36mLEFT Arrow\033[0m -> Rotate anti-clockwise
        \033[36mRIGHT Arrow\033[0m -> Rotate clockwise

        \033[3;32mVelocity:\033[0m
        \033[35mZ\033[0m -> Increase velocity
        \033[35mX\033[0m -> Decrease velocity

        \033[3;32mSpecial Commands:\033[0m
        \033[31mT\033[0m -> Takeoff
        \033[31mL\033[0m -> Land
        \033[31m1\033[0m -> Flip left
        \033[31m2\033[0m -> Flip right
        \033[31m3\033[0m -> Flip forward
        \033[31m4\033[0m -> Flip backward
        \033[31m9\033[0m -> Throw and takeoff
        \033[31mSpace\033[0m -> Emergency stop
        \033[31mO\033[0m -> Start ArUco tracking
        \033[31mP\033[0m -> Stop ArUco tracking
        """

        print(instructions)
        
        

    def on_press(self, key):
        self.key_pressed.add(key)
        self.update_twist(key, pressed=True)
        self.handle_special_keys(key, pressed=True)

    def on_release(self, key):
        self.key_pressed.discard(key)
        self.update_twist(key, pressed=False)
        self.handle_special_keys(key, pressed=False)

    def update_twist(self, key, pressed):
        try:
            if key.char == 'w':
                self.current_twist.linear.x = 1.0 * self.velocity_factor if pressed else 0.0
            elif key.char == 's':
                self.current_twist.linear.x = -1.0 * self.velocity_factor if pressed else 0.0
            elif key.char == 'a':
                self.current_twist.linear.y = 1.0 * self.velocity_factor if pressed else 0.0
            elif key.char == 'd':
                self.current_twist.linear.y = -1.0 * self.velocity_factor if pressed else 0.0
            elif key.char == 'z' and pressed:  # Increase velocity
                self.velocity_factor *= 1.1
            elif key.char == 'x' and pressed:  # Decrease velocity
                self.velocity_factor /= 1.1

        except AttributeError:
            if key == keyboard.Key.up:
                self.current_twist.linear.z = 1.0 * self.velocity_factor if pressed else 0.0
            elif key == keyboard.Key.down:
                self.current_twist.linear.z = -1.0 * self.velocity_factor if pressed else 0.0
            elif key == keyboard.Key.left:
                self.current_twist.angular.z = 1.0 * self.velocity_factor if pressed else 0.0
            elif key == keyboard.Key.right:
                self.current_twist.angular.z = -1.0 * self.velocity_factor if pressed else 0.0

        self.publisher.publish(self.current_twist)

    def handle_special_keys(self, key, pressed):
        try:
            if key.char == 't' and pressed:
                self.takeoff_publisher.publish(Empty())
            elif key.char == 'l' and pressed:
                self.land_publisher.publish(Empty())
            elif key.char == '1' and pressed:
                self.flip_publisher.publish(String(data='l'))
            elif key.char == '2' and pressed:
                self.flip_publisher.publish(String(data='r'))
            elif key.char == '3' and pressed:
                self.flip_publisher.publish(String(data='f'))
            elif key.char == '4' and pressed:
                self.flip_publisher.publish(String(data='b'))
            elif key.char == '9' and pressed:
                self.throw_takeoff_publisher.publish(Empty())
            elif key.char == 'o' and pressed:
                self.aruco_tracking_start.publish(Empty())
            elif key.char == 'p' and pressed:
                self.aruco_tracking_stop.publish(Empty())

        except AttributeError:
            if key == keyboard.Key.space and pressed:
                self.emergency_publisher.publish(Empty())

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    keyboard_controller = KeyboardControllerNode()
    keyboard_controller.run()
    keyboard_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
