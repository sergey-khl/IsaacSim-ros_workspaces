#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
import threading
from cv_bridge import CvBridge
import cv2


class DinoController(Node):
    def __init__(self):
        super().__init__('dino_controller_node')

        self.get_logger().info('Starting dino controller node')

        # Current target position
        self.target_position = 0.04  # Start open
        self.position_lock = threading.Lock()

        # Publisher to Isaac Sim
        self.isaac_pub = self.create_publisher(
                JointState,
                '/isaac_joint_commands',
                10
                )

        self.timer = self.create_timer(0.05, self.publish_gripper_command)

        self.cv_bridge = CvBridge()
        self.latest_color_img = None
        self.latest_depth_img = None
        self.center_depth_value = 0.0

        self.color_sub = self.create_subscription(
            Image,
            '/realsense/color/image_raw',
            self.color_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/realsense/depth/image_rect_raw',
            self.depth_callback,
            10
        )

    def color_callback(self, msg):
        self.latest_color_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.latest_depth_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        height, width = self.latest_depth_img.shape
        center_y, center_x = height // 2, width // 2
        self.center_depth_value = self.latest_depth_img[center_y, center_x]
        
        self.get_logger().info(f'Depth at center: {self.center_depth_value}')

    def publish_gripper_command(self):
        """Continuously publish the current gripper target to Isaac Sim"""
        with self.position_lock:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['panda_finger_joint1', 'panda_finger_joint2']
            msg.position = [self.target_position, self.target_position]
            self.isaac_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DinoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


