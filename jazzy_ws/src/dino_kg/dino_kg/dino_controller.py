#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped, PoseStamped
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image
import threading
from cv_bridge import CvBridge
import cv2
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ServoCommandType
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
import numpy as np
import os
from scipy.spatial.transform import Rotation as R

# dino control comes from https://gist.github.com/normandipalo/fbc21f23606fbe3d407e22c363cb134e
import torch
import numpy as np 
import matplotlib.pyplot as plt 
from torchvision import transforms,utils
from PIL import Image as PILImage
import torchvision.transforms as T
import warnings 
import glob
import time
from dino_kg.correspondences import find_correspondences, draw_correspondences

# servoing similar to what is done here: https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/demos/servo_keyboard_input.cpp
# tf transform stuff based on https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html
class DinoControllerNode(Node):
    def __init__(self):
        super().__init__('dino_controller')
        self.declare_parameter('skill', 'robot_pickup_mug') # underscore seperated triplet used in behaviour graph
        self.skill = self.get_parameter('skill').get_parameter_value().string_value


        # this is used to get correct robot state so we can do servoing
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self.move_group_client.wait_for_server()


        # Publisher to Isaac Sim
        self.isaac_pub = self.create_publisher(
                JointState,
                '/joint_commands',
                10
                )

        # moveit servo publisher
        self.servo_pub = self.create_publisher(
                PoseStamped, 
                '/servo_node/pose_target_cmds', 
                10
                )
        # we need to instantiate the type of servo mode for moveit. 1 means twist. 2 is pose which we will use for the actual dino control
        self.command_type_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')

        # wait for servo srv
        while not self.command_type_client.wait_for_service(timeout_sec=1.0):
            pass

        req = ServoCommandType.Request()
        req.command_type = 2
        self.command_type_client.call_async(req)

        self.tlock = threading.Lock()

        self.is_recording = False
        self.recorded_trajectory = []
        self.bottleneck_rgb = None
        self.bottleneck_depth = None

        self.cv_bridge = CvBridge()
        self.latest_color_img = None
        self.latest_depth_img = None

        # replay stuff
        traj_path = f"/workspace/data/{self.skill}/trajectory.npy"
        self.trajectories = np.load(traj_path)
        self.gripper_position = 0.04
        self.idx = 0

        # used to find ee pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.base_frame = 'panda_link0'
        self.ee_frame = 'panda_link8'

        # should make faster subscription processing
        self.traj_cb_group = MutuallyExclusiveCallbackGroup()
        self.img_cb_group = MutuallyExclusiveCallbackGroup()

        self.color_sub = self.create_subscription(
                Image,
                '/realsense/color/image_raw',
                self.color_callback,
                10,
                callback_group=self.img_cb_group
                )

        self.depth_sub = self.create_subscription(
                Image,
                '/realsense/depth/image_rect_raw',
                self.depth_callback,
                10,
                callback_group=self.img_cb_group
                )

        self.traj_sub = self.create_subscription(
                JointTrajectory,
                '/panda_arm_controller/joint_trajectory',
                self.trajectory_callback,
                1,
                callback_group=self.traj_cb_group
                )

        # 20 hz
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("STARTED dino controller...")

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            return

        # converting trajectory to joint state
        js = JointState()
        js.header = msg.header
        js.name = list(msg.joint_names) + ['panda_finger_joint1', 'panda_finger_joint2']

        js.position = list(msg.points[0].positions)  + [self.gripper_position, self.gripper_position]
        js.velocity = list(msg.points[0].velocities) + [0, 0]

        self.isaac_pub.publish(js)

    def control_loop(self):
        with self.tlock:
            if self.idx < len(self.trajectories):
                pose_data = self.trajectories[self.idx]
                tx, ty, tz, rx, ry, rz, rw, self.gripper_position = pose_data

                target_pose = PoseStamped()
                target_pose.header.frame_id = self.ee_frame
                target_pose.header.stamp = self.get_clock().now().to_msg()
                target_pose.pose.position.x = tx
                target_pose.pose.position.y = ty
                target_pose.pose.position.z = tz
                target_pose.pose.orientation.x = rx
                target_pose.pose.orientation.y = ry
                target_pose.pose.orientation.z = rz
                target_pose.pose.orientation.w = rw


                self.servo_pub.publish(target_pose)

                self.idx += 1



    def color_callback(self, msg):
        self.latest_color_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.latest_depth_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

def main(args=None):
    rclpy.init(args=args)

    controller = DinoControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


