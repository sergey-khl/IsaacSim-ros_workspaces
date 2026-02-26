#!/usr/bin/env python3

from geometry_msgs.msg import TwistStamped, TransformStamped
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
from pynput import keyboard
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
import numpy as np
import os


# servoing similar to what is done here: https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/demos/servo_keyboard_input.cpp
# tf transform stuff based on https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html
class DinoRecordingNode(Node):
    def __init__(self):
        super().__init__('dino_recorder')
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
                TwistStamped, 
                '/servo_node/delta_twist_cmds', 
                10
                )

        # we need to instantiate the type of servo mode for moveit. 1 means twist. 2 is pose which we will use for the actual dino control
        self.command_type_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')

        # wait for servo srv
        while not self.command_type_client.wait_for_service(timeout_sec=1.0):
            pass

        req = ServoCommandType.Request()
        req.command_type = 1
        self.command_type_client.call_async(req)

        self.tlock = threading.Lock()

        self.is_recording = False
        self.recorded_trajectory = []
        self.bottleneck_rgb = None
        self.bottleneck_depth = None

        self.cv_bridge = CvBridge()
        self.latest_color_img = None
        self.latest_depth_img = None
        self.gripper_position = 0.04
        self.linear_speed = 0.1
        self.angular_speed = 0.1

        # used to find ee pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.base_frame = 'panda_link0'
        self.ee_frame = 'panda_link8'
        self.bottleneck_frame = 'bottleneck'

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
        self.pressed = set()
        self.listener = keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release).start()

        self.get_logger().info("STARTED dino recorder...")

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

    def start_recording(self):
        try:
            t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_frame,
                    rclpy.time.Time()
                    )

            t.header.stamp = self.get_clock().now().to_msg()
            t.child_frame_id = self.bottleneck_frame

            # initialize bottleneck frame. we use this to find relative pose transform in subsequent frames
            self.tf_broadcaster.sendTransform(t)

            if self.latest_color_img is not None:
                self.bottleneck_rgb = self.latest_color_img.copy()
            if self.latest_depth_img is not None:
                self.bottleneck_depth = self.latest_depth_img.copy()

            self.recorded_trajectory = []
            self.is_recording = True

            self.get_logger().info("starting recording")
        except Exception as e:
            self.get_logger().error(f"failed to start recording {e}")

    def save_recording(self):
        save_dir = f"/workspace/data/{self.skill}"
        os.makedirs(save_dir, exist_ok=True)

        if len(self.recorded_trajectory) > 0 and self.bottleneck_rgb is not None and self.bottleneck_depth is not None:
            traj_array = np.array(self.recorded_trajectory)
            np.save(os.path.join(save_dir, "trajectory.npy"), traj_array)

            cv2.imwrite(os.path.join(save_dir, "bottleneck_rgb.png"), self.bottleneck_rgb)
            cv2.imwrite(os.path.join(save_dir, "bottleneck_depth.png"), self.bottleneck_depth)

            self.get_logger().info("saved recordiung")
        else:
            self.get_logger().error("nothing to save")

        
        self.recorded_trajectory = []
        self.bottleneck_rgb = None
        self.bottleneck_depth = None
        self.is_recording = False

    def discard_recording(self):
        self.is_recording = False
        self.recorded_trajectory = []
        self.bottleneck_rgb = None
        self.bottleneck_depth = None

        self.get_logger().info("discarded recording")

    def on_press(self, key):
        with self.tlock:
            try:
                char = key.char.lower()
                self.pressed.add(char)

                # Action keys (Trigger once per press)
                if char == 'c' and not self.is_recording:
                    self.start_recording()
                elif char == 'f' and self.is_recording:
                    self.save_recording()
                elif char == 'r':
                    self.discard_recording()

            except AttributeError:
                if key == keyboard.Key.shift:
                    self.pressed.add('shift')
                elif key == keyboard.Key.ctrl or key == keyboard.Key.ctrl_l:
                    self.pressed.add('ctrl')

    def on_release(self, key):
        with self.tlock:
            try:
                char = key.char.lower()
                if char in self.pressed:
                    self.pressed.remove(char)
            except AttributeError:
                if key == keyboard.Key.shift and 'shift' in self.pressed:
                    self.pressed.remove('shift')
                elif (key == keyboard.Key.ctrl or key == keyboard.Key.ctrl_l) and 'ctrl' in self.pressed:
                    self.pressed.remove('ctrl')

    def process_keys(self):
        # horizontal
        twist = TwistStamped()
        twist.header.frame_id = self.base_frame
        twist.header.stamp = self.get_clock().now().to_msg()

        # wasd and cntrl shift for cartesian position
        if 'w' in self.pressed: twist.twist.linear.x = self.linear_speed
        if 's' in self.pressed: twist.twist.linear.x = -self.linear_speed
        if 'a' in self.pressed: twist.twist.linear.y = self.linear_speed
        if 'd' in self.pressed: twist.twist.linear.y = -self.linear_speed
        if 'shift' in self.pressed: twist.twist.linear.z = self.linear_speed
        if 'ctrl' in self.pressed:  twist.twist.linear.z = -self.linear_speed

        # j and k for pitch, h and l for yaw
        if 'h' in self.pressed: twist.twist.angular.y = self.angular_speed
        if 'j' in self.pressed: twist.twist.angular.y = -self.angular_speed
        if 'g' in self.pressed: twist.twist.angular.z = self.angular_speed
        if 'l' in self.pressed: twist.twist.angular.z = -self.angular_speed

        # gripper open with i and close with u
        if 'i' in self.pressed: self.gripper_position = 0.04
        if 'u' in self.pressed: self.gripper_position = 0

        return twist

    def control_loop(self):
        with self.tlock:
            twist = self.process_keys()
            self.servo_pub.publish(twist)

            if self.is_recording:
                try:
                    t = self.tf_buffer.lookup_transform(
                            self.bottleneck_frame,
                            self.ee_frame,
                            rclpy.time.Time()
                            )

                    pose = [
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z,
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w,
                        self.gripper_position
                    ]
                    self.recorded_trajectory.append(pose)
                except Exception as e:
                    self.get_logger().error(f"could not find relative transform between ee and bottleneck: {e}")


    def color_callback(self, msg):
        self.latest_color_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.latest_depth_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

def main(args=None):
    rclpy.init(args=args)

    recorder = DinoRecordingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(recorder)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


