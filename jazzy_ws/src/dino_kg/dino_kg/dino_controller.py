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
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
import numpy as np
import os
from scipy.spatial.transform import Rotation

# dino stuff
import torch
import numpy as np 
import matplotlib.pyplot as plt 
from PIL import Image as PILImage
import time
from dino_kg.correspondences import find_correspondences, draw_correspondences

import matplotlib
matplotlib.use('Agg')

# TODO: eventually want to get these from the ros topic but this is faster for now
COLOR_K = [914.834228515625, 0.0, 653.8088989257812, 0.0, 914.9916381835938, 345.4898376464844, 0.0, 0.0, 1.0]
DEPTH_K = [422.7320861816406, 0.0, 424.1393737792969, 0.0, 422.7320861816406, 239.05503845214844, 0.0, 0.0, 1.0]

# servoing similar to what is done here: https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/demos/servo_keyboard_input.cpp
# tf transform stuff based on https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html
# dino speefic code is gotten from https://gist.github.com/normandipalo/fbc21f23606fbe3d407e22c363cb134e
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

        # traj replay stuff
        skill_path = f"/workspace/data/{self.skill}"
        self.trajectories = np.load(os.path.join(skill_path, "trajectory.npy"))
        self.gripper_position = 0.04
        self.idx = 0

        # alignment stuff
        self.num_pairs = 8 
        self.load_size = 224 
        self.layer = 9 
        self.facet = 'key' 
        self.bin=True 
        self.thresh=0.05 
        self.model_type='dino_vits8' 
        self.stride=4
        self.bottleneck_rgb = PILImage.open(os.path.join(skill_path, "bottleneck_rgb.png")).convert('RGB')
        self.bottleneck_depth = cv2.imread(os.path.join(skill_path, "bottleneck_depth.tiff"), cv2.IMREAD_UNCHANGED)
        self.moving = False
        self.last_alignment_time = time.time() # to account for any jerkiness at startup

        # state management
        self.state = "alignment" # alignment or replay

        # used to find ee pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.base_frame = 'panda_link0'
        self.ee_frame = 'panda_link8'
        self.bottleneck_frame = 'bottleneck'
        self.goal_frame = 'correspondence_goal'
        self.cam_frame = 'sim_camera'

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
                '/realsense/aligned_depth_to_color/image_raw',
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
        # collect dino correspondeces asynchronously
        threading.Thread(target=self.correspondences_loop, daemon=True).start()

        self.get_logger().info("STARTED dino controller...")

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            return

        if np.any(msg.points[0].velocities):
            self.moving = True
        else:
            self.moving = False

        # converting trajectory to joint state
        js = JointState()
        js.header = msg.header
        js.name = list(msg.joint_names) + ['panda_finger_joint1', 'panda_finger_joint2']

        js.position = list(msg.points[0].positions)  + [self.gripper_position, self.gripper_position]
        js.velocity = list(msg.points[0].velocities) + [0, 0]

        self.isaac_pub.publish(js)

    def correspondences_loop(self):
        while rclpy.ok():
            if self.state != "alignment" or self.latest_color_img is None or self.latest_depth_img is None or self.moving or time.time() - self.last_alignment_time < 1.0:
                time.sleep(0.1)
                continue

                
            with torch.no_grad():
                points1, points2, image1_pil, image2_pil = find_correspondences(
                    self.latest_color_img, self.bottleneck_rgb, self.num_pairs, 
                    self.load_size, self.layer, self.facet, self.bin, 
                    self.thresh, self.model_type, self.stride
                )


            # # debug stuff
            # fig1, fig2 = draw_correspondences(points1, points2, image1_pil, image2_pil)
            # fig1.savefig("latest.png", bbox_inches='tight', pad_inches=0)
            # fig2.savefig("bottleneck.png", bbox_inches='tight', pad_inches=0)
            # plt.close('all')

            scale_u = self.latest_color_img.size[0] / image1_pil.size[0]
            scale_v = self.latest_color_img.size[1] / image1_pil.size[1]
            # finding necessary transform from our latest in order to get to the bottleneck
            points1 = self.project_to_3d(points1, self.latest_depth_img, scale_u, scale_v, 'lat.png')
            points2 = self.project_to_3d(points2, self.bottleneck_depth, scale_u, scale_v, "bot.png")
            self.get_logger().info(str(points1))
            self.get_logger().info(str(points2))

            goal_transform = self.find_transformation(points1, points2)

            # we need to publish relative to base of robot so goal does not move when ee moves
            # t = self.tf_buffer.lookup_transform(
            #         self.base_frame,
            #         self.ee_frame,
            #         rclpy.time.Time()
            #         )
            # cam_trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            # cam_quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            #
            # cam_transform = np.eye(4)
            # cam_transform[:3, :3] = Rotation.from_quat(cam_quat).as_matrix()
            # cam_transform[:3, 3] = cam_trans
            #
            # goal_relative_to_base = cam_transform @ goal_transform

            error = np.linalg.norm(np.array(points1) - np.array(points2))

            t_cam = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.cam_frame,
                rclpy.time.Time()
            )
            cam_base_trans = [t_cam.transform.translation.x, t_cam.transform.translation.y, t_cam.transform.translation.z]
            cam_base_quat = [t_cam.transform.rotation.x, t_cam.transform.rotation.y, t_cam.transform.rotation.z, t_cam.transform.rotation.w]

            cam_to_base_matrix = np.eye(4)
            cam_to_base_matrix[:3, :3] = Rotation.from_quat(cam_base_quat).as_matrix()
            cam_to_base_matrix[:3, 3] = cam_base_trans

            t_cam_to_ee = self.tf_buffer.lookup_transform(
                self.ee_frame,
                self.cam_frame,
                rclpy.time.Time()
            )
            cam_ee_trans = [t_cam_to_ee.transform.translation.x, t_cam_to_ee.transform.translation.y, t_cam_to_ee.transform.translation.z]
            cam_ee_quat = [t_cam_to_ee.transform.rotation.x, t_cam_to_ee.transform.rotation.y, t_cam_to_ee.transform.rotation.z, t_cam_to_ee.transform.rotation.w]

            cam_to_ee_matrix = np.eye(4)
            cam_to_ee_matrix[:3, :3] = Rotation.from_quat(cam_ee_quat).as_matrix()
            cam_to_ee_matrix[:3, 3] = cam_ee_trans


            # find goal transform in cam frame
            goal_cam_in_base = cam_to_base_matrix @ np.linalg.inv(goal_transform)

            # find transform in ee frame
            goal_ee_in_base = goal_cam_in_base @ np.linalg.inv(cam_to_ee_matrix)
            
            with self.tlock:
                t = t_cam
                quat = Rotation.from_matrix(goal_ee_in_base[:3, :3]).as_quat()
                trans = goal_ee_in_base[:3, 3]

                # ee -> goal
                t.child_frame_id = self.goal_frame
                t.transform.translation.x = trans[0]
                t.transform.translation.y = trans[1]
                t.transform.translation.z = trans[2]
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                self.tf_broadcaster.sendTransform(t)
                self.moving = True
                self.last_alignment_time = time.time()

                self.get_logger().info(f"Error: {error}")
                if error <= 0.1:
                    self.get_logger().info("Finished alignment stage")
                    self.state = "replay"

    # NOTE: see camera_ros.py for realsense specs
    def project_to_3d(self, points, depth_img, scale_u, scale_v, name):
        fx_rgb, cx_rgb, fy_rgb, cy_rgb = COLOR_K[0], COLOR_K[2], COLOR_K[4], COLOR_K[5]
        
        # fx_depth, cx_depth, fy_depth, cy_depth = DEPTH_K[0], DEPTH_K[2], DEPTH_K[4], DEPTH_K[5]
        
        depth_scale = 1.5
        
        points = np.asarray(points, dtype=float)
        
        u_rgb = points[:, 1] * scale_u
        v_rgb = points[:, 0] * scale_v
        
        # u_depth = ((u_rgb - cx_rgb) / fx_rgb) * fx_depth + cx_depth
        # v_depth = ((v_rgb - cy_rgb) / fy_rgb) * fy_depth + cy_depth
        
        # u_idx = np.round(u_depth).astype(int)
        # v_idx = np.round(v_depth).astype(int)
        u_idx = np.round(u_rgb).astype(int)
        v_idx = np.round(v_rgb).astype(int)
        self.get_logger().info(str(depth_img.shape))
        
        # u_idx = np.clip(u_idx, 0, depth_img.shape[0] - 1)
        # v_idx = np.clip(v_idx, 0, depth_img.shape[1] - 1)

        depth_vis = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        depth_vis_color = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

        for ix, iy in zip(u_idx, v_idx):
            cv2.circle(depth_vis_color, (ix, iy), radius=5, color=(0, 0, 255), thickness=-1)

        # Save the image
        cv2.imwrite(name, depth_vis_color)
        
        z = depth_img[v_idx, u_idx] / depth_scale
        
        x = (u_rgb - cx_rgb) * z / fx_rgb
        y = (v_rgb - cy_rgb) * z / fy_rgb
        
        return np.column_stack((x, y, z))

    def find_transformation(self, X, Y):
        """
        Inputs: X, Y: lists of 3D points
        Outputs: R - 3x3 rotation matrix, t - 3-dim translation array.
        Find transformation given two sets of correspondences between 3D points.
        """
        # Calculate centroids
        cX = np.mean(X, axis=0)
        cY = np.mean(Y, axis=0)
        # Subtract centroids to obtain centered sets of points
        Xc = X - cX
        Yc = Y - cY
        # Calculate covariance matrix
        C = np.dot(Xc.T, Yc)
        # Compute SVD
        U, S, Vt = np.linalg.svd(C)
        # Determine rotation matrix (need to do a flip on z cus svf some
        R = np.dot(Vt.T, U.T)
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)

        # Determine translation vector
        t = cY - np.dot(R, cX)

        # create the transformation matrix we can use to find goal
        transform = np.eye(4)
        transform[:3, :3] = R
        transform[:3, 3] = t
        return transform

    def alignment(self):
        try:
            t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.goal_frame,
                    rclpy.time.Time()
                    )
        except Exception as e:
            # self.get_logger().debug(f"probably just dont have corr yet {e}")
            return

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = t.transform.translation.x
        target_pose.pose.position.y = t.transform.translation.y
        target_pose.pose.position.z = t.transform.translation.z
        target_pose.pose.orientation.x = t.transform.rotation.x
        target_pose.pose.orientation.y = t.transform.rotation.y
        target_pose.pose.orientation.z = t.transform.rotation.z
        target_pose.pose.orientation.w = t.transform.rotation.w

        self.servo_pub.publish(target_pose)

    def replay(self):
        if self.idx < len(self.trajectories):
            if self.idx == 0:
                t = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        self.ee_frame,
                        rclpy.time.Time()
                        )

                t.header.stamp = self.get_clock().now().to_msg()
                t.child_frame_id = self.bottleneck_frame

                # initialize bottleneck frame. we use this to find relative pose transform in subsequent frames
                self.tf_broadcaster.sendTransform(t)


            pose_data = self.trajectories[self.idx]
            tx, ty, tz, rx, ry, rz, rw, self.gripper_position = pose_data

            target_pose = PoseStamped()
            target_pose.header.frame_id = self.bottleneck_frame
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
        else:
            self.get_logger().info("Finished replay stage")

    def control_loop(self):
        with self.tlock:
            # NOTE for sergio: panda moves uses link8 as its ee not panda_hand
            if self.state == "alignment":
                self.alignment()
            elif self.state == "replay":
                self.replay()
            else:
                self.get_logger().error(f"unknown state {self.state}")


    def color_callback(self, msg):
        rgb_array = cv2.cvtColor(self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), cv2.COLOR_BGR2RGB)

        self.latest_color_img = PILImage.fromarray(rgb_array)

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


