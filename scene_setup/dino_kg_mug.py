# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# THIS code is expanded from standalone_examples/api/isaacsim.ros2.bridge/moveit.py

import sys

import numpy as np
from isaacsim import SimulationApp

FRANKA_STAGE_PATH = "/Franka"
FRANKA_USD_PATH = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"

MUG_STAGE_PATH = "/Mug"
MUG_USD_PATH = "/Isaac/Props/Mugs/SM_Mug_A2.usd"
CAMERA_PATH = f"{FRANKA_STAGE_PATH}/panda_hand/RealSense"
PANDA_HAND_PATH = f"{FRANKA_STAGE_PATH}/panda_hand"
CAMERA_USD_PATH = "/Isaac/Sensors/Intel/RealSense/rsd455.usd"

CONFIG = {"renderer": "RaytracedLighting", "headless": False}

# Example ROS2 bridge sample demonstrating the manual loading of stages
# and creation of ROS components
simulation_app = SimulationApp(CONFIG)
import carb
import omni.graph.core as og
import omni.kit.commands
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions, prims, rotations, stage
from omni.physx.scripts import utils
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import Articulation
import omni.kit.hotkeys.core
from pxr import Gf, UsdPhysics, UsdGeom
from omni.kit.viewport.utility import get_active_viewport

# enable ROS2 bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("omni.graph.window.action")
extensions.enable_extension("omni.graph.window.core")

# see bezier_curve_edits.py . remove hotkeys so it doesent interfere with recording
from omni.kit.hotkeys.core import get_hotkey_registry

hotkey_registry = get_hotkey_registry()
discovered_hotkeys = hotkey_registry.get_all_hotkeys()
for hotkey in discovered_hotkeys.copy():
    hotkey_registry.deregister_hotkey(hotkey)


simulation_app.update()

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Preparing stage

# Loading the simple_room environment
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

# Loading the franka robot USD
robot = prims.create_prim(
    FRANKA_STAGE_PATH,
    "Xform",
    position=np.array([0, -0.64, 0]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=assets_root_path + FRANKA_USD_PATH,
)

# load mug
mug = prims.create_prim(
    MUG_STAGE_PATH,
    "Xform",
    position=np.array([0, -0.2, 0]),
    orientation=rotations.euler_angles_to_quat([0, 0, 90], degrees=True),
    usd_path=assets_root_path + MUG_USD_PATH,
)

# https://docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_scene_based_sdg.html#creating-the-cameras-and-the-writer
panda_hand_prim = simulation_context.stage.GetPrimAtPath(PANDA_HAND_PATH)
hand_tf = omni.usd.get_world_transform_matrix(panda_hand_prim)

trans_mat = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.04, 0, 0.04))
rot_mat = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90))
local_offset_tf = rot_mat * trans_mat
rot_mat = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180))
local_offset_tf = rot_mat * local_offset_tf
cam_tf = local_offset_tf * hand_tf
cam_trans = cam_tf.ExtractTranslation()
cam_quat = cam_tf.ExtractRotationQuat()
cam_quat_xyzw = (cam_quat.GetReal(), *cam_quat.GetImaginary())

realsense_prim = prims.create_prim(
    CAMERA_PATH,
    "Xform",
    position=cam_trans, 
    orientation=cam_quat_xyzw,
    usd_path=assets_root_path + CAMERA_USD_PATH,
)

# NOTE: see camera_ros.py for realsense specs

# pixel_size is 3 microns from the datasheet
def apply_intrinsics(cam_prim_path, width, height, K, pixel_size=3.0 * 1e-3):
    cam = UsdGeom.Camera(simulation_context.stage.GetPrimAtPath(cam_prim_path))
    
    fx, cx, fy, cy = K[0], K[2], K[4], K[5]
    
    horizontal_aperture = width * pixel_size
    vertical_aperture = height * pixel_size
    
    focal_length = ((fx + fy) / 2.0) * pixel_size
    
    cam.GetFocalLengthAttr().Set(focal_length)
    cam.GetHorizontalApertureAttr().Set(horizontal_aperture)
    cam.GetVerticalApertureAttr().Set(vertical_aperture)

color_cam_path = f"{CAMERA_PATH}/RSD455/Camera_OmniVision_OV9782_Color"
depth_cam_path = f"{CAMERA_PATH}/RSD455/Camera_Pseudo_Depth"

# got this from realsense d435i in lab
COLOR_K = [914.834228515625, 0.0, 653.8088989257812, 0.0, 914.9916381835938, 345.4898376464844, 0.0, 0.0, 1.0]
DEPTH_K = [422.7320861816406, 0.0, 424.1393737792969, 0.0, 422.7320861816406, 239.05503845214844, 0.0, 0.0, 1.0]

apply_intrinsics(color_cam_path, width=1280, height=720, K=COLOR_K)
apply_intrinsics(depth_cam_path, width=848, height=480, K=DEPTH_K)

viewport = get_active_viewport()
viewport.camera_path = f"{CAMERA_PATH}/RSD455/Camera_OmniVision_OV9782_Color"

# Set variant selections for the Franka robot
robot.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
robot.GetVariantSet("Mesh").SetVariantSelection("Quality")


# enable physics
# https://docs.isaacsim.omniverse.nvidia.com/5.1.0/python_scripting/environment_setup.html#enable-physics-and-collision-for-a-mesh
utils.setRigidBody(mug, "convexDecomposition", False)
# https://docs.isaacsim.omniverse.nvidia.com/5.1.0/python_scripting/environment_setup.html#set-mass-properties-for-a-mesh
mass_api = UsdPhysics.MassAPI.Apply(mug)
# need to make super light or gripper cant lift it 
mass_api.CreateMassAttr(0.01)


simulation_app.update()

# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                # camera stuff
                ("RenderProduct_RGB", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RenderProduct_Depth", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGBHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("ColorInfoHelper", "isaacsim.ros2.bridge.ROS2PublishCameraInfo"),
                ("DepthInfoHelper", "isaacsim.ros2.bridge.ROS2PublishCameraInfo"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                # camera
                ("OnImpulseEvent.outputs:execOut", "RenderProduct_RGB.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "RenderProduct_Depth.inputs:execIn"),
                ("RenderProduct_RGB.outputs:execOut", "RGBHelper.inputs:execIn"),
                ("RenderProduct_Depth.outputs:execOut", "DepthHelper.inputs:execIn"),
                ("RenderProduct_RGB.outputs:execOut", "ColorInfoHelper.inputs:execIn"),
                ("RenderProduct_Depth.outputs:execOut", "DepthInfoHelper.inputs:execIn"),
                ("RenderProduct_RGB.outputs:renderProductPath", "RGBHelper.inputs:renderProductPath"),
                ("RenderProduct_Depth.outputs:renderProductPath", "DepthHelper.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", FRANKA_STAGE_PATH),
                ("PublishJointState.inputs:topicName", "joint_states"),
                ("SubscribeJointState.inputs:topicName", "joint_commands"),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(FRANKA_STAGE_PATH)]),
                # camera
                ("RenderProduct_RGB.inputs:cameraPrim", [usdrt.Sdf.Path(color_cam_path)]),
                ("RenderProduct_RGB.inputs:width", 1280),
                ("RenderProduct_RGB.inputs:height", 720),
                # ("RenderProduct_Depth.inputs:cameraPrim", [usdrt.Sdf.Path(f"{CAMERA_PATH}/RSD455/Camera_Pseudo_Depth")]),
                ("RenderProduct_Depth.inputs:cameraPrim", [usdrt.Sdf.Path(color_cam_path)]),
                # ("RenderProduct_Depth.inputs:width", 848),
                # ("RenderProduct_Depth.inputs:height", 480),
                ("RenderProduct_Depth.inputs:width", 1280),
                ("RenderProduct_Depth.inputs:height", 720),
                ("RGBHelper.inputs:type", "rgb"),
                ("RGBHelper.inputs:topicName", "/realsense/color/image_raw"),
                ("RGBHelper.inputs:frameId", "realsense_color_frame"),
                ("DepthHelper.inputs:type", "depth"),
                # ("DepthHelper.inputs:topicName", "/realsense/depth/image_rect_raw"),
                ("DepthHelper.inputs:topicName", "/realsense/aligned_depth_to_color/image_raw"),
                ("DepthHelper.inputs:frameId", "realsense_color_frame"),

                ("ColorInfoHelper.inputs:topicName", "/realsense/color/camera_info"),
                ("ColorInfoHelper.inputs:frameId", "camera_color_optical_frame"),
                ("ColorInfoHelper.inputs:width", 1280),
                ("ColorInfoHelper.inputs:height", 720),
                ("ColorInfoHelper.inputs:k", COLOR_K),
                # ("DepthInfoHelper.inputs:topicName", "/realsense/depth/camera_info"),
                # ("DepthInfoHelper.inputs:frameId", "camera_depth_optical_frame"),
                # ("DepthInfoHelper.inputs:width", 848),
                # ("DepthInfoHelper.inputs:height", 480),
                # ("DepthInfoHelper.inputs:k", DEPTH_K),
            ],
        },
    )
except Exception as e:
    print(e)

simulation_app.update()

# need to initialize physics getting any articulation etc.
simulation_context.initialize_physics()

# https://docs.isaacsim.omniverse.nvidia.com/5.1.0/manipulators/manipulators_lula_trajectory_generator.html#generating-a-c-space-trajectory
franka_articulation = Articulation(FRANKA_STAGE_PATH)
initial_pos = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04])
franka_articulation.set_joint_positions(initial_pos)
franka_articulation.set_joint_velocities(np.zeros_like(initial_pos))

simulation_context.play()

while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState and Publish Clock nodes each frame
    og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)

simulation_context.stop()
simulation_app.close()
