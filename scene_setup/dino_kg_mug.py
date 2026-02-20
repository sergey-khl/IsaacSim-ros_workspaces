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


# NOTE: see camera_ros.py for realsense specs

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
from isaacsim.core.utils import extensions, prims, rotations, stage, viewports
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf

# enable ROS2 bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Preparing stage
viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

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
    position=np.array([0, 0, 0]),
    usd_path=assets_root_path + MUG_USD_PATH,
)

# https://docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_scene_based_sdg.html#creating-the-cameras-and-the-writer
panda_hand_prim = simulation_context.stage.GetPrimAtPath(PANDA_HAND_PATH)
hand_tf = omni.usd.get_world_transform_matrix(panda_hand_prim)

trans_mat = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.05, 0, 0))
rot_mat = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), -90))
local_offset_tf = rot_mat * trans_mat
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

# Set variant selections for the Franka robot
robot.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
robot.GetVariantSet("Mesh").SetVariantSelection("Quality")

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
                ("RenderProduct_RGB.outputs:renderProductPath", "RGBHelper.inputs:renderProductPath"),
                ("RenderProduct_Depth.outputs:renderProductPath", "DepthHelper.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Setting the /Franka target prim to Articulation Controller node
                ("ArticulationController.inputs:robotPath", FRANKA_STAGE_PATH),
                ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(FRANKA_STAGE_PATH)]),
                # camera
                ("RenderProduct_RGB.inputs:cameraPrim", [usdrt.Sdf.Path(f"{CAMERA_PATH}/RSD455/Camera_OmniVision_OV9782_Color")]),
                ("RenderProduct_Depth.inputs:cameraPrim", [usdrt.Sdf.Path(f"{CAMERA_PATH}/RSD455/Camera_Pseudo_Depth")]),
                ("RGBHelper.inputs:type", "rgb"),
                ("RGBHelper.inputs:topicName", "/realsense/color/image_raw"),
                ("RGBHelper.inputs:frameId", "realsense_color_frame"), # It's good practice to separate these frames in ROS
                ("DepthHelper.inputs:type", "depth"),
                ("DepthHelper.inputs:topicName", "/realsense/depth/image_rect_raw"),
                ("DepthHelper.inputs:frameId", "realsense_depth_frame"),
            ],
        },
    )
except Exception as e:
    print(e)

simulation_app.update()

# need to initialize physics getting any articulation etc.
simulation_context.initialize_physics()

simulation_context.play()

while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState and Publish Clock nodes each frame
    og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)

simulation_context.stop()
simulation_app.close()
