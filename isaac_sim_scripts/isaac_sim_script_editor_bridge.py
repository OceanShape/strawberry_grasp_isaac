import sys
import threading
import numpy as np
import builtins
import os
import time

# [중요] 시스템 ROS 2 패키지와 Isaac Sim 내장 ROS 2 패키지 충돌 방지
sys.path = [p for p in sys.path if '/opt/ros' not in p]

import omni.kit.app
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose
from pxr import UsdGeom

import omni.physx
import omni.timeline
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

class ScriptBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge_node')
        self.joint_states_pub = self.create_publisher(JointState, '/dsr01/joint_states', 10)
        self.joint_cmd_sub = self.create_subscription(JointState, '/joint_command', self.joint_command_cb, 10)
        self.strawberry_pub = self.create_publisher(PoseArray, '/isaac_sim/strawberries', 10)
        
        self.target_action = None
        self.dof_names_cache = None
        self.stiffness_set = False
        self.last_pub_time = 0.0

    def joint_command_cb(self, msg):
        self.target_action = np.array(msg.position)

def start_bridge():
    if not rclpy.ok():
        rclpy.init()
        
    # [중요] 노드와 스레드가 여러 번 생성되는 것 방지 (중복 Run 클릭 대비)
    if not hasattr(builtins, "my_ros_node") or builtins.my_ros_node is None:
        node = ScriptBridgeNode()
        builtins.my_ros_node = node
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    else:
        node = builtins.my_ros_node
        
    if hasattr(builtins, "my_physx_sub"):
        builtins.my_physx_sub = None
        
    robot = Articulation(prim_path="/World/robot_recent/strawberry_grasp_robot", name="doosan_robot")
    builtins.my_robot = robot
    
    def on_physics_step(step_size):
        if not omni.timeline.get_timeline_interface().is_playing():
            return
            
        # 초기화가 안 되어있으면 초기화 시도 (아이작 심 내부 뷰 버그 에러는 무시)
        if getattr(robot, 'num_dof', None) is None:
            try:
                robot.initialize()
            except Exception:
                pass # RigidPrimView AttributeError 무시
            return
            

        # 액션 적용
        if node.target_action is not None:
            cmd = node.target_action
            if len(cmd) < robot.num_dof:
                current_pos = robot.get_joint_positions()
                full_cmd = current_pos.copy()
                full_cmd[:len(cmd)] = cmd
                cmd = full_cmd
            robot.apply_action(ArticulationAction(joint_positions=np.array(cmd)))
            node.target_action = None

        # 상태 발행
        if node.dof_names_cache is None:
            node.dof_names_cache = [robot.dof_names[i] for i in range(robot.num_dof)]

        positions = robot.get_joint_positions()
        if positions is None:
            return # 물리 뷰가 아직 초기화 안 되었거나 종료 중일 때

        js_msg = JointState()
        js_msg.header.stamp = node.get_clock().now().to_msg()
        js_msg.name = node.dof_names_cache
        js_msg.position = positions.tolist()
        node.joint_states_pub.publish(js_msg)

        # 1초마다 딸기 위치 전송
        current_time = time.time()
        if current_time - node.last_pub_time > 1.0:
            node.last_pub_time = current_time
            stage = omni.usd.get_context().get_stage()
            pose_array = PoseArray()
            pose_array.header.stamp = node.get_clock().now().to_msg()
            pose_array.header.frame_id = "world"
            
            for prim in stage.Traverse():
                # 딸기 오브젝트 찾기 (이름에 strawberry가 포함되고, 로봇이 아닌 것)
                prim_name = prim.GetName().lower()
                if "strawberry" in prim_name and "robot" not in prim_name:
                    if prim.IsA(UsdGeom.Xformable):
                        xform = UsdGeom.Xformable(prim)
                        # 월드 트랜스폼 계산
                        time_code = omni.timeline.get_timeline_interface().get_current_time()
                        world_transform = xform.ComputeLocalToWorldTransform(time_code)
                        translation = world_transform.ExtractTranslation()
                        
                        pose = Pose()
                        pose.position.x = float(translation[0])
                        pose.position.y = float(translation[1])
                        pose.position.z = float(translation[2])
                        pose.orientation.w = 1.0
                        pose_array.poses.append(pose)
            
            if len(pose_array.poses) > 0:
                node.strawberry_pub.publish(pose_array)

    builtins.my_physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
    print("Script Editor ROS 2 Bridge Started Successfully!")

start_bridge()
