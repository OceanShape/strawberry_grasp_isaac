import sys
import threading
import numpy as np
import builtins
import os

# [중요] 시스템 ROS 2 패키지와 Isaac Sim 내장 ROS 2 패키지 충돌 방지
sys.path = [p for p in sys.path if '/opt/ros' not in p]

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import omni.kit.app
import omni.physx
import omni.timeline
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

# ROS 2 익스텐션 강제 활성화 (최신 API 방식)
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

class ScriptBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge_node')
        self.joint_states_pub = self.create_publisher(JointState, '/dsr01/joint_states', 10)
        self.joint_cmd_sub = self.create_subscription(JointState, '/joint_command', self.joint_command_cb, 10)
        
        self.target_action = None
        self.dof_names_cache = None

    def joint_command_cb(self, msg):
        self.target_action = np.array(msg.position)

def start_bridge():
    # 여러 번 Run 버튼을 눌러도 꼬이지 않도록 초기화
    if not rclpy.ok():
        rclpy.init()
    if hasattr(builtins, "my_ros_node") and builtins.my_ros_node is not None:
        builtins.my_ros_node.destroy_node()
    if hasattr(builtins, "my_physx_sub"):
        builtins.my_physx_sub = None
        
    robot = Articulation(prim_path="/World/robot_recent/strawberry_grasp_robot", name="doosan_robot")
    node = ScriptBridgeNode()
    builtins.my_ros_node = node
    
    # 아이작 심 GUI의 물리 엔진 재생(Play) 루프에 기생(Hooking)
    def on_physics_step(step_size):
        # Play 버튼(스페이스바)이 눌려있을 때만 동작
        if not omni.timeline.get_timeline_interface().is_playing():
            return
            
        # 초기화 안 된 상태면 초기화 진행
        is_initialized = getattr(robot, '_is_initialized', False)
        if not is_initialized and getattr(robot, 'num_dof', None) is None:
            robot.initialize()
            return
            
        # 액션 적용 (패딩 로직 포함)
        if node.target_action is not None:
            cmd = node.target_action
            if len(cmd) < robot.num_dof:
                current_pos = robot.get_joint_positions()
                full_cmd = current_pos.copy()
                full_cmd[:len(cmd)] = cmd
                cmd = full_cmd
            robot.apply_action(ArticulationAction(joint_positions=np.array(cmd)))
            node.target_action = None

        # 상태 발행 (캐싱 로직 포함)
        if node.dof_names_cache is None:
            node.dof_names_cache = [robot.dof_names[i] for i in range(robot.num_dof)]

        js_msg = JointState()
        js_msg.header.stamp = node.get_clock().now().to_msg()
        js_msg.name = node.dof_names_cache
        js_msg.position = robot.get_joint_positions().tolist()
        node.joint_states_pub.publish(js_msg)

    # 이벤트 구독 등록
    builtins.my_physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
    
    # 백그라운드 스핀
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print("Script Editor ROS 2 Bridge Started!")

start_bridge()
