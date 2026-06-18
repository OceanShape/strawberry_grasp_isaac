#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Isaac Sim Virtual Sensor & Controller Bridge Node
이 스크립트는 기존 curobo_planner_node와 scan_executor_node를 수정하지 않고,
Isaac Sim 환경과 ROS 2 간의 양방향 통신을 구현합니다.

주의: 이 스크립트는 Isaac Sim 환경 내장 Python (또는 ./python.sh)으로 실행해야 합니다.
"""

import sys
import threading
import time

# Isaac Sim Standalone App 설정
try:
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": False})
except ImportError:
    try:
        from omni.isaac.kit import SimulationApp
        simulation_app = SimulationApp({"headless": False})
    except ImportError:
        print("[Error] Isaac Sim Python 환경에서 실행해주세요 (예: ./python.sh isaac_sim_bridge_node.py)")
        sys.exit(1)

# [중요] 시스템 ROS 2(Humble, Python 3.10)와 Isaac Sim(Python 3.11) 충돌 방지
import os
sys.path = [p for p in sys.path if '/opt/ros' not in p]

# Isaac Sim 내장 ROS 2 브릿지 익스텐션 활성화
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")
simulation_app.update()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# Doosan 로봇용 커스텀 서비스 (에러 방지를 위해 try-except 블록 사용)
try:
    from dsr_msgs2.srv import MoveSplineJoint, MoveLine
    HAS_DSR_MSGS = True
except ImportError:
    HAS_DSR_MSGS = False
    print("[Warning] dsr_msgs2 패키지를 찾을 수 없습니다. 관련 서비스는 사용할 수 없거나 Mocking 됩니다.")

import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.world import World

# ==========================================
# [사용자 설정 영역]
# Isaac Sim 환경 내 USD Prim Path를 이곳에 맞게 수정하세요.
# ==========================================
ROBOT_PRIM_PATH = "/World/robot"           # 로봇 팔(Articulation Root)의 Prim Path
TARGET_PRIM_PATH = "/World/strawberry"     # 목표물(딸기)의 Prim Path
# ==========================================

class IsaacSimBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge_node')

        # 1. 가상 센서 퍼블리셔
        self.pick_pose_pub = self.create_publisher(PoseStamped, '/dsr01/curobo/pick_pose', 10)
        self.joint_states_pub = self.create_publisher(JointState, '/dsr01/joint_states', 10)

        # 2. 가상 제어기 서비스 서버 (커스텀 메시지 C-Extension 에러 방지)
        if HAS_DSR_MSGS:
            try:
                self.srv_move_spline = self.create_service(MoveSplineJoint, '/dsr01/motion/move_spline_joint', self.move_spline_joint_cb)
                self.srv_move_line = self.create_service(MoveLine, '/dsr01/motion/move_line', self.move_line_cb)
            except Exception as e:
                self.get_logger().error(f"dsr_msgs2 C-Extension 로드 실패. 커스텀 서비스를 비활성화합니다: {e}")
        else:
            self.get_logger().warn("dsr_msgs2 서비스가 없어 MoveSplineJoint 및 MoveLine 서버를 생략합니다.")
        
        # [테스트용] 표준 ROS2 토픽으로 로봇을 제어하기 위한 서브스크라이버 추가
        self.joint_cmd_sub = self.create_subscription(JointState, '/joint_command', self.joint_command_cb, 10)

        # 가상 그리퍼 구동용 (Trigger 서비스로 가정)
        self.srv_gripper = self.create_service(Trigger, '/gripper_service/set_position', self.gripper_cb)

        # Isaac Sim 객체 초기화 대기
        self.robot = None
        self.target = None
        
        # 30Hz 주기로 상태 발행
        self.create_timer(1.0 / 30.0, self.publish_states)
        self.get_logger().info("Isaac Sim Virtual Bridge Node Started.")

    def init_isaac_sim_objects(self):
        """Isaac Sim 물리 세계의 로봇과 타겟 객체를 연동합니다."""
        if prim_utils.is_valid_prim(ROBOT_PRIM_PATH) and prim_utils.is_valid_prim(TARGET_PRIM_PATH):
            if self.robot is None:
                self.robot = Articulation(prim_path=ROBOT_PRIM_PATH, name="doosan_robot")
                self.robot.initialize()
                self.get_logger().info(f"로봇 에셋 연결 완료: {ROBOT_PRIM_PATH}")
            if self.target is None:
                self.target = XFormPrim(prim_path=TARGET_PRIM_PATH, name="strawberry_target")
                self.target.initialize()
                self.get_logger().info(f"타겟 에셋 연결 완료: {TARGET_PRIM_PATH}")
        else:
            # 아직 로드되지 않은 경우를 위한 안내 (최초 1회만 출력하도록 제어 가능)
            pass

    def publish_states(self):
        # 런타임에 에셋을 초기화 시도
        if self.robot is None or self.target is None:
            self.init_isaac_sim_objects()
            return
            
        try:
            # --- 1) Pick Pose 발행 (타겟 객체의 위치) ---
            target_pos, target_rot = self.target.get_world_pose()
            robot_pos, robot_rot = self.robot.get_world_pose()
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"
            
            # 절대 좌표계(World)를 로봇 base_link 기준으로 상대적 변환 (간이 변환 로직)
            pose_msg.pose.position.x = float(target_pos[0] - robot_pos[0])
            pose_msg.pose.position.y = float(target_pos[1] - robot_pos[1])
            pose_msg.pose.position.z = float(target_pos[2] - robot_pos[2])
            
            # 회전값 할당
            pose_msg.pose.orientation.w = float(target_rot[0])
            pose_msg.pose.orientation.x = float(target_rot[1])
            pose_msg.pose.orientation.y = float(target_rot[2])
            pose_msg.pose.orientation.z = float(target_rot[3])
            
            self.pick_pose_pub.publish(pose_msg)
            
            # --- 2) Joint States 발행 ---
            js_msg = JointState()
            js_msg.header.stamp = pose_msg.header.stamp
            
            # dof_names와 관절 위치 배열 획득
            js_msg.name = [self.robot.dof_names[i] for i in range(self.robot.num_dof)]
            js_msg.position = self.robot.get_joint_positions().tolist()
            
            self.joint_states_pub.publish(js_msg)
        except Exception as e:
            # Isaac Sim 환경에서 에셋이 삭제되거나 에러 발생 시 예외처리
            self.get_logger().error(f"상태 발행 중 에러 발생: {e}")

    def move_spline_joint_cb(self, request, response):
        """플래너로부터 받은 궤적(Trajectory) 데이터를 가상 로봇에 적용"""
        if self.robot is not None:
            # request.pos 가 궤적을 담고 있다고 가정, 최후의 목표 관절 각도로 조작 (Mocking)
            # 향후 Dynamic Control을 통해 보간 궤적으로 부드럽게 움직일 수 있도록 확장 가능합니다.
            if len(request.pos) > 0:
                target_joints = list(request.pos[-1]) 
                self.robot.set_joint_position_targets(target_joints)
                self.get_logger().info("가상 제어기: move_spline_joint 궤적 수신 및 적용 완료")
            response.success = True
        else:
            response.success = False
            self.get_logger().warn("가상 로봇이 아직 초기화되지 않았습니다.")
        return response

    def move_line_cb(self, request, response):
        """직선 이동 명령 수신 (Cartesian Space)"""
        self.get_logger().info("가상 제어기: move_line 수신 (Mocking 처리됨)")
        # 아이작 심의 역기구학 모듈을 통해 조인트 목표값을 계산하거나,
        # 단순히 모션 플래너가 IK를 계산해주리라 가정하고 응답만 반환
        response.success = True
        return response

    def joint_command_cb(self, msg):
        """표준 JointState 토픽을 통한 관절 직접 제어 (테스트 및 범용)"""
        if self.robot is not None:
            self.robot.set_joint_position_targets(msg.position)
            self.get_logger().info("가상 제어기: /joint_command 수신 및 적용 완료")

    def gripper_cb(self, request, response):
        """그리퍼 제어 명령 수신"""
        self.get_logger().info("가상 그리퍼: set_position 수신 및 구동")
        # 여기서 Isaac Sim의 그리퍼 조인트를 닫는 코드를 추가합니다.
        # 예: self.robot.set_joint_position_targets([0.0, 0.0], indices=[gripper_idx1, gripper_idx2])
        response.success = True
        response.message = "Virtual gripper action completed"
        return response

def main():
    rclpy.init()
    node = IsaacSimBridgeNode()
    
    # ROS 2 노드를 백그라운드 스레드에서 실행
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # USD 자동 로드
    from omni.isaac.core.utils.stage import open_stage
    open_stage("/home/sun/strawberry_grasp_isaac/robot/strawberry_grasp_robot.usd")
    
    world = World()
    world.reset()
    
    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print("시뮬레이션을 종료합니다.")
    
    rclpy.shutdown()
    simulation_app.close()

if __name__ == '__main__':
    main()
