#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from dsr_msgs2.srv import MoveSplineJoint
except ImportError:
    MoveSplineJoint = None
    print("[Warning] dsr_msgs2 패키지를 찾을 수 없어 서비스 호출이 불가능할 수 있습니다.")

import sys
sys.path.append("/home/sun/.local/lib/python3.11/site-packages")
import ikpy.chain

class TestMoveNode(Node):
    def __init__(self):
        super().__init__('test_move_node')
        
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_cb, 10)
        
        self.current_joints = None
        
        # URDF 로드 및 IK 체인 설정 (base_link부터 link_6까지만 활성화)
        urdf_path = "/home/sun/strawberry_grasp_environment/robot.urdf"
        try:
            # ikpy는 첫 번째 링크(base)를 False로, 이후 joint 1~6을 True로 설정해야 합니다.
            # 로봇의 자유도(6)에 맞게 마스크 설정
            self.chain = ikpy.chain.Chain.from_urdf_file(
                urdf_path,
                active_links_mask=[False, True, True, True, True, True, True, False, False, False, False]
            )
            self.get_logger().info(f"URDF 로드 성공. 링크 수: {len(self.chain.links)}")
        except Exception as e:
            self.get_logger().error(f"URDF 로드 실패: {e}")
            self.chain = None
            
        # 순차적 이동 시퀀스 쓰레드 시작
        self.test_thread = threading.Thread(target=self.run_sequence)
        self.test_thread.start()

    def joint_cb(self, msg):
        # Doosan 관절 상태 (6축) 저장
        if self.current_joints is None:
            # joint_1 ~ joint_6의 순서가 보장된다고 가정 (일반적으로 처음 6개)
            self.current_joints = list(msg.position[:6])

    def wait_for_joints(self):
        self.get_logger().info("관절 상태를 기다리는 중 (/dsr01/joint_states)...")
        while self.current_joints is None and rclpy.ok():
            time.sleep(0.1)
        self.get_logger().info("관절 상태 수신 완료!")

    def send_spline_joint_target(self, target_joints):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 브릿지에서 position 배열만 읽어 할당하므로 name은 생략 가능하나 관례상 작성
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = target_joints
        
        self.pub.publish(msg)
        self.get_logger().info("이동 명령(/joint_command) 전송 완료!")

    def run_sequence(self):
        self.wait_for_joints()
        if self.chain is None:
            return

        # 현재 조인트 각도를 IK 체인 배열 길이에 맞게 패딩 (기본적으로 첫번째는 base 고정이므로 0 추가)
        ik_current_joints = [0.0] + self.current_joints + [0.0] * (len(self.chain.links) - 7)
        
        # 현재 위치 (Forward Kinematics)
        current_pose = self.chain.forward_kinematics(ik_current_joints)
        initial_translation = current_pose[:3, 3].copy()
        
        self.get_logger().info(f"초기 TCP 위치: {initial_translation}")

        # 이동 시퀀스 (단위: m) -> 10cm = 0.1m
        # +x, -x, +y, -y, +z, -z
        moves = [
            (" +X (10cm)", np.array([0.1, 0.0, 0.0])),
            (" -X (10cm)", np.array([-0.1, 0.0, 0.0])),
            (" +Y (10cm)", np.array([0.0, 0.1, 0.0])),
            (" -Y (10cm)", np.array([0.0, -0.1, 0.0])),
            (" +Z (10cm)", np.array([0.0, 0.0, 0.1])),
            (" -Z (10cm)", np.array([0.0, 0.0, -0.1])),
        ]

        for name, offset in moves:
            self.get_logger().info(f"====={name} 이동 시작 =====")
            target_pose = current_pose.copy()
            target_pose[:3, 3] = current_pose[:3, 3] + offset

            # 역기구학 연산 (orientation 유지)
            target_ik_joints = self.chain.inverse_kinematics(
                target_position=target_pose[:3, 3],
                target_orientation=target_pose[:3, :3],
                orientation_mode="all",
                initial_position=ik_current_joints
            )
            
            # 계산된 IK 결과에서 6축 데이터 추출 (index 1~6)
            cmd_joints = list(target_ik_joints[1:7])
            
            # 시뮬레이션 브릿지로 전송
            self.send_spline_joint_target(cmd_joints)
            
            # 이동할 시간 대기 및 원위치 복귀
            time.sleep(3.0)
            
            self.get_logger().info("원위치(초기)로 복귀")
            self.send_spline_joint_target(self.current_joints)
            time.sleep(3.0)
            
            # 갱신 (선택적)
            ik_current_joints = [0.0] + self.current_joints + [0.0] * (len(self.chain.links) - 7)

        self.get_logger().info("모든 테스트 이동이 완료되었습니다!")

def main():
    rclpy.init()
    node = TestMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
