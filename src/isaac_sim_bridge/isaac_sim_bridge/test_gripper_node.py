import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import threading

class TestGripperNode(Node):
    def __init__(self):
        super().__init__('test_gripper_node')
        
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.sub = self.create_subscription(JointState, '/dsr01/joint_states', self.joint_cb, 10)
        
        self.current_joints = None
        self.received_first_state = False

        # 비동기로 시퀀스를 실행하기 위한 스레드 시작
        self.sequence_thread = threading.Thread(target=self.run_sequence)
        self.sequence_thread.start()

    def joint_cb(self, msg):
        self.received_first_state = True
        self.current_joints = list(msg.position)

    def run_sequence(self):
        self.get_logger().info("관절 상태를 기다리는 중 (/dsr01/joint_states)...")
        while not self.received_first_state:
            time.sleep(0.1)
            
        self.get_logger().info("관절 상태 수신 완료! 그리퍼 테스트를 시작합니다.")
        time.sleep(1.0)
        
        # 1. 그리퍼 열기 (Open)
        self.get_logger().info("===== 그리퍼 열기 (Open) =====")
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 팔 각도는 유지, 마지막 4개 그리퍼 각도는 0.0
        gripper_open = self.current_joints[:6] + [0.0, 0.0, 0.0, 0.0]
        msg.position = gripper_open
        self.pub.publish(msg)
        time.sleep(3.0)

        # 2. 그리퍼 닫기 (Close)
        self.get_logger().info("===== 그리퍼 닫기 (Close) =====")
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 팔 각도는 유지, 마지막 4개 그리퍼 각도는 1.1 (최대 닫힘)
        gripper_close = self.current_joints[:6] + [1.1, 1.1, 1.0, 1.0]
        msg.position = gripper_close
        self.pub.publish(msg)
        time.sleep(3.0)

        self.get_logger().info("그리퍼 테스트가 모두 완료되었습니다!")

def main():
    rclpy.init()
    node = TestGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
