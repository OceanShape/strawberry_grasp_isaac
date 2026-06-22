import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Float64MultiArray
import time

class FakeVisionNode(Node):
    def __init__(self):
        super().__init__('fake_vision_node')
        
        # 실제 비전 노드(strawberry_fusion_node.py)가 발행하는 토픽과 동일한 퍼블리셔 생성
        self.pick_pub = self.create_publisher(
            PoseStamped, "/strawberry/detection/pick_pose", 20)
        self.scene_pub = self.create_publisher(
            Float64MultiArray, "/strawberry/detection/scene_positions", 10)
            
        # Isaac Sim으로부터 딸기 좌표 수신
        self.strawberry_sub = self.create_subscription(
            PoseArray, '/isaac_sim/strawberries', self.strawberry_cb, 10)
            
        self.get_logger().info("Fake Vision Node initialized. (Mocking strawberry_fusion_node)")
        self.last_pub_time = 0.0

    def strawberry_cb(self, msg: PoseArray):
        current_time = time.time()
        
        # 너무 빈번한 발행을 막기 위해 0.5초 간격으로 발행
        if current_time - self.last_pub_time < 0.5:
            return
        self.last_pub_time = current_time

        if not msg.poses:
            return

        # 1. scene_positions 발행 (모든 딸기의 좌표 배열)
        scene_msg = Float64MultiArray()
        for pose in msg.poses:
            scene_msg.data.extend([pose.position.x, pose.position.y, pose.position.z])
        self.scene_pub.publish(scene_msg)
        
        # 2. pick_pose 발행 (각 딸기마다 하나씩 순차적으로 발행)
        # scan_executor는 Dwell 시간 동안 이 토픽들을 수집하여 중복 제거 후 리스트를 만듭니다.
        for pose in msg.poses:
            pick_msg = PoseStamped()
            pick_msg.header.stamp = self.get_clock().now().to_msg()
            pick_msg.header.frame_id = "base_link" # 로봇 베이스 기준이라고 가정
            
            # 카메라가 아닌 월드/베이스 기준의 좌표를 그대로 사용
            pick_msg.pose = pose
            
            # 딸기 중심보다 약간 위/앞을 파지 목표로 설정할 경우 여기에 오프셋을 줄 수 있습니다.
            # 하지만 현재는 시뮬레이션의 정확한 좌표를 그대로 넘겨줍니다.
            self.pick_pub.publish(pick_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
