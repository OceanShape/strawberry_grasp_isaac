import os
import time
import threading
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from dsr_msgs2.srv import MoveSplineJoint, MoveJoint, MoveLine, ChangeOperationSpeed
from dsr_gripper_tcp_interfaces.srv import SetPosition, GetState
from dsr_gripper_tcp_interfaces.action import SafeGrasp

try:
    import torch
    from curobo.types.math import Pose
    from curobo.types.robot import RobotConfig
    from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
    from curobo.types.base import TensorDeviceType
    import yaml
    CUROBO_AVAILABLE = True
except ImportError:
    CUROBO_AVAILABLE = False


class SimExecutorBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_executor_bridge_node')
        self.get_logger().info("Sim Executor Bridge Node initializing...")

        self.cb_group = ReentrantCallbackGroup()
        self.current_joints = [0.0] * 6
        self.gripper_position = 600

        self.joint_sub = self.create_subscription(
            JointState, '/dsr01/joint_states', self.joint_cb, 10, callback_group=self.cb_group)
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)

        # Doosan Motion Services
        self.srv_move_spline = self.create_service(
            MoveSplineJoint, '/dsr01/motion/move_spline_joint', self.move_spline_cb, callback_group=self.cb_group)
        self.srv_move_joint = self.create_service(
            MoveJoint, '/dsr01/motion/move_joint', self.move_joint_cb, callback_group=self.cb_group)
        self.srv_move_line = self.create_service(
            MoveLine, '/dsr01/motion/move_line', self.move_line_cb, callback_group=self.cb_group)
        self.srv_change_speed = self.create_service(
            ChangeOperationSpeed, '/dsr01/motion/change_operation_speed', self.change_speed_cb, callback_group=self.cb_group)

        # Gripper Services / Action
        self.srv_set_position = self.create_service(
            SetPosition, '/gripper_service/set_position', self.set_position_cb, callback_group=self.cb_group)
        self.srv_get_state = self.create_service(
            GetState, '/gripper_service/get_state', self.get_state_cb, callback_group=self.cb_group)
        self.action_safe_grasp = ActionServer(
            self, SafeGrasp, '/gripper_service/safe_grasp', self.safe_grasp_cb, callback_group=self.cb_group)

        # IK Solver Setup
        self.ik_solver = None
        self.init_curobo_ik()

        self.get_logger().info("Sim Executor Bridge Node Ready! (Listening to Doosan & Gripper services)")

    def init_curobo_ik(self):
        if not CUROBO_AVAILABLE:
            self.get_logger().warn("curobo is not installed. MoveLine IK will be skipped.")
            return

        try:
            from ament_index_python.packages import get_package_share_directory
            config_dir = os.path.join(
                get_package_share_directory("e0509_gripper_description"),
                "config", "curobo"
            )
            robot_config_name = "e0509_gripper.yml" # fallback
            config_path = os.path.join(config_dir, robot_config_name)
            if not os.path.exists(config_path):
                self.get_logger().warn(f"Robot config not found at {config_path}")
                return

            with open(config_path, "r", encoding="utf-8") as f:
                robot_cfg_data = yaml.safe_load(f)
            
            robot_kin = robot_cfg_data["robot_cfg"]["kinematics"]
            robot_kin["urdf_path"] = os.path.join(config_dir, "e0509_gripper.urdf")
            
            tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
            robot_cfg = RobotConfig.from_dict(robot_cfg_data, tensor_args=tensor_args)
            ik_config = IKSolverConfig.build_from_robot_config(
                robot_cfg,
                tensor_args=tensor_args,
                use_cuda_graph=False
            )
            self.ik_solver = IKSolver(ik_config)
            self.get_logger().info("cuRobo IK Solver successfully initialized for MoveLine.")
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize cuRobo IK: {e}")

    def joint_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position)[:6]

    def _publish_joint_command(self, joint_positions_deg):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        msg.position = [math.radians(j) for j in joint_positions_deg]
        self.joint_pub.publish(msg)

    # --- Motion Services ---

    def move_spline_cb(self, req, res):
        self.get_logger().info(f"MoveSplineJoint called with {req.pos_cnt} points")
        # 간단한 보간 실행 (시뮬레이션 용)
        for i, point in enumerate(req.pos):
            self._publish_joint_command(point.data)
            time.sleep(0.1) # 가상의 이동 시간
        res.success = True
        return res

    def move_joint_cb(self, req, res):
        self.get_logger().info(f"MoveJoint called to {req.pos}")
        self._publish_joint_command(req.pos)
        time.sleep(req.time if req.time > 0 else 0.5)
        res.success = True
        return res

    def move_line_cb(self, req, res):
        self.get_logger().info(f"MoveLine called (pos={req.pos}, ref={req.ref}, mode={req.mode})")
        if self.ik_solver is None:
            self.get_logger().warn("IK Solver not available, faking MoveLine success.")
            res.success = True
            return res

        # 현재 FK 구하기
        start_state = torch.tensor([self.current_joints], device="cuda:0", dtype=torch.float32)
        try:
            fk_result = self.ik_solver.kinematics.get_state(start_state)
            current_pose = Pose(position=fk_result.ee_position, quaternion=fk_result.ee_quaternion)
            
            # mode=1 (REL), ref=0 (BASE) 라고 가정하고 구현 (SIMULATION_INTERFACE_SPEC에 따름)
            delta_m = [req.pos[0]/1000.0, req.pos[1]/1000.0, req.pos[2]/1000.0]
            
            goal_pos = current_pose.position.clone()
            goal_pos[0, 0] += delta_m[0]
            goal_pos[0, 1] += delta_m[1]
            goal_pos[0, 2] += delta_m[2]

            goal_pose = Pose(position=goal_pos, quaternion=current_pose.quaternion)
            
            ik_result = self.ik_solver.solve_single(goal_pose, start_state)
            if ik_result.success.item():
                target_joints_rad = ik_result.solution[0].cpu().numpy().tolist()
                self._publish_joint_command([math.degrees(j) for j in target_joints_rad])
                time.sleep(0.5) # 이동 딜레이
            else:
                self.get_logger().warn("MoveLine IK Failed!")
                
        except Exception as e:
            self.get_logger().error(f"MoveLine Exception: {e}")

        res.success = True
        return res

    def change_speed_cb(self, req, res):
        res.success = True
        return res

    # --- Gripper Services / Action ---

    def set_position_cb(self, req, res):
        self.get_logger().info(f"SetPosition called to {req.position}")
        self.gripper_position = req.position
        res.success = True
        res.message = "Simulated Gripper Moved"
        return res

    def get_state_cb(self, req, res):
        res.success = True
        res.state.present_position = self.gripper_position
        res.state.present_current = 200 # 모의 전류값
        return res

    def safe_grasp_cb(self, goal_handle):
        self.get_logger().info(f"SafeGrasp action called to {goal_handle.request.target_position}")
        
        # 실제 파지 시뮬레이션
        time.sleep(1.0)
        self.gripper_position = 670 # 실제 잡혔을 때의 위치(예: 670)로 모의
        
        goal_handle.succeed()
        
        result = SafeGrasp.Result()
        result.success = True
        result.final_position = self.gripper_position
        result.final_current = 250
        result.reason = "GRASP_CONTACT_DETECTED"
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SimExecutorBridgeNode()
    
    # Use MultiThreadedExecutor for action servers and multiple services
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
