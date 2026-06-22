# 시뮬레이션 환경 구축 과제 및 히스토리

본 문서는 Strawberry Grasp 시뮬레이션 환경 구축 과정의 진행 상황과 과거 아키텍처 프롬프트 히스토리를 요약합니다.

## 1. 진행 완료된 작업
* **Isaac Sim 연동 및 브릿지 최적화**: 
  * 기존 ROS 2 코어 노드(`isaac_sim_bridge_node.py`) 방식을 폐기하고, Isaac Sim 5.1.0 스크립트 에디터 환경에서 구동되는 브릿지로 단일화.
  * 로봇 트리 내 조인트 `DriveAPI`에 강제 `Stiffness`(1,000,000)를 주입하여 관절 흔들림(꿀렁임) 완벽 해결.
* **가짜 비전 노드 (`fake_vision_node`) 구현**: 
  * Isaac Sim 내 딸기 객체의 3D 좌표를 원본 비전 노드(`strawberry_fusion`)와 100% 동일한 메시지(`Float64MultiArray`, `PoseStamped`)로 발행.
* **가상 제어기 브릿지 (`sim_executor_bridge_node`) 구현**: 
  * `MoveLine`, `MoveJoint`, `MoveSplineJoint` 등 실제 하드웨어의 서비스와 액션을 가로채서 가짜 성공 응답을 반환.
  * 목표 관절 및 역기구학 좌표를 `/joint_command` 토픽으로 변환하여 Isaac Sim 내부의 로봇을 구동시킴.
* **의존성 모킹 (Mocking)**: 
  * 메인 코드를 수정하지 않기 위해 `strawberry_motion` 가짜 패키지를 구축하고 안전 함수(`scan_safety.py`)를 흉내 내어 파이프라인 구동 성공.

## 2. 향후 세부 과제 (Upcoming Tasks)
* **J1/J2 Swing 초과 에러 해결**: `scan_executor`가 대기 자세(Overview)에서 바로 스캔을 수행한 뒤, `curobo_planner`가 딸기 위치로 IK를 계산할 때, 딸기 좌표(`X=-304, Y=672` 등)가 대기 자세에서 너무 꺾인 위치에 있어 J2 관절이 90도 이상 크게 회전해야 한다는 이유로 안전 검사(`trajectory_has_reasonable_swing`)에 걸려 파지가 취소(reject)되는 현상이 발견되었습니다.
  * **해결 방안 1**: 아이작 심 상에서 딸기의 위치를 로봇이 대기 자세에서 쉽게 손을 뻗을 수 있는 정면 방향으로 이동.
  * **해결 방안 2**: `scan_pose_candidates.yaml`의 `curobo_start_joints_deg`와 `endpoint_joints_deg` 자체를 딸기 방향을 자연스럽게 바라보는 자세로 완전히 새로 세팅하여 로봇을 그 자세에서부터 시작하게 함.
* **물리적 정합성(TCP 오프셋) 검증**: 시뮬레이션 상의 로봇 TCP 오프셋과 실물 로봇의 오프셋 간의 100mm 오차 여부를 실기팀과 확인 및 동기화.
* **충돌 모델 정교화**: `measured_tcp_260mm` 툴 프로필 적용 시 발생하는 로봇-테이블 간의 `INVALID_START_STATE_WORLD_COLLISION` 오판 문제는 `config/environment.yaml`의 더미 큐브 생성으로 우회 완료.

---

## [참고] 초기 아키텍처 제안 프롬프트 히스토리
*(기록용)* 초기 시뮬레이션 환경 구축 시 AI에게 두산 로봇 제어 아키텍처를 지시하기 위해 사용했던 프롬프트 요약입니다.
1. **`doosan_real_hardware_bridge`**: `/dsr01/servol_cmd`(`Twist`)를 받아 `RobotAction`으로 서보 API 직접 제어.
2. **`arm_controller_node`**: MoveIt 2 기반 경로 계획. `/move_to_pick`, `/move_to_place`, `/move_to_home` Trigger 서비스 제공.
3. **`gripper_node`**: 객체별 (bread, snack, bottle 등) 파지력(Force) 매핑 (`GRASP_FORCE_TABLE`). `/gripper/grasp` 서비스 등을 통한 강건한 파지력 제어.
