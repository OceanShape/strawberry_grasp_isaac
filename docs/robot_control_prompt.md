# 두산 로봇 & 그리퍼 제어 아키텍처 프롬프트

다른 시뮬레이션 프로젝트에서 두산 로봇 및 그리퍼의 제어 아키텍처를 구현하도록 AI나 개발자에게 지시할 때 아래의 프롬프트 텍스트를 사용할 수 있습니다.

---

**[컨텍스트]**
우리는 두산 로봇 팔과 커스텀 그리퍼를 위한 ROS 2 시뮬레이션 환경을 구축하고 있습니다. 하드웨어 브릿지(Hardware Bridge)와 모션 컨트롤러(Motion Controller) 노드를 구현해야 합니다. 다음 아키텍처 명세에 따라 ROS 2 Python 노드들을 작성해 주세요.

**[아키텍처 명세]**

### 1. 두산 실물 하드웨어 브릿지 노드 (`doosan_real_hardware_bridge.py`)
- **역할**: 고수준의 Policy 액션을 두산의 네이티브 서보 API로 변환하는 실시간 통신 브릿지 역할을 합니다.
- **노드 이름**: `doosan_real_hardware_bridge`
- **Subscribers (구독)**:
  - Topic: `/dsr01/servol_cmd`
  - Type: `geometry_msgs/msg/Twist`
  - 목적: 디퓨전 폴리시(Diffusion Policy) 또는 고수준 제어기로부터 목표 TCP 밀리미터 변위(선속도 및 각속도)를 30Hz로 수신합니다.
- **Publishers (발행)**:
  - Topic: `/dsr01/servo_vector_cmd`
  - Type: `dsr_msgs/msg/RobotAction` (두산 DART 플랫폼 Humble 라이브러리 제공)
  - 목적: 직렬화된 벡터 명령을 두산 실물 컨트롤러의 서보 API(`servol_rt`)로 직접 전송합니다.
- **동작 방식**: `Twist` 메시지의 선형 및 각형 벡터를 `RobotAction.data` 배열에 지속적으로 매핑하고 발행하여 초저지연 실시간 통신을 보장합니다.

### 2. 암 컨트롤러 노드 (`arm_controller_node.py`)
- **역할**: 고수준 모션 계획 및 실행 (일반적으로 MoveIt 2 사용).
- **노드 이름**: `arm_controller_node`
- **Subscribers (구독)**:
  - Topic: `/place_target`
  - Type: `geometry_msgs/msg/PoseStamped`
  - 목적: 물체를 놓을 목표 포즈를 수신합니다.
- **Publishers (발행)**:
  - Topic: `/joint_command`
  - Type: `sensor_msgs/msg/JointState`
  - 목적: 명시적인 조인트 궤적 명령을 전송할 때 사용할 수 있습니다.
- **Services (서비스) (Type: `std_srvs/srv/Trigger`)**:
  - `/move_to_pick`: 파지(Grasp) 위치로의 이동을 계획하고 실행합니다.
  - `/move_to_place`: 목표 배치(Place) 포즈로의 이동을 계획하고 실행합니다.
  - `/move_to_home`: 미리 정의된 홈(Home) 포지션으로 팔을 복귀시킵니다.
- **세부 사항**: 역기구학 및 궤적 생성을 처리하기 위해 모션 플래닝 인터페이스(예: `MoveItPy`)를 초기화해야 하며, 이를 통해 `_move_to_pose` 및 `_move_to_named_target` 내부 로직을 구현합니다.

### 3. 그리퍼 노드 (`gripper_node.py`)
- **역할**: 물체의 재질 속성과 원하는 파지력에 기반하여 그리퍼의 물리적 작동을 제어합니다.
- **노드 이름**: `gripper_node`
- **설정**:
  - 물체 타입별로 이상적인 파지력과 최대 파지력(단위: N)을 매핑하는 `GRASP_FORCE_TABLE`이 미리 정의되어 있어야 합니다. (예시):
    - 'bread' (식빵 - 소프트 바디): force 10.0, max 15.0
    - 'snack' (과자 - 봉지): force 20.0, max 30.0
    - 'bottle' (페트병 - 원통형): force 40.0, max 60.0
    - 'can' (캔 - 금속): force 80.0, max 100.0
- **Subscribers (구독)**:
  - `/object_class` (`std_msgs/msg/Float32`): 조작 중인 물체 타입에 따라 파지력 목표치를 동적으로 설정합니다.
  - `/grasp_force` (`std_msgs/msg/Float32`): 파지력을 수동으로 덮어쓸 때 사용합니다.
- **Publishers (발행)**:
  - `/gripper_command` (`sensor_msgs/msg/JointState`): 그리퍼 하드웨어/시뮬레이션으로 목표 명령을 전송합니다.
  - 세부 사항: 조인트 이름으로 `rh_r1`을 사용합니다. 목표 `position`은 가변적이며(예: 열림 `0.0`, 닫힘 `1.101`), `effort`는 계산된 파지력을 나타냅니다.
- **Services (서비스) (Type: `std_srvs/srv/Trigger`)**:
  - `/gripper/open`: 그리퍼를 완전히 엽니다 (position: 0.0, effort: 0.0).
  - `/gripper/close`: 기본 또는 최대 힘으로 그리퍼를 완전히 닫습니다.
  - `/gripper/grasp`: 현재 물체 클래스에 맞춰 `GRASP_FORCE_TABLE`에 정의된 정확한 effort 한계를 사용하여 그리퍼를 닫습니다.

**[태스크]**
위에서 설명한 토픽 이름, 메시지 타입, 그리고 구조적 로직을 엄격하게 준수하여 이 세 가지 노드(`doosan_real_hardware_bridge.py`, `arm_controller_node.py`, `gripper_node.py`)의 완전한 Python 소스 코드를 작성해 주세요.
