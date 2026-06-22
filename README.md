# Isaac Sim + ROS 2 통합 시뮬레이션 환경 (Strawberry Grasp)

본 프로젝트는 두산 로봇(e0509)과 커스텀 그리퍼(RH-P12-RN-A)를 활용하여 아이작 심(Isaac Sim) 환경에서 딸기 수확을 시뮬레이션하는 테스트 베드입니다. 

실제 하드웨어 구동을 위해 작성된 모션 플래너(`docs/curobo_planner_node.py`, `docs/scan_executor_node.py`) 코드를 단 한 줄도 수정하지 않고, 시뮬레이션 환경에 그대로 연동하여 동작시키는 것을 목표로 설계되었습니다.

---

## 1. 핵심 아키텍처 및 노드 설명

이 워크스페이스는 실제 환경을 완벽하게 모방(Mocking)하기 위한 여러 컴포넌트로 구성되어 있습니다.

### 🍓 1) `fake_vision_node` (가상 비전 노드)
* **역할**: 아이작 심에서 렌더링된 딸기 3D 좌표를 실시간으로 받아, 실제 카메라 비전 노드(`strawberry_fusion`)가 뿜어내는 것과 완벽히 동일한 형식의 토픽(`/strawberry/detection/scene_positions`, `/strawberry/detection/pick_pose`)으로 변환하여 송출합니다.
* **위치**: `src/strawberry_sim_core/strawberry_sim_core/fake_vision_node.py`

### 🤖 2) `sim_executor_bridge_node` (가상 제어기 브릿지)
* **역할**: 실제 플래너가 두산 로봇 제어기에 쏘는 모든 동작 서비스(`MoveLine`, `MoveJoint`, `MoveSplineJoint`) 및 그리퍼 액션(`SafeGrasp`)을 가로채서 가짜 성공 응답을 주고, 아이작 심의 관절 목표 토픽(`/joint_command`)으로 변환하여 시뮬레이션 로봇을 움직입니다.
* **IK 내장**: 직선 이동(`MoveLine`)의 역기구학 연산을 안정적으로 수행하기 위해, 프로젝트에 세팅된 `curobo` 라이브러리를 이 노드 내부에서 직접 활용합니다.
* **위치**: `src/strawberry_sim_core/strawberry_sim_core/sim_executor_bridge_node.py`

### 🛠 3) `isaac_sim_scripts` 디렉토리의 아이작 심 스크립트들
* **`isaac_sim_script_editor_bridge.py`**: 아이작 심의 내장 스크립트 에디터에서 구동되는 파이썬 스크립트입니다. 시뮬레이션 내 딸기 객체 좌표를 추출하여 퍼블리시하고, 수신된 관절 각도(`joint_command`)를 물리엔진 상의 로봇에 주입(Articulation)합니다.
* **`self_collision_logger_script.py`**: 아이작 심 내부에서 발생하는 로봇 자기 충돌(Self-Collision)을 감지하고, 인접하지 않은(유효한) 부품 간의 물리적 충돌 로그를 `~/isaac_sim_self_collision.log` 파일로 남깁니다.

### 📦 4) `dsr_gripper_tcp_interfaces` 패키지
* 원래 로봇 구동 환경에만 존재하는 커스텀 그리퍼 제어 패키지(액션, 서비스 정의)를 시뮬레이션 워크스페이스에도 껍데기 형태로 구현하여, 플래너 실행 시 발생하는 `ImportError`를 방지합니다.

---

## 2. 시뮬레이션 구동 가이드

본 섹션은 원본 메인 코드(`curobo_planner_node`, `scan_executor`)를 수정 없이 그대로 실행하여, **시뮬레이션 상의 가상 로봇이 실제로 물리적으로 수확 동작을 수행하도록 만드는 전체 실행 과정**을 설명합니다. 테스트를 진행하려면 **아이작 심 실행 터미널**과 **ROS 2 노드 실행 터미널**을 분리해야 합니다.

### 🖥️ 단계 1: 아이작 심(Isaac Sim) 깨끗하게 실행하기
우분투 터미널에 ROS 2 환경변수가 설정되어 있으면 아이작 심 내부 모듈이 꼬이면서 즉시 크래시가 발생합니다. **반드시 환경변수를 무시하는 아래 명령어로 실행하세요.**

```bash
cd /home/sun/isaacsim
env -u LD_LIBRARY_PATH -u PYTHONPATH -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION ./isaac-sim.sh
```

### 🌉 단계 2: 아이작 심 내부 브릿지 가동 (로봇 및 딸기 활성화)
1. 아이작 심이 켜지면, 로봇과 딸기 객체들이 배치된 USD 씬(Scene)을 로드합니다.
2. 상단 메뉴바의 **[Window] -> [Script Editor]** 를 엽니다.
3. 프로젝트의 `isaac_sim_scripts/isaac_sim_script_editor_bridge.py` (필요시 `self_collision_logger_script.py`도 함께) 코드를 전체 복사하여 스크립트 에디터에 붙여넣고 하단의 **[Run]** 버튼을 클릭합니다.
4. 뷰포트 왼쪽의 **재생(Play ▷)** 버튼을 눌러 시뮬레이션을 활성화합니다.
   * 이 시점부터 시뮬레이션 상의 딸기 좌표가 ROS 2로 송출되며, 가상 로봇은 외부 명령을 받아 움직일 준비를 마칩니다.

### 🎭 단계 3: 모방 환경 구성 (가짜 비전 및 가상 제어기 노드 켜기)
**새로운 터미널 창**을 열어 원본 노드들을 속이기 위한 모방 서버들을 실행합니다.

```bash
# 1. 환경 소싱 (필수)
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
source ~/strawberry_grasp_isaac/install/setup.bash

# 2. 가짜 비전 노드 실행 (딸기 인식 모방)
# 시뮬레이션의 딸기 좌표를 받아 진짜 비전 토픽과 동일하게 송출 시작
ros2 run strawberry_sim_core fake_vision_node &

# 3. 가상 제어기 실행 (로봇 제어 모방)
# 플래너가 호출하는 두산 로봇 서비스를 수신하기 위해 가짜 서비스 서버 오픈
ros2 run strawberry_sim_core sim_executor_bridge_node &
```

### 🧠 단계 4: 수확 파이프라인 메인 노드 구동 (동작 시작!)
모방 환경이 완벽하게 갖추어졌습니다. 이제 실제 현장에서 사용하는 원본 메인 노드들을 실행합니다.

**또 다른 새로운 터미널 창**을 열어 동일하게 환경을 소싱합니다.
```bash
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
source ~/strawberry_grasp_isaac/install/setup.bash
cd ~/strawberry_grasp_isaac
```

아래 명령어로 메인 파이프라인 노드를 실행합니다.
```bash
# 1. 경로 계산 및 모션 제어 노드 구동 (테이블 충돌 오판 방지를 위해 검증된 legacy 프로필 적용)
python3 docs/curobo_planner_node.py --ros-args -p tool_model_profile:=legacy_160mm &

# 2. 스캔 및 타겟 전달 관리자 노드 구동 (모든 타겟 구역 스캔 옵션 추가)
python3 docs/scan_executor_node.py --ros-args -p target_cell:=all
```

위 노드들이 켜지면 `scan_executor`는 즉시 동작하지 않고 대기 상태(`ready`)로 진입합니다.

### 🎯 단계 5: 스캔 및 수확 시퀀스 수동 시작 (Trigger)
실제 연구실 로봇 환경에서는 안전상의 이유로 노드를 켰다고 해서 로봇이 바로 움직이지 않습니다. 사람이 물리적인 시작 버튼을 누르거나 명시적인 시작 명령을 내려주어야만 파이프라인이 구동됩니다. 원본 코드를 그대로 사용하는 본 시뮬레이션에서도 이와 똑같이 **수동 서비스 호출(Service Call)**을 날려주어야 합니다.

**새로운 5번째 터미널 창**을 열어 동일하게 환경을 소싱한 뒤, 아래 명령어로 시작 트리거를 보내주세요.
```bash
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
source ~/strawberry_grasp_isaac/install/setup.bash

# 파이프라인 수동 시작 명령!
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger
```

이 명령이 들어가면, `scan_executor`가 가짜 비전 데이터를 통해 타겟을 확정하고 `curobo_planner`가 경로를 계산한 뒤 **가상 제어기**로 제어 서비스를 호출하게 됩니다. 이를 받은 가상 제어기는 최종적으로 아이작 심 내부 로봇을 물리적으로 구동시켜 수확 시퀀스를 완벽히 모사합니다.

---

## 3. 사전 필수 설치 가이드 (CUDA 12.1 툴킷 및 cuRobo v0.7.8)

**[중요]** 본 프로젝트는 실제 하드웨어 구동을 위해 작성된 원본 메인 플래너 노드(`curobo_planner_node.py`)의 코드를 **단 한 줄도 수정하지 않고 그대로 사용하는 것**을 원칙으로 합니다.
따라서 최신 cuRobo(0.8.0 이상)가 아닌 원본 코드가 의존하는 **구버전 cuRobo(v0.7.8)**를 반드시 소스 빌드해야 합니다. 구버전 빌드를 위해서는 시스템에 거대한 C++ CUDA 컴파일러(`nvcc`)가 필수입니다.

터미널을 열고 아래 명령어를 순서대로 실행하세요:

```bash
# 1. RTX 5080 (sm_120) 아키텍처 호환을 위한 CUDA 12.8 툴킷 설치 (관리자 권한 필요)
sudo apt-get install -y cuda-toolkit-12-8

# 2. RTX 5080 지원을 위한 PyTorch 2.6.0 (cu124) 업그레이드
python3 -m pip install torch==2.6.0 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124 --upgrade

# 3. NVIDIA 공식 저장소에서 cuRobo 소스코드 클론 및 v0.7.8 강제 체크아웃 후 빌드
cd ~
git clone https://github.com/NVlabs/curobo.git
cd curobo
git reset --hard v0.7.8
export CUDA_HOME=/usr/local/cuda-12.8
python3 -m pip install -e . --no-build-isolation
```

---

## ⚠️ 4. cuRobo Config 확인 필요 사항

`config/curobo/` 디렉토리에는 cuRobo 모션 플래너가 사용하는 로봇 설정 파일이 들어 있습니다.

| 파일 | 출처 | 상태 |
| --- | --- | --- |
| `e0509_gripper.yml` | `doosan_ws`에서 복사 | ✅ 검증됨 (legacy 160mm 모델) |
| `e0509_gripper.urdf` | `doosan_ws`에서 복사 | ✅ 검증됨 |
| `e0509_spheres.yml` | `doosan_ws`에서 복사 | ✅ 검증됨 |
| **`e0509_gripper_measured_tcp.yml`** | **시뮬팀에서 자동 생성** | **🔴 실기팀 확인 필요** |

### `e0509_gripper_measured_tcp.yml` 확인 사항

이 파일은 `curobo_planner_node.py`의 상수값(`MEASURED_FLANGE_TO_GRASP_CENTER_M = 0.260`)을 기반으로 **자동 생성**된 파일입니다.

- `ee_link`를 `gripper_rh_p12_rn_base`에서 +Z 100mm 떨어진 가상 링크(`grasp_tcp_link`)로 설정
- 이 100mm = `MEASURED_FLANGE_TO_GRASP_CENTER_M(260mm) - MEASURED_FLANGE_TO_GRIPPER_M(160mm)`

**실기 담당자에게 확인 필요:**

1. TCP offset 100mm (`gripper_base` → `grasp_center`)가 실제 하드웨어 측정값과 일치하는지
2. 실기 환경에서 사용하던 원본 `e0509_gripper_measured_tcp.yml`이 별도로 존재하는지
3. `attached_object`의 parent가 `grasp_tcp_link`로 변경된 것이 의도와 맞는지

> **참고**: 실기팀 확인 전까지는 `legacy_160mm` 모드로 실행할 수 있습니다:
> ```bash
> python3 docs/curobo_planner_node.py --ros-args -p tool_model_profile:=legacy_160mm
> ```
