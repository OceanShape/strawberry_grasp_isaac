# 시뮬레이션 환경 실행 매뉴얼 (User Manual)

본 문서는 아이작 심(Isaac Sim) 기반 로봇 제어 통합 시뮬레이션을 실행하기 위한 절차를 안내합니다.

## 1. 아이작 심(Isaac Sim) 셋업
1. Isaac Sim 5.1.0을 구동합니다.
2. `robot` 폴더에 있는 `strawberry_grasp_robot.usd` 파일을 스테이지로 로드합니다.
3. 로봇 팔의 Prim Path(`/World/doosan_robot`)와 타겟 딸기의 Prim Path(`/World/strawberry_ripe`)가 브릿지 스크립트와 일치하는지 확인합니다.

## 2. 브릿지 노드 구동
모방 환경 구성을 위해 두 개의 핵심 가상 브릿지 노드를 백그라운드에서 실행합니다.

```bash
# ROS 2 환경 소싱
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
source ~/strawberry_grasp_isaac/install/setup.bash
cd ~/strawberry_grasp_isaac

# 가상 비전 노드 실행 (딸기 인식 모방)
ros2 run strawberry_sim_core fake_vision_node &

# 가상 제어기 브릿지 실행 (액션 가로채기 및 아이작 심 제어)
ros2 run strawberry_sim_core sim_executor_bridge_node &
```

## 3. 메인 파이프라인 구동
실제 하드웨어 구동을 위해 작성된 원본 노드들을 수정 없이 실행합니다.
**반드시 새로운 터미널 창을 열고 아래와 같이 ROS 2 환경을 다시 소싱한 뒤 실행해야 합니다.**

```bash
# ROS 2 환경 소싱 (필수)
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
source ~/strawberry_grasp_isaac/install/setup.bash
cd ~/strawberry_grasp_isaac

# 1. 경로 계산 플래너 노드 구동 (기본 measured_tcp_260mm 프로필 사용)
python3 docs/curobo_planner_node.py &

# 2. 스캔 및 타겟 전달 노드 구동 (모든 타겟 스캔)
python3 docs/scan_executor_node.py --ros-args -p target_cell:=all
```

## 4. 수확 시퀀스 시작 (Trigger)
모든 노드가 준비(ready) 상태에 진입하면, 새로운 터미널에서 수동으로 시작 명령을 내립니다.

```bash
# ROS 2 환경 소싱 (새 터미널이므로 필수)
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
source ~/strawberry_grasp_isaac/install/setup.bash

# 시작 서비스 호출
ros2 service call /strawberry/scan/start std_srvs/srv/Trigger
```
명령 인가 후, 로봇이 딸기를 향해 물리적으로 움직이며 수확 시퀀스를 모사합니다.
