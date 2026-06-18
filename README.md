# Isaac Sim + ROS 2 통합 시뮬레이션 환경 (Strawberry Grasp)

본 프로젝트는 두산 로봇(e0509)과 커스텀 그리퍼를 활용하여 아이작 심(Isaac Sim) 환경에서 딸기 수확을 시뮬레이션하는 테스트 베드입니다. 기존의 ROS 2 코어 노드들을 전혀 수정하지 않고 양방향 통신을 지원하는 가상 브릿지를 구축했습니다.

## 1. 노드 설명
* **`isaac_sim_bridge_node.py`**: Isaac Sim 내부의 객체(딸기) 좌표를 ROS 2로 퍼블리싱하고, ROS 2에서 하달된 로봇 관절 모션(`/dsr01/motion/move_spline_joint`)을 받아 가상 로봇을 직접 구동하는 핵심 브릿지입니다.
* **`test_move_node.py`**: IK 연산(역기구학)을 수행해 로봇 그리퍼의 끝단을 10cm씩 왕복 이동시키며 브릿지의 정상 동작 여부를 테스트하는 노드입니다.

---

## 2. 실행 가이드 (임시 테스트 방법)

테스트를 진행하려면 반드시 2개의 터미널이 필요합니다. 하나는 **아이작 심 환경과 브릿지**를 돌리고, 다른 하나는 **ROS 2 명령(테스트 노드)**을 전송합니다.

### 터미널 1: 브릿지 노드 구동 (중요)
브릿지 노드는 `omni.isaac.core`를 포함한 Isaac Sim의 파이썬 앱(`SimulationApp`)을 통째로 실행합니다. **따라서 이 명령어를 치기 전에 이미 켜져 있는 Isaac Sim 창이 있다면 반드시 모두 종료(Close)해야 합니다!** (메모리 부족 및 포트 충돌 방지)

브릿지를 실행하면 자동으로 새로운 아이작 심 창이 뜨고 `strawberry_grasp_robot.usd` 환경이 로드됩니다.

```bash
# 1. 워크스페이스 이동
cd /home/sun/strawberry_grasp_isaac

# 2. 기존 아이작 심이 켜져있다면 종료 후 실행
/home/sun/isaacsim/python.sh src/isaac_sim_bridge/isaac_sim_bridge/isaac_sim_bridge_node.py
```
> **실행 확인**: `Failed to upload DomeLight texture` 같은 빨간색 경고 메시지는 무시하셔도 됩니다. 잠시 뒤 아이작 심 창이 뜨고 터미널에 `Isaac Sim Virtual Bridge Node Started` 로그가 뜨면 성공입니다.

### 터미널 2: 테스트 모션 노드 구동
브릿지가 켜지고 통신 준비가 끝났다면, 새 터미널 창을 열고 테스트 노드를 실행하여 명령을 내립니다.

```bash
# 1. 워크스페이스로 이동하여 빌드된 ROS 2 환경을 활성화합니다.
cd /home/sun/strawberry_grasp_isaac
source install/setup.bash

# 2. 로봇을 10cm씩 십자 방향으로 움직이는 테스트 노드 실행
ros2 run isaac_sim_bridge test_move_node
```

---

## 3. 테스트 진행 확인 요소
1. `test_move_node`가 구동되면, 가장 먼저 아이작 심 브릿지가 발행하는 `/dsr01/joint_states` 토픽을 수신해 로봇의 초기 위치를 기록합니다.
2. 로봇 끝단의 **기존 각도를 철저히 유지**한 채로, 역기구학 엔진(`ikpy`)이 X, Y, Z 축에 대해 +10cm, -10cm 위치의 관절 각도를 알아서 계산합니다.
3. 브릿지 서버로 목표 위치를 전송하며, 시뮬레이션 화면에서 로봇이 10cm 단위로 정확히 움직였다가 복귀하는지 시각적으로 점검합니다.
