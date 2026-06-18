# 아이작 심 통합 시뮬레이션 사용자 작업 매뉴얼

본 문서는 아이작 심(Isaac Sim) 기반 로봇 제어 통합 시뮬레이션 구축을 위해 **사용자가 직접 수행해야 하는 필수 작업**들을 안내합니다.

## 단계 1. Isaac Sim 환경 및 USD 구성

1. **Isaac Sim 5.1.0 구동**: Isaac Sim을 실행합니다.
2. **로봇 USD 및 타겟 USD 배치**:
   - 하단 Content 창에서 제공해주신 `robot` 폴더로 이동합니다.
   - `strawberry_grasp_robot.usd` 파일을 스테이지로 드래그 앤 드롭하여 로드합니다.
3. **Prim Path 기록 (중요)**:
   - 우측 Stage 트리에서 로봇 팔(Articulation Root)의 정확한 Prim Path를 찾습니다. (예: `/World/doosan_robot`)
   - 타겟 딸기 객체의 정확한 Prim Path를 찾습니다. (예: `/World/strawberry_ripe`)
   - 확인한 경로를 `src/isaac_sim_bridge_node.py`의 `ROBOT_PRIM_PATH`와 `TARGET_PRIM_PATH` 변수에 입력합니다.

## 단계 2. ROS 2 환경 준비

1. 터미널을 열고 로봇 워크스페이스(ROS 2)로 이동하여 환경 변수를 활성화합니다.
   ```bash
   source install/setup.bash
   ```

## 단계 3. 시뮬레이션 브릿지 노드 구동

1. Isaac Sim의 파이썬 실행기(`python.sh`)를 사용하여 브릿지 스크립트를 독립 모드(Standalone)로 실행합니다.
   ```bash
   # 예시 (경로는 실제 Isaac Sim 설치 경로에 맞게 변경하세요)
   ~/.local/share/ov/pkg/isaac-sim-5.1.0/python.sh /home/sun/strawberry_grasp_isaac/src/isaac_sim_bridge_node.py
   ```
2. 터미널에 퍼블리셔와 서비스 서버가 활성화되었다는 로그가 뜨는지 확인합니다.

## 단계 4. 모션 플래너 노드 구동 및 테스트

1. **새 터미널 창**을 열고 ROS 2 환경을 소싱한 뒤, 모션 플래너 노드를 실행합니다.
   ```bash
   ros2 run <패키지명> curobo_planner_node.py
   ```
2. **또 다른 새 터미널 창**을 열고 스캔/실행 관리자 노드를 실행합니다.
   ```bash
   ros2 run <패키지명> scan_executor_node.py
   ```
3. 트리거 서비스를 호출하여 수확 사이클을 시작합니다.
   ```bash
   ros2 service call /strawberry/scan/start std_srvs/srv/Trigger
   ```
4. **검증**: Isaac Sim 뷰포트 내에서 가상 두산 로봇이 딸기를 향해 이동하고 그리퍼가 동작하는지 시각적으로 확인합니다.
