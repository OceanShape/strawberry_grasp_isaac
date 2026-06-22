# 트러블슈팅 가이드 (Troubleshooting)

이 문서는 시뮬레이션 환경 구축 및 실행 과정에서 발생할 수 있는 주요 에러와 해결 방법을 정리합니다.

## 1. `scan_executor`가 아무 동작 없이 종료될 때
* **현상**: 시작 트리거(`ros2 service call /strawberry/scan/start ...`)를 보냈음에도 로봇이 움직이지 않고 바로 `SCAN_COMPLETE`를 띄우며 종료됨.
* **로그 확인**: `SINGLE_CELL_SCAN_STARTED target=` 뒤에 타겟 이름이 비어 있음.
* **원인**: 실행 시 스캔할 타겟 구역(`target_cell`)을 지정하지 않아, 빈 이름의 타겟을 찾다가 스킵해버림.
* **해결법**: 파라미터 `target_cell:=all`을 추가하여 노드를 다시 실행합니다.
  ```bash
  python3 docs/scan_executor_node.py --ros-args -p target_cell:=all
  ```

## 2. `INVALID_START_STATE_WORLD_COLLISION` (테이블 충돌 에러)
* **현상**: 타겟 스캔은 정상적으로 수행했으나, 파지를 위한 경로 계획(Cartesian plan)을 시작하자마자 `static:table`과 충돌했다는 에러가 발생하며 모든 파지 후보가 Reject 됨.
* **원인**: 플래너 노드 실행 시 외부 환경 파일이 없으면 무조건 로봇 발밑에 가상의 테이블을 생성하도록 하드코딩되어 있는데, 이 테이블이 로봇 밑동과 겹쳐 충돌 오판을 일으킴.
* **해결법**: `config/environment.yaml` 파일을 생성하여 허공에 더미 큐브를 하나 배치해두면, 플래너가 하드코딩된 테이블을 생성하지 않아 문제가 해결됩니다. (현재 해당 파일이 세팅되어 있으므로 기본 명령어 `python3 docs/curobo_planner_node.py &`로 실행하시면 됩니다.)
