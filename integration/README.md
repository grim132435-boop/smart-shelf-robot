<!-- 모션팀 전송용 — 시뮬 stage7 → 실기 cuRobo 모션 이관 패키지 -->

# 모션팀 이관 패키지 — cuRobo 플래너 (stage7 기반)

비전 노드(`webcam_seg_node`)가 발행하는 GraspGen 후보를 받아, **plan_grasp(goalset) 선택 + 관절한계 모션 + 두산 실행**을 하는 모션 측 코어입니다. 시뮬 `stage7_graspgen_e0509.py`의 모션 책임만 추출했습니다.

## 구성

| 파일 | 내용 |
|------|------|
| `INTERFACE_CONTRACT.md` | **먼저 읽기.** 토픽 계약 + 좌표/쿼터니언 규약 + 이관 리스크 4가지 + stage7 매핑 |
| `curobo_planner_core.py` | 모션 코어 — 데이터소스 무관. 관절한계 URDF, plan_grasp goalset, 리프트/운반/홈 |
| `curobo_planner_node.py` | ROS2 배선 스켈레톤 — 비전 토픽 구독 → 코어 호출 → 두산 궤적/그리퍼 |

## 시작점 (TODO 채울 곳)

1. `core` 상수의 `ROBOT_DIR` 등 경로를 실기로 교체, TCP 깊이 실측 보정 (리스크③).
2. `node`의 `arm_joint_names`(6개), 두산 궤적 전송(`_send_joint_traj`), 그리퍼 전류(`_gripper_grasp`), obstacles 파싱(`_parse_obstacles_to_world`).
3. **리스크① 관절한계가 MotionGen 빌드에 들어갔는지** 반드시 확인 — 없으면 팔 플립.

## 핵심 한 줄

비전은 "어디를 잡을지(후보)"까지, 모션은 "그 중 무엇을 어떻게 도달할지(goalset+궤적)"를 책임집니다. 두 노드의 **좌표/쿼터니언/오프셋 규약**(계약서 2·3장)만 맞으면 시뮬에서 검증된 로직이 그대로 실기로 이어집니다.
