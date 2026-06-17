# Stage 3 파이프라인 상세

> 최종 업데이트: 2026-06-02

---

## 전체 흐름

```
GraspGen ZMQ 서버 (port 5556)
  ← 큐브 점구름 2048점 전송
  → (M, 4, 4) 파지 자세 + (M,) 점수 반환
       ↓
  상위 50개 후보 → select_best_reachable_grasp()
       ↓ (첫 번째 통과 파지)
  cuRobo MotionGen → Isaac Sim 실행
```

---

## 핵심 파일

| 파일 | 위치 |
|------|------|
| Stage 3 메인 | `~/curobo_ws/stage3_graspgen_curobo_gui.py` (653줄) |
| 사본 | `~/shelf_grasp_dev/stage3_graspgen_curobo_gui.py` |
| GraspGen 클라이언트 | `~/graspgen_ws/GraspGen/grasp_gen/serving/zmq_client.py` |
| 오프라인 시각화 | `~/shelf_grasp_dev/visualize_grasps.py` |

---

## 상수 (stage3_graspgen_curobo_gui.py L56-72)

```python
CUBE_SIZE           = 0.05     # m
CUBE_MASS           = 0.15     # kg
TABLE_Z             = 0.0
CUBE_Z              = 0.025    # m (테이블 위 큐브 중심)
ROBOTIQ_TO_FRANKA_Z = +0.0897  # m (Robotiq 195mm → Franka EE 105.3mm)
PREGRASP_STANDOFF   = 0.15     # m
FINGER_OPEN         = 0.04     # m
FINGER_GRASP        = 0.015    # m
NUM_PC_POINTS       = 2048
```

---

## 파지 자세 선택 로직

**함수**: `select_best_reachable_grasp()` L199–248

필터 순서 (전부 통과해야 선택):
1. `is_in_franka_workspace()` — 수평 0.15~0.75m, 높이 0.0~0.70m, x > 0.1m
2. 하향 접근 — approach Z < −0.65
3. 수평 닫힘축 — |closing Z| ≤ 0.40
4. `snap_grasp_roll_90()` — 닫힘축을 world X/Y 축에 90° 스냅
5. `IKSolver.solve_single()` — 20-seed IK 가능 여부

기준 통과 첫 번째 파지 반환. 없으면 큐브 위치 무작위 이동 후 재시도.

---

## 속도 설정 (plan_config L357–366)

```python
plan_config.time_dilation_factor = 0.7   # pre-grasp/lift
slow_config.time_dilation_factor = 0.65  # grasp approach
```

→ 자세한 튜닝 가이드: docs/CUROBO_SPEED_TUNING.md

---

## 상태 머신

```
IDLE → QUERY_GRASP → OPEN_GRIPPER → PLAN_PREGRASP
     → MOVE_PREGRASP → PLAN_GRASP → MOVE_GRASP
     → CLOSE_GRIPPER → PLAN_LIFT → MOVE_LIFT
     → HOLD → OPEN_DROP → IDLE
```

---

## 실행 방법

```bash
# 터미널 1: GraspGen ZMQ 서버
bash ~/shelf_grasp_dev/start_graspgen_server.sh

# 터미널 2: Stage 3 Isaac Sim GUI (env_isaaclab)
conda activate env_isaaclab
source ~/IsaacLab/_isaac_sim/setup_conda_env.sh
cd ~/curobo_ws
python stage3_graspgen_curobo_gui.py --port 5556
```

환경 문제 발생 시 → RUNBOOK.md 참조.

---

## 알려진 이슈 (디버깅 중)

| 이슈 | 현상 | 원인 추정 |
|------|------|-----------|
| 속도 느림 | pre-grasp/lift 이동이 너무 느려 실용적이지 않음 | time_dilation_factor 0.65~0.7이 과도하게 낮음 |
| 파지 위치 부적절 | 선택된 파지 자세가 실제 큐브를 잡기 어려운 위치 | snap_grasp_roll_90 또는 ROBOTIQ_TO_FRANKA_Z 오프셋 문제 가능성 |

→ Stage 3 디버깅 작업 계획: docs/STAGE3_DEBUG_PLAN.md
