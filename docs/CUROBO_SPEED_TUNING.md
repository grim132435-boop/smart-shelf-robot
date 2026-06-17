# cuRobo 속도 튜닝 레퍼런스

> 조사일: 2026-06-02 | 출처: NVlabs/curobo 공식 레포 분석

---

## time_dilation_factor 의미

**정의**: 최적화된 궤적의 시간 축을 늘리는 후처리 파라미터 (경로 자체는 변하지 않음).

```
new_dt = optimized_dt × (1.0 / time_dilation_factor)
```

| 값 | 궤적 시간 배율 | 속도 |
|----|---------------|------|
| 0.99 | 1.01× | 거의 최적 속도 |
| 0.75 | 1.33× | 약간 느림 |
| 0.65 | 1.54× | 현재 stage3 approach |
| 0.70 | 1.43× | 현재 stage3 pre-grasp |
| 0.50 | 2.00× | 절반 속도 (기본 예제) |
| 0.33 | 3.03× | 1/3 속도 |

### 핵심 규칙
- **값이 높을수록 빠름** (dilation이 적음)
- **유효 범위: 0.0 < value < 1.0** (1.0 이상 시 cuRobo 내부에서 에러)
- `velocity_scale` / `acceleration_scale` 쓰지 말 것 → 비용함수 재튜닝 필요
- `time_dilation_factor`만 쓸 것 → 경로 재계산 없이 순수 시간 재스케일링

### 관련 소스
- 정의: `~/curobo/src/curobo/wrap/reacher/motion_gen.py` L1017, L1339
- 사용 예: `~/curobo/examples/motion_gen_example.py` L219-232
- 테스트: `~/curobo/tests/motion_gen_speed_test.py`

---

## Stage 3 현재 설정 (stage3_graspgen_curobo_gui.py)

```python
# plan_config (pre-grasp 이동, lift) — L360
time_dilation_factor = 0.7   # 1.43× 느림

# slow_config (grasp approach 최종 접근) — L365
time_dilation_factor = 0.65  # 1.54× 느림
```

### 속도 빠르게 하려면
```python
# 더 빠르게 (권장 튜닝 범위)
plan_config.time_dilation_factor = 0.85   # 1.18× 만 느림 (거의 실시간)
slow_config.time_dilation_factor = 0.75   # 1.33× (안전한 접근)
```

---

## MotionGenPlanConfig 주요 파라미터

```python
from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

cfg = MotionGenPlanConfig(
    enable_graph=False,          # RRT 비활성 (최적화만)
    enable_graph_attempt=4,      # 4회 실패 후 그래프 폴백
    max_attempts=8,              # 최대 재시도 횟수
    enable_finetune_trajopt=True,# 궤적 파인튜닝 활성
    time_dilation_factor=0.7,    # 속도 조절 (핵심)
)
```

---

## USD 궤적 애니메이션 내보내기 (디버깅용)

```python
from curobo.util.usd_helper import UsdHelper

UsdHelper.write_trajectory_animation_with_robot_usd(
    robot_file="path/to/robot.yml",
    world_model=world_model,
    q_start=q_start,
    q_traj=trajectory,
    save_path="debug_traj.usd",
    robot_color=[0.5, 0.5, 0.2, 1.0],
    flatten_usd=True,
)
```

파일: `~/curobo/src/curobo/util/usd_helper.py`
예제: `~/curobo/examples/usd_example.py`
