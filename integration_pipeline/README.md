<!-- 모션팀 전송용 — stage7 모듈화 핸드오프 (재사용 모듈 + 핵심 2주제). integration/ 와 별도 폴더(독립 업데이트) -->

# 모션팀 이관 — 모듈화 파이프라인 핸드오프

기존 `integration/`(ROS2 cuRobo 플래너 코어)과 **별개 폴더**입니다. 이쪽은 시뮬 `stage7`을 **유지보수·재사용하기 쉽게 모듈화**한 결과 + 모션팀이 요청한 **두 주제**(누운 물체 파지 / 모션 끊김)를 정리한 것입니다. 두 폴더는 동일 계층에 두고 **독립적으로 업데이트**합니다.

> 이 폴더는 모션팀 측 작업/통합용 **참고 + 재사용 모듈**입니다. 실 ROS2 배선은 `integration/` 패키지를 쓰세요.

## 무엇이 들어있나

| 파일 | 내용 | 모션팀 활용 |
|---|---|---|
| **`LYING_PICK.md`** | 캔/페트병 **누워있을 때 파지**(축 감지→위에서 파지→직립 재배향, 병=캡-위) | 통합 대상 ① |
| **`MOTION_STUTTER.md`** | 실기 **모션 끊김(뚝뚝)** 원인 + 수정안 | 통합 대상 ② |
| `pp_geometry.py` | **순수** 파지 기하(파지자세 합성·프레임 변환·누운픽). numpy/scipy/trimesh만 의존 → cuRobo·Isaac 없이 **그대로 import 가능** | 재사용 |
| `pp_motion.py` | 모션 실행(cuRobo 궤적/moveL/직접IK/그리퍼)+충돌간격. omni/curobo 의존 | 참고 |
| `pp_phases.py` | 단계 함수(현재: 그랩젠 생성=추론+월드변환) | 참고 |

전체 오케스트레이터(상태머신)는 개발 레포의 **`stages/pipeline/pick_place_main.py`** 입니다(여기엔 미포함 — 거대 FSM, 시뮬 전용). 거기 FSM 상단에 **단계 맵 주석**(실행 순서)이 있습니다.

## 파이프라인 구조 (한눈에)

```
[단계1] 타겟 선택(다물체 우선순위)  ─ FSM(pick_place_main)
[단계2] 그랩젠 생성  ─ pp_phases.query_graspgen → 점구름→파지후보(월드)
[단계3] 쿠로보 파지점 선택  ─ select_best_reachable_grasp / plan_grasp(goalset)
[단계4] 파지→리프트→attach
[단계5] 운반(plan_single 충돌회피)
[단계6] 진입(+y)→하강(-z) 안착
[단계7] 후퇴(-y)→home
        헬퍼: 파지기하=pp_geometry, 모션실행/간격=pp_motion
```

## 좌표·그리퍼 규약 (필수)

- 단위 meter, world 프레임. 로봇 base ≈ `[-0.25,-0.04,0.73]`. cuRobo는 base 원점 계획 → **월드 타겟에서 base offset 차감**(`pp_motion._ROBOT_BASE_OFFSET`, `set_base_offset()`).
- RH-P12 side 파지 프레임: **Z=approach(수평 진입), Y=손가락 분리축, X=그리퍼 상단(카메라축, +z로 강제)**. `side_grasp_from_approach()` 참조.
- 파지점 TCP 깊이 `RHP12_TCP_DEPTH=0.110`(실기 실측 보정 필요).

## 빠른 사용 (누운픽만 쓰려면)

```python
from pp_geometry import can_is_lying, lying_grasp_from_axis  # cuRobo/Isaac 불필요
# 자세한 통합은 LYING_PICK.md 참조
```
