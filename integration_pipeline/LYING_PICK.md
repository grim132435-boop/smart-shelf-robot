# 누운 물체 파지 (캔 / 페트병이 누워있을 때)

검증: 시뮬에서 캔(서/눕 0°) + 병(서/눕 45°) 혼합 4개 → 누운 것 포함 4/4 직립 적치 성공.

## 원리 (3줄)

1. 물체가 누웠는지 감지(축의 z성분이 작으면 눕힘).
2. 누우면 GraspGen 후보 대신 **물체 축 기준 "위에서(top-down) 파지"** 자세를 합성 — 그리퍼 X축(상하축)을 **물체 축**에 맞춤.
3. 그 자세로 잡고 적치 pose(X=위)로 옮기면 **물체가 저절로 직립**한다. 별도 "세우기" 코드 없음 — 그리퍼 X축이 물체 축이라, X를 위로 두면 물체가 선다.

## 핵심 코드 (pp_geometry.py — 순수, 의존 numpy/scipy만)

```python
def can_is_lying(obj):
    # 물체 자세 quat → 회전행렬의 z축(물체 축) 월드 z성분 |az|<0.5면 눕힘
    _w,_x,_y,_z = obj.get_world_pose()[1]
    az = Rotation.from_quat([_x,_y,_z,_w]).as_matrix()[:,2][2]
    return abs(float(az)) < 0.5

def lying_grasp_from_axis(can_axis, obj_center, tcp_depth):
    # X(그리퍼 상하축)=물체 축(수평투영), Z(approach)=아래(-z), Y=Z×X, EE=중심-tcp_depth·Z
    ...  # 4x4 파지자세(월드) 반환
```

## FSM 통합 스니펫 (그랩젠 생성 단계에서)

```python
if can_is_lying(target_cube):
    _ax = obj_R @ np.array([0,0,1])          # 물체 로컬 +z(월드). 병은 이게 '캡' 방향
    if obj_type == "bottle":
        # ★병은 위아래(캡) 있음 → 캡이 그리퍼 X축(=재배향 후 위/카메라축)과 같은 쪽인 +캡만
        #   → 적치 시 캡-위 + 카메라-위 동시 보장 (joint_6 롤이 이 정렬을 결정)
        grasp_cands = [lying_grasp_from_axis(+_ax, center, TCP_DEPTH)]
    else:
        # 캔(대칭) → ±축 모두 주고 plan_grasp이 손목 덜 트는 쪽 선택
        grasp_cands = [lying_grasp_from_axis(+_ax, center, TCP_DEPTH),
                       lying_grasp_from_axis(-_ax, center, TCP_DEPTH)]
    grasp_world = grasp_cands[0]
# 파지(닫기) 직후:
if can_is_lying(target_cube):
    grip_z_offset = 0.0   # 재배향 후 TCP=물체중심이라 오프셋 0 강제
```

## ★ 캡-위 / 카메라-위 = joint_6 (모션팀 통찰)

페트병은 위아래가 있어 **적치 시 캡이 위로, 그리퍼 카메라도 위로** 가야 함. 둘은 **파지 순간 손목 롤(joint_6)** 로 한 번에 결정됨:
- 누운 병 위에서 파지할 때 그리퍼 X축(=카메라/위축)을 **캡 방향(+축)** 에 맞추면 → 재배향(X→위) 시 캡-위 + 카메라-위가 동시 성립.
- 반대 부호(-축)로 잡으면 캡이 아래(거꾸로). 그래서 병은 **+캡 후보만** 채택.
- 캔은 대칭이라 ± 무관.

## 머지 체크리스트

- [ ] `pp_geometry.py`의 `can_is_lying`, `lying_grasp_from_axis` 가져오기(또는 동등 구현).
- [ ] 그랩젠 단계: 누우면 위 스니펫처럼 `grasp_cands` 합성(병=+캡, 캔=±).
- [ ] 파지 후 누우면 `grip_z_offset=0`.
- [ ] 누운 물체 spawn/감지: 안착높이=반경, 구름 방지 댐핑(angular 50/linear 5) 권장.
- [ ] 적치 pose는 X=위(`side_grasp_from_approach`)라 재배향 자동 — 별도 세우기 불필요.
- [ ] 실기: 비전이 물체 축 + (병)캡/넥 방향(위아래)을 추정해 `+_ax`(캡)를 정확히 줘야 함.
