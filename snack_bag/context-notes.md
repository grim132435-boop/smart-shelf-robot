# 과자봉지 적치 개선 컨텍스트 노트 (결정과 근거)

이어받는 세션이 결정을 다시 도출하지 않도록 작업 중 판단을 누적 기록. 체크리스트 = [checklist.md](checklist.md).

## 진단 (2026-06-18, snack55 라이브 관찰)
사용자 관찰: ① carry 중 매대와 박음(충돌회피 안 됨) ② 봉지를 높은 데서 투하해 거치대에서 흘러내림 ③ 봉지를 수직으로 놓음(빗변 평행 원함).

- **충돌 근본원인**: [stage7:2011](../stages/stage7_graspgen_e0509.py#L2011) `if args.obj_type != "snack" ...` 가 snack에서 `update_world`를 통째로 건너뜀 → cuRobo world = 책상 슬랩 cuboid 하나뿐([1774](../stages/stage7_graspgen_e0509.py#L1774)). 매대·거치대가 충돌모델에 미등록이라 `_plan_move`(plan_single)가 직선경로로 매대 관통. "회피 실패"가 아니라 "회피 대상 미등록".
- **수직 적치**: 적치 모션([2182-2185](../stages/stage7_graspgen_e0509.py#L2182-L2185))은 `side_grasp_from_approach` 수직 자세. tilt용 `_place_ee`/`_theta`(≈21°)([2162-2170](../stages/stage7_graspgen_e0509.py#L2162-L2170))는 정의만 되고 호출 안 됨 = 폐기된 tilt 접근의 죽은 코드.
- **흘러내림**: 거치대([add_snack_stand](snack_bag_module.py#L106))는 빗면 쐐기뿐, 앞쪽 받침 턱 없음. +0.15(15cm)에서 투하.

## tilt 접근 (Phase C) — 막다른 벽 회피 전략
핸드오프 막다른 벽: "bag-center 역산+tilt = 추종오차 17°·매대 통과로 폐기". 사용자가 재요청.
- 폐기 추정 원인: **마지막 하강에서만** 자세를 비틀어 IK 급변 → 추종오차 폭증.
- 사용자 제안(채택): 매대 앞 포인트(carry_above)**부터** 베이스 x축 둘레 회전 `Rx(θ)`한 일관 자세로 moveL 진입. 자세 변화가 없어 추종오차 폭증 안 함.
- 빗변 = y-z 평면 앞아래→뒤위 → 봉지 평행은 base-x 둘레 pitch가 맞음. 부호(±θ)는 side_grasp 방위 의존 → 시뮬로 확정.
- 순서: A·B 먼저(2026-06-18 사용자 결정). 받침 턱이 흘러내림을 막으면 tilt를 약하게(또는 0) 둘 수 있음 → tilt는 효과 본 뒤 약한 각도부터.

## Phase A 구현 결정
- snack도 `update_world` 호출하되 `ignore_substring`에 봉지(`/World/snack_bag`)·파티클(`snackParticleSystem`) 추가. 이유: 봉지 cloth 메시를 장애물로 잡으면 cuRobo가 자기 봉지와 충돌 판정→정지. 거치대(`/World/snack_stand`)·매대는 미포함(=회피 대상).
- snack 핸들러는 step>80에 1회 블로킹 실행이라 step50 등록분으로 충분.

## Phase B 구현 결정 (받침 턱)
- 사용자가 Isaac 에디터에서 직접 받침판을 배치하고 Transform 스크린샷 제공: pos(0.29287, 0.43078, 1.14607), Orient X=37.427°, Scale(0.11294, 0.00535, 0.01181).
- 좌표 해석: x≈거치대중심(_STAND_X 0.29), y=0.431≈거치대 앞면 y0(_STAND_Y 0.50 − depth/2 0.07 = 0.43), z=1.146≈base_z(1.14)+6mm. → `add_snack_stand` 안에서 상대좌표(cx, y0, base_z+0.006)로 재현.
- prim = `UsdGeom.Cube` size=1.0 + scale → dims=scale(폭 width, 두께 5.35mm, 높이 11.8mm), `AddRotateXOp(37.427)`, `CollisionAPI`(box 정적). 빗변 앞에 평행히 서서 봉지가 빗변 따라 내려오다 걸림.

## Phase C 구현 결정 (tilt 진입) — 사용자가 B와 동시 요청
- 사용자 지시: "매대 앞 포인트로 거치대·봉지 평행하게 간 뒤 그 상태로 매대쪽 moveL 진입".
- 구현: `_tilt_pose(xyz)` = `side_grasp_from_approach`(새 np.eye(4) 반환 → in-place 안전)에 `Rx(_TILT_DEG)`를 회전부에 곱함. ① carry_above=매대 앞(SHELF3_PRE_Y, +0.20)으로 cuRobo plan(tilt 자세) ② +y moveL로 _PY(_STAND_Y−0.06, +0.15) 진입 ③ retract도 tilt 유지.
- `_TILT_DEG = -37.427` 시작값(베이스 x축, 사용자 제안 -방향, 빗변 37.4°). 부호·크기는 라이브 보고 조정.
- 봉지중심 보정(_bagc)은 안 넣음 — 폐기된 bag-center 역산이 추종오차 17° 유발했기에. EE 회전만 단순 적용, 받침 턱이 위치 오차 흡수. 어긋나면 재검토.
