# 체크리스트 — 과자봉지를 캔/병 모션경로로 통합 (실기 배포 준비)

배경·결정·gotcha는 `context-notes_snack_unify.md` 참조. 대상 `stages/pipeline/stage8_main.py` + `pp_geometry.py` + `snack_bag/snack_bag_module.py`.
원칙 — 한 Phase 구현 → Isaac 검증(로그+PNG, 사용자에게 라이브 확인) → 다음. 라인번호는 편집하며 밀리니 grep 재확인.

## 한 줄 목표
봉지는 **GraspGen 미사용**(Phase1서 side만 줌 확정) → **봉지 좌표로 top-down 직접 파지**. 소프트 처리(파지후 rigidify / 적치시 soften)는 유지. **남은 핵심 = 적치 충돌/투하 해소** — carry/place를 검증된 캔/병 시퀀스로. 실기 입력 = PC 아닌 봉지 좌표(ZYZ[0,180,yaw]).

## Phase 1 — GraspGen on 봉지 검증 [완료 2026-06-19]
- [x] 결과 — 후보 100개 score0.92~0.96이나 **전부 side(top 0개)** → 봉지 top-down엔 부적합. **GraspGen 미사용 결정.** 스파이크/PC/pillow 헬퍼 원복(제거).

## 착수 전
- [ ] `context-notes_snack_unify.md` 읽기.
- [ ] 베이스라인 회귀 기준 확보 — `--obj-type snack` 현행 핸들러 1회 돌려 PNG 확보(전/후 비교용).
- [ ] 현 snack 핸들러 블록 위치 재확인 — stage8_main.py L1796~1968 (`if obj_type == "snack":` … `continue`).

## Phase 2 — 파지 = 봉지 좌표 top-down (GraspGen 미사용, 현 핸들러 유지)
- [x] 결정 — `_ee_side`(approach -Z, 닫힘축=월드X 폭압축) 좌표기반 top-down 유지. 봉지 중심 (cx,cy)+yaw가 입력. close=폐루프 스퀴즈+rigidify 유지.
- [ ] (실기 매핑) 봉지 좌표를 외부(비전 pick_pose)에서 받도록 인터페이스 정리 — 시뮬은 spawn 좌표 사용.

## Phase 3 — 리프트/운반/적치/홈을 검증된 캔/병 시퀀스로
- [ ] 커스텀 lift/carry/tilt-place/retreat(L1911~1954) 폐기 → 캔/병 FSM(PLAN_LIFT→MOVE_LIFT(attach)→PLAN_CARRY→INSERT(+y moveL)→LOWER(-z moveL)→release→RETREAT(-y moveL)→HOME) 적용.
- [ ] attach = rigidify_bag의 AABB로 Cuboid 프록시(기존 코드 재사용). release 시 detach + `soften_bag`.
- [ ] snack_follow(수동 추종) — 물리 그립이 강체화 봉지를 운반하면 제거. 슬립하면 잠정 유지(test-driven, context-notes에 근거).
- [ ] tilt 적치 폐기 — 봉지는 거치대에 캔/병처럼 직립/안착(거치대 빗변 평행은 후속).
- [ ] 검증 — `--obj-type snack` 픽→3층 적치 완주, **그리퍼-매대 충돌 0**(HANDOFF 미해결 이슈 해소 확인). 사용자 라이브 확인.

## Phase 4 — 회귀
- [ ] `--mixed`(캔2+병2) 4/4 회귀 유지(snack 변경이 캔/병 경로 안 깸).
- [ ] `--mixed` 3종(캔+병+봉지) — 봉지가 통합 경로로 적치되는지(보너스).
- [ ] 커밋(한 문장 단위).

## Phase 5 — 실기 배포 매핑(문서)
- [ ] rigidify/soften = **시뮬 전용** 명시(실봉지는 물리적 소프트). 실기 전이 = 모션 궤적 + grasp_class=snack_bag→300mA.
- [ ] 실기 실행 = **movesj 단일호출**(관절궤적) / Cartesian 금지 / interpolation_dt=두산RT주기 (스터터 대응, context-notes 참조).

## 하지 않을 것
- [ ] 봉지 물성 튜닝(사용자 다른 페이지 담당).
- [ ] FEM 전환(cloth 확정).
- [ ] tilt 빗변 평행 안착 재시도(추종오차 17°·매대통과로 폐기 이력 — 필요 시 별도).
