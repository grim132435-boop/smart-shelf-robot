# Stage8 봉지 통합 체크리스트 — 3종 적치 + 텍타임

계획 = `snack3_plan.md`, 결정 근거 = `snack3_notes.md`. 대상 = `stages/pipeline/stage8_main.py` + `snack_bag/snack_bag_module.py`.
원칙: 한 Phase 구현 → Isaac 검증 → 다음. 라인번호는 편집하며 밀리니 grep으로 재확인.

## 착수 전
- [ ] `snack3_notes.md` 읽기. 다른 창 stage8/stage7 인스턴스 없는지 확인(Isaac Run Rules).
- [ ] 회귀 기준: `bash run_stage8.sh --mixed --place` 4/4 1회 → PNG 확보(이식 후 비교).

## Phase 0 — 텍타임 계측 골격
- [x] 단계 타임스탬프 유틸 추가(`TactTimer`, grasp/carry/place/home/done 마킹). stage8_main.py:267.
- [x] 종료/HALT 시 콘솔 요약표 + `logs/tact_*.csv` 기록(`_TACT.report()`).
- [x] 검증: 기존 `--mixed --place` 4/4 회귀 없음 + 텍타임 요약표 출력 (2026-06-19 사용자 "문제 없어").

## Phase 1 — 강체전환 토글(단독 검증)
- [x] `snack_bag_module.rigidify_bag` → 파티클 정지 + 월드 AABB(center,dims) 반환.
- [x] `snack_bag_module.soften_bag` → 파티클 재활성.
- [x] stage8 snack 경로: squish 그립 닫힘 후 rigidify → Cuboid 프록시 `attach_external_objects_to_robot` → EE 추종 콜백 → carry(plan_single). + snack도 `update_world`(매대 등록).
- [x] 큐보이드 dims 봉지 bbox×1.1(시작값). IK_FAIL 시 ×1.0~1.05로 하향 스윕.
- [ ] 검증: `--obj-type snack --place` 봉지가 매대 앞(PRE)까지 plan_single 무충돌 carry(`snack_06_carry` PNG 확인).

## Phase 2 — 봉지 place 분기(거치대+tilt+물성복귀)
- [x] 단독 snack 씬에 `add_snack_stand` spawn(3층 우측 0.31/0.53, stage7값 재사용).
- [x] snack 분기: tilt reorientation(PRE) → tilt moveL(PLCE) → 추종해제 + detach + 그리퍼 개방 + `soften_bag`(파티클 재활성) → 후퇴.
- [x] `_tilt_pose`(Rx(-37.427°)) 이식.
- [x] 검증: 단독 snack 봉지가 거치대 빗면에 같은 기울기로 안착(2026-06-19 사용자 확인). PRE·PLCE 동일 tilt·회전없는 +y moveL, z1.35/y0.57, 받침턱 Z 0.02892로 키움.

## Phase 3 — --mixed 3종
- [x] mixed 레이아웃을 캔1(2층)+병1(3층)+봉지1(3층 거치대)로 변경(DR은 기존 4개 유지).
- [x] 3층 슬롯 비충돌: 병=좌측 슬롯(0.165,0.56) 1곳, 봉지=거치대(0.31,0.53). GPU dynamics mixed서 활성화.
- [x] IDLE 디스패치: snack 타겟이면 강체 안정화/QUERY 건너뛰고 top-of-loop squish 핸들러로(게이트 obj_type 기준). 핸들러 종료 시 placed 마킹+IDLE 복귀.
- [x] update_world: 봉지·파티클 항상 ignore(거치대는 장애물 유지).
- [ ] 검증: 캔1+병1+봉지1 연속 적치 성공(PNG + 텍타임 3행).

## Phase 4 — 텍타임 리포트
- [ ] 물체별 구간 CSV + 콘솔 요약표(캔/병/봉지 grasp·carry·place·home·total).
- [ ] 검증: 3종 1런 결과표 사용자 보고.
