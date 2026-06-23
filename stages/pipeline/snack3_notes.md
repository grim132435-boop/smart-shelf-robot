# Stage8 봉지 통합 컨텍스트 노트 (결정과 근거)

이어받는 세션이 결정을 다시 도출하지 않도록 기록. 계획 = `snack3_plan.md`, 작업 = `snack3_checklist.md`.

## 대상 파일 (반드시 확인)
- **정본 = `stages/pipeline/stage8_main.py`**(모듈본, `run_stage8.sh` 29행이 실행). pp_geometry/pp_motion/pp_phases import.
- `stages/stage8_motion_e0509.py` = 스테일 모놀리식(파일 2행 경고). **건드리지 말 것.**
- 봉지 적재 로직 원본 = `stages/stage7_graspgen_e0509.py`([2081-2283](../stage7_graspgen_e0509.py#L2081-L2283), 시각 캐리-보조 방식). 이식 참고용.
- 봉지 에셋/물성 = `snack_bag/snack_bag_module.py`(`CLOTH_PARAMS`, `add_snack_stand`, `spawn_snack_bag`).

## 핵심 결정 (2026-06-19)
1. **강체전환 채택(사용자)**. stage7의 시각 캐리-보조(파티클 정지+메시 추종 콜백+direct IK)는 carry_above plan_single이
   tilt 포함 EE까지 충돌회피 궤적을 못 찾아 폐기. 대신 그립 닫힘 시 봉지를 bbox 큐보이드 강체 프록시로 바꿔
   `attach_external_objects_to_robot`로 attach → 캔/병과 동일 상태머신 → release 시 파티클 재활성으로 물성 복귀.
   tilt는 plan_single 제약이 아니라 release 직전 USD 포즈로만 적용 → 충돌회피와 무관해짐.
2. **3종 연속(--mixed 확장)**. 기존 mixed=캔2+병2(4/4 검증). 회귀 보호 위해 봉지는 신규 타겟으로 추가하되 검증된
   슬롯/충돌월드 골격 재사용. 깔끔한 3종 데모는 캔1+병1+봉지1 구성 권장(3층 슬롯 1+거치대 1로 경합 최소).
3. **거치대+tilt 적치(사용자)**. 봉지를 거치대 빗면에 평행히 기대 세움. `add_snack_stand`엔 받침턱(lip, rotX 37.427°)
   이미 있음. tilt 포즈 부호(-37.427°)는 stage7서 미검증 → Phase 2 라이브 확정.

## 이식 대상 vs 재사용
- 이식(봉지 전용): squish 파지(이미 stage8에 있음)·CLOTH_PARAMS·add_snack_stand·강체토글(신규)·tilt 포즈.
- 재사용(손대지 않음): PLAN_CARRY/INSERT/LOWER/RETREAT/GO_HOME, attach 메커니즘, update_world, 게인/중력보상, save_shot, mixed 순회.

## 미해결·인계 리스크
- tilt 부호·받침턱 효과 미검증(stage7 인계). Phase 2서 확정.
- attach 큐보이드 dims: 봉지 bbox보다 약간 크게(매대 회피) but 과대 시 IK_FAIL → Phase 1 스윕.
- 병·봉지 둘 다 3층 라우팅(`_TYPE_LEVEL`). 슬롯 vs 거치대 공간 경합 → 거치대 우측 고정, 병 좌측 슬롯.
- 파티클 재활성 타이밍: release 위치가 거치대 빗면 바로 위여야 안착(높으면 흘러내림 — stage7 관찰).

## Phase 1 gotcha (2026-06-19 라이브)
- **추종·강체전환은 반드시 리프트 전(그립 직후)에 걸 것.** 1차 시도서 리프트를 cloth 상태로 먼저 하고 추종을
  나중에 걸었더니 봉지가 슬립→리프트된 EE 기준으로 스냅샷돼 공중부양·좌표 어긋남·매대 관통. stage7 순서(그립 직후
  파티클정지+W0스냅샷+follow콜백 → 그 다음 리프트)로 바로잡음. 봉지가 EE 로컬프레임 고정점으로 리프트·carry 내내 추종.
- 거치대(`add_snack_stand`)는 stage8 단독 snack엔 아직 미spawn(Phase2서 추가). stage7엔 spawn돼 있음.

## Phase 2 결정 (2026-06-19 라이브)
- **tilt 진입 폐기.** Rx(-37.427°) tilt 포즈가 손목 IK 도달불가(사용자 관찰). PRE→PLCE 표준 자세 +y 직선
  moveL 진입으로 변경. 봉지 빗면 평행은 거치대 받침턱(lip)이 흘러내림 막아 대체. `_tilt_pose`/`_TILT_DEG` 제거.
- 적치 시퀀스: carry(PRE 표준)→ +y moveL(PLCE 표준)→ 추종해제+detach+그리퍼개방+`soften_bag`(파티클재활성)→ 후퇴(PRE).

## Phase 3 gotcha (2026-06-19 라이브)
- **mixed 캔 적치 실패 cascade.** GPU dynamics(cloth용) 켜진 mixed에서 캔이 파지 중 옆으로 회전 → 길어진 캔이
  매대 release 부분개방(GRIP_RELEASE 0.35≈81mm)으로 안 빠짐 → 그리퍼에 물린 채 매대 왕복 → 물린 캔 prim이
  update_world에 장애물로 등록 → 다음 타겟(병) GraspGen/IK 전부 거부(빨강) → 병 스킵 → 봉지로. 봉지 attach 시
  잔존 attach와 충돌 위험(방어적 detach로 보강).
- 조치: mixed는 매대 release를 **풀오픈(GRIP_OPEN)** — 1물체/슬롯이라 이웃 없어 안전, 회전 캔도 떨굼. 봉지 attach 전 detach.
- 근본 확정(2026-06-19): **마찰파지가 GPU dynamics와 안 맞음.** 위치드라이브가 effort cap(25N)까지 강체를 압착 →
  GPU에서 손가락 SDF에 캔이 침투·welded → 풀오픈해도 안 빠짐. 파지각 조절은 정상상태 압착력 동일이라 무효.
- 운동학 고정 파지 시도 → **폐기**(2026-06-19). 닫기 후 캡처라 회전캔 그대로 운반 + kinematic 캔에 그리퍼 박혀
  모션 요동. 사용자 "물리 파지 복원" 선택.
- **진짜 원인 규명(사용자 질문 계기).** baseline 4/4가 됐던 건 GPU dynamics OFF였기 때문(게이트 `args.obj_type==snack`,
  mixed서 False). Phase3에서 봉지 cloth 위해 `or args.mixed`로 GPU dynamics ON → **그게 강체 파지를 깨뜨린 단일 원인.**
  유력 메커니즘: 손가락 SDF 충돌이 GPU(TGS) 솔버서 관통(CPU PGS선 안정).
- **해결(물리 파지 복원 + GPU 호환):** ① 운동학 고정 전부 revert(적응형 close + GRIP_RELEASE 복원) ② mixed는
  `set_finger_sdf_collision` 생략 → 손가락 convexHull 유지(GPU 안정) ③ 로봇 articulation 솔버 iteration↑(pos64/vel8).
  미검증: SDF가 정말 범인인지 라이브 확인 필요. 부족하면 can/bottle 강체에도 iteration↑.

## 검증 명령
- 회귀: `bash stages/pipeline/run_stage8.sh --mixed --place`
- 단독 봉지: `bash stages/pipeline/run_stage8.sh --obj-type snack --place --no-graspgen`
- 종료: `touch /tmp/stage7_stop` (STOP_FILE) 또는 kill -9.
