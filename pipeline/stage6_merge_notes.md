# Stage5 ↔ Stage6 코드 병합 가이드 (Stage7 진입 전)

작성 2026-06-12. 세 파일(stage4/5/6_graspgen_e0509.py)의 실측 diff 기반.

## 결론 한 줄

**stage5를 베이스로 삼고, stage6 고유분 2개만 가져오면 끝.** 단방향 병합이라 충돌 없음.
공유 코드(SDF 손가락·마찰·curl·TCP)는 두 파일에서 **바이트 단위로 동일** → 손댈 것 없음.

## 왜 단방향인가 (파일 계보)

- stage4 = 동결 베이스. **이미 Stage6 선행분(SDF B1 / 마찰 max B2 / curl 로깅)을 포함**.
- stage5 = stage4 사본(2026-06-12) + **추종 정밀화 + 다물체**.
- stage6 = stage4 사본(2026-06-12) + **부분열림 릴리즈 + --gripper-test**.
- 검증: 공유 상수 `RHP12_TCP_DEPTH` / `resolution=256` / `static_friction` / `GRIP_CLOSE` / `GRIP_OPEN`
  전부 두 파일 일치. `set_finger_sdf_collision`·`FrictionCombineMode max`·curl 로깅도 양쪽 동일.
- 함수 레벨 diff: stage6 고유 함수 **0개**. stage5 고유 함수 6개(아래 "stage5가 더 가진 것").

## stage5가 이미 더 가진 것 (stage6엔 없음 — 그대로 둠)

- 중력보상: `enable_gravity_comp` (j2 추종 4.55°→0.17°)
- 간격 로깅: `min_world_clearance` / `make_clearance_fn` / `_clr_probe` / `_clr_report`
- moveL settle=0 (핸드오프 블렌딩)
- 다물체: `slot_feasible`, `--objects`/`--obj-gap`, `SHELF3_SLOTS`(2열), targets 오케스트레이션,
  매대측 우선 순서, 스킵+사유 로깅

## stage6에서 stage5로 가져올 것 — 딱 2개

### 1) [필수] 매대 안 부분열림 릴리즈 (GRIP_RELEASE) — 다물체에서 더 중요

풀오픈(0.0)은 근위바가 좌우로 벌어져 측벽·이웃캔과 간섭. 캔(66mm)만 놓을 만큼만 연다(81.5mm).
**다물체 모드는 매대에 기적치 캔이 있어 이 간섭이 실제로 발생** → 반드시 반영.

**(a) 상수 추가** — stage5 `GRIP_CLOSE = 1.05` 아래에 삽입:
```python
# 매대 안 릴리즈용 부분 열림(Stage6): 풀오픈(0.0)은 패드 간격 107mm + 근위바 좌우 스윙으로
#   매대 측벽/이웃캔 간섭 위험 → 캔(66mm)만 놓을 만큼만 연다.
#   내면 간격 = 2*(0.008 + 0.0494cos q − 0.0285sin q − 0.0039) → q=0.35에서 81.5mm(캔+양측 7.7mm).
GRIP_RELEASE = 0.35
```

**(b) 릴리즈 호출 교체** — stage5 **line 2228** (LOWER_SHELF 안착):
```python
# 변경 전 (stage5)
set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=40)   # 캔 안착
# 변경 후 (stage6 방식)
# ★매대 안에서는 부분 열림(GRIP_RELEASE) — 풀오픈은 근위바가 좌우로 벌어져 측벽/이웃캔 간섭
set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_RELEASE, steps=40)   # 캔 안착
```
P5 로그 문구도 stage6처럼 "부분열림(...rad≈간격81mm, 매대간섭 회피)"로 바꾸면 일관.

> ⚠️ **절대 `GRIP_OPEN`을 일괄 치환하지 말 것.** stage5에 `set_gripper(..., GRIP_OPEN, ...)` 호출이
> 9곳 있는데 **딱 한 곳(매대 안 안착)만** GRIP_RELEASE로 바뀐다. 나머지 8곳은 책상쪽/재시도 릴리즈라
> 풀오픈이 맞다(부분열림으로 바꾸면 책상 위 재파지·해제가 망가짐). 바꿀 한 줄은 **`# 캔 안착` 주석이
> 달린 그 줄(현재 line 2228)** 유일 — 이 주석 문자열로 정확히 식별해 그 줄만 교체.
> 유지해야 할 GRIP_OPEN 8곳: pre-grasp 열기(~1728), plan_grasp 실패 해제(~1850), 다물체 grasp_slip/
> lift_plan_fail 해제(~1972·2007·2074·2089), RELEASE 단일모드 해제(~2109). (라인번호는 병합 중 밀릴 수
> 있으니 숫자 말고 맥락·주석으로 식별.)

주의: RETREAT_SHELF 후퇴는 그리퍼가 이미 부분열림 상태로 빠져나옴 → 후퇴 경로가 캔을 다시 안 건드리는지
첫 런에서 [간격:moveL -y후퇴] 확인(현재 stage5 풀오픈 기준 +5~47mm 여유라 부분열림이면 더 안전).

### 2) [선택] --gripper-test 진단 모드

빈손 open→풀클로즈 형상·각도를 실물 사진과 대조하는 1회성 진단(Stage6 링크구조 판별에 썼던 것).
운영 파이프라인과 무관(게이트 `if args.gripper_test`)하니 **유지해도 무해, 버려도 무방**.
가져온다면: `--gripper-test` 인자(stage6 line 32) + `_gtest_done` 플래그(line 1336) +
판별실험 블록(line 1380~, `_glog`/`_gcam`) 3덩어리를 stage5 대응 위치에 복사.

## 충돌 없음을 보장하는 근거

- diff 헝크 대부분은 stage5가 **추가한** 다물체/추종 코드라서 stage6에 "없는" 것(삭제처럼 보임) — 정상.
- stage6의 `>` 추가 라인 중 코드는 위 2개 블록 + 헤더 주석뿐. 공유 함수 본문 변경 0.
- 따라서 **stage5 파일에 위 (1)·(2)만 얹으면 = 완전한 병합본**. 별도 3-way 머지 불필요.

## 병합 후 Stage7 진입 체크

1. 병합본으로 `--place --objects 3` 1회 — 부분열림으로도 직립 z=1.208 유지 확인.
2. 4캔 스킵경로 수정분(리프트 실패 재선별 / RELEASE 재배치 금지) 재검증 — `--place --objects 4`.
3. 파일명: 병합본을 stage7 베이스로 승격(stage7_graspgen_e0509.py 권장) + run_stage7.sh / /tmp/stage7_stop 분리.
