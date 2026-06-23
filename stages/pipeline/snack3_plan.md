# Stage8 봉지 통합 계획 — 3종(캔·병·봉지) 한 씬 적치 + 텍타임

목적. stage8(`stages/pipeline/stage8_main.py`, 모듈본) 골격을 그대로 쓰고, 과자봉지 적재에 필요한 것만 이식해
`--mixed`를 3종 연속 적치로 확장하고 물체별 텍타임을 계측한다. 결정·근거는 `snack3_notes.md`, 작업항목은 `snack3_checklist.md`.

## 확정된 설계 결정 (2026-06-19, 사용자)
- **적치 방식**: 거치대(`add_snack_stand`)+tilt 기대세움 (stage7 방식 이식). 봉지는 빗면에 평행하게 기댐.
- **텍타임 범위**: 한 씬 3종 연속 (`--mixed` 확장). 캔1+병1+봉지1이 순차 적치.
- **봉지 운반 메커니즘**: 사용자 아이디어 채택 — 그립 닫힘 시 **강체 프록시로 전환** → cuRobo attach → 캔/병과
  **동일한 carry/place 상태머신** 사용 → release 순간 **봉지 물성(파티클) 복귀**. stage7의 시각 캐리-보조(메시
  추종 콜백+direct IK)는 폐기. 강체 attach가 plan_single 충돌회피를 그대로 살려 carry_above 실패가 원천 소멸.

## 이식 대상 (봉지 전용만)
1. squish 파지 핸들러 — stage8에 이미 있음([1684-1768](stage8_main.py#L1684-L1768)). 그립 닫힘까지 재사용.
2. 봉지 물성 — `snack_bag_module.CLOTH_PARAMS`(cloth6 검증값). 그대로.
3. 거치대 — `snack_bag_module.add_snack_stand`(빗면 프리즘+받침턱). mixed 씬에 spawn.
4. 강체전환/복귀 토글 — **신규**(snack_bag_module에 헬퍼 추가).
5. release tilt 포즈 — stage7 `_tilt_pose`(Rx(-37.427°)) 이식. 부호 라이브 확정.

## 재사용 (stage8 골격, 손대지 않음)
- PLAN_CARRY → INSERT_SHELF → LOWER_SHELF → RETREAT_SHELF → GO_HOME 상태머신.
- `attach_external_objects_to_robot` + attached_object 링크 슬롯([1036-1049](stage8_main.py#L1036-L1049)).
- 충돌월드 갱신(`update_world`), 게인/중력보상, save_shot, --mixed 타겟 순회.

## 통합 플로우 (봉지)
```
QUERY → squish 파지(봉지 전용) → 그립 닫힘
  → [rigidify] 파티클 정지 + bag bbox 큐보이드 프록시 + attach_external_objects_to_robot
  → PLAN_LIFT → PLAN_CARRY → INSERT_SHELF → LOWER_SHELF      ← 캔/병과 공유
  → [place 분기:snack] tilt 포즈 안착 → 그리퍼 개방 → detach + soften(파티클 재활성) → 거치대 빗면 안착
  → RETREAT_SHELF → GO_HOME
```

## 단계 계획 (각 단계 독립 검증 후 다음)
- **Phase 0 — 계측 골격 + 회귀 기준**. 텍타임 마킹 유틸(물체별 grasp/carry/place/home/total) + 기존 `--mixed` 4/4 PNG 확보.
  검증: 4/4 적치 회귀 없음, 텍타임 로그 출력.
- **Phase 1 — 강체전환 토글(단독 검증)**. `snack_bag_module`에 `rigidify_bag`/`soften_bag` 추가. stage8 단독 snack
  경로를 squish→rigidify→attach→상태머신 진입으로 확장.
  검증: `--obj-type snack --place` 봉지가 거치대 앞까지 plan_single 무충돌 carry.
- **Phase 2 — 봉지 place 분기**. LOWER_SHELF/RELEASE의 snack 분기에 tilt 안착+detach+soften. mixed 씬에 거치대 spawn.
  검증: 단독 snack 봉지가 거치대 빗면에 흘러내림 없이 기댐(tilt 부호 확정).
- **Phase 3 — --mixed 3종**. mixed 레이아웃에 봉지 타겟 추가(3층 병 슬롯 ↔ 거치대 비충돌). 루프 디스패치 분기.
  검증: 캔1+병1+봉지1 연속 적치 성공.
- **Phase 4 — 텍타임 리포트**. 물체별 구간 CSV + 콘솔 요약표.

## 리스크 (인계)
- tilt 부호(-37.427°)·받침턱 효과 stage7서 미검증 → Phase 2 라이브 확정.
- attach 큐보이드 dims: 봉지 bbox보다 약간 크게(매대 회피) but 과대 시 IK_FAIL. 스윕 필요.
- 병·봉지 둘 다 3층 → 슬롯/거치대 공간 경합. 거치대 우측 고정 + 병 좌측 슬롯.
- 파티클 재활성 타이밍: release 위치가 거치대 빗면 바로 위여야 안착.
