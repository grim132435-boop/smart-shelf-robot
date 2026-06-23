# 과자봉지 적치 개선 체크리스트 (2026-06-18~)

진단: 핸드오프 [HANDOFF_snack_pickplace.md](HANDOFF_snack_pickplace.md). 결정 근거는 [context-notes.md](context-notes.md).

## Phase A — 충돌회피 복구 (1순위: "매대랑 박는다")
- [x] snack 경로에서 `update_world` 활성화 (봉지·파티클시스템 ignore, 매대·거치대 포함)
- [ ] 실행 → `[충돌월드] cuRobo 장애물 N개` 로그에 매대 mesh 뜨는지 확인
- [ ] 라이브 관찰: carry_above/carry_in 중 그리퍼·팔이 매대와 안 박음

## Phase B — 거치대 앞 받침 턱 (흘러내림 방지)
- [x] `add_snack_stand`에 빗변 앞-아래 끝(y0) 직육면체 턱 추가 (사용자 에디터값 재현: rotX 37.427°, 폭×5.35mm×11.8mm)
- [ ] 라이브 관찰: 봉지가 거치대에서 안 흘러내리고 턱에 걸림

## Phase C — 빗변 평행 tilt (사용자 요청으로 B와 동시 구현)
- [x] carry_above(매대 앞 PRE_Y)부터 `Rx(_TILT_DEG)` 일관 자세 → +y moveL 진입 (마지막만 비트는 폐기방식 회피)
- [x] retract도 tilt 자세 유지 (매대 안 자세 급변 방지)
- [x] 죽은 코드 `_place_ee`/`_theta`/`_bagc` 정리
- [ ] 회전 부호·각도 시뮬 확인 (_TILT_DEG=-37.427 시작값, 라이브 보고 부호/크기 조정)
- [ ] 라이브 관찰: 빗변 평행 안착, 추종오차·매대 통과 없음
