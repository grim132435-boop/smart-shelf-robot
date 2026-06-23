# Stage8 체크리스트 — 다물체 파지 우선순위 개선

상세 배경·위치·실행법은 `context-notes.md` 참조. 대상 파일 `stages/pipeline/stage8_main.py`.
원칙: 한 단계 구현 → Isaac 검증 → 다음. 라인번호는 편집하며 밀리니 **grep으로 재확인**할 것.

## 착수 전
- [ ] `context-notes.md` 읽기 (현재 검증된 동작·파라미터·gotcha).
- [ ] 베이스라인 회귀 기준 확보: `--mixed --place` 4/4 1회 돌려 PNG 확인(개선 후 비교용).
- [ ] (권장) `--seed` 인자 추가 → DR 결정적 재현(전/후 동일 씬 비교용). `np.random.seed(args.seed)`.

## Phase 1 — 보류-재시도 (non-Markov, 핵심)
- [ ] 상수 `MAX_TARGET_TRIES = 3` 추가.
- [ ] targets dict 초기화에 `"tries": 0` 추가 (line ~1206, ~1250, ~1345).
- [ ] 타겟 선택 키 변경: `_tgt_key` → `(tries, y, x)` (덜 시도한 것 우선, 동률 periphery). line ~1838-1844.
- [ ] 실패 6경로에서 `status="skipped"` 대신 **`tries+=1; status` 유지(pending)**, `tries>=MAX`일 때만 `skipped`(reason). 위치:
  - [ ] 1959 `no_grasp`
  - [ ] 2019 `unreachable`
  - [ ] 2194 `unreachable`
  - [ ] 2227 `lift_plan_fail`
  - [ ] 2363 `no_slot_runtime`
  - [ ] 1818 `no_slot` (IDLE 일괄) — 매대 만석은 재시도 무의미하니 정책 결정(즉시 skip 유지 권장).
- [ ] 무한루프 방지 확인: 모든 pending이 MAX 도달하면 `_pend` 비어 HALT 됨.
- [ ] 컴파일 `python3 -m py_compile stage8_main.py`.
- [ ] 검증: `--mixed --dr` 동일 시드 전/후 비교 — 적치수↑ 또는 "초기 unreachable→재시도 적치" 로그 1건↑.
- [ ] 회귀: `--mixed` 4/4 유지.
- [ ] 커밋 (한 문장 요약 가능 단위).

## Phase 1.5 — HITL GUI 에스컬레이션 (사용자 아이디어)
- [ ] **착수 전 사용자 확인**: 시뮬 시연용 vs 실배포 HMI 설계.
- [ ] 종료조건(모든 pending 소진)에서 HALT 대신 "사람 개입 필요" 표시.
- [ ] 사유 분류: `unreachable`(범위밖) / `ungraspable`(클러터간섭) / `no_slot`(만석) → 메시지 다르게.
- [ ] v1 = 방법 B(뷰포트 배너 + `/tmp/stage8_rescan` 센티널 대기). 사람 정리 후 touch → `tries` 리셋 + 월드 재동기화 + IDLE 재스캔.
- [ ] (후속) 방법 A omni.ui 팝업+[재시도] 버튼으로 업그레이드.
- [ ] (실배포) 오퍼레이터 HMI 이벤트 인터페이스(팀레포 ROS) 매핑 — 별도 협의.
- [ ] 검증: 겹쳐 못 잡는 씬 구성 → GUI 뜸 → 수동 정리 후 재시도 → 적치 완료.

## Phase 2 — 정밀화 (Phase1 후 부족하면만)
- [ ] 실패 grasp 반복 금지: 재시도 시 직전 실패 goalset 후보 배제(SFO 회피).
- [ ] 도달성 2순위 키: 근소차일 때 IK 도달 후보 많은 것 우선(Dex-Net). 기존 IK필터 재사용.

## 하지 않을 것
- [ ] 최적순서 solver / RL 순서학습 (NP-hard·과설계).
- [ ] dense bin-picking 정책 통째 이식 (우리는 포즈 알려진 소수 구조적 적치).
