# 시뮬 3종 통합 — 이슈 · 목표 · 해결방안 플랜

작성 2026-06-22. 대상 `stages/pipeline/stage8_main.py` + `snack_bag/snack_bag_module.py`. 관련 노트 `context-notes_snack_unify.md`.

## 목표
- 캔+병+봉지 **3종을 한 씬에서 깨끗하게** 픽앤플레이스(적치) — `--mixed --place`.
- 3물체 **텍타임**(grasp/carry/place/home/total) 측정 → **실기 텍타임과 비교**(sim↔real).
- (상위) 실기 배포.

## 이미 해결됨
- **봉지 거치대 적치** — 하이브리드(cuRobo plan_single 매대앞 도달 + moveL 칸 안 틸트 진입·하강) + 동결유지(soften 금지, snack_hold) + −y 직선 후진→plan_single_js home. 빠른 그리퍼(증분0.10/8/한계0.70)·빠른 진입(90/12). **단독 snack 안정.**
- **캔/병 단독 4/4** (CPU 물리).

---

## 미해결 이슈

### 이슈1 — 캔 welded/관통 (혼합 모드)
- **원인(코드 확정, L1567-1583)** — `--mixed`는 봉지 particle cloth 때문에 **GPU dynamics ON** → **GPU TGS 솔버 + SDF 손가락 = 강체 캔 관통·welded(붙음)**. 단독 캔/병은 CPU라 없음. **모듈화 무관 / 봉지 추가로 GPU 켜진 게 원인.** (주석에 "GPU(TGS)서 SDF 손가락이 강체를 관통" 명시)
- **현재** — 로봇 articulation(L1579) + 캔 강체(L1273-1275) 둘 다 솔버 iter 64/8 적용했으나 **불충분**.
- **영향** — 캔 파지 재시도 루프(GraspGen 재질의 반복) → 텍타임 부풀림.
- **해결 후보** — (a) 손가락/캔 `contact_offset`·`rest_offset` 키워 접촉 조기 해소(미설정 상태), (b) 강체 파지 시 SDF→convexHull 손가락(SDF는 soft wrap용), (c) SDF resolution↑(현재 256), (d) 시뮬 아티팩트로 인정(실기 무관).
- **검증** — 형 라이브 관찰(관통/회전 없음) + 로그 재시도 0회.

### 이슈2 — 봉지 폭발 (혼합 모드에서만)
- **원인** — cloth 불안정(혼합서만). **GPU 버퍼↑ 효과 없음**(버퍼 초과 아님). 봉지가 캔/병 처리되는 긴 시간 cloth로 떠 있다 솔버 불안정 누적 의심.
- **해결 후보** — (a) **봉지 차례 전까지 파티클 동결(off), 잡을 때만 on** → 떠 있는 시간 제거(폭발 원천차단, 추천), (b) 폭발 타이밍 확인 후 표적 수정.
- **미확인(형 관찰 필요)** — 폭발 **타이밍**: 스폰 직후? 캔/병 잡을 때? 로봇이 봉지 옆 지날 때?
- **검증** — 형 관찰(안 터짐).
- **주의** — 봉지 물성 자체는 형 다른 페이지 담당. 여기선 씬/타이밍(코드)만.

### 이슈3 — 텍타임 측정·비교
- **인프라** — `_TACT`(TactTimer, L286): 물체별 grasp/carry/place/home/total + CSV(`logs/tact_*.csv`). 준비됨.
- **블로커** — 이슈1(재시도)·이슈2(폭발)가 깨끗한 측정 방해(텍타임 왜곡).
- **실기** — 실기 노드(curobo_planner_node)에 **동일 _TACT 단계 경계**(grasp→carry→place→home→done) 이식 → 같은 구간 비교. 실기는 movesj 실행+통신지연 포함이라 sim보다 길게 나옴.

---

## 우선순위 / 순서
1. **이슈2 봉지 폭발** → 동결 방식(폭발 원천차단). [형: 폭발 타이밍 관찰]
2. **이슈1 캔 welded** → contact_offset 조정 또는 강체용 convexHull 손가락. [형: 관통/회전 관찰]
3. **3종 클린 런** → 텍타임 측정(시뮬 베이스라인) + CSV.
4. **실기 노드 _TACT 이식** → 실기 텍타임 → sim↔real 비교.

## ★중요 — 시뮬 vs 실기
- 이슈1(캔 관통)·이슈2(봉지 폭발)는 **시뮬 물리 아티팩트라 실기엔 전이 안 됨**(실기는 실강체·실소프트). 깨끗한 **시뮬 데모/텍타임**엔 필요하나, **실기 배포 자체엔 무관**. → 깨끗한 sim 텍타임 비교가 목적이면 1·2 해결 필요. 실기 배포가 급하면 건너뛰어도 됨.

## 실행/검증 규칙
- 실행 `nohup bash /home/devuser/shelf_grasp_dev/stages/pipeline/run_stage8.sh --mixed --place > logs/X.log 2>&1 & disown` (절대경로 — cd 누락 시 run_stage8.sh 못 찾음). GraspGen 서버(PID 8436대) 생존 확인.
- 종료 `pkill -9 -f 'stage8_main[.]py'`(단독 줄). 컴파일 먼저.
- ★시뮬 거동은 **Claude가 PNG로 판단 금지** — 형이 보고 알려줌. Claude는 코드+로그 수치만. [[sim_ask_dont_judge]]
