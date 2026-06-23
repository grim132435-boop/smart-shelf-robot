# Stage8 컨텍스트 노트 (다른 Claude 세션 인계용)

작성 2026-06-18. 동일 워크스테이션의 다른 Claude가 바로 이어가기 위한 결정·근거·위치 기록.

> ## ⚠️ Stage8은 두 케이스 (반드시 확인)
> 모듈 분리하며 **모놀리식 / 모듈** 두 케이스로 진행됨(stage7 모듈화와 동일 패턴).
> - **모듈 케이스 = `stages/pipeline/stage8_main.py`** — `pp_geometry/pp_motion/pp_phases` import(헬퍼 분리). **`run_stage8.sh`(29행)가 실행하고, 이번 세션 4/4 검증·모든 수정(standoff0.15·-y이탈·2층hfrac·클리어리프트)이 들어간 최신본. → 작업은 여기서.**
> - **모놀리식 케이스 = `stages/stage8_motion_e0509.py`** — pp_ 미사용, 헬퍼 전부 인라인(단일 자립 파일). 어떤 러너도 미참조. **★이번 세션 수정 미반영 = 스테일.** 유지하려면 모듈본 수정을 수동 동기화해야 하고, 안 쓸 거면 retire 결정 필요(미정).
>
> 헬퍼: `pp_geometry.py`(순수)·`pp_motion.py`·`pp_phases.py`. 원본 `stages/stage7_graspgen_e0509.py`(Stage7 DR본)는 별개로 보존.

---

## 0. 지금 상태 (검증됨, 건드리면 회귀 주의)

- `--mixed --place` (캔2→2층 / 병2→3층 고정) → **4/4 직립 적치**.
- `--mixed --dr --place` (위치·서/눕·yaw 무작위) → **4/4 직립 적치**.
- 검증 로그: `logs/stage8_yexit_*.log`, `logs/stage8_drcheck_*.log`. 시점 PNG: `logs/shots/`.

### 검증된 동작 시퀀스 (FSM, 이 순서가 정답 — 효율화로 단계 빼면 터짐)
파지 → **PLAN_LIFT(+z 0.12 클리어리프트)** → MOVE_LIFT(attach) → PLAN_CARRY(매대 앞 plan_single 회피) → **INSERT(+y moveL)** → **LOWER(-z moveL 하강)** → release(부분개방) → **RETREAT(-y 직선 moveL, +z 생략)** → GO_HOME(plan_single_js, 매대 밖서 시작).

### 검증된 핵심 파라미터/결정 (코드 주석에 [복원]/[P6]/★ 마커)
- `PREGRASP_STANDOFF = 0.15` (cuRobo 기본). 0.04은 pregrasp가 파지점 4cm 앞이라 실제 메시 그리퍼가 톨보틀 접촉(off-center 밀어냄) + pregrasp IK_FAIL. 0.15면 접근 무충돌·도달성↑.
- **2층(천장 SHELF_CEIL=1.11) 파지높이: `_hfracs`에서 위쪽(+) 후보 제외, 중심·아래만** (line ~665). 선 캔을 GraspGen이 중심+39mm 높게 잡으면 TCP↑ → 손목이 천장 박음. grasp_frac=0.
- 병 안착: **직행안착 금지, -z 수직 하강(P4)이 직립의 핵심** (직행은 비스듬 도착→기움).
- 이탈: **-y 직선 moveL만(+z 생략)** — 사용자 승인. 부분개방 그리퍼(간격81mm > 캔60·병70mm)라 안 끌림. home은 매대 밖(PRE_Y)서 시작해야 적치물/매대 안 침.
- attach: `robot_cfg`에 `attached_object` 링크(extra_collision_spheres 100) 필수 — 없으면 조용히 False → 든 물체 무인지 → 매대에 박음. (line ~1028-1041)
- 병 컨투어 메시: mass 600g, 바닥 평평·넓게(r0.0335) — `pp_geometry._BOTTLE_PROFILE`.
- 리프트 전 대기 12스텝(45→12 단축).

---

## 1. 다음 작업 = 다물체 파지 우선순위 개선 (플랜은 checklist.md)

### 현재 우선순위 (변경 대상)
- 타겟 선택: `stage8_main.py` **line ~1834-1844**, `_tgt_key = (y, x)` → `cur_tgt_i = min(_pend, key=_tgt_key)`. 매대 먼쪽(y작은)+로봇 가까운(x작은) = periphery-first. **정적 기하**.
- 실패 시 **즉시 영구 skipped**. 스킵 지점 6곳(grep `"skipped"`):
  - 1818 `no_slot` (IDLE 사전검사, pending 일괄)
  - 1959 `no_grasp` (GraspGen 빈 결과)
  - 2019 `unreachable`
  - 2194 `unreachable`
  - 2227 `lift_plan_fail` (PLAN_LIFT)
  - 2363 `no_slot_runtime` (PLAN_CARRY, 잡고나서 슬롯없음)
- targets 정의: line 1125, append 1206·1250, 단일 1345. dict 키 `{obj, path, status:pending|placed|skipped, reason, (혼합)type/spec/level/frac}`.
- 종료: IDLE `_pend` 비면 `_idle_done` → HALT (line 1819-1832).

### 평가 (웹 근거 — 단정 말고 확인된 사실)
- **periphery-first는 문헌상 옳음** (Tang&Yu: 최적순서 NP-hard라 휴리스틱이 정답 / 주변부부터 치워 연쇄붕괴 최소화). 큰 그림 OK.
- 약점 = **정적, 실패이력 무시**. 개선은 "보류-재시도"(non-Markov)가 ROI 최고.
- NVIDIA Isaac Manipulator도 정교한 순서 없이 도달/충돌 grasp 필터(우리 slot_feasible+IK필터)에 의존 → 우리 구조는 정석에 부합.
- 근거: Goldberg/Danielczuk Non-Markov(실패 기억→재배치 MPPH+107%, arxiv 2007.10420), Tang&Yu(1905.13530), Dex-Net, periphery(arxiv 2603.02511).

### 사용자가 추가한 아이디어 — HITL GUI (Phase 1.5)
실 편의점 매대는 상품이 겹쳐 못 잡는 경우가 있음 → 무한시도/조용한 정지 대신 **"상품 겹침, 흐트려 놓고 재시도" GUI 창**을 띄워 사람 개입 요청(실 배포 정석 HITL). Phase 1의 "모든 pending 소진" 상태가 트리거.
- **트리거 정확도 중요**: 사유 구분(`unreachable`=범위밖 / `ungraspable`=클러터간섭 / `no_slot`=만석)해서 메시지 다르게.
- 구현 옵션: A) omni.ui 팝업+버튼(뷰포트 headless=False라 가능, 버튼콜백 복잡) / B) 뷰포트 배너+`/tmp/stage8_rescan` 센티널 재개(견고한 v1). 실배포는 오퍼레이터 HMI(팀레포 ROS 이벤트)로 매핑.
- 사용자 미정: 시뮬 시연용인지 실배포 HMI 설계까지인지 → 착수 전 확인할 것.

---

## 2. 실행/검증 방법 (중요 — 안 지키면 셸이 죽거나 GPU OOM)

### 실행
```
cd ~/shelf_grasp_dev/stages/pipeline
rm -f /tmp/stage7_stop
nohup bash ./run_stage8.sh --mixed --place > ~/shelf_grasp_dev/logs/stage8_X.log 2>&1 &
disown
```
- DR 검증: `--mixed --dr --place`. 컴파일 먼저: `python3 -m py_compile stage8_main.py`.
- 결과는 PNG로 확인: `logs/shots/shot_*_placed.png`, `*_place_done.png` (창이 사용자에게 안 보일 수 있음 → Read로 PNG 확인).

### 종료 (★셸 자기살해 주의)
- `set +e` 쓰고, pkill은 **단독 줄**에서 브래킷 트릭: `pkill -9 -f 'stage8_main[.]py'`.
- `py_compile ...py`(평문 파일명)와 pkill을 **같은 명령에 두지 말 것** (pkill이 자기 명령줄 매칭 → 셸 죽음, exit 137). 메모리 [[shell_pkill_selfkill]] 참조.
- graceful: `touch /tmp/stage7_stop` (로그경로는 `/tmp/stage7_logpath`에 기록됨).

### GPU
- RTX 5080 16GB. GraspGen 서버 PID(11661, ~0.5GB)는 **죽이지 말 것**(파지 추론 ZMQ 5556).
- ★사용자가 **다른 페이지에서 snack 런(stage7 --obj-type snack)**을 돌릴 때가 있음(~6.9GB). 그건 **사용자 작업이라 절대 kill 금지**. GPU 여유<7.5GB면 OOM 위험 → 사용자에게 확인.
- 좀비 확인: `nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader`.

### 검증 성공기준 (목표주도)
- Phase1: 동일 DR 시드 씬에서 개선 후 적치수 ≥ 개선 전 + "초기 unreachable→재시도로 적치" 로그 1건↑. 고정 `--mixed` 4/4 회귀 없음.
- (현재 `--seed`는 미구현 — Phase1 검증 위해 추가하면 결정적 재현 가능. DR 무작위는 `np.random` 사용.)

---

## 3. 충돌 에셋 단계별 (참고)
- 월드 동기화: 현재 타겟만 ignore → 다른 물체 자동 장애물. `collision_checker_type=MESH`.
- QUERY_GRASP(line ~1919): 타겟도 장애물 추가(접근 스윕 방지). 파지 닫은 뒤 `enable_obstacle(False)` 해제(line ~2170). MOVE_LIFT서 attach(Cuboid bbox).
- cuRobo `plan_grasp`: 접근은 그리퍼충돌 ON, 최종진입만 `disable_collision_links` OFF(소스 motion_gen.py 4291/4318).
- 노션 상세: "[기술문서] cuRobo 모션플래닝 > [디버깅] Stage8 모션효율화 + 무간섭 3제품 적치".

## 4. 규칙 (CLAUDE.md 발췌)
- 코드 다수 수정 전 묻기. 보고→승인→진행. 한 단계 독립 검증 후 다음.
- 한국어 출력, 문장 종결 콜론 금지. 새 소스 파일 첫 줄 한국어 헤더 주석.
- 공식 문서/소스 먼저(추측 금지). cuRobo v0.7.8 v1 API만, GraspGen PointNet++만.
