# 마스터 플랜 — 시뮬 모션 파이프라인 실기체 배포 (매대 정리 픽앤플레이스)

작성 2026-06-19. 세부 결정·gotcha는 `stages/pipeline/context-notes_snack_unify.md`, 단계 체크는 `stages/pipeline/checklist_snack_unify.md`, 인터페이스는 `integration/INTERFACE_CONTRACT.md` 참조.

---

## 1. 최종 목표

시뮬(Isaac Sim)에서 검증한 **GraspGen → cuRobo 모션 파이프라인을 실기체(두산 E0509 + 로보티즈 RH-P12-RN)에 배포**한다. 로봇 장착 카메라(RealSense)로 비전팀 노드(webcam_seg_node)가 물체를 인식해 내 모션 노드로 데이터를 주면, 내 노드가 모션플래닝 → 두산 실행. 매대 정리 픽앤플레이스(캔/병/봉지 3종)를 실로봇에서 자율 동작시키는 게 목표.

분담 — 비전(인식·좌표/PC) = 비전팀 / **모션(파지선택·궤적·두산실행) = 나** / 봉지 물성 = 사용자 다른 페이지.

---

## 2. 아키텍처 (확정)

```
RealSense → webcam_seg_node (비전팀)
   ├─ 캔/병:  물체별 원시 PC (N,3 float32, base·m, 'point_cloud')
   └─ 봉지:   봉지 중심 좌표 + yaw (top-down pose, ZYZ[0,180,yaw])

→ curobo_planner_node (내 노드, 실기)
   ├─ 캔/병:  PC → GraspGen.infer → plan_grasp(goalset) 무충돌 best → lift/carry/place/home
   └─ 봉지:   좌표 → top-down 직접 파지(스퀴즈) → side 재배향 운반 → 병 적치 시퀀스
   → 두산 관절궤적 movesj 단일호출 + 그리퍼 전류(class별)
        ↑ main_controller_node FSM 오케스트레이션
```

핵심 결정.
- **GraspGen은 내 측**(비전이 원시 PC를 줌). PTV3 금지·PointNet++만(CUDA12.8). ZMQ 서버(:5556).
- **봉지는 GraspGen 미사용** — Phase1 검증서 GraspGen이 봉지 PC에 side 후보만 줌(top 0/100). 봉지는 top-down 파지 필요라 좌표 기반 직접 파지로 결정.
- **실기 실행 = 관절궤적 movesj 단일호출**(Cartesian 금지 — IK분기 플립=되돌아감 방지), interpolation_dt=두산 RT주기 일치, time_dilation 점진 상향(스터터 대응).
- 팀 레포는 보고만, 내 로컬이 정본 → 재이관.

---

## 3. 현재 진행 상황

### 시뮬 (검증됨)
- **캔+병 mixed 4/4 적치**(stage8, plan_grasp→PLAN_LIFT→CARRY→INSERT→LOWER→RETREAT→HOME).
- **봉지 통합 (이번 세션 진행)**
  - Phase1 — GraspGen 봉지 검증 → side만 → **GraspGen 미사용 확정**. 스파이크 코드 원복.
  - 봉지 좌표 확보 — 월드 중심 [0.25,-0.04,~0.74], 거치대 [0.31,0.53,1.14], 치수 134×~180×71mm.
  - top-down 파지(폐루프 스퀴즈 74mm + rigidify) **유지**.
  - 적치를 **병 방식으로 교체** — side 재배향 plan_single 운반 + moveL(+y진입/-z하강/-y이탈). → **운반·3층 도달 해결(사용자 "모션 OK" 확인)**.
  - 적치 위치 튜닝 중 — v3에서 place_z +0.07 / x +0.05 / soften 복귀 적용(라이브 확인 대기 중 사용자 중단).

### 자료
- 모션 코어/노드 스켈레톤 + 인터페이스 계약 — `integration/`.
- 비전팀 파일 — `vision/`(webcam_seg_node, 모델, 캘리브).
- 팀 레포 참조 — `feat/integration-controller` `src/{motion,integration,vision,dashboard}`. motion/curobo_planner_node.py ~85%.

---

## 4. 해결해야 할 문제점

### A. 봉지 적치 마무리 (시뮬, 진행 중)
- 적치 시 **관통**(봉지 바닥이 매대판 아래) — place_z 상향으로 대응 중(v3 +0.07). 라이브 확인 필요.
- **soften 후 봉지 형상 복귀** — v3에서 재활성(매대판 위로 띄워 터널링 방지). 안착 확인 필요.
- **x/깊이 정렬** — 사용자 "x+로" 반영(v3 +0.05). 미세조정 가능(_SNACK_PLACE_DX/DY/DZ 보정변수).
- (주의) 봉지-매대 cloth 충돌이 안정적인지 — 깊은 관통=터널링 위험. 물성은 사용자 다른 페이지 협의.

### B. 실기 배선 (캔/병+봉지 공통)
- `curobo_planner_node` 실기화 — 두산 **movesj 관절궤적** 전송, 그리퍼 전류(snack300/bottle600/can1000), obstacles String→WorldConfig 파서, arm_joint_names 6개, TCP 깊이 실측.
- 봉지 좌표를 비전 pick_pose(또는 전용 토픽)로 받는 인터페이스 배선.

### C. 이관 리스크 (계약서 §3)
- **리스크① 관절한계 URDF** — MotionGen 빌드에 j4±180°/j5 0~135° 반드시(없으면 팔 플립).
- **충돌구체 link-name 정합** — plan_grasp goalset이 44-구체 모델과 link명 불일치로 조용히 실패(레포 TODO#1).
- **리스크③ 오프셋** — standoff는 plan_grasp 전담, TCP 깊이(0.110) 실측 보정(이중적용 금지).
- **리스크② 누운 캔** — 실테이블 누운 캔 처리(lying_grasp 또는 비전 판정).

### D. 실기 모션 품질 (스터터 — 기보고 증상 대응)
- 되돌아감=IK분기 플립 → 관절궤적 전송, Cartesian 금지, j4/j5 URDF.
- 끊김=웨이포인트 쪼갬/주기 불일치 → movesj 단일호출, interpolation_dt=RT주기.
- 추종 실패 → time_dilation_factor 0.1→0.3→1.0.

### E. 문서
- INTERFACE_CONTRACT를 원시 PC 입력 + 봉지 좌표 경로로 개정.

---

## 5. 플랜 (단계)

### Phase A — 봉지 시뮬 적치 완성 [현재]
1. v3 라이브 확인(관통/형상복귀/x정렬) → 보정변수(_SNACK_PLACE_DZ/DX/DY) 미세조정. 검증 = 봉지 3층 안착, 관통 0, 형상 복귀.
2. (선택) `--mixed` 3종(캔+병+봉지) 통합 적치 + 캔/병 4/4 회귀.
3. 커밋.

### Phase B — 실기 모션 코어 재이관
1. stage8에서 캔/병(GraspGen+plan_grasp+FSM) + 봉지(좌표 top-down+side 적치) 로직을 데이터소스 무관 코어로 추출(integration/core 개정, PC·좌표 둘 다 입력).
2. 검증 = 저장된 PC/좌표로 오프라인 호출 → 후보·궤적 생성(로봇 없이).

### Phase C — ROS2 노드 + 두산 실행
1. curobo_planner_node — 토픽 구독(PC/좌표/class/obstacles/joint_states) → 코어 → 두산.
2. 두산 movesj 관절궤적 전송 + 그리퍼 전류 + obstacles 파서. interpolation_dt=RT주기, time_dilation 저속.
3. 검증 = 계약서 §5 순서(FK→고정후보 공중 저속→obstacles 무충돌→비전 연결→그리퍼→풀시퀀스→정상속도). 영상서 되돌아감 0·세그먼트내 정지 0.

### Phase D — 통합·오케스트레이션
1. main_controller FSM 연동, 다물체 우선순위(periphery-first).
2. 검증 = 캔+병(+봉지) 실기 1사이클 무충돌 완주.

### Phase E — 문서·핸드오프
1. INTERFACE_CONTRACT 개정, 재이관 패키지, 핸드오프.

---

## 6. 실행/검증 규칙 (요약)
- Isaac 한 번에 하나, 실행 전 `pgrep -af stage8|grep python` 확인. GraspGen 서버 PID 죽이지 말 것. 사용자 다른페이지 snack 런 kill 금지.
- 종료 — `pkill -9 -f 'stage8_main[.]py'`(단독 줄, py_compile와 분리). 결과는 PNG로(`logs/shots/`).
- 시뮬 결과 로그로 단정 금지 → 사용자 라이브 확인. [[sim_ask_dont_judge]]
- 보고→승인→진행, 단계 독립검증. 한국어·종결콜론 금지. cuRobo v0.7.8 v1 API만.
