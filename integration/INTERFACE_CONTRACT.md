<!-- 비전(webcam_seg_node) ↔ 모션(curobo_planner_node) 인터페이스 계약 + 이관 리스크 -->

# 비전 ↔ 모션 인터페이스 계약 (매대 정리 픽앤플레이스, 실기)

> 작성: 남정혁(시뮬·그래스프) | 대상: 모션팀 | 근거: 시뮬 `stage7_graspgen_e0509.py` + 비전 `webcam_seg_node.py`
> 핵심: **GraspGen 추론·후보 자세 합성은 비전 노드가 끝낸다.** 모션 노드는 후보를 받아 **plan_grasp(goalset) 선택 + 충돌없는 궤적 + 두산 실행**만 한다.

---

## 1. 시스템 분리

```
RealSense → webcam_seg_node (비전)            → curobo_planner_node (모션, 이 패키지) → dsr(두산)+그리퍼
  YOLO seg + SAM 마스크                          grasp_candidates 구독
  깊이 deproject + T_cam2base(eye-in-hand)       → plan_grasp(goalset) 선택
  GraspGen.infer(pc) + 후보 자세(옆면 수평·LEVEL)  → 관절한계 MotionGen + 충돌월드
  발행: grasp_candidates / class / obstacles      → 관절궤적 + 그리퍼 전류
```

---

## 2. 토픽 계약

| 토픽 | 타입 | 생산→소비 | 프레임/단위 | 내용 |
|------|------|----------|------------|------|
| `/dsr01/curobo/grasp_candidates` | PoseArray | 비전→모션 | **base · meter** | GraspGen 후보 EE pose N개 (goalset 입력) |
| `/dsr01/curobo/grasp_class` | String | 비전→모션 | — | `can`/`bottle`/`snack_bag` → 그리퍼 전류 |
| `/dsr01/curobo/obstacles` | String | 비전→모션 | base · meter | 매대·이웃물체 장애물 → cuRobo world |
| `/dsr01/curobo/pick_pose` | PoseStamped | 비전→모션 | base · meter | 단일 파지 타겟 / 실행 트리거 |
| `/dsr01/joint_states` | JointState | 두산→모션 | rad | 현재 관절(플래닝 시작상태) |
| (두산 궤적) | dsr_msgs2 | 모션→두산 | rad | **관절궤적**(Cartesian 아님 — 리스크① 참조) |
| `/gripper_service/set_position` | SetPosition | 모션→그리퍼 | — | 닫기(클래스별 전류) |

**좌표 규약 (반드시 일치)**
- 모든 위치 = **robot base 프레임, meter**. (비전이 `T_cam2base`로 변환 완료 → 모션은 base offset = 0)
- **Quaternion 순서** — ROS `Pose`는 `xyzw`, cuRobo `Pose`는 `wxyz`. 노드에서 변환함(`cb_candidates`). **양팀 모두 이 순서 확인.**
- 그리퍼 전류(mA) — snack_bag 300 / bottle 600 / can 1000.

---

## 3. 이관 리스크 — 모션팀이 반드시 챙길 4가지

시뮬에서 **모션 측에 있던 것**들이라 비전 노드엔 없다. curobo_planner_core.py 에 반영해 둠.

### 리스크① 관절 4/5 한계 (팔 안 돌게) — **최우선**
- 시뮬은 `make_jlim_urdf`로 joint_4 ±180° / joint_5 0~135°를 **URDF에 박아 MotionGen 빌드** (stage7 L1335).
- cuRobo BoundCost는 URDF를 init에서 clone → **런타임 텐서 수정은 무효, 반드시 URDF로** 줘야 함.
- → `build_motion_gen()`에 구현. 이걸 빠뜨리면 후보가 멀쩡해도 **실기 팔이 플립**한다.
- 또한 두산에 **관절궤적으로 전송**할 것. Cartesian(movel)로 주면 두산이 자기 IK 분기를 골라 플립함.

### 리스크② 누운 캔 분기
- 비전 노드는 캔/병을 **옆면 수평 파지(|az|<0.45)** 로 가정. **누운 캔**이면 끝을 잡게 돼 틀림.
- 시뮬엔 `lying_grasp_from_axis`(위에서 캔축 파지 → carry가 직립화)가 있었음(stage7 L679, L2082).
- → 실 테이블에 캔이 누울 수 있으면, **비전이 누움/직립을 판정해 누운캔 후보를 따로 합성**하거나, 모션이 받은 후보의 approach가 캔축과 평행하면 거르는 처리 필요. **양팀 합의 항목.**

### 리스크③ 오프셋 규약 (이중 적용 금지)
- 비전 상수: `gripper_offset 0.11`, `pregrasp_standoff 0.06`. 시뮬 상수: `RHP12_TCP_DEPTH 0.110`, `PREGRASP_STANDOFF 0.10`.
- **grasp_candidates의 pose가 "TCP가 물체에 닿는 최종 파지 EE pose"인지, 오프셋이 이미 적용된 것인지** 명확히 할 것.
- 모션의 `plan_grasp`는 `grasp_approach_offset`으로 standoff를 **자기가** 더한다 → 비전이 또 빼면 이중 적용.
- → **계약: 비전은 '최종 파지 EE pose(오프셋 0)'를 발행, standoff는 모션의 plan_grasp가 전담.** TCP 깊이는 실측 보정.

### 리스크④ 다물체 우선순위
- 시뮬은 `_tgt_key`로 **자동**(매대 먼쪽 y작은 + 로봇 가까운 x작은 = 바깥부터, stage7 L1964).
- 비전 노드는 현재 **키보드 수동 선택(1-9 lock)**. 자율 다물체로 가려면 이 우선순위를 **비전 자동선택 또는 모션 측**에 넣어야 함.

---

## 4. stage7 → 모션 코어 매핑

| 기능 | stage7 위치 | 이 패키지 |
|------|-------------|-----------|
| 관절한계 URDF | `make_jlim_urdf` L1017 + L1335 | `core.make_jlim_urdf` / `build_motion_gen` |
| MotionGen 빌드 | main L1328-1356 | `core.build_motion_gen` |
| plan_grasp goalset 선택 | main L2174-2220 | `core.plan_grasp_goalset` |
| 리프트/운반 | plan_single L2337/L2516 | `core.plan_to_pose` |
| 홈 복귀 | plan_single_js L2604 | `core.plan_to_joints` |
| 충돌월드 갱신 | `update_world` L2041 | `core.plan_pick_motion` |
| (참고) 후보 합성·GraspGen | `select_best_reachable_grasp` L727 등 | **비전 노드가 담당(이관 불필요)** |

---

## 5. 검증 순서 (안전 우선)

```
1. build_motion_gen → FK 검증(관절→EE base == 실측)  ★관절한계 적용 확인
2. 고정 후보 1개로 plan_grasp → 두산 저속 공중 실행(물체 없이)
3. obstacles(매대) 등록 → 무충돌 궤적 확인
4. 비전 grasp_candidates 연결(고정 캔) → goalset 선택 실행
5. 그리퍼 닫기 전류 → 리프트/운반/적치/홈 단계 추가
6. 저속(--max_speed 0.1) → 정상속도
```

---

## 6. 파일

- `curobo_planner_core.py` — 모션 코어(데이터소스 무관, stage7 알고리즘 추출)
- `curobo_planner_node.py` — ROS2 배선 스켈레톤(비전 토픽 → 코어 → 두산)
- 본 문서 — 인터페이스 계약 + 리스크
