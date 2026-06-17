# 다물체 매대정리 파이프라인 — 컨텍스트 노트

작업 중 내린 결정과 근거. 계속 덧붙임. (다음 세션이 결정을 재도출하지 않도록.)

## 아키텍처 결정 (2026-06-10, 사용자 승인)
사이클(물체당):
1. world 갱신: 책상 모든 물체 + 매대 = 장애물, **현재 타겟만 ignore**
2. GraspGen(타겟) → N 파지
3. **cuRobo plan_grasp(N개 + approach/retract) → 무충돌·도달 best 1개 (한 호출)**
   - 전부 실패 → 다음 타겟 스킵, "unreachable" 로깅
4. 접근(plan_grasp) → 그리퍼 닫기 → **attach_object**
5. cuRobo 전이 → 층별 매대앞 핸드오프 pose (attach 부피로 회피)
6. **moveL 삽입 → 빈 슬롯(점유맵) → release → detach**
7. moveL 후퇴 → cuRobo 홈
8. world 갱신(물체: 책상→매대) → 반복

근거: 하이브리드(cuRobo 전이 + moveL 제약삽입)는 역할분리가 옳음. 순차 "실패→GraspGen 재호출"은
느리고 비결정적 → **후보 묶음을 plan_grasp/goalset에 통째로** 넘겨 한 번에 선택. 클러터/적재누적
대응 위해 **attach + 다물체 장애물화 + 동적 월드 + 슬롯관리** 추가.

## 확인된 사실 (설치본 검증)
- cuRobo 소스: /home/devuser/IsaacLab/src/curobo/src/curobo (v0.7.x, v1 API).
- motion_gen.py에 존재: plan_single(1544), plan_goalset(1585), update_world(1825),
  plan_single_js(2059), attach_objects_to_robot(2327), attach_external_objects_to_robot(2425),
  detach_object_from_robot(2612), attach_spheres_to_robot(2620), **plan_grasp(4198)**.
- plan_grasp 시그니처: (start_state, grasp_poses:Pose(1,num_grasps,7), plan_config,
  grasp_approach_offset, grasp_approach_path_constraint, retract_offset, retract_path_constraint,
  disable_collision_links, plan_approach_to_grasp, plan_grasp_to_retract, ...) → GraspPlanResult.
  approach→grasp→retract 3모션을 goalset trajopt로. disable_collision_links로 그리퍼-물체 접촉 허용.

## Phase 0 검증 결과 (2026-06-10) — plan_grasp 동작 확인
- plan_grasp(start, grasp_poses:Pose(1,N,7), plan_cfg, disable_collision_links=[...]) → GraspPlanResult.
  - success(tensor[bool]), status, planning_time, goalset_index(선택된 후보).
  - grasp_trajectory(approach→grasp, [T,6]) / retract_trajectory([T,6]) / grasp_interpolated_trajectory([T,6]).
  - 하위: approach_result / grasp_result / retract_result / goalset_result.
- attach_spheres_to_robot(sphere_tensor=(M,4)[x,y,z,r], link_name="attached_object") + detach_object_from_robot("attached_object"): 동작. attached_object 링크에 cfg가 미리 4스피어 할당(extra_collision_spheres) 필요. ★스피어를 그리퍼 메시와 안 겹치게 배치(안 그럼 INVALID_START_SELF_COLLISION).
- cuda graph: single↔goalset 전환 시 "changing goal type, cuda graph reset not available" → 실시스템은 warmup(n_goalset=N), 검증은 use_cuda_graph=False.
- 헤드리스 MESH 체커는 warp.torch 미존재로 불가 → PRIMITIVE. Isaac Sim(Phase1)은 MESH 정상.
- 스파이크 파일: multiobj_pipeline/phase0_spike_plan_grasp.py, 러너 run_phase0_spike.sh (헤드리스, ~12초).

## Stage4에서 이월된 교훈 (적용 유지)
- grasp/lift는 plan_single(무충돌). 직접IK 우회 금지 — plan_single 거부=실제 충돌.
- side grasp 높이 0.7·half(상단부)라야 손목이 책상 위로 떠 plan_single 통과(0.3은 INVALID_START_WORLD_COLLISION).
- 캔은 TCP보다 grip_z_offset(≈0.02~0.04, 실측) 아래에서 잡힘 → 적치 높이 = 바닥+half+offset.
- 매대(3단) world 좌표: 바닥top 1.14, 앞턱top 1.15, 천장 없음. [[stage4-shelf-coords]]
- 추종오차: joint_2 ~4°(중력처짐, 보상없음) — Phase4에서 해결. ★사용자 관찰(6/10): 모션 출렁임+마지막 접근서 책상 살짝 접촉 → Stage5(Phase4)서 잡기로 확정.
- 그리퍼 간이 오프셋: RHP12_TCP_DEPTH=0.110 (0.060=너무 깊어 캔 기울어 잡힘, 0.160=안 닿음). 정밀화는 Stage6.
- RELEASE 재배치: 캔 재생성 전에 로봇 홈복귀(plan_single_js) 추가 — 팔이 새 캔 안 침(사용자 요청 반영).
- 실행 함정: **pkill을 런처와 같은 Bash 호출에 넣지 말 것**(자기 명령문자열 매칭해 자살). GraspGen 서버(5556) 재사용.

## 확정 (2026-06-12 오후) — Stage6 그리퍼 정밀화 완료 + 파일 분기
- **파일 분기**: stage4_graspgen_e0509.py 동결(참조용). Stage5(추종)=stage5_graspgen_e0509.py(별도 세션),
  Stage6(그리퍼)=stage6_graspgen_e0509.py + run_stage6.sh. 이후 그리퍼 작업은 stage6 파일에서만.
- **슬립 근본원인 = 손가락 collision convexHull** (그리퍼 "데이터"는 무죄 — 기구학·질량·mimic·TCP는
  공식 RH-P12-RN-A와 이미 동일함을 doosan_ws/src/RH-P12-RN-A 대조로 확인. 공식 effort/관성도 placeholder).
- **처방**: set_finger_sdf_collision(SDF res=256, play() 전 호출) + frictionCombineMode=max.
  → 부족구동 curl 자연 발생(r2>r1 +0.3rad, 실물처럼 감싸쥠), 2단 램프 불필요(투기 코드 안 넣음).
- **캔 기울기 해소**: 3위치 회귀 안착 z=1.208 정확 일치. TCP 0.110 유지.
- **캔 생성 위치 정책(사용자)**: dy=-0.2(가장자리)는 접근 경로가 캔을 침 → 중앙쪽(|dy|≤0.1) 사용.
- 참고 레퍼런스: robotis_lab OMY(RH-P12 정석 actuator 100/4/30, combine=max), Franka 2e3/200.
  SurfaceGripper는 5.0+/CPU 전용/흡착 모델이라 실기체 검증 목적에 부적합 판정.

## 확정 (2026-06-12 사용자) — 파지점 선정은 plan_grasp(goalset)에 일임
- **Phase 3.5 마감**: j5≥0 비대칭 한계가 -y/중앙 위치 파지를 굶지 않음(세 위치 모두 j5=+102° 도달·P8). 한계 완화 불요.
- **-y 첫 파지 슬립의 원인=그리퍼 물리 모델**(예제 robotiq 합성값 적용, RH-P12 정확값 아님) → 선정 로직이 아니라
  Stage6(그리퍼 오프셋)에서 잡음. 지금 선정단에서 우회(-y 가중치 등) 금지 — 잘못된 층 반창고.
- **선정 로직 정리(코드)**: `select_best_reachable_grasp` side 정렬에서 **장면 의존 편향 제거**.
  정면페널티(0.1·|deg|)·클리어런스 가산을 빼고 **축부하(jcost)만 prior**로 정렬(chosen·cands 풀 둘 다).
  근거: 최종 파지점은 plan_grasp(goalset)이 11개 후보 전체를 월드충돌·도달로 직접 선택(IK필터 순서 무관).
  휴리스틱 편향은 장면 바뀌면 재튜닝 유발(Phase2.5 교훈). 책상침범은 safe 하드필터(p[7]≥table_top)가 보장.
  ★검증 필요: dy=-0.2/-0.04/+0.2 각 1회 P8 + j5≥0 유지(아직 미실행 — 사용자 트리거).

## 확정 (2026-06-10 사용자)
- **시작점 = Phase 0 키스톤 스파이크** (plan_grasp/goalset/attach 동작·시그니처 먼저 검증).
- **perception = Mock** — 시뮬 spawn pose를 GraspGen·cuRobo월드에 직접 주입. 비전팀 인터페이스는 나중.
- 추종정밀도/부드러움(원 Stage5)은 Phase 4로 흡수, 다물체 파이프라인이 우선.

## Stage5(Phase4) 추종 정밀화 (2026-06-12 진행)
- **별도 파일**: stage4를 다른 창(Stage6)에서 수정 중이라 `stage5_graspgen_e0509.py` 사본으로 작업
  (사용자 요청). 러너 `run_stage5.sh`, 센티널 `/tmp/stage5_stop` 분리. ★나중에 stage4/6 변경과 머지 필요.
- **간격로깅(단계1)**: motion_gen.world_coll_checker의 `get_sphere_distance(..., compute_esdf=True)` —
  커널이 구체 반경을 빼고 반환(양수=침투, 음수=간격 → 간격=-d). max_distance(0.1m) 근방 포화.
  ★패딩구체(r≤0)는 커널이 0을 써 min을 오염 → r>0 마스크 필수. 5스텝마다 질의(GPU 동기화 절감).
  별도 RobotWorld 안 만들고 motion_gen 월드 공유(이중관리 없음).
- **기준선(run3, P8 클린)**: grasp +33.9/lift +64.6/carry 포화/moveL진입 +43.4/하강 +26.6/후퇴 +39.8mm,
  전 구간 무침투. joint_2 추종오차 grasp 4.55°/lift 4.36°/home 4.12°(중력처짐 재확인).
- **중력보상(단계2)**: physics callback 1곳 등록(execute_plan/moveL/정착 루프 전부 커버).
  `view.get_generalized_gravity_forces()` → arm 6관절만 `view.set_joint_efforts`(=set_dof_actuation_forces,
  PD 드라이브와 독립 가산). 그리퍼 0(파지력 간섭 방지). 게인 유지. j2 중력토크 ≈ -54Nm.
  효과(gravcomp_run1): grasp 추종오차 4.55°→**0.17°**, lift 4.36°→**0.26°**, 간격 grasp +33.9→+44.0mm.
- **pkill/pgrep 함정 재확인**: 같은 명령 문자열 내 경로도 자기매칭(py_compile 경로 거짓양성 포함)
  → 브래킷 트릭 `stage5_graspgen_e0509[.]py` 필수. 실행 전 다른 창 인스턴스 확인(사용자 지시 2026-06-12).
- **--place 필수**: 플래그 없으면 파지·리프트 테스트 루프만(3사이클 후 종료) — 풀 P8 검증은 `--place --cycles 1`.
- **핸드오프 블렌딩(단계4)**: moveL 진입/하강/후퇴 settle 10→0(중력보상 후 추종 0.2°라 정착 불요).
  그리퍼 램프오픈 40스텝·캔 안정 120스텝은 물체 물리 정착이라 유지. 파지 후 45/리프트 후 20스텝도 유지(보수).
- **단계3(출렁임) 별도 튜닝 안 함**: 4° 처짐의 정체가 PD 정상상태오차(53.7Nm/kp800=3.8°)로 확인 —
  중력보상이 곧 출렁임 해법. time_dilation 0.7 유지.
- **최종 검증(final_run1, P8)**: 추종오차 전 관절 ≤0.40°, 전 구간 간격>0(최소 +28.7mm), 직립 z=1.208, 회귀 없음.
  **Phase 4 마감(2026-06-12)**. 잔여: stage5 변경(중력보상·간격로깅·settle0)을 stage4/6 본류와 머지.

## Phase 3 다물체 슬롯 설계 (2026-06-12)
- **1열 3슬롯(x 0.14/0.25/0.36, 피치 0.11) 실패** (multiobj_run1, 캔 지름 기준 설계의 오류):
  - x=0.14: 캔은 들어가지만 **그리퍼 스윕 반폭 ~0.07**이 이웃 캔(0.236)·좌벽(0.077)과 간섭
    → 하강 -3.5mm·후퇴 -9.0mm 침투, 캔 기울어짐(z=1.175). 간격로깅이 원인 즉시 수치화.
  - x=0.36: base(-0.25)에서 3D 0.99m = **도달한계** → 진입 moveL 웨이포인트 38/40 IK 실패.
  - 중앙 0.25만 성공(직립 z=1.208). 교훈: 슬롯 간격은 캔 폭이 아니라 **그리퍼 스윕폭**이 결정.
- **재설계 = 2열 지그재그 + 실현성 사전검사**:
  - SHELF3_SLOTS=(x,in_y) [(0.165,0.56),(0.34,0.44),(0.21,0.44)] — 쌍별 ≥0.128, 벽 ≥0.088,
    깊은 슬롯 먼저(후속 하강이 기적치 옆을 띄워 지남). 단일 모드는 기존 (0.25,0.50) 유지.
  - `slot_feasible()`: ①기적치 중심거리 ≥0.125(기하) ②삽입·하강 IK ③삽입자세 간격>0(부착 캔 포함).
    IDLE에서 추정 오프셋(0.047)으로 사전검사(잡고 갈 곳 없는 상황 차단), PLAN_CARRY에서 실측으로 본검사.
    하드코딩 좌표 신뢰 대신 매번 검사 — 장면이 바뀌어도 스스로 불가 슬롯 거름(Phase2.5 철학 동일).

## 다물체 선택 정책 (2026-06-15 사용자 확정·구현) — stage7 반영 완료
- **확정 정책**: **매대 먼쪽(y 작은) + 로봇 가까운(x 작은) 물체 먼저.** 1순위 y오름차순, 동률 x오름차순.
  근거: 바깥쪽(로봇 앞·매대 반대)부터 치우면 후속 carry가 남은 물체 위를 안 지나 충돌·모션 효율↑.
- **구현(stage7)**: IDLE 타겟선택 `max(_pend, key=_tgt_y)` → `min(_pend, key=lambda: (y, x))`로 교체.
  (이전 "매대측 우선"·"효율 기반" 설명은 폐기 — 이 y작은/x작은 우선이 최종.)
- 비전 역할 축소(2026-06-12 사용자): **비전은 탐지·6D pose까지만**. 파지 성공/유지(들림·접촉·직립) 확인은
  비전이 아니라 **그리퍼 전류 기반 판정**으로. 표/흐름도에서 비전 파지확인 항목 제거.

## 아직 미정 (Phase 2~3 진입 시 결정)
- 물체 형상: 현재 box/cylinder 2종 → a,b,c,d 형상/크기 정의.
- 매대 슬롯: 층당 슬롯 수·간격(moveL만으로 충분히 떨어졌는지).
- 스테이지 번호 표기(다물체=Stage5로 흡수, 그리퍼=6, 랜더=7).

## 누운 캔 픽 구현 (2026-06-15) — Stage7
- **핵심 설계**: 누운 캔을 위에서 "그리퍼 X축(상하축)=캔 축"으로 파지 → 이후 carry/place가 X=위로 강제하며
  캔이 **직립으로 자연 재배향**. 별도 재배향 상태 불필요. (lying_grasp_from_axis: X=캔축, Z=-z 위에서, Y=Z×X)
- **트리거**: `--can-pose lying` → LYING_PICK=True → IDLE/QUERY 넘어짐 리셋 가드 우회 + 누운 파지 사용.
- **grip_z_offset=0 강제**: 누운 캔은 TCP가 캔 중심(축 방향)에서 잡혀 직립 후 TCP=캔중심 → 오프셋 0(측정값 무시).
- **j6 180° 플립 수정(사용자 지적)**: 캔축 ±180° 대칭(손가락 교체)을 안 풀어 IK가 j6=-180° 선택했음.
  → +축/−축 두 후보를 grasp_cands로 주고 plan_grasp(goalset)이 손목 최소 선택 → j6 0°, 간격도 +16.6→+25.4mm 개선.
- **검증**: 단일 누운 캔(축 -y) → 2층 직립 적치 z=1.018, 하강 간격 +25.4mm, P8. (서있는 캔과 동일 경로 합류)
- ★남음: 혼합자세 다물체 spawn(현재 다물체 전부 직립) + 2층 다슬롯 → 서있는+누운 동시 검증. 물체 3종(페트병·과자).

## 혼합(서있는+누운) 캔 동시 2층 적치 (2026-06-15) — Stage7 ✅
- **결과**: --objects 2 --can-pose lying --shelf-level 2 → obj_0(직립)·obj_1(눕힘) **둘 다 2층 직립 적치(2/2)**, P8.
- **타겟별 누운 감지**: can_is_lying(obj) (축 |az|<0.5) → 전역 LYING_PICK 폐기. 가드/파지/오프셋 분기가 타겟별.
- **혼합 spawn**: 다물체에서 홀수 인덱스 눕힘(orientation 90°X, z=반경), 짝수 직립.
- **★누운 실린더 굴러감 = 핵심 함정**: 누운 캔이 굴러 도달 밖(x→0.37)으로 이탈 → 파지 위치 이탈로 그리퍼가
  캔 침/미끄러짐/plan_grasp 실패. **angular_damping=50 + linear_damping=5**로 제자리 안착시켜 해결.
  (사용자 "그리퍼가 캔 쳐서 날아감"의 진짜 원인 = 접근 충돌누락 아니라 이 롤링. 접근 충돌회피는 정상.)
  ★실기체에선 댐핑 대신 실물 물성(질량·마찰·구름저항)으로 대체 필요(개선항목8).
- **함정**: _PhysxSchema가 함수 뒤쪽 import라 spawn(앞)서 UnboundLocalError → lying 블록 내 재import로 해결.
- 2층 다슬롯 마진: 슬롯 (0.155,0.561)·(0.338,0.45)서 하강 간격 +7.4/+4.7mm(중앙단일 +24보다 빡빡, 양수).
  실기체 비전오차 마진 위해 2층 다물체 슬롯은 worst-case 더 봐야(개선).

## 물체 3종 — 페트병 통합 ✅ (2026-06-15)
- **CYLSPEC 패턴**: 하드코딩 OBJ_SPECS["cylinder"] 16곳을 CYLSPEC(=args.obj_type=="bottle"면 bottle spec)로
  치환 + `_obj_type/obj_type == "cylinder"` → `in ("cylinder","bottle")`로 일반화. 페트병=실린더 계열 재사용.
- OBJ_SPECS["bottle"]={z0.1125, r0.035, h0.225, side} (파워에이드600ml 실측 22.5cm 근사, 그립 가능 7cm 지름).
- **검증**: --obj-type bottle --place → side 파지(j5+104°)·리프트+0.118·3층 직립 z=1.253·P8. 3층 개방이라 간격 +60~98mm 넉넉.
- 과자봉지: FEM box(0.16×0.23×0.07, youngs 0.3MPa) 중앙 top 파지 변형 검증 완료(deformable_bag_spike.py).
- ★남은 3종 통합: per-object 타입+층(캔2/병3/과자3) 라우팅(현 전역 --shelf-level → per-target), 과자 FEM 파이프라인 통합.

## 과자봉지 실 로봇 파지 — 제약·기법 (2026-06-15 사용자)
- ★**그리퍼<봉지**: RH-P12 최대 개방 ~106mm < 봉지 160×230mm → 가로질러 못 감쌈. **중앙 squish만 가능**.
- ★실기체 노하우(사용자): **부분 개방(어느정도 벌림) + 위에서 누르기 → 봉지 중앙이 손가락 사이로 솟음 → 닫기**.
  = press-then-grip. (완전개방 straddle 아님). FEM서도 부분개방 손가락으로 누르면 중앙 bulge → 닫아 그립.
- 봉지 메시: snack_bag_pillow.usd (베개형, 중심 7cm/가장자리 0.8cm, 1632v 전부quad watertight, 외곽두께 실물반영).
- 봉지 파지 결정: **중앙 squish(press-grip)**, top 접근. 3층 적치 목표.
- 통합 난점: FEM은 cuRobo 장애물 직접 불가(bbox 근사), 강체 attach 불가(carry 그립마찰 의존, 미끄럼 위험).
- stage7 통합 진행중: obj-type "snack" 추가됨. 남음: deformableUtils import, GPU dynamics enable(play前),
  FEM 봉지 spawn, press-grip 시퀀스(부분개방→하강 누르기→닫기→리프트), 검증(우선 --place 없이 파지+리프트).
