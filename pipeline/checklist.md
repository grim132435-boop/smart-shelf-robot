# 다물체 매대정리 픽앤플레이스 — 체크리스트

목표: 책상 위 여러 물체(a,b,c,d…)를 GraspGen 파지 + cuRobo 무충돌 전이 + moveL 매대삽입으로
가능한 것부터 각 층에 순차 적재 후 홈복귀. 클러터/적재누적에 견고.

성공기준(전체): N개 물체를, 책상·이웃물체·매대턱/벽·기적치물 무충돌로 지정 층에 직립 적치,
불가물체는 스킵·기록, 매 사이클 홈복귀. 궤적 중 구체-장애물 최소간격 로그로 무접촉 증명.

---

## Phase 0 — cuRobo 키스톤 API 스파이크 (de-risk) — ✅ 통과 (2026-06-10)
  스파이크: multiobj_pipeline/phase0_spike_plan_grasp.py (헤드리스, PRIMITIVE 월드)
- [x] plan_grasp 단발 호출 → 9후보 중 best 선택 + approach/grasp/retract 무충돌 반환
      결과: success=True, grasp_traj[62,6]/retract[31,6]/interp[86,6], goalset_index, planning_time 3.0s
- [x] attach_spheres_to_robot → plan_single → detach_object_from_robot: API 동작 OK
      (mock 캔 스피어가 그리퍼와 겹쳐 INVALID_START_SELF_COLLISION — Phase2서 스피어 위치만 조정)
- [~] disable_collision_links: 그리퍼 링크 전달 후 plan_grasp 성공(접촉허용 동작은 Phase1서 실측)
- [~] plan_goalset 직접호출: Pose(1,N,4) shape가 pose_cost_metric 경로와 안 맞아 예외.
      단 plan_grasp 내부 plan_goalset은 정상 → 실사용 무관, 직접 쓸 때 shape만 맞추면 됨.
  부수학습: cuda graph는 single↔goalset 전환 충돌 → warmup(n_goalset=N) 또는 use_cuda_graph=False.
            헤드리스 MESH는 warp.torch 미존재로 불가(PRIMITIVE 사용); Isaac Sim Phase1선 MESH 정상.

## Phase 1 — 깔끔한 파지 접근 (NVIDIA 공식 패턴 정렬) — ✅ 통과 (2026-06-10)
  ★공식 권장은 plan_grasp가 아니라 plan_single+접근메트릭 (simple_stacking.py). 추측한 plan_grasp+cuda_off → 공식으로 교체.
- [x] use_cuda_graph=**True** (계획 빠름). plan_grasp(goalset) 폐기.
- [x] 파지 접근: QUERY_GRASP(side)→PLAN_GRASP에서 grasp_world로 **plan_single + PoseCostMetric.create_grasp_approach_metric**
      (offset_position=PREGRASP_STANDOFF, linear_axis=2=grasp Z=approach, tstep_fraction=0.8). PLAN_PREGRASP 건너뜀.
- [x] 메트릭은 **plan_config.clone()**에만 실어 호출(누수 없음). carry/home은 base plan_config(무메트릭) — cuda graph 공존 OK(에러 없음).
      검증: 단일 캔 클린 성공(캔-EE0.100, 물리파지+0.111, 3단 직립z1.208, P8, 실패·재배치 0, changing-goal 에러 없음).
- [ ] (Phase2) 파지 후 attach_objects_to_robot(메시 스피어 자동피팅) / 적치 후 detach — 클러터에서.

## Phase 2 — attach + 다물체 월드  (구현 완료, 검증 대기 2026-06-10)
  ★범위=메커니즘만(이웃 장애물+attach). N개 순차적재·슬롯맵은 Phase 3. 외과적: 단일타겟 상태머신 유지.
- [x] 책상에 클러터 캔 N개 spawn: `--clutter N`, 타겟 ±y 0.12m·바깥확장, kinematic 정적 장애물(실린더 동일).
- [x] world 동기화: 타겟만 ignore 유지 → 클러터는 ignore 목록 밖이라 get_obstacles_from_stage가 자동 장애물.
- [x] 잡은 캔 attach: `attach_external_objects_to_robot`(외부 Cuboid 프록시, 캔이 world_model에 없어 external 사용)
      → carry/home plan_single이 캔 부피 인지. 안착 후 `detach_object_from_robot`. (attached_object 스피어 4개)
- [x] (검증✅ 2026-06-10) `--clutter 1 --place`: clutter_0(+y) 장애물19개 포함, 파지후보 walk로 −y쪽 접근 채택
      (옆 클러터 안 침), [attach]→carry plan_single 회피→캔 직립 적치 z1.208→[detach]→P8 완료. shots 001~009.
- [x] **파지후보 walk 추가**: 접근 실패 시 GraspGen 재추론 대신 `cands`를 순차 시도(다음 azimuth) → 막힌 방향
      거부 시 자유 방향 자동 선택. (이전엔 동일 #1 재선택 무한루프였음). cands 상한 제거(전 azimuth 도달).
- [x] (검증✅ 2026-06-11) **--clutter 2(양옆)** — 캔 0.58m(`--obj-dist`, side IK 도달구간)+간격 0.17m.
      goalset이 정면(approach≈[0.94,0.34]) 선택 → 클러터 사이 진입, 둘 다 안 침, 직립 적치 z1.2, P8. shots.

## Phase 2.5 — 룰베이스 선택 → plan_grasp(goalset) (사용자 지적: 상황적합 생성) — ✅ (2026-06-11)
  ★문제: side 후보를 손튜닝 가중치(손목부하+정면페널티+클리어런스)로 단일선택 → 장면 바뀔 때마다 재튜닝(하루 3회).
   선택 책임이 휴리스틱에 있어 클러터 무시. 정석=월드를 아는 플래너가 선택.
- [x] 후보 묶음 전체 → **`motion_gen.plan_grasp(cu_js, gposes(1,N,7), plan_config, grasp_approach_offset,
      disable_collision_links=그리퍼링크, plan_grasp_to_retract=False)`**. goalset_index로 best, 2단계 진입
      (offset 풀충돌인지 → 직선 최종진입+손가락충돌면제). 룰베이스 walk/메트릭/정면페널티 폐기.
- [x] use_cuda_graph **False**(goalset↔single 혼용의 changing-goal 회피, Phase0). 계획 ~2s. warmup 그대로.
- [x] 효과: 접근중 캔 이동 0.4m→**5mm**(안 침). 양옆 클러터서 goalset이 정면 자동선택. 넘어짐 가드(QUERY 재진입).
- [ ] (TODO) 속도 — cuda graph off로 계획 느림. Phase4서 warmup(n_goalset=N)+후보수 고정 패딩으로 graph 재활성 검토.
- [ ] (TODO) +x 순수 정면(deg=0)은 캔 0.50m에선 여전히 side IK_FAIL(도달거리). 0.58m+서 가능. 실거리 정책 Phase3.

## Phase 3 — 다물체 오케스트레이션 + 적재 — ✅ 핵심 검증 (2026-06-12, stage5_graspgen_e0509.py)
  ★층별 적재는 불가(2단 천장 개구 0.16m, 옆파지 진입 불가 — 2026-06-10 확정) → 3단 한 층 슬롯 분할.
- [x] N개 물체 등록: `--objects N --obj-gap`, /World/obj_i 동적 스폰, targets 상태목록(pending/placed/skipped).
      **매대측(+y) 우선 순서**(사용자 지시): 운반 경로에 걸리는 캔부터 치움. 실측 y 기준 선택.
- [x] 매대 빈슬롯 점유맵: SHELF3_SLOTS=(x,in_y) 2열 지그재그 [(0.165,0.56),(0.34,0.44),(0.21,0.44)].
      ★1열 피치 0.11 실패 교훈: 슬롯 간격은 캔 폭이 아니라 **그리퍼 스윕폭(~0.07 half)**이 결정.
      x0.14=이웃/벽 간섭(-3.5mm 침투), x0.36=도달한계(IK 실패) → 2열 + `slot_feasible()` 사전검사
      (①기적치 중심거리≥0.125 ②삽입·하강 IK ③삽입자세 간격>0). IDLE 사전검사 + PLAN_CARRY 본검사.
- [x] 불가물체 스킵+사유: no_grasp/unreachable/grasp_slip/lift_plan_fail/no_slot → 홈 복귀 후 다음 타겟, 요약 표.
- [x] (검증✅ --objects 3) 3캔→3슬롯 P8×3: 전 세그먼트 무침투(최소 +2.1mm, 우벽 근접), 직립 z=1.208×3,
      기적치 캔 사이 하강 +28.2mm, 매 사이클 홈복귀, 요약 "적치 3/3". shots 027.
- [~] (부분검증 --objects 4 매대측 우선) 1번째(obj_3) 적치 성공 + 순서 동작 확인. 도중 발견:
      리프트 plan 실패 → 단일모드 RELEASE 텔레포트 오작동 → **수정**(다물체: lift실패=재선별1회→스킵,
      RELEASE=재배치 금지·홈만). 수정 후 재검증은 Stage7에서(사용자 결정 2026-06-12 — 여기까지).

## Phase 3.5 — 모션 자세 정밀화 (간섭 위험 제거) — ✅ (2026-06-11)
  ★사용자 관찰: 운반 중 팔이 손목을 뒤집고(link4,5 아래), 홈복귀 시 joint_4 ~180° 플립 → 이웃 간섭 위험.
  ★진단(관절각 측정, log_arm_deg): cuRobo가 운반에서 파지 분기를 버리고 손목-플립 분기로 이주.
   파지 j4=143°/j5=+81°(좋음) → 운반 j4=269°/j5=−92°(손목 아래) → 홈복귀 j4 204° 되감기.
- [x] 원인=cuRobo는 자세 비용이 없어 IK 분기를 경로비용만으로 고름. ±360° j4가 과회전·플립 허용.
- [x] 실패한 시도: 대칭 position_limit_clip ±135° → 파지(j4=143°)까지 무력화→정면 강제→미끄러짐. (대칭은 부적합)
- [x] **정확 처방(비대칭 한계, URDF 복사본)**: BoundCost가 한계를 init에서 clone → 런타임 텐서수정 무효.
      `make_jlim_urdf`로 원본 불변, joint_5=[0,+135°](손목 위만=플립분기 차단), joint_4=±180°(과회전 차단).
      ARM_JOINT 클램프도 일치. 검증: j5 전구간 양수(+92), 홈 j4 되감기 204°→25°, 파지·직립적치 유지.
- [x] (확인✅ 2026-06-11~12) 다른 캔 위치(-y/중앙)서도 j5≥0로 파지 가능한지 — **feasible 확인**:
      중앙(dy-0.04)/-y(dy-0.2)/+y 세 위치 모두 j5=+102°로 IK 도달·P8 성공(center/negy/diag 로그). 한계 완화 불요.
      ★단 -y 첫 파지가 미끄러져 캔을 넘어뜨리는 건 **그리퍼 물리 모델(예제값) 문제 → Stage6(Phase5)로 이관**, 선정 로직 무관.
- [x] (2026-06-12) 선정 로직 정리: IK 필터 side 정렬에서 **장면 의존 편향(정면페널티·클리어런스 가산) 제거,
      축부하(jcost)만 prior 유지**. 최종 파지점은 plan_grasp(goalset) 월드선택에 일임(Phase2.5 정석 확정).

## Phase 4 — 품질 (기존 Stage5 흡수) — ✅ (2026-06-12, stage5_graspgen_e0509.py)
  ★사용자 관찰(2026-06-10): "모션이 출렁이면서 책상에 좀 부딪힌다" — 이걸 Stage5에서 잡기로.
   증상=① 이동 시작/끝에서 팔이 처졌다 출렁임(joint_2 중력처짐 ~4°) ② 마지막 접근 모션서 책상 살짝 접촉.
  ★별도 파일 작업: stage4를 다른 창(Stage6)에서 수정 중 → stage5 사본+run_stage5.sh+/tmp/stage5_stop 분리.
   Stage6의 SDF 손가락 변경과 머지 필요(추후).
- [x] joint_2 중력처짐(~4°) → 중력보상 피드포워드(게인 유지). physics callback 1곳서
      get_generalized_gravity_forces→set_joint_efforts(arm 6관절만, PD와 독립 가산). j2 중력토크 -53.7Nm
      = kp800 기준 정상상태오차 3.8° — 관측 4°의 정체. 효과: 추종오차 grasp 4.55°→**0.17°**, 전 구간 ≤0.40°.
- [x] 출렁임 완화 — 중력보상으로 해소 판정(추가 time_dilation/kd 튜닝 불요).
      운반 시작 하향 휘청(사용자 관찰 6/12) 소멸: lift 간격 +64.6→+100mm 포화.
- [x] 전이(cuRobo)↔moveL 핸드오프: moveL 3구간(진입/하강/후퇴) settle 10→0 연속화.
      그리퍼 램프오픈(40스텝)·캔 안정 대기(120스텝, 물체 물리)는 유지.
- [x] 구체-장애물 최소간격 로깅: world_coll_checker ESDF(compute_esdf=True, r>0 마스크, 5스텝 샘플)
      → 세그먼트별 [간격:tag] running-min. 검증 런 전 구간 무침투(최소 +28.7mm, -z하강).
      검증(final_run1, P8 클린): 추종오차 전 관절 ≤0.40°, 전 구간 간격>0, settle=0 연속 모션,
      캔 직립 z=1.208 정확, 홈복귀 OK. 기준선(run3) 대비 회귀 없음.

## Phase 5 — 그리퍼 오프셋·접촉 정밀화 [Stage6] — ✅ (2026-06-12, stage6_graspgen_e0509.py)
  ★진단: 슬립·손끝 펴짐의 근본원인 = 손가락 collision **convexHull**(physics.usd 명시) — 오목 패드를
   메워 캔과 점접촉. 기구학·질량·mimic은 공식 ROBOTIS RH-P12-RN-A와 이미 동일(레포 대조 완료, TCP 0.110 타당).
- [x] B1 손가락 collision convexHull→**SDF(res=256)** 런타임 교체 → 오목 패드 면접촉 복원.
      효과: 기존 닫기 명령만으로 **부족구동 curl**(원위 r2/l2가 근위 r1보다 +0.25~0.35rad 더 감김) 자연 발생
      — 실물처럼 감싸쥠. 별도 2단 램프 불필요 판정.
- [x] B2 마찰 frictionCombineMode=**max**·restitution=min (robotis_lab pick_place 정석).
- [x] r2/l2 각도 로깅 — curl 발생을 매 런 수치 증명.
- [x] 캔 미세 기울기 → **해소**: 회귀 3위치(dy 0/-0.1/+0.1) 모두 안착 z=1.208(직립기준 정확 일치).
      SDF 면접촉으로 캔이 반듯하게 물려 TCP/축 추가 보정 불요.
      검증: 3위치 모두 첫 시도 리프트 성공(재시도 0), 접근 캔이동 0.000, P8 완료.
  ★별개 발견: dy=-0.2(책상 가장자리)는 접근 경로가 캔을 침 — 그립 물리 아닌 **생성 위치-경로 간섭**
   (사용자 확인: 중앙쪽 dy≥-0.1 사용). 추후 접근경로 회피 정밀화는 Stage5(추종) 쪽 소관.
- [x] (마감 2026-06-12) **링크 구조 판별실험**(`--gripper-test`, 실물 사진 3장 대조): 빈손 풀클로즈
      r1=1.094/r2=1.095 = **완전 평행**. FK 검산 — 닫힘 패드 피벗간격 10.6mm, 내면 맞닿음. 실물 사진의
      "부리" 모양은 원위 링크(r2.stl) 꺾인 팁 형상이지 관절 추가회전이 아님 → **관절 기구학·mimic·한계는
      실물과 일치, 수정 불요**. 캔 파지 시 r2>r1 토우인(시뮬)도 실물 적응 모드와 동일 거동.
      잔여 외형 차이 = 수동 평행바(4절 링키지 2번째 바) visual 부재 — 성능 무관, 필요 시 r1 fixed 자식
      thin-box로 추가 가능(보류, 사용자 결정).

## Phase 6 — 도메인 랜더마이제이션 [Stage7]
- [ ] 실환경 동일 USD + 조명/텍스처/포즈 랜더마이즈 검증
