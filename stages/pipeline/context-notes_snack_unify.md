# 컨텍스트 노트 — 과자봉지 캔/병 모션경로 통합 + 실기 배포 (세션 인계용)

작성 2026-06-19. 동일 워크스테이션 다른 Claude 세션이 결정·근거·위치를 재도출 없이 이어가기 위한 기록.
체크리스트 = `checklist_snack_unify.md`. stage8 우선순위 작업의 `context-notes.md`와는 별개 작업.

---

## 0. 목표와 결정 사슬 (사용자 합의)

상위 목표 = 시뮬 모션을 실기(E0509+RH-P12)에 배포. 비전팀(webcam_seg_node)이 **원시 포인트클라우드**를 내 노드로 주면 내가 GraspGen→cuRobo→두산 실행. 1차 범위 = **캔+병**, 그리고 **과자봉지 먼저 통합** 후 배포.

사용자 결정(질문-답).
1. 비전→내 노드 = **원시 PC**(물체별 (N,3) float32, base·m, key 'point_cloud'). → **GraspGen이 내 모션 노드 쪽**으로 옴(기존 INTERFACE_CONTRACT는 비전이 candidates 발행이라 개정 필요).
2. 팀 레포 ~85% curobo_planner_node.py는 참조용, **내 로컬이 정본**, 재이관.
3. ~~snack 실기 = 캔/병과 동일 모션경로(GraspGen→plan_grasp)~~ → **철회.** Phase1 검증서 GraspGen이 봉지 PC에 **side 후보만(100개 중 top 0개)** 줌 — 봉지는 **위에서 아래(top-down) 파지** 필요라 GraspGen 부적합. **결정: snack은 GraspGen 미사용, 봉지 중심 좌표+yaw로 top-down 직접 파지.** 실기 입력 = PC 아님, **봉지 좌표(ZYZ[0,180,yaw], pick_pose 류)**.
4. **봉지는 소프트** → 파지 후 `rigidify_bag`(강체화, 시뮬 시각화용) → 운반·적치 → `soften_bag`(복귀)는 **유지**. 봉지 grip은 시뮬상 시각화일 뿐, 실기 전이되는 건 **좌표기반 top-down 모션**.
5. 착수 = **시뮬 통합 먼저**. carry/place는 검증된 캔/병 시퀀스(plan_single+moveL) 채택 검토(적치 충돌 해소 목적) — 단 파지 선택은 좌표 top-down 유지.

### Phase1 결과 (2026-06-19, 완료)
- `--obj-type snack` 실행 → `[GraspGen] 100개 파지 수신`, `score[0.92~0.96]`, `approach_az[-0.42~0.48]` (top|az|>0.7:**0** / side|az|<0.45:**99**). → GraspGen은 봉지에 side만 → top-down 필요한 봉지엔 부적합 확정. 스파이크 코드는 원복(제거)함.
- 봉지 좌표 실측 — 월드 중심 [0.25,-0.04,~0.74](base [0.50,0,~0.01]), 책상윗면 z0.70, 거치대 [0.31,0.53,1.14]. 치수 폭134×길이~180×두께71mm. RHP12_TCP_DEPTH 0.110.

---

## 1. 현재 snack 흐름 (변경 대상, stage8_main.py)

- 전용 핸들러 = L1796~1968 `if obj_type == "snack":` … 끝에 `continue`(강체 FSM 미진입). GraspGen/plan_grasp **우회**.
- 흐름 — open → plan_single 접근(above) → 직접IK enter(의도적 접촉) → 폐루프 스퀴즈(_TARGET_GAP=0.074) → **rigidify_bag**(파티클 정지+AABB) + snack_follow 콜백(메시를 EE에 수동 핀) + cuRobo Cuboid attach → plan_single 리프트 → tilt 자세 carry(plan_single) → +y 직접IK moveL 적치(tilt) → detach+open+**soften_bag** → 직접IK 후퇴.
- 즉 접근/리프트/운반은 **이미 cuRobo plan_single 사용**(IK플립 없음). 안 쓰는 건 (a) 파지 pose가 GraspGen 아님(_ee_side 고정), (b) enter/squeeze/place가 직접IK, (c) tilt 적치(HANDOFF 미해결 충돌 원인).
- 관련 — `_TYPE_LEVEL`(L181) snack→3층 라우팅 이미 있음. `OBJ_SPECS["snack"]`(L242) z0.035/h0.07/top. `sample_object_pc`(L624)는 bottle/cylinder만(snack PC 없음 — GraspGen 안 쓰니까). mixed snack spawn L1282~1296, 단독 spawn L1387~1403, 타겟 dispatch L2067~2070.
- snack 모듈 — `snack_bag/snack_bag_module.py`: `spawn_snack_bag(mode="cloth")`=절차 베개(_make_pillow_mesh, USD 안 씀), `rigidify_bag`(L428), `soften_bag`(L447), `add_snack_stand`(L106). CLOTH_PARAMS: pressure11/stretch6000/friction2/pbd_damping18/max_vel0.2, 폭 hx0.06(12cm)·길이 hy0.086(17.25cm), 52g.

## 2. 통합 설계 (무엇을 바꾸고 무엇을 유지)

| 유지 | cloth 봉지 spawn, rigidify_bag(파지후 강체화→cuRobo가 부피 무충돌 인지+운반), soften_bag(적치시 복귀), 폐루프 스퀴즈 close |
| 합류(캔/병 동일) | 봉지 PC→GraspGen→plan_grasp(goalset) 파지 pose 선택, PLAN_LIFT/CARRY/INSERT/LOWER/RETREAT/HOME 검증 시퀀스 |
| 폐기 | _ee_side 고정 파지, snack_follow(물리그립 되면), tilt 적치, 직접IK 운반/후퇴 |

핵심 통찰 — rigidify가 봉지를 강체화하므로 그 다음은 캔/병과 동일하게 cuRobo attach+표준 carry/place가 먹힌다. rigidify/soften 사이의 "어떻게 움직이나"만 검증된 경로로 교체.

## 3-DONE. 봉지 거치대 적치 — 완성 (2026-06-22)
최종 해법(stage8_main snack 핸들러). 사용자 "snack 완료" 확정.
- 파지 = top-down 스퀴즈(좌표기반, GraspGen 미사용) + `rigidify_bag`(파티클 동결). 스퀴즈 빠르게(증분0.10·steps8·한계0.70, 목표 gap74mm). 진입 `move_direct_ik` steps90/settle12(200/40서 단축).
- 적치 = **하이브리드** — cuRobo `plan_single`로 매대 앞(tilt 자세) 도달(어려운 reach·충돌회피) → `move_linear_ik`(waypoints15)로 칸 안 **+y 진입→−z 하강**(좁고 의도된 접촉이라 moveL). 거치대 빗면 각 `_TILT_DEG`.
- ★도달 통찰(사용자) — 빗면 수직진입은 PRE 과도상승→도달불가. **수평(xy평행) 진입 + 봉지만 틸트**로 해결. 틸트 **20°가 도달 가능 한계**(37.4°는 +y진입 웨이포인트25/40서 IK실패). 거치대 빗면도 20°로 맞추면 flush(미적용 — 사용자 OK).
- ★soften 금지 — 물성복귀하면 cloth가 빗면서 흘러내림(사용자 관찰). 대신 놓는 순간 자세로 **동결 유지**(snack_hold 콜백, placed 월드점=EE@_Plocal 재계산. GetPointsAttr().Get()은 rest=원점 반환 버그).
- home = **−y 직선 후진(moveL, 칸 밖)** → `plan_single_js` 복귀(캔/병 동일, 사용자 요청).
- 좌표 — 거치대 _STAND_X0.31/_STAND_Y0.53(받침턱 y0=0.46), 적치 _SYf=_STAND_Y−0.04, _rest_z=FLOOR+0.13, entry_z=LIP+0.15(1.30).

## 3. (과거) 남은 핵심 이슈 = 적치(place) 충돌/투하 (HANDOFF_snack_pickplace.md)
- GraspGen 리스크는 Phase1서 해소(미사용 결정). 이제 핵심 = **봉지 적치 시 그리퍼-매대 충돌 또는 중간 투하**.
- 현 핸들러 place = tilt 자세(_TILT_DEG -37.4°) carry→+y moveL(stage8_main L1914~1958). HANDOFF엔 tilt 적치가 추종오차로 폐기 이력 — 코드 상태가 그 이후 버전이라 **베이스라인 재실행으로 실제 거동 확인부터**(checklist Phase0).
- 후보 해법 — 검증된 캔/병 place 시퀀스(PLAN_LIFT→CARRY plan_single→INSERT +y moveL→LOWER -z moveL→release→RETREAT -y moveL→HOME)로 교체. rigidify로 봉지가 강체라 캔/병 attach+place 그대로 먹힐 것.
- 단정 금지 — 시뮬 결과는 사용자에게 라이브로 물어볼 것. [[sim_ask_dont_judge]]

## 4. ★실기 배포 gotcha (NVIDIA/cuRobo/GraspGen/두산 문서 근거 — 2026-06-19 조사)

### "한 모션에서 뚝뚝 끊기고 가다가 되돌아감" 근본원인 (사용자 기보고 증상)
- **되돌아감(reverse) = IK 분기 플립.** 두산에 Cartesian(movel)/목표 pose만 주고 두산이 자체 IK 재해석 → 웨이포인트마다 분기 달라 손목이 갔다 되돌아옴. → **cuRobo 관절궤적 그대로 전송, Cartesian 금지.** + j4/j5 URDF 한계로 분기 모호성 제거(리스크①).
- **뚝뚝 끊김 = 웨이포인트 쪼개 블로킹 전송**(매 점 가감속 정지) 또는 **단일목표만 보내 중간 웨이포인트 손실**. → 궤적을 **한 번에** 전송 — `movesj`(스플라인 조인트, 블렌딩) v1 권장, 또는 `servoj_rt` 스트리밍.
- **스트리밍 흔들림 = interpolation_dt ≠ 컨트롤러 주기.** servoj_rt는 고정주기(두산 RT 최대 1kHz, set_rt_control_output) 필요. cuRobo `get_interpolated_plan(interpolation_dt=두산RT주기)`로 정확히 일치. 안 맞으면 버퍼 언더/오버런→따라잡기·되돌리기 보정.
- **추종 실패(estop) = 속도 과대.** `time_dilation_factor`<1.0로 감속(실기 첫 런 0.1~0.3). 구버전 `servoj`는 펌웨어 stutter+estop 보고됨(issue#117) → `servoj_rt`/`movesj` 사용.
- **권장 v1 = cuRobo 관절 웨이포인트를 `movesj` 단일 서비스 호출**(주기문제 회피, 블렌딩, 중간점 보존). 정밀 필요시 servoj_rt(주기일치). 어느 쪽이든 Cartesian 금지.

### 기타 문서 근거
- GraspGen — **PTV3는 CUDA12.8/Blackwell 미지원 → PointNet++만**(공식). ZMQ 독립서버(:5556) 운용, 내 노드가 클라이언트. 서버 PID 죽이지 말 것.
- cuRobo — MotionGen 인스턴스 유지(warmup 비쌈), 충돌월드는 `update_world`만(재빌드 금지). 25Hz 타겟→120Hz 궤적 구조라 궤적 자체는 부드러움(jerk 최소).
- 레포 노드 실제 TODO #1 — plan_grasp goalset이 **44-구체 충돌모델과 link-name 불일치로 비활성**돼 있었음. goalset 쓰려면 충돌구체 YAML 링크명 ↔ disable_collision_links 정합 필수(안 그럼 조용히 실패).
- Sim→Real — base offset=0(비전 T_cam2base 적용), TCP 깊이(RHP12 0.110) 실측 보정(리스크③).
- 출처 — cuRobo discussion #210/#406, report.pdf; 두산 ROS2 python_api·RT tutorials·issue#117; NVlabs/GraspGen.

## 5. 실행/검증 (안 지키면 셸 죽음·GPU OOM — stage8 context-notes.md 동일)
- 실행 — `cd ~/shelf_grasp_dev/stages/pipeline; rm -f /tmp/stage7_stop; nohup bash ./run_stage8.sh --obj-type snack > ~/shelf_grasp_dev/logs/stage8_snackuni_X.log 2>&1 & disown`. 컴파일 먼저 `py_compile`.
- 종료 — `set +e`, pkill 단독 줄 브래킷 트릭 `pkill -9 -f 'stage8_main[.]py'`. py_compile(평문 파일명)과 pkill 같은 명령 금지(자기살해 exit137). [[shell_pkill_selfkill]]. graceful=`touch /tmp/stage7_stop`.
- 결과는 PNG로 — `logs/shots/snack_*`. 창 안 보일 수 있음 → Read로 확인.
- GPU — RTX5080 16GB. GraspGen 서버 PID(~0.5GB) 죽이지 말 것. 사용자가 다른 페이지서 snack 런(~6.9GB) 돌릴 수 있음 → 절대 kill 금지, 여유<7.5GB면 확인.
- Isaac 한 번에 하나. 실행 전 `pgrep -af stage8 | grep python` 확인.

## 6. 규칙 (CLAUDE.md 발췌)
- 코드 다수 수정 전 묻기. 보고→승인→진행, 단계 독립검증. 한국어 출력·문장 종결 콜론 금지. 새 소스 첫 줄 한국어 헤더. 공식문서 먼저(추측 금지). cuRobo v0.7.8 v1 API만, GraspGen PointNet++만.
