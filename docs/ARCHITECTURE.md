# ARCHITECTURE.md — Sim/Real 공용 GraspGen+cuRobo 파지 파이프라인

> 작성: 남정혁 | 2026-06 | 대상: shelf_grasp_dev (개인 작업) ↔ smart-shelf-robot (팀 ROS2 레포)
> 목적: **시뮬에서 검증한 GraspGen+cuRobo 코어를 코드 변경 없이 실기체(ROS2)로 이관**할 수 있는 구조.

---

## 1. 배경 / 목표

- 실로봇 설치 전 **Isaac Sim에서 태스크 검증**: 다양한 물체의 점구름 → GraspGen이 최선 파지점 선정 → cuRobo로 이동.
- 검증되면 **동일한 GraspGen·cuRobo를 실기체에 적용**.
- 따라서 **GraspGen+cuRobo 코어는 sim/real 공용**으로 두고, 바뀌는 부분(점구름 입력, 로봇·그리퍼)만 어댑터로 분리한다.

팀 레포(smart-shelf-robot) 기준 (functional_spec / interface_definition):
- 로봇 **Doosan E0509** + 그리퍼 **RH-P12-RN-A**, RealSense **D455f**(eye-in-hand), **ROS2 Humble**, `ROS_DOMAIN_ID=100`.
- "온라인 파지 생성 노드"는 **isaacsim conda 환경에서 ZMQ로 GraspGen 질의 + cuRobo 실행**, `/move_to_pick` 서비스로 트리거 → **본 프로젝트가 그 노드의 코어**.

---

## 2. 핵심 원리: 공용 코어 + 교체 가능한 어댑터

```
            [ 점구름 입력 ]          [ 공용 코어 ]              [ 로봇 실행 ]
sim:   Isaac Sim 물체 PC  ─┐                                ┌─ Isaac Sim Franka+Robotiq
                          ├─▶ GraspGen 파지선택 → cuRobo 궤적 ─┤
real:  /object_pointcloud ─┘   (sim/real 동일 코드)           └─ ROS2 E0509 + RH-P12-RN-A
       (RealSense+SAM)                                          (/joint_command, /gripper/grasp)
```

실기체 이관 = **perception_bridge + robot_adapter 구현만 교체**. 코어(grasp_inference, motion_planning, transforms)는 불변.

---

## 3. 디렉토리 구조 (제안)

```
shelf_grasp_dev/
├── grasp_inference/        # [공용] GraspGen
│   ├── client.py           #   ZMQ 클라이언트 (.infer: PC(N,3) → 파지 후보 (M,4,4)+score)
│   └── selection.py        #   파지 선택 필터 (작업공간/수직도 APPROACH_Z_MAX/수평닫힘/IK + fallback)
├── motion_planning/        # [공용] cuRobo v1 API
│   └── planner.py          #   MotionGen/IKSolver 래퍼 (목표 pose → 충돌없는 궤적)
├── common/
│   ├── transforms.py       # [공용] 좌표변환 (grasp→ee, obj→world(회전포함), mat4↔Pose)
│   └── types.py            #   GraspCandidate, Trajectory 등 공용 dataclass
├── perception_bridge/      # 점구름 입력 추상화 (sim/real 교체점)
│   ├── base.py             #   PointCloudSource: get_object_pcs() → {label:(N,3)} (robot base frame)
│   ├── sim_source.py       #   Isaac Sim 물체 → PC (현 sample_cube_pc 일반화, 다물체)
│   └── ros2_source.py      #   /object_pointcloud(PointCloud2) 구독 → (N,3)  [Stage4]
├── robot_adapter/          # 로봇·그리퍼 추상화 (sim/real 교체점)
│   ├── base.py             #   RobotAdapter: get_joint_state(), execute_traj(), gripper_grasp(current), EE 오프셋
│   ├── sim_franka.py       #   Isaac Sim Franka + Robotiq (현재, ROBOTIQ_TO_FRANKA_Z 캡슐화)
│   └── ros2_e0509.py       #   ROS2 E0509 + RH-P12-RN-A (/joint_command, /gripper/grasp)  [Stage4]
├── pipeline/
│   ├── grasp_pipeline.py   # [공용] 조립: PC → 파지선택 → 궤적 (source/adapter 주입)
│   ├── sim_runner.py       #   Isaac Sim 상태머신 (현 stage3_graspgen_curobo_gui.py 역할)
│   └── ros2_node.py        #   ROS2 노드: /move_to_pick(Trigger) → grasp_pipeline  [Stage4]
├── grasp_viz.py            # 시각화 (Isaac Sim USD 좌표축 + Viser) — 이미 분리됨
└── docs/
```

> 현재는 `stage3_graspgen_curobo_gui.py` 한 파일에 위 책임이 모두 섞여 있음 → 위 구조로 점진 분리.

---

## 4. 팀 레포 ROS2 인터페이스 ↔ 본 프로젝트 매핑

> ⚠️ **이 인터페이스는 잠정(미확정)이며 계속 바뀔 수 있다.** 따라서 토픽/서비스/메시지 이름에
> 코어를 직접 결합하지 않는다. 변경은 **어댑터 계층(perception_bridge / robot_adapter)에서만 흡수**하고,
> 공용 코어(grasp_inference / motion_planning / transforms)는 ROS2·팀 레포를 **전혀 모르는 순수 입출력**
> (PC(N,3) → 파지 → 궤적)으로 유지한다. → 팀 파이프라인이 바뀌어도 어댑터 한 파일만 고치면 됨.

| 팀 레포 인터페이스 (잠정) | 타입 | 본 프로젝트 모듈 |
|---|---|---|
| `/object_pointcloud` | sensor_msgs/PointCloud2 | perception_bridge/ros2_source.py |
| `/object_class` | (분류) | pipeline (클래스→그리퍼 전류 매핑) |
| `/move_to_pick` | std_srvs/Trigger | pipeline/ros2_node.py (트리거) |
| `/joint_command` | (관절 궤적) | robot_adapter/ros2_e0509.py |
| `/gripper/grasp` | control_msgs/GripperCommand (전류) | robot_adapter/ros2_e0509.py |
| `/gripper/open` | std_srvs/Trigger | robot_adapter/ros2_e0509.py |
| cuRobo 궤적 | — | motion_planning/planner.py (공용) |

**클래스별 그리퍼 전류** (robot_adapter에서 적용): snack 300mA / bottle 600mA / can 1000mA.

---

## 5. 데이터 흐름

### Sim 검증 모드 (현재)
```
sim_source(Isaac Sim 물체 PC) → grasp_pipeline
  → grasp_inference.client.infer(PC) → selection (수직도+IK 필터)
  → motion_planning.planner (cuRobo 궤적)
  → sim_franka.execute_traj + gripper → grasp_viz
```

### Real ROS2 모드 (Stage4, 예정)
```
/move_to_pick (Trigger) → ros2_node → grasp_pipeline
  → ros2_source(/object_pointcloud) → [동일 코어: infer → selection → cuRobo]
  → ros2_e0509.execute_traj(/joint_command) + gripper_grasp(/gripper/grasp, 클래스별 전류)
```

---

## 6. 이관 체크리스트 (sim → real)

- [ ] perception_bridge: sim_source → ros2_source (`/object_pointcloud` 구독, 좌표를 robot base 프레임으로)
- [ ] robot_adapter: sim_franka → ros2_e0509 (E0509 관절, RH-P12-RN-A 전류 그리퍼)
- [ ] EE 오프셋 재계산: `ROBOTIQ_TO_FRANKA_Z` → RH-P12-RN-A 기준 (robot_adapter 내부)
- [ ] cuRobo 로봇 config: franka.yml → e0509_gripper.yml (이미 `~/curobo_ws/robots/e0509_gripper/` 존재)
- [ ] 충돌 월드: collision_table.yml → 실제 매대 USD/충돌 모델
- [ ] 좌표계: 점구름·파지가 모두 **robot base 프레임** 기준인지 확인 (비전팀과 합의)
- [ ] 코어(grasp_inference / motion_planning / transforms)는 **변경 없음** 확인

---

## 7. 진행 방식

1. 본 문서 = 목표 구조 (합의용).
2. **점진 리팩토링**: 한 모듈씩 분리하며 매 단계 `sim_runner`(=현 stage3) 동작 유지.
   순서 제안: transforms → grasp_inference → motion_planning → perception_bridge(base+sim) → robot_adapter(base+sim) → pipeline 조립.
3. Stage4에서 ros2_source / ros2_e0509 / ros2_node 추가 → 팀 레포 `feat/rl/src`에 통합.

> 업로드 대상: https://github.com/StealthBlack66/smart-shelf-robot/tree/feat/rl/src
> (CLAUDE.md 격리 규칙상 팀 레포 직접 수정은 사용자 승인하에 별도 단계로 진행)
