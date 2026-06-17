# 실기체 통합 가이드 — Vision PointCloud → GraspGen → cuRobo → 매대 적치 (시뮬팀 → 실기체)

작성 2026-06-15. 시뮬(Isaac Sim, stage7_graspgen_e0509.py)에서 검증한 파이프라인을 실기체 워크스테이션으로 이식.
실기체 워크스테이션의 Claude Code 세션이 이 문서를 따라 통합. 환경은 우리와 다를 수 있으니 **환경 구축을 가장 조심**.

## 참고 파일 (이 저장소 — 경로는 GitHub 업로드 기준 상대경로)

| 파일 | 용도 |
|---|---|
| `multiobj_pipeline/REAL_ROBOT_INTEGRATION.md` | 이 문서 (통합 메인 가이드) |
| `stage7_graspgen_e0509.py` | 레퍼런스 구현 (제어로직 발췌 대상, Isaac Sim 종속부는 무시) |
| `docs/HANDOFF_shelf2_moveL.md` | 매대 적치 moveL 좌표·진입값 (실측·검증) |
| `docs/ARCHITECTURE.md` | Sim/Real 공용 GraspGen+cuRobo 구조 (이관 관점) |
| `multiobj_pipeline/pipeline_flow.md` | 동작 흐름도 + 단계별 기능표 |
| `docs/GRASPGEN_VIS_API.md` | GraspGen API 레퍼런스 |
| `docs/CUROBO_SPEED_TUNING.md` | cuRobo 속도 튜닝(plan_grasp warmup 등) |
| `robots/e0509_gripper/` (실기체 동일 시) | robot_cfg yml·URDF·충돌구체 yml — 실기체가 E0509+RH-P12면 직접 사용 |

## 0. 데이터 흐름

```
실비전(RealSense+검출) → 물체 점구름(N,3) + 물체 월드 pose(pos,quat)
   → GraspGen 서버(ZMQ, 자체 env) → 파지 후보 M개(물체프레임 4x4)+score
   → [클라이언트 측] robotiq→실그리퍼 오프셋 → 월드 변환
   → cuRobo plan_grasp(goalset, 후보 M개 통째) → 무충돌·도달 best 1개 선택
   → 접근·파지·리프트·attach → 매대앞 이동 → moveL(+y진입/-z하강/부분개방/-y후퇴) → detach → 홈
   → (다물체) 남은 물체 중 y작은·x작은 먼저 반복
```

## 1. 환경 구축 — 가장 조심할 부분

### GraspGen은 "서버/클라이언트" 2분할 (env 충돌 회피의 핵심)
- **서버**: 무거움(torch/CUDA/모델). **기존 환경 그대로 둔다 — conda든 venv든 무관.**
  우리 예: `graspgen_venv` 활성화 후 `graspgen_server.py --gripper_config <yml> --port 5556 --host 127.0.0.1`.
  실기체는 이미 ZMQ로 구축돼 있다 했으니 **서버는 손대지 말고 그대로 띄우기만** 한다.
- **클라이언트**: 매우 가벼움. `grasp_gen/serving/zmq_client.py` 주석 명시 —
  **"Only depends on pyzmq, msgpack, msgpack-numpy, numpy — no torch/CUDA needed."**
  → 따라서 **cuRobo(제어) 환경에 이 4개만 설치**하면 클라이언트가 거기서 돈다. torch 버전 충돌 걱정 없음.
  ```bash
  # cuRobo 쪽 env에서:
  pip install pyzmq msgpack msgpack-numpy numpy
  ```
- 두 환경은 ZMQ(127.0.0.1:port)로만 통신 → **서로의 파이썬/CUDA/torch 버전이 달라도 됨.** 이게 분리 설계의 이유.

### cuRobo — 버전·API 확인 필수
- **v0.7.x(v1 API)만**. 우리가 쓰는 함수: `MotionGen.plan_single / plan_grasp / plan_single_js`, `IKSolver`,
  `MotionGenConfig.load_from_robot_config`, `attach_external_objects_to_robot / detach_object_from_robot`.
  V2 API(MotionPlanner.plan_pose/GoalToolPose)면 호출 시그니처가 다르니 먼저 `import curobo; curobo.__version__` 확인.
- **로봇 config는 실기체 것으로 교체** — URDF, 충돌구체 yml, 관절한계, ee_link, base 위치는 로봇마다 다름(2절 참고).

### 점검 순서(통합 전 sanity)
```python
# 1) ZMQ 왕복 확인 (클라이언트 env에서, 더미 점구름)
import numpy as np, sys; sys.path.insert(0, "<GraspGen 경로>")
from grasp_gen.serving.zmq_client import GraspGenClient
c = GraspGenClient("127.0.0.1", 5556, wait_for_server=True)
g, s = c.infer(np.random.randn(2048,3).astype(np.float32), num_grasps=50)
print(g.shape, s.shape)   # (M,4,4),(M,) 나오면 통신 OK
# 2) cuRobo: MotionGen 로드 + plan_single 1회 성공 확인(로봇 config로)
```

## 2. GraspGen 인터페이스 계약

- **입력**: 물체 표면 점구름 `pc_obj` = `(N,3) float32`, **물체 중심 프레임**(물체 원점에 센터링). N=2048 사용.
- **호출**: `grasps_obj, scores = client.infer(pc_obj, num_grasps=K)`  (우리 K=400, 여러 후보 받아 cuRobo가 선택).
  - 반환 `grasps_obj` = `(M,4,4)` 물체프레임 파지 자세, `scores` = `(M,)` 신뢰도.
- **실비전 교체점**: 시뮬은 `sample_object_pc()`(trimesh mock). **실기체는 RealSense+검출(SAM 등) 점구름으로 교체** —
  - 물체 점들을 **물체 중심으로 평행이동**(centroid 또는 검출 pose 원점)해서 `pc_obj`(물체프레임)로.
  - 그 **물체의 월드 pose(pos, quat[w,x,y,z])는 따로 보관** → 4절 월드 변환에 사용.
  - GraspGen은 robotiq_2f_140 체크포인트로 학습 → 출력은 robotiq EE 프레임. 그리퍼 오프셋은 4절에서 보정.

## 3. 변환 — 물체프레임 파지 → 월드 (이식 가능, 함수 그대로)

```python
# robotiq(GraspGen) EE → 실그리퍼 EE Z 깊이 보정. Z는 그리퍼 접촉깊이 차(실측·시각화로 보정).
def robotiq_grasp_to_target(grasp_4x4, Z_OFFSET):
    return grasp_4x4 @ tra.translation_matrix([0, 0, Z_OFFSET])   # 우리 RH-P12는 Z=0.0

# 물체프레임 → 월드 (물체 위치+회전 반영). quat=[w,x,y,z]
def grasp_to_world(grasp_obj, obj_world_pos, obj_world_quat):
    T = np.eye(4); T[:3,3] = obj_world_pos
    w,x,y,z = obj_world_quat; T[:3,:3] = Rotation.from_quat([x,y,z,w]).as_matrix()
    return T @ grasp_obj
```
- `Z_OFFSET`는 **실그리퍼 고유값**(robotiq→실그리퍼 접촉깊이 차). 우리는 0.0, TCP 깊이는 별도 `TCP_DEPTH`로 관리.

## 4. cuRobo 선택 — plan_grasp(goalset)에 후보 통째로 (이식 가능 패턴)

서버가 준 후보 M개를 **전부** plan_grasp에 넘기고 플래너가 월드 충돌·도달로 best 1개를 고른다(휴리스틱 사전선택 금지).
```python
# 월드 파지후보 리스트 grasps_w(M,4,4) → base 프레임 Pose(1,N,7)
_pl = np.stack([g[:3,3] - ROBOT_BASE_OFFSET for g in grasps_w])[None]      # (1,N,3)
_ql = np.stack([Rotation.from_matrix(g[:3,:3]).as_quat()[[3,0,1,2]] for g in grasps_w])[None]  # (1,N,4) wxyz
gposes = Pose(position=ta.to_device(_pl), quaternion=ta.to_device(_ql))
gres = motion_gen.plan_grasp(
    cu_js.unsqueeze(0), gposes, plan_config.clone(),
    grasp_approach_offset=Pose.from_list([0,0,-PREGRASP_STANDOFF, 1,0,0,0]),
    disable_collision_links=GRIPPER_COLL_LINKS,   # 그리퍼-물체 접촉 허용(실그리퍼 링크명으로)
    plan_grasp_to_retract=False)
if gres.success.item():
    best = grasps_w[int(gres.goalset_index.item())]   # 선택된 파지
    # gres.approach_result / grasp_result 의 interpolated plan을 실행
```
- `use_cuda_graph=False` 권장(goalset↔single 혼용 시 changing-goal 에러 회피). 속도 필요시 warmup(n_goalset=N).
- **로봇 고유**: `GRIPPER_COLL_LINKS`(그리퍼 링크명), `ROBOT_BASE_OFFSET`(로봇 base 월드좌표), `PREGRASP_STANDOFF`.

## 5. 다물체 우선 파지 알고리즘 (공유 요청분)

**정책: 매대에서 먼(y 작은) + 로봇에 가까운(x 작은) 물체 먼저.** 바깥쪽(로봇 앞·매대 반대)부터 치워야 후속
운반(carry)이 남은 물체 위를 안 지나 충돌이 줄고 모션 효율이 오른다. 1순위 y오름차순, 동률 x오름차순.
```python
# 남은(pending) 타겟 중 실측 월드좌표로 선택
def _tgt_key(ti):
    p = targets[ti]["obj"].get_world_pose()[0]   # 실기체: 비전이 준 월드 pose
    return (float(p[1]), float(p[0]))            # (y, x) 오름차순
cur = min(pending_indices, key=_tgt_key)
```
- ★프레임 주의: y·x 부호는 **매대가 +y, 로봇 앞이 +x**인 우리 프레임 기준. 실기체 프레임이 다르면 부호/축을
  맞춰라(매대 반대쪽=작은 값이 되도록). 매대 방향 축을 먼저 정의하고 그 축 오름차순으로.
- 매대 빈 슬롯 점유맵·실현성 사전검사(IK·충돌·기적치 간격 ≥0.125m)도 stage7 `slot_feasible()` 참고(슬롯 간격은
  물체 폭이 아니라 **그리퍼 스윕폭**으로 잡을 것 — 1열 조밀 배치는 이웃·벽 간섭).

## 6. 매대 삽입 moveL (이식 가능 기하)

옆파지 자세로 매대 앞 시작 → +y 진입 → -z 하강 → 부분개방 릴리즈 → -y 후퇴. 시작 TCP 자세는
`side_grasp_from_approach(approach=[0,1,0], center=[x, PRE_Y, entry_z], TCP_DEPTH)`로 합성(그리퍼 상단=+z 위).
매대 층별 좌표·진입값은 `docs/HANDOFF_shelf2_moveL.md`(실측·검증본) 참고. ★2단(천장 개구 ~16cm)은
**캔을 위에서 잡으면 그리퍼+RealSense가 천장에 걸림** → 캔 위로 그리퍼가 안 솟게 잡고 **RealSense를 충돌모델에 포함**해
플래닝하면 적치 성공(그 문서 처방). 3단(천장 개방)은 제약 없음.

## 7. 이식 가능 vs 로봇 고유 (체크리스트)

| 구분 | 그대로 이식 | 실기체 고유로 교체 |
|---|---|---|
| GraspGen | ZMQ 계약·infer·점구름(N,3)·변환함수·후보 다개 전달 | 서버 env(기존), 실비전 점구름, Z_OFFSET |
| cuRobo | plan_grasp(goalset) 패턴, 다물체 선택 로직, moveL 기하 | robot_cfg(URDF/구체/관절한계/ee_link), base offset, cuRobo 설치/버전 |
| 그리퍼 | 부분개방 릴리즈 개념, 충돌링크 면제 | TCP_DEPTH, GRIP_OPEN/CLOSE/RELEASE 각, 링크명, 전류제어 인터페이스 |
| 매대 | 진입/하강/후퇴 시퀀스 | 매대 실측 좌표(층 높이·개구·앞턱), 슬롯 X |
| 프레임 | 월드↔base 변환 구조 | 로봇 base 월드좌표, 축 부호 |

## 8. 통합 순서 (실기체 Claude Code가 따라갈)

1. **ZMQ 왕복 확인**(1절 sanity) — 클라이언트 env에 pyzmq/msgpack/msgpack-numpy/numpy 설치 후 더미 infer.
2. **cuRobo 로드 확인** — 실기체 robot_cfg로 MotionGen 생성 + plan_single 1회 성공(버전/API 확인).
3. **단일 물체 경로** — 실비전 점구름 1개 → infer → 변환 → plan_grasp → 접근·파지·리프트까지.
4. **매대 적치** — attach → 매대앞 → moveL 삽입 → 부분개방 → detach → 후퇴 → 홈.
5. **다물체** — 5절 선택 + 슬롯 점유맵·사전검사 + 불가물체 스킵·사유로깅.
6. 각 단계 **구체-장애물 최소간격 로깅**(stage7 ESDF, `min_world_clearance`)으로 무접촉 수치 검증.

참고 구현 전체: `stage7_graspgen_e0509.py` (시뮬, Isaac Sim 종속부는 제어로직과 분리해 발췌).
흐름도·단계표는 `multiobj_pipeline/pipeline_flow.md`.
