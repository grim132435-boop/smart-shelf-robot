# E0509 모션 실행 + 충돌간격 코어(라이브러리) — cuRobo 궤적 실행·moveL/직접IK·그리퍼·간격질의.
"""1단계 모듈화(종류별 라이브러리). 실행 흐름(단계)이 아니라 '도구'. main/단계함수가 호출.
의존: numpy, omni ArticulationAction, curobo(Pose·CollisionQueryBuffer). 순수기하는 pp_geometry.
★_ROBOT_BASE_OFFSET는 런타임 결정(robot base) → set_base_offset로 in-place 갱신(모듈 간 공유)."""
import numpy as np
from omni.isaac.core.utils.types import ArticulationAction
from curobo.geom.sdf.world import CollisionQueryBuffer
from curobo.types.math import Pose

RENDER_EVERY = 3   # [런타임] 모션/대기 중 N스텝당 1회 렌더(물리는 매 스텝). 마지막 스텝은 캡처 위해 렌더.

# cuRobo 월드는 robot base 프레임 → 월드좌표에서 base offset 차감. main이 set_base_offset로 채움.
_ROBOT_BASE_OFFSET = np.zeros(3, dtype=np.float32)


def set_base_offset(robot_base):
    """robot base(월드)를 in-place로 설정 — import한 쪽도 같은 배열을 보므로 값 공유됨."""
    _ROBOT_BASE_OFFSET[:] = np.asarray(robot_base, dtype=np.float32)


def mat4_to_curobo_pose(mat4, tensor_args):
    from scipy.spatial.transform import Rotation
    pos    = (mat4[:3, 3] - _ROBOT_BASE_OFFSET).astype(np.float32)   # 월드→base
    q_xyzw = Rotation.from_matrix(mat4[:3, :3]).as_quat().astype(np.float32)
    q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
    return Pose(
        position=tensor_args.to_device(pos.reshape(1, 3)),
        quaternion=tensor_args.to_device(q_wxyz.reshape(1, 4)),
    )


def xyz_to_curobo_pose(pos_xyz, quat_wxyz, tensor_args):
    pos = (np.array(pos_xyz, dtype=np.float32) - _ROBOT_BASE_OFFSET)   # 월드→base
    return Pose(
        position=tensor_args.to_device(pos.reshape(1, 3)),
        quaternion=tensor_args.to_device(np.array([quat_wxyz], dtype=np.float32)),
    )


def log_arm_deg(robot, arm_joint_names, tag):
    """현재 arm 6관절 각도(deg) 로깅 — 자세 분기 진단(joint_4·6 180° 플립 추적)."""
    try:
        jp = robot.get_joint_positions()
        deg = [round(np.degrees(float(jp[robot.get_dof_index(n)])), 1) for n in arm_joint_names]
        print(f"  [관절deg:{tag}] j1~6 = {deg}", flush=True)
    except Exception as e:
        print(f"  [관절deg:{tag}] 실패: {e}", flush=True)


# [간격로깅] 실측 관절각 → 로봇 충돌구체(부착 캔 포함)-월드 장애물 최소간격(m). 세그먼트 running-min.
_CLEARANCE = {"fn": None, "buf": None, "shape": None, "w": None, "act": None}


def min_world_clearance(motion_gen, q_dev):
    """측정 관절각(cuRobo 관절순서, device tensor)에서 구체-장애물 최소간격(m).
    world_coll_checker ESDF 질의(compute_esdf=True): d>0=침투깊이, d<0=간격 → 간격 = -d.
    유효반경(r>0) 구체만 집계. 간격은 max_distance 근방 포화. 실패 시 None."""
    try:
        cc = motion_gen.world_coll_checker
        st = motion_gen.kinematics.get_state(q_dev.view(1, -1))
        sph = st.get_link_spheres().detach().view(1, 1, -1, 4).contiguous()
        radii = sph[0, 0, :, 3]
        c = _CLEARANCE
        if c["buf"] is None or c["shape"] != sph.shape:
            ta = motion_gen.tensor_args
            c["buf"] = CollisionQueryBuffer.initialize_from_shape(sph.shape, ta, cc.collision_types)
            c["shape"] = sph.shape
            c["w"] = ta.to_device([1.0])
            c["act"] = ta.to_device([0.0])
        d = cc.get_sphere_distance(sph, c["buf"], c["w"], c["act"], compute_esdf=True).view(-1)
        mask = radii > 0.0
        if not bool(mask.any()):
            return None
        return float(-(d[mask].max().item()))
    except Exception:
        return None


def make_clearance_fn(robot_art, motion_gen, tensor_args):
    """로봇 실측 관절각을 읽어 min_world_clearance를 질의하는 클로저 생성(메인 루프서 1회 등록)."""
    names = list(motion_gen.kinematics.joint_names)
    def _fn():
        try:
            jp = robot_art.get_joint_positions()
            q = np.array([float(jp[robot_art.get_dof_index(n)]) for n in names], dtype=np.float32)
            return min_world_clearance(motion_gen, tensor_args.to_device(q))
        except Exception:
            return None
    return _fn


def _clr_probe(run_min, every=5, _cnt=[0]):
    """K스텝마다 간격 질의해 running-min 갱신(GPU 동기화 비용 절감). run_min(None 가능) 반환."""
    fn = _CLEARANCE["fn"]
    if fn is None:
        return run_min
    _cnt[0] += 1
    if _cnt[0] % every != 0:
        return run_min
    c = fn()
    if c is None:
        return run_min
    return c if (run_min is None or c < run_min) else run_min


def _clr_report(run_min, tag):
    """세그먼트 종료 시 최소간격 1줄 출력. 음수=침투(접촉 발생)."""
    if run_min is None:
        return
    mark = "OK" if run_min > 0.0 else "★침투"
    print(f"  [간격:{tag}] 구체-장애물 최소 {run_min*1000:+.1f}mm ({mark})", flush=True)


def execute_plan(cmd, sim_js_names, robot, ctrl, my_world, extra_steps=1, track_tag=None,
                 arm_only=False, viz=None):
    idx_list, common = [], []
    for x in sim_js_names:
        if x in cmd.joint_names:
            # arm_only: 그리퍼 관절 제외 → 캔 든 채 운반/복귀 시 그리퍼 열려 캔 떨구는 것 방지
            if arm_only and str(x).startswith("gripper_rh"):
                continue
            idx_list.append(robot.get_dof_index(x))
            common.append(x)
    cmd_ord = cmd.get_ordered_joint_state(common)
    clr_min = None                             # [간격로깅] 세그먼트 최소간격 누적
    _nlast = len(cmd_ord.position) - 1
    for i in range(len(cmd_ord.position)):
        ctrl.apply_action(ArticulationAction(
            cmd_ord.position[i].cpu().numpy(),
            cmd_ord.velocity[i].cpu().numpy(),
            joint_indices=idx_list,
        ))
        _rnd = (i % RENDER_EVERY == 0) or (i == _nlast)   # [런타임] 렌더 솎기(마지막은 캡처 위해 렌더)
        for _ in range(extra_steps):
            my_world.step(render=_rnd)
        clr_min = _clr_probe(clr_min)
        if viz is not None and i % 10 == 0:    # 동작 중 충돌구체 실시간 갱신(팔 따라감)
            viz()
    # 추종오차(관절공간): 마지막 명령 관절각 vs 정착 후 실제 측정.
    if track_tag is not None:
        try:
            for _ in range(8):              # 정착(settle)
                my_world.step(render=True)
            last_cmd = cmd_ord.position[-1].cpu().numpy()
            jp = robot.get_joint_positions()
            meas = np.array([float(jp[robot.get_dof_index(n)]) for n in common])
            derr = np.degrees(np.abs(last_cmd - meas))
            print(f"  [추종오차:{track_tag}] 관절 최대={derr.max():.2f}° 평균={derr.mean():.2f}° "
                  f"(각축 {np.round(derr,2)})", flush=True)
        except Exception as e:
            print(f"  [추종오차:{track_tag}] 계산실패: {e}", flush=True)
    _clr_report(clr_min, track_tag or "plan")


def set_gripper(ctrl, robot_art, sim_js_names, my_world, angle, steps=60):
    """RH-P12 그리퍼 닫기/열기(물리 제어, 점진 램프). r1/l1/r2/l2 모두 mimic(×1)이라 같은 각.
    한번에 주면 stiffness 2000으로 슬램 → 현재각서 목표로 점진 보간. angle: GRIP_OPEN(0)~GRIP_CLOSE(1.05)."""
    gidx = [robot_art.get_dof_index(n) for n in sim_js_names if str(n).startswith("gripper_rh")]
    if not gidx:
        return
    try:
        cur = float(robot_art.get_joint_positions()[gidx[0]])
        # ★그리퍼 균일속도(사용자): 스텝을 |목표-현재| 각도변화에 비례 → 파지·오픈 속도 일정.
        #   0.035rad/스텝(사용자: 파지속도 느림 → 0.02→0.035로 빠르게). 작은 변화는 최소 4스텝.
        steps = max(4, int(round(abs(float(angle) - cur) / 0.035)))
        for t in np.linspace(0.0, 1.0, max(steps, 2)):
            tgt = (1.0 - t) * cur + t * float(angle)
            ctrl.apply_action(ArticulationAction(
                np.array([tgt] * len(gidx), dtype=np.float32), joint_indices=gidx))
            my_world.step(render=True)
    except Exception as e:
        print(f"  [그리퍼] 제어 실패: {e}", flush=True)


def move_direct_ik(target_world, ik_solver, tensor_args, cu_js_pos, arm_joint_names,
                   robot_art, ctrl, my_world, steps=50, settle=8, viz=None):
    """월드 4x4 목표로 직접 IK + 관절 보간 이동(짧은 동작, start-collision 검사 우회). 성공 시 True."""
    ikr = ik_solver.solve_single(mat4_to_curobo_pose(target_world, tensor_args),
                                 cu_js_pos.view(1, -1), cu_js_pos.view(1, 1, -1))
    if not ikr.success.item():
        return False
    arm_idx = [robot_art.get_dof_index(n) for n in arm_joint_names]
    q_now = cu_js_pos.view(-1).cpu().numpy()
    q_tgt = ikr.solution.view(-1)[:len(arm_joint_names)].cpu().numpy()
    clr_min = None                             # [간격로깅]
    for j, t in enumerate(np.linspace(0.0, 1.0, steps)):
        q = ((1 - t) * q_now + t * q_tgt).astype(np.float32)
        ctrl.apply_action(ArticulationAction(q, joint_indices=arm_idx))
        my_world.step(render=True)
        clr_min = _clr_probe(clr_min)
        if viz is not None and j % 10 == 0:    # 동작 중 충돌구체 실시간 갱신
            viz()
    for _ in range(settle):
        my_world.step(render=True)
    _clr_report(clr_min, "directIK")
    return True


def lowest_sphere_bottom_world(kin, q_vec, base):
    """관절각 q에서 로봇 충돌구체 중 최저 구체의 바닥 월드 z. 책상 침범 판정용. 실패/미지원 시 None."""
    if kin is None:
        return None
    try:
        st  = kin.get_state(q_vec.view(1, -1))
        sph = st.get_link_spheres().reshape(-1, 4).detach().cpu().numpy()
    except Exception:
        return None
    low = None
    for (x, y, z, r) in sph:
        if r <= 0.0:
            continue
        b = float(z) + base[2] - float(r)
        if low is None or b < low:
            low = b
    return low


def highest_sphere_top_world(kin, q_vec, base):
    """관절각 q에서 로봇 충돌구체 중 최고 구체의 꼭대기 월드 z. 매대 천장 간섭 판정용. 실패/미지원 시 None."""
    if kin is None:
        return None
    try:
        st  = kin.get_state(q_vec.view(1, -1))
        sph = st.get_link_spheres().reshape(-1, 4).detach().cpu().numpy()
    except Exception:
        return None
    top = None
    for (x, y, z, r) in sph:
        if r <= 0.0:
            continue
        t = float(z) + base[2] + float(r)
        if top is None or t > top:
            top = t
    return top


def move_linear_ik(start_world, target_world, ik_solver, tensor_args, cu_js_pos,
                   arm_joint_names, robot_art, ctrl, my_world,
                   waypoints=40, substeps=2, settle=10, viz=None, tag=""):
    """TCP를 start→target 데카르트 직선으로 이동(moveL). 위치 선형보간, 자세는 start→target 보간.
    start_world와 target_world의 회전이 다르면 t에 따라 회전도 선형보간(틸트 점진 적용 등).
    웨이포인트마다 IK 풀어 따라감. IK 실패 시 그 지점에서 중단하고 False(부분 이동)."""
    arm_idx = [robot_art.get_dof_index(n) for n in arm_joint_names]
    nA = len(arm_joint_names)
    p0 = np.asarray(start_world[:3, 3],  dtype=np.float64)
    p1 = np.asarray(target_world[:3, 3], dtype=np.float64)
    R0 = np.asarray(start_world[:3, :3],  dtype=np.float64)
    R1 = np.asarray(target_world[:3, :3], dtype=np.float64)
    _interp_R = not np.allclose(R0, R1, atol=1e-6)   # 회전이 다를 때만 보간
    # ★TCP 균일속도(사용자): 웨이포인트를 거리비례로 산출(고정 waypoints면 짧은구간 느리고 긴구간 빠름).
    #   웨이포인트당 12mm → 모든 moveL 구간의 TCP 선속도 동일. (4mm는 너무 잘아 텍타임 3배 → 12mm로 적정화)
    _dist = float(np.linalg.norm(p1 - p0))
    waypoints = max(8, int(round(_dist / 0.012)))
    q_prev = cu_js_pos.view(-1).cpu().numpy()[:nA].astype(np.float32)
    seed   = cu_js_pos.view(1, 1, -1)
    clr_min = None                             # [간격로깅]
    for i in range(1, waypoints + 1):
        t  = i / waypoints
        Tw = np.eye(4, dtype=np.float32)
        if _interp_R:
            # 회전 선형보간 후 QR 재정규화(짧은 회전에서 충분히 정확)
            _Ri = (1.0 - t) * R0 + t * R1
            _U, _, _Vt = np.linalg.svd(_Ri); Tw[:3, :3] = (_U @ _Vt).astype(np.float32)
        else:
            Tw[:3, :3] = R1
        Tw[:3, 3]  = (1.0 - t) * p0 + t * p1
        ikr = ik_solver.solve_single(mat4_to_curobo_pose(Tw, tensor_args),
                                     cu_js_pos.view(1, -1), seed)
        if not ikr.success.item():
            print(f"  [moveL{(' ' + tag) if tag else ''}] 웨이포인트 {i}/{waypoints} IK 실패 → 중단", flush=True)
            return False
        q_tgt = ikr.solution.view(-1)[:nA].cpu().numpy().astype(np.float32)
        _rnd = (i % RENDER_EVERY == 0) or (i == waypoints)   # [런타임] 렌더 솎기(마지막 웨이포인트는 렌더)
        for s in range(1, substeps + 1):          # 드라이브 추종용 미세 보간
            q = ((1.0 - s / substeps) * q_prev + (s / substeps) * q_tgt).astype(np.float32)
            ctrl.apply_action(ArticulationAction(q, joint_indices=arm_idx))
            my_world.step(render=(_rnd and s == substeps))
        q_prev = q_tgt
        seed   = ikr.solution.view(1, 1, -1)      # 다음 IK는 현 해로 시드 → 관절 연속성
        clr_min = _clr_probe(clr_min)
        if viz is not None and i % 5 == 0:
            viz()
    for _ in range(settle):
        my_world.step(render=True)
    _clr_report(clr_min, f"moveL{(' ' + tag) if tag else ''}")
    return True
