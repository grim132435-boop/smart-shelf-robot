#!/usr/bin/env python3
# Phase0 스파이크: cuRobo plan_grasp/plan_goalset/attach가 e0509 cfg+매대월드에서 동작하는지 헤드리스 검증
#   (Isaac Sim 불필요 — 순수 cuRobo. 키스톤 API 동작·시그니처·반환을 빠르게 확인해 Phase1 통합 리스크 제거)
import numpy as np
from scipy.spatial.transform import Rotation

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.geom.types import WorldConfig, Cuboid
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.util_file import load_yaml, join_path, get_world_configs_path

ROBOT_DIR = "/home/devuser/curobo_ws/robots/e0509_gripper"
ROBOT_YML = f"{ROBOT_DIR}/e0509_gripper.yml"
RETRACT   = [-0.610865, 0.785398, 1.396263, 1.134464, 2.007129, -0.698132]
TCP_DEPTH = 0.060
ROBOT_BASE_Z = 0.73          # 월드 로봇 base z (월드0 바닥을 base프레임으로 내릴 때 사용)

# 캔(목표) — base 프레임. stage4: 월드[0.25,-0.04,0.769] = base[0.50,0.0,0.039]
CAN_CENTER = np.array([0.50, 0.0, 0.039])
CAN_HALF   = 0.0675          # 13.5cm/2
GRASP_FRAC = 0.7             # 캔 상단부(stage4 검증값) — 손목 책상 클리어


def side_grasp(approach, center, tcp_depth):
    """stage4 side_grasp_from_approach 동일 로직(순수 numpy). 4x4 반환(base 프레임)."""
    a = np.asarray(approach, float).copy(); a[2] = 0.0
    n = np.linalg.norm(a)
    if n < 1e-3:
        return None
    a /= n
    Y = np.array([-a[1], a[0], 0.0])
    X = np.cross(Y, a); X /= (np.linalg.norm(X) + 1e-9)
    if X[2] < 0:
        X, Y = -X, -Y
    Y = np.cross(a, X); Y /= (np.linalg.norm(Y) + 1e-9)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = X, Y, a
    T[:3, 3] = np.asarray(center, float) - tcp_depth * a
    return T


def build_world():
    """floor 슬랩(월드0으로 내림) + 책상 + 3단 매대 큐보이드 (base 프레임, cuRobo 장애물 실측값)."""
    wt = WorldConfig.from_dict(load_yaml(join_path(get_world_configs_path(), "collision_table.yml")))
    wt.cuboid[0].pose[2] = -ROBOT_BASE_Z - wt.cuboid[0].dims[2] / 2.0   # 슬랩 윗면=월드0
    extra = [
        Cuboid(name="table1",  pose=[0.25, 0.04,  -0.38, 1, 0, 0, 0], dims=[1.2, 0.6, 0.7]),
        Cuboid(name="table2",  pose=[0.25, 0.565, -0.36, 1, 0, 0, 0], dims=[1.8, 0.45, 0.72]),
        Cuboid(name="s3_floor", pose=[0.5,  0.54, 0.395, 1, 0, 0, 0], dims=[0.365, 0.285, 0.03]),
        Cuboid(name="s3_lip",   pose=[0.5,  0.41, 0.415, 1, 0, 0, 0], dims=[0.325, 0.025, 0.01]),
        Cuboid(name="s3_wallR", pose=[0.673, 0.54, 0.49, 1, 0, 0, 0], dims=[0.02, 0.285, 0.16]),
        Cuboid(name="s3_wallL", pose=[0.327, 0.54, 0.49, 1, 0, 0, 0], dims=[0.02, 0.285, 0.16]),
        Cuboid(name="s3_back",  pose=[0.5,  0.673, 0.49, 1, 0, 0, 0], dims=[0.325, 0.02, 0.16]),
    ]
    return WorldConfig(cuboid=list(wt.cuboid) + extra)


def make_grasp_poses(tensor_args):
    """캔 상단부 side 파지 후보 N개(정면±azimuth 스윕) → Pose(1,N,7) base 프레임."""
    gc = CAN_CENTER + np.array([0, 0, GRASP_FRAC * CAN_HALF])
    ang0 = np.arctan2(gc[1], gc[0])     # base 원점(로봇)→캔 정면
    pos_list, quat_list = [], []
    for deg in [0, -20, 20, -40, 40, -60, 60, -80, 80]:
        ang = ang0 + np.radians(deg)
        a = np.array([np.cos(ang), np.sin(ang), 0.0])
        T = side_grasp(a, gc, TCP_DEPTH)
        if T is None:
            continue
        q = Rotation.from_matrix(T[:3, :3]).as_quat()   # xyzw
        pos_list.append(T[:3, 3])
        quat_list.append([q[3], q[0], q[1], q[2]])      # wxyz
    N = len(pos_list)
    pos = tensor_args.to_device(np.array(pos_list, dtype=np.float32)).view(1, N, 3)
    quat = tensor_args.to_device(np.array(quat_list, dtype=np.float32)).view(1, N, 4)
    return Pose(position=pos, quaternion=quat), N


def main():
    tensor_args = TensorDeviceType()

    # ── robot cfg (stage4와 동일 수정) ──
    robot_cfg = load_yaml(ROBOT_YML)["robot_cfg"]
    robot_cfg["kinematics"]["cspace"]["retract_config"] = list(RETRACT)
    robot_cfg["kinematics"]["urdf_path"]       = f"{ROBOT_DIR}/e0509_gripper_abs.urdf"
    robot_cfg["kinematics"]["asset_root_path"] = ROBOT_DIR
    robot_cfg["kinematics"]["collision_spheres"] = f"{ROBOT_DIR}/e0509_spheres.yml"
    robot_cfg["kinematics"]["lock_joints"]     = {"gripper_rh_r1": 0.0}
    _kin = robot_cfg["kinematics"]
    if "base_link" in _kin.get("collision_link_names", []):
        _kin["collision_link_names"] = [l for l in _kin["collision_link_names"] if l != "base_link"]
    if isinstance(_kin.get("self_collision_ignore"), dict):
        _kin["self_collision_ignore"].pop("base_link", None)
        for _k in _kin["self_collision_ignore"]:
            _kin["self_collision_ignore"][_k] = [x for x in _kin["self_collision_ignore"][_k] if x != "base_link"]
    if isinstance(_kin.get("self_collision_buffer"), dict):
        _kin["self_collision_buffer"].pop("base_link", None)
    j_names = robot_cfg["kinematics"]["cspace"]["joint_names"]
    print(f"[cfg] arm joints={j_names}", flush=True)

    world_cfg = build_world()
    print(f"[world] cuboid {len(world_cfg.cuboid)}개 (floor+책상+3단)", flush=True)

    mg_cfg = MotionGenConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args,
        collision_checker_type=CollisionCheckerType.PRIMITIVE,   # 월드가 전부 큐보이드 → MESH(warp.torch 의존) 불필요
        num_trajopt_seeds=12, num_graph_seeds=12, interpolation_dt=0.03,
        collision_cache={"obb": 60, "mesh": 60},
        collision_activation_distance=0.005,
        optimize_dt=True, trajopt_dt=None, trajopt_tsteps=32, trim_steps=[1, None],
        use_cuda_graph=False,   # 스파이크: single↔goalset 전환 시 cuda graph 충돌 회피(속도만 느림)
    )
    motion_gen = MotionGen(mg_cfg)
    print("[warmup] ...", flush=True)
    motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
    print("[ready] cuRobo MotionGen", flush=True)

    start = JointState.from_position(
        tensor_args.to_device(np.array(RETRACT, dtype=np.float32)).view(1, -1),
        joint_names=list(j_names),
    )
    grasp_poses, N = make_grasp_poses(tensor_args)
    print(f"[grasp] 후보 {N}개 생성 (캔 상단부 side, base프레임)", flush=True)

    plan_cfg = MotionGenPlanConfig(enable_graph=False, enable_graph_attempt=4,
                                   max_attempts=8, enable_finetune_trajopt=True,
                                   time_dilation_factor=0.7)

    # ── 1) plan_grasp: 후보 묶음 → best 선택 + approach/grasp/retract ──
    print("\n=== [1] plan_grasp ===", flush=True)
    try:
        gripper_links = [l for l in _kin.get("collision_link_names", []) if "gripper" in l]
        res = motion_gen.plan_grasp(
            start, grasp_poses, plan_cfg,
            disable_collision_links=gripper_links,
        )
        print(f"  success={getattr(res,'success',None)} status={getattr(res,'status',None)}", flush=True)
        for attr in ("grasp_trajectory", "approach_trajectory", "retract_trajectory",
                     "grasp_interpolated_trajectory", "goalset_index", "planning_time"):
            if hasattr(res, attr):
                v = getattr(res, attr)
                try:
                    info = v.shape if hasattr(v, "shape") else (len(v) if hasattr(v, "__len__") else v)
                except Exception:
                    info = type(v)
                print(f"    {attr}: {info}", flush=True)
        print(f"    [속성목록] {[a for a in dir(res) if not a.startswith('_')]}", flush=True)
    except Exception as e:
        import traceback; traceback.print_exc()
        print(f"  plan_grasp 예외: {e}", flush=True)

    # ── 2) plan_goalset: 여러 목표 중 best ──
    print("\n=== [2] plan_goalset ===", flush=True)
    try:
        res_gs = motion_gen.plan_goalset(start, grasp_poses, plan_cfg)
        print(f"  success={res_gs.success} status={getattr(res_gs,'status',None)} "
              f"goalset_index={getattr(res_gs,'goalset_index',None)}", flush=True)
        if bool(res_gs.success.any()):
            traj = res_gs.get_interpolated_plan()
            print(f"    무충돌 궤적 길이={traj.position.shape}", flush=True)
    except Exception as e:
        import traceback; traceback.print_exc()
        print(f"  plan_goalset 예외: {e}", flush=True)

    # ── 3) attach_spheres → plan_single → detach ──
    print("\n=== [3] attach_spheres / detach ===", flush=True)
    try:
        # 캔 근사 스피어 4개(attached_object 링크에 pre-alloc 되어 있어야 함; cfg에 extra 4개 있음)
        sph = tensor_args.to_device(np.array([
            [0.0, 0.0,  0.03, 0.03],
            [0.0, 0.0,  0.00, 0.03],
            [0.0, 0.0, -0.03, 0.03],
            [0.0, 0.0, -0.06, 0.03],
        ], dtype=np.float32))
        motion_gen.attach_spheres_to_robot(sphere_tensor=sph, link_name="attached_object")
        print("  attach_spheres_to_robot OK (attached_object에 캔 스피어 4개 부착)", flush=True)
        # 부착 상태로 home 위 어딘가로 plan_single (부피 반영 확인용 단발)
        goal = Pose(position=tensor_args.to_device(np.array([[0.4, 0.0, 0.3]], dtype=np.float32)),
                    quaternion=tensor_args.to_device(np.array([[0, 1, 0, 0]], dtype=np.float32)))
        r2 = motion_gen.plan_single(start, goal, plan_cfg)
        print(f"  부착상태 plan_single success={r2.success} status={getattr(r2,'status',None)}", flush=True)
        motion_gen.detach_object_from_robot("attached_object")
        print("  detach_object_from_robot OK", flush=True)
    except Exception as e:
        import traceback; traceback.print_exc()
        print(f"  attach 테스트 예외: {e}", flush=True)

    print("\n[Phase0 스파이크 종료]", flush=True)


if __name__ == "__main__":
    main()
