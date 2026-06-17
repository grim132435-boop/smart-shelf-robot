#!/usr/bin/env python3
"""
Stage 3: GraspGen 파지 자세 → cuRobo 목표 연결 (GUI)
아키텍처:
  [GraspGen ZMQ Server:5556] ← 큐브 점구름 전송
                              → 6-DOF 파지 자세 (4x4, robotiq frame)
  Z오프셋 변환: robotiq(depth=0.195m) → franka(depth=0.1053m)
  cuRobo plan_single() → Isaac Sim 실행
"""

try:
    import isaacsim
except ImportError:
    pass

import torch
_ = torch.zeros(4, device="cuda:0")

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--robot",    type=str,   default="franka.yml")
parser.add_argument("--port",     type=int,   default=5556)
parser.add_argument("--cycles",   type=int,   default=3)
parser.add_argument("--viser-port", type=int, default=8081,
                    help="파지 Viser 웹뷰어 포트 (Stage1 visualize_grasps.py 8080과 겹치지 않게)")
parser.add_argument("--obj-type", type=str, default="box", choices=["box", "cylinder"],
                    help="파지 대상 물체 형상 (box=snack 근사, cylinder=can 근사)")
parser.add_argument("--attach-mode", type=str, default="physical", choices=["physical", "kinematic"],
                    help="physical=실제 물리 파지(마찰·무게 적용, 안 잡히면 실패), "
                         "kinematic=강제 부착(데모용, 항상 들림)")
parser.add_argument("--grip-force", type=float, default=0.0,
                    help="그리퍼 손가락 max_effort(파지 조임력). 0=USD 기본값 유지, "
                         "양수면 그 값으로 설정 (물리 파지 시 무거운/미끄러운 물체용). 실기체 전류제어에 대응")
args = parser.parse_args()

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": "1920", "height": "1080"})

import sys, time
import numpy as np
import trimesh
import trimesh.transformations as tra
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics

import carb
# 이 파일은 shelf_grasp_dev 사본 → 실제 helper.py는 curobo_ws에 있음
sys.path.insert(0, "/home/devuser/curobo_ws")
from helper import add_extensions, add_robot_to_scene
# 파지 시각화 모듈 (이 스크립트와 같은 디렉토리)
from pathlib import Path as _Path
sys.path.insert(0, str(_Path(__file__).resolve().parent))
from grasp_viz import (
    draw_grasp_candidates_usd, clear_grasp_viz_usd, ViserGraspViz,
)
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.materials import PhysicsMaterial

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.util.logger import setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import (
    get_robot_configs_path, get_world_configs_path, join_path, load_yaml,
)
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

# ── 상수 ──────────────────────────────────────────────────────────────────────
CUBE_SIZE         = 0.05
CUBE_MASS         = 0.15
TABLE_Z           = 0.0
CUBE_Z            = TABLE_Z + CUBE_SIZE / 2        # 0.025m

# Robotiq→Franka EE 프레임 변환 오프셋 (Z축)
# 수학적 유도: contact = T_robotiq @ [0,0,0.195,1] = T_franka @ [0,0,0.1053,1]
# → T_franka = T_robotiq @ T([0,0,+(0.195-0.1053)])  (양수: EE를 큐브 방향으로)
ROBOTIQ_TO_FRANKA_Z = +(0.195 - 0.1053)           # +0.0897m

# 파지 전 후퇴 거리 (pre-grasp)
PREGRASP_STANDOFF = 0.15                           # m

# 파지 approach 수직도 임계 (월드 Z성분). -1.0=완전수직, 값이 -1에 가까울수록 엄격.
# -0.90 ≈ 수평 기울기 25° 이내만 허용 (사선 파지 배제). 후보 없으면 호출부에서 완화.
APPROACH_Z_MAX        = -0.90
APPROACH_Z_MAX_RELAX  = -0.80                      # fallback(완화) 임계 (약 37° 이내)

FINGER_OPEN   = 0.04
FINGER_GRASP  = 0.015
NUM_PC_POINTS = 2048

# 물체 종류별 사양 (B1: Isaac Sim 기본도형으로 도메인 물체 근사)
#   box = snack 근사(5cm 큐브), cylinder = can 근사(r3cm h10cm)
#   grasp_mode: "top"=윗면 수직 파지, "side"=옆면 수평 파지(캔/병을 세워서 매대에)
OBJ_SPECS = {
    "box":      {"z": CUBE_SIZE / 2,                        "mass": 0.15, "grasp_mode": "top"},
    "cylinder": {"z": 0.05, "radius": 0.03, "height": 0.10, "mass": 0.50, "grasp_mode": "side"},  # 음료캔 500g
}

class GS:
    IDLE          = "IDLE"
    QUERY_GRASP   = "QUERY_GRASP"
    OPEN_GRIPPER  = "OPEN_GRIPPER"
    PLAN_PREGRASP = "PLAN_PREGRASP"
    MOVE_PREGRASP = "MOVE_PREGRASP"
    PLAN_GRASP    = "PLAN_GRASP"
    MOVE_GRASP    = "MOVE_GRASP"
    CLOSE_GRIPPER = "CLOSE_GRIPPER"
    PLAN_LIFT     = "PLAN_LIFT"
    MOVE_LIFT     = "MOVE_LIFT"
    HOLD          = "HOLD"
    OPEN_DROP     = "OPEN_DROP"


# ── 헬퍼 ──────────────────────────────────────────────────────────────────────

def rand_yaw_quat() -> np.ndarray:
    """랜덤 yaw(Z축 회전) 쿼터니언 [w,x,y,z].
    큐브를 다양한 방향으로 배치 → 회전 큐브에서 '면' 파지가 되는지 검증 + 실환경(임의 자세) 모사.
    """
    yaw = np.random.uniform(-np.pi, np.pi)
    return np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])


def apply_gripper(ctrl, finger_idx, pos):
    ctrl.apply_action(ArticulationAction(
        joint_positions=np.array([pos, pos]),
        joint_indices=np.array(finger_idx),
    ))


def get_ee_world_pos(stage, path="/World/panda/panda_hand"):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    T = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array([T[3][0], T[3][1], T[3][2]])


def set_kinematic(stage, path, enabled):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(enabled)


def sample_object_pc(obj_type="box", n=NUM_PC_POINTS):
    """물체 종류별 표면 점구름 (오브젝트 중심 기준 프레임). B1: 기본도형 근사.
    실기체에선 이 함수를 RealSense+SAM 점구름(perception_bridge)으로 교체.
    """
    if obj_type == "cylinder":
        s = OBJ_SPECS["cylinder"]
        mesh = trimesh.creation.cylinder(radius=s["radius"], height=s["height"])
    else:  # box
        mesh = trimesh.creation.box([CUBE_SIZE] * 3)
    pts, _ = trimesh.sample.sample_surface(mesh, n)
    return pts.astype(np.float32)           # 이미 0 중심


def robotiq_grasp_to_franka(grasp_4x4: np.ndarray) -> np.ndarray:
    """robotiq EE 프레임 → franka panda_hand 프레임 변환
    GraspGen FAQ: new_grasp = grasp @ tra.translation_matrix([0, 0, -Z_OFFSET])
    """
    return grasp_4x4 @ tra.translation_matrix([0, 0, ROBOTIQ_TO_FRANKA_Z])


def grasp_to_world(grasp_obj: np.ndarray, cube_world_pos: np.ndarray,
                   cube_world_quat: np.ndarray = None) -> np.ndarray:
    """오브젝트 프레임 파지 자세 → 월드 프레임 (큐브 위치 + 회전 반영).
    cube_world_quat: Isaac Sim 쿼터니언 [w,x,y,z]. None이면 회전 무시(이전 동작).
    """
    T_world = np.eye(4)
    T_world[:3, 3] = cube_world_pos
    if cube_world_quat is not None:
        w, x, y, z = cube_world_quat
        T_world[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
    return T_world @ grasp_obj


def mat4_to_curobo_pose(mat4: np.ndarray, tensor_args):
    """4x4 행렬 → cuRobo Pose (쿼터니언 [w,x,y,z])"""
    pos    = mat4[:3, 3].astype(np.float32)
    q_xyzw = Rotation.from_matrix(mat4[:3, :3]).as_quat().astype(np.float32)
    q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
    return Pose(
        position=tensor_args.to_device(pos.reshape(1, 3)),
        quaternion=tensor_args.to_device(q_wxyz.reshape(1, 4)),
    )

# 이전 이름 호환
grasp_to_curobo_pose = mat4_to_curobo_pose


def is_in_franka_workspace(pos: np.ndarray) -> bool:
    """Franka 작업공간 기본 필터 (x,y,z 범위)"""
    x, y, z = pos
    r = np.sqrt(x**2 + y**2)
    return (0.15 < r < 0.75 and        # 수평 거리
            0.0  < z < 0.70 and         # 높이
            x    > 0.1)                 # 로봇 뒤편 제외


def snap_grasp_roll_90(grasp_4x4: np.ndarray, cube_R: np.ndarray = None) -> np.ndarray:
    """닫힘축(X)을 큐브 면(로컬 X/Y축)에 90° 단위로 스냅 → 큐브 평행면 파지 보장.
    cube_R: 큐브 회전행렬(3x3). None이면 세계 X/Y축 기준(이전 동작, 큐브 회전 0 가정).
    ※ 큐브가 회전한 경우 cube_R을 줘야 모서리가 아닌 '면'을 잡는다.
    approach 방향(-Z world에 가까움)을 축으로 회전 조정.
    """
    R        = grasp_4x4[:3, :3]
    approach = R[:, 2]          # gripper +Z (approach)
    closing  = R[:, 0]          # gripper +X (닫힘)
    grip_y   = R[:, 1]          # gripper +Y

    # 닫힘축을 정렬시킬 후보 축: 큐브 로컬 X/Y(회전 반영) 또는 세계축(fallback)
    if cube_R is not None:
        ax, ay = cube_R[:3, 0], cube_R[:3, 1]
        candidates = [ax, -ax, ay, -ay]
    else:
        candidates = [np.array([1., 0, 0]), np.array([-1., 0, 0]),
                      np.array([0, 1., 0]), np.array([0, -1., 0])]
    dots = [np.dot(closing, c) for c in candidates]
    best_world = candidates[int(np.argmax(dots))]

    # approach와 수직인 성분으로 투영
    best_perp = best_world - np.dot(best_world, approach) * approach
    norm = np.linalg.norm(best_perp)
    if norm < 1e-6:
        return grasp_4x4
    best_perp /= norm

    # 현재 closing과 target 사이 각도 계산
    cos_a = np.clip(np.dot(closing, best_perp), -1, 1)
    sin_a = np.dot(np.cross(closing, best_perp), approach)
    angle = np.arctan2(sin_a, cos_a)

    if abs(angle) < 0.05:       # 이미 정렬됨
        return grasp_4x4

    # Rodrigues 회전 (approach 축 기준)
    c, s = np.cos(angle), np.sin(angle)
    def rot(v):
        return c*v + s*np.cross(approach, v) + (1-c)*np.dot(approach, v)*approach

    new_closing = rot(closing)
    new_y       = rot(grip_y)
    new_R = np.column_stack([new_closing, new_y, approach])
    out = grasp_4x4.copy()
    out[:3, :3] = new_R
    return out


def select_best_reachable_grasp(
    grasps_world: np.ndarray,
    scores: np.ndarray,
    ik_solver: IKSolver,
    tensor_args,
    cu_js_seed,
    top_k: int = 100,
    approach_z_max: float = APPROACH_Z_MAX,
    cube_R: np.ndarray = None,
    grasp_mode: str = "top",
    max_candidates: int = 8,
) -> tuple:
    """방향 필터(모드별) + IK 성공 후보 중 '현재 관절과 변화 최소' 파지 선택.
      grasp_mode = "top"  : 윗면 수직 파지 (approach가 아래로, snap으로 면 정렬)
                 = "side" : 옆면 수평 파지 (캔/병을 세워 매대에) — approach가 수평
    모션비용(관절변화) 최소를 골라 6번 등 과회전을 줄인다.
    반환: (grasp_4x4, pre_grasp_4x4) or (None, None)
    """
    order = np.argsort(scores)[::-1][:top_k]
    print(f"  [IK 필터] mode={grasp_mode}, 상위 {top_k}개 탐색 (모션비용 최소 선택)...", flush=True)
    q_now  = cu_js_seed.position.view(-1)
    passed = []   # (joint_cost, score, g_use, pre, rank, approach)

    for rank, idx in enumerate(order):
        g   = grasps_world[idx]
        pos = g[:3, 3]
        if not is_in_franka_workspace(pos):
            continue
        approach = g[:3, 2]
        closing  = g[:3, 0]

        if grasp_mode == "side":
            # 옆면 파지: approach가 수평(캔 측면으로 접근)이어야 함 (약간 완화 0.6)
            if abs(approach[2]) > 0.6:
                continue
            g_use = g                          # 원통은 roll 대칭 → snap 생략
        else:  # top
            if approach[2] > approach_z_max:   # 수직(아래)에 충분히 가깝게
                continue
            if abs(closing[2]) > 0.40:         # 닫힘은 수평 (윗면 가르며 파지)
                continue
            g_use = snap_grasp_roll_90(g, cube_R=cube_R)

        # cuRobo IK
        cpose = mat4_to_curobo_pose(g_use, tensor_args)
        ik_result = ik_solver.solve_single(
            cpose, q_now.view(1, -1), q_now.view(1, 1, -1),
        )
        if not ik_result.success.item():
            continue

        # 모션비용 = 현재 관절 대비 IK 해의 관절 변화 합 (작을수록 과회전 적음)
        q_sol = ik_result.solution.view(-1)[:q_now.shape[0]]
        jcost = float((q_sol - q_now).abs().sum().item())
        pre   = pregrasp_from_grasp(g_use, PREGRASP_STANDOFF)
        passed.append((jcost, float(scores[idx]), g_use, pre, rank, approach.round(2)))
        if len(passed) >= max_candidates:      # 충분히 모으면 중단(속도)
            break

    if not passed:
        print("  [IK 필터] 도달 가능 파지 없음", flush=True)
        return None, None

    passed.sort(key=lambda x: x[0])            # 관절변화 최소 우선
    jcost, sc, g_sel, pre_sel, rank, appr = passed[0]
    print(f"  [IK 필터] 선택 rank={rank+1}, score={sc:.3f}, approach={appr}, "
          f"관절변화합={jcost:.2f}rad (후보 {len(passed)}개 중 최소)", flush=True)
    return g_sel, pre_sel


def pregrasp_from_grasp(grasp_world: np.ndarray, standoff: float) -> np.ndarray:
    """grasp에서 approach 방향으로 standoff만큼 후퇴한 pre-grasp 포즈"""
    pre = grasp_world.copy()
    approach = grasp_world[:3, 2]              # +Z 열 = approach 방향
    pre[:3, 3] = grasp_world[:3, 3] - approach * standoff
    return pre


def plan_to(motion_gen, cu_js, curobo_pose, plan_cfg):
    result = motion_gen.plan_single(cu_js.unsqueeze(0), curobo_pose, plan_cfg)
    if not result.success.item():
        return None
    cmd = motion_gen.get_full_js(result.get_interpolated_plan())
    return cmd


def execute_plan(cmd, sim_js_names, robot, ctrl, finger_idx,
                 finger_pos, my_world, extra_steps=2):
    idx_list, common = [], []
    for x in sim_js_names:
        if x in cmd.joint_names:
            idx_list.append(robot.get_dof_index(x))
            common.append(x)
    cmd = cmd.get_ordered_joint_state(common)
    for i in range(len(cmd.position)):
        ctrl.apply_action(ArticulationAction(
            cmd.position[i].cpu().numpy(),
            cmd.velocity[i].cpu().numpy(),
            joint_indices=idx_list,
        ))
        apply_gripper(ctrl, finger_idx, finger_pos)
        for _ in range(extra_steps):
            my_world.step(render=True)


# ── 메인 ──────────────────────────────────────────────────────────────────────

def main():
    setup_curobo_logger("warn")

    # ZMQ 클라이언트 (graspgen_venv 서버에 연결)
    sys.path.insert(0, "/home/devuser/graspgen_ws/GraspGen")
    from grasp_gen.serving.zmq_client import GraspGenClient
    print("[ZMQ] GraspGen 서버 연결 중 (port 5556)...", flush=True)
    grasp_client = GraspGenClient("127.0.0.1", args.port, wait_for_server=True)
    print(f"[ZMQ] 연결 완료: {grasp_client._server_metadata}", flush=True)

    # 파지 Viser 웹뷰어 (선택: 실패해도 파이프라인 진행)
    viser_viz = ViserGraspViz(port=args.viser_port)

    # Isaac Sim 월드
    my_world = World(stage_units_in_meters=1.0)
    stage    = my_world.stage

    obj_type = args.obj_type
    obj_z    = OBJ_SPECS[obj_type]["z"]        # 물체 중심 높이 (재배치/판정 공용)
    print(f"[물체] 종류={obj_type}, 중심높이={obj_z:.3f}m", flush=True)
    if obj_type == "cylinder":
        from omni.isaac.core.objects import cylinder as _cyl
        s = OBJ_SPECS["cylinder"]
        target_cube = _cyl.DynamicCylinder(
            prim_path="/World/target_cube", name="target_cube",
            position=np.array([0.45, 0.0, obj_z]),
            orientation=rand_yaw_quat(),
            radius=s["radius"], height=s["height"],
            color=np.array([1.0, 0.4, 0.0]), mass=s["mass"],
        )
    else:  # box
        target_cube = cuboid.DynamicCuboid(
            prim_path="/World/target_cube", name="target_cube",
            position=np.array([0.45, 0.0, obj_z]),
            orientation=rand_yaw_quat(),          # 회전 물체로 면-파지 검증
            size=CUBE_SIZE, color=np.array([1.0, 0.4, 0.0]), mass=OBJ_SPECS["box"]["mass"],
        )
    # 콜라캔(알루미늄)–그리퍼 고무패드 마찰 수준. ★물체에 실제로 적용해야 효력 발생
    cube_mat = PhysicsMaterial(
        prim_path="/World/Physics_Materials/cube_mat",
        static_friction=0.6, dynamic_friction=0.5, restitution=0.0,
    )
    target_cube.apply_physics_material(cube_mat)

    # 점구름 시각화 USD Points prim (업데이트 용)
    from pxr import UsdGeom as _UsdGeom, Gf as _Gf, Vt as _Vt
    pc_prim_path = "/World/debug_pc"
    pc_prim = _UsdGeom.Points.Define(stage, pc_prim_path)

    robot_cfg_path = get_robot_configs_path()
    robot_cfg = load_yaml(join_path(robot_cfg_path, args.robot))["robot_cfg"]
    j_names        = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_config = robot_cfg["kinematics"]["cspace"]["retract_config"]
    robot, robot_prim_path = add_robot_to_scene(robot_cfg, my_world)

    world_cfg_table = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
    )
    world_cfg_table.cuboid[0].pose[2] -= 0.02
    world_cfg = WorldConfig(cuboid=world_cfg_table.cuboid)

    usd_help    = UsdHelper()
    tensor_args = TensorDeviceType()

    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args,
        collision_checker_type=CollisionCheckerType.MESH,
        num_trajopt_seeds=12, num_graph_seeds=12,
        interpolation_dt=0.03,
        collision_cache={"obb": 10, "mesh": 10},
        optimize_dt=True, trajopt_dt=None, trajopt_tsteps=32, trim_steps=[1, None],
    )
    motion_gen = MotionGen(motion_gen_config)
    print("warming up...", flush=True)
    motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)

    # IK 솔버 (파지 도달 가능 필터용)
    ik_config = IKSolverConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args,
        rotation_threshold=0.05, position_threshold=0.005,
        num_seeds=20, self_collision_check=False,
        self_collision_opt=False,
        collision_checker_type=CollisionCheckerType.MESH,
        collision_cache={"obb": 10, "mesh": 10},
    )
    ik_solver = IKSolver(ik_config)
    print("Curobo is Ready", flush=True)

    plan_config = MotionGenPlanConfig(
        enable_graph=False, enable_graph_attempt=4,
        max_attempts=8, enable_finetune_trajopt=True,
        time_dilation_factor=0.8,   # pre-grasp/lift: 빠르게 (0.7→0.8)
    )
    slow_config = MotionGenPlanConfig(
        enable_graph=False, enable_graph_attempt=4,
        max_attempts=8, enable_finetune_trajopt=True,
        time_dilation_factor=0.75,  # grasp 접근 (0.65→0.75)
    )

    add_extensions(simulation_app, None)
    usd_help.load_stage(my_world.stage)
    usd_help.add_world_to_stage(world_cfg, base_frame="/World")
    my_world.scene.add_default_ground_plane()
    my_world.play()

    # 루프 변수
    ctrl          = None
    arm_idx       = None
    finger_idx    = None
    sim_js_names  = None

    state         = GS.IDLE
    wait_cnt      = 0
    cycle         = 0
    grasp_world   = None    # 선택된 파지 자세 (월드 프레임 4x4)
    pre_world     = None    # pre-grasp 자세 (월드 프레임 4x4)
    attached      = False
    attach_offset = None

    print(f"Stage 3 시작. {args.cycles}사이클 실행.", flush=True)

    while simulation_app.is_running():
        my_world.step(render=True)
        step = my_world.current_time_step_index

        if ctrl is None:
            ctrl = robot.get_articulation_controller()

        if step < 10:
            all_idx = [robot.get_dof_index(x) for x in j_names]
            robot._articulation_view.initialize()
            robot.set_joint_positions(default_config, all_idx)
            robot._articulation_view.set_max_efforts(
                values=np.array([5000]*len(all_idx)), joint_indices=all_idx
            )
            continue
        if step < 20:
            continue

        if arm_idx is None:
            sim_js_names  = robot.dof_names
            arm_names     = [n for n in sim_js_names if "finger" not in n]
            finger_names  = [n for n in sim_js_names if "finger" in n]
            arm_idx       = [robot.get_dof_index(n) for n in arm_names]
            finger_idx    = [robot.get_dof_index(n) for n in finger_names]
            print(f"조인트: 팔={arm_names}, 손가락={finger_names}", flush=True)
            if args.grip_force > 0:    # 그리퍼 파지힘(max_effort) 조절
                robot._articulation_view.set_max_efforts(
                    values=np.array([args.grip_force] * len(finger_idx)),
                    joint_indices=np.array(finger_idx),
                )
                print(f"  그리퍼 파지힘(max_effort)={args.grip_force}", flush=True)

        if step == 50 or step % 1000 == 0:
            obs = usd_help.get_obstacles_from_stage(
                only_paths=["/World"], reference_prim_path=robot_prim_path,
                ignore_substring=[robot_prim_path, "/World/target_cube",
                                   "/World/defaultGroundPlane", "/curobo"],
            ).get_collision_check_world()
            motion_gen.update_world(obs)

        # 현재 조인트 상태
        sim_js = robot.get_joints_state()
        if sim_js is None:
            continue
        cu_js = JointState(
            position=tensor_args.to_device(sim_js.positions),
            velocity=tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=sim_js_names,
        ).get_ordered_joint_state(motion_gen.kinematics.joint_names)

        cube_pos, cube_quat = target_cube.get_world_pose()   # quat=[w,x,y,z]

        # ── 상태 머신 ────────────────────────────────────────────────────────

        if state == GS.IDLE:
            if step > 60:
                cycle += 1
                print(f"\n{'='*55}", flush=True)
                print(f"[사이클 {cycle}/{args.cycles}] 큐브 위치={cube_pos.round(3)}", flush=True)
                state = GS.QUERY_GRASP

        elif state == GS.QUERY_GRASP:
            # ── GraspGen ZMQ 요청 ─────────────────────────────────────────
            print("  [GraspGen] 점구름 샘플링 & 추론 요청...", flush=True)
            pc_obj = sample_object_pc(obj_type)

            # 점구름 시각화 (USD Points prim 업데이트) — 큐브 회전 반영
            _w, _x, _y, _z = cube_quat
            _R_cube = Rotation.from_quat([_x, _y, _z, _w]).as_matrix()
            pc_world = (_R_cube @ pc_obj.T).T + cube_pos   # 월드 프레임(회전+위치)
            pc_prim.CreatePointsAttr().Set(
                _Vt.Vec3fArray([_Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pc_world])
            )
            pc_prim.CreateWidthsAttr().Set(_Vt.FloatArray([0.003] * len(pc_world)))
            pc_prim.CreateDisplayColorAttr().Set(
                _Vt.Vec3fArray([_Gf.Vec3f(0.2, 0.8, 0.2)] * len(pc_world))
            )
            print(f"  [PC] 시각화 갱신: {len(pc_world)}개 점 (초록색)", flush=True)

            t0 = time.time()
            grasps_obj, scores = grasp_client.infer(pc_obj, num_grasps=400)
            elapsed = time.time() - t0
            print(f"  [GraspGen] {len(grasps_obj)}개 파지 수신 ({elapsed:.2f}s)", flush=True)

            if len(grasps_obj) == 0:
                print("  [GraspGen] 파지 없음! 재시도", flush=True)
                state = GS.IDLE
                continue

            top5 = np.argsort(scores)[::-1][:5]
            print(f"  [GraspGen] Top-5 scores: {scores[top5].round(3)}", flush=True)

            # robotiq → franka 프레임 변환 + 월드 프레임 변환 (전체 후보)
            grasps_franka_obj = np.array([robotiq_grasp_to_franka(g) for g in grasps_obj])
            grasps_w = np.array([grasp_to_world(g, cube_pos, cube_quat) for g in grasps_franka_obj])
            print(f"  [변환] robotiq→franka Z오프셋: {ROBOTIQ_TO_FRANKA_Z:.4f}m", flush=True)

            # IK 필터: 물체별 파지모드 + 모션비용 최소 선택
            grasp_mode = OBJ_SPECS[obj_type].get("grasp_mode", "top")
            grasp_world, pre_world = select_best_reachable_grasp(
                grasps_w, scores, ik_solver, tensor_args, cu_js,
                approach_z_max=APPROACH_Z_MAX, cube_R=_R_cube, grasp_mode=grasp_mode,
            )
            if grasp_world is None and grasp_mode == "top":
                # top 전용 fallback: 수직 파지 없으면 사선 허용 완화
                print("  [IK 필터] 수직 파지 없음 → 사선 허용 완화 재시도", flush=True)
                grasp_world, pre_world = select_best_reachable_grasp(
                    grasps_w, scores, ik_solver, tensor_args, cu_js,
                    approach_z_max=APPROACH_Z_MAX_RELAX, cube_R=_R_cube, grasp_mode="top",
                )

            # ── 파지 시각화 (후보 전체 점수순 + 선택 강조) ──
            # grasp_world(=None 포함)를 그대로 넘김 → 선택 실패해도 후보를 눈으로 확인
            _order = np.argsort(scores)[::-1]
            draw_grasp_candidates_usd(
                stage, grasps_w[_order], scores[_order], selected_T=grasp_world
            )
            viser_viz.update(
                grasps_w[_order], scores[_order],
                selected_T=grasp_world, point_cloud_world=pc_world,
            )

            if grasp_world is None:
                print("  → 도달 가능한 파지 없음. 큐브 재배치 후 재시도", flush=True)
                new_x = np.random.uniform(0.35, 0.6)
                new_y = np.random.uniform(-0.15, 0.15)
                target_cube.set_world_pose(position=np.array([new_x, new_y, obj_z]),
                                           orientation=rand_yaw_quat())
                state = GS.IDLE
                continue

            print(f"  [파지] 월드 위치: {grasp_world[:3,3].round(3)}", flush=True)
            print(f"  [파지] approach(+Z): {grasp_world[:3,2].round(3)}", flush=True)
            print(f"  [pre-grasp] 위치: {pre_world[:3,3].round(3)}", flush=True)

            state = GS.OPEN_GRIPPER
            wait_cnt = 0

        elif state == GS.OPEN_GRIPPER:
            apply_gripper(ctrl, finger_idx, FINGER_OPEN)
            wait_cnt += 1
            if wait_cnt > 40:
                print("  [1] 그리퍼 열기 완료", flush=True)
                state = GS.PLAN_PREGRASP
                wait_cnt = 0

        elif state == GS.PLAN_PREGRASP:
            cpose = grasp_to_curobo_pose(pre_world, tensor_args)
            print(f"  [2] Pre-grasp 플래닝 → pos={pre_world[:3,3].round(3)}", flush=True)
            cmd = plan_to(motion_gen, cu_js, cpose, plan_config)
            if cmd is not None:
                print(f"     → 궤적 {len(cmd.position)} steps", flush=True)
                execute_plan(cmd, sim_js_names, robot, ctrl, finger_idx,
                             FINGER_OPEN, my_world, extra_steps=1)  # 빠른 이동
                print("  [3] Pre-grasp 도달", flush=True)
                state = GS.PLAN_GRASP
            else:
                print("     → 실패, 재시도", flush=True)

        elif state == GS.PLAN_GRASP:
            cpose = grasp_to_curobo_pose(grasp_world, tensor_args)
            print(f"  [4] Grasp 플래닝 → pos={grasp_world[:3,3].round(3)}", flush=True)
            cmd = plan_to(motion_gen, cu_js, cpose, slow_config)
            if cmd is not None:
                print(f"     → 궤적 {len(cmd.position)} steps", flush=True)
                execute_plan(cmd, sim_js_names, robot, ctrl, finger_idx,
                             FINGER_OPEN, my_world, extra_steps=1)  # 1로 단축 (빠른 접근)
                print("  [5] Grasp 위치 도달", flush=True)
                state    = GS.CLOSE_GRIPPER
                wait_cnt = 0
            else:
                print("     → 실패, 재시도", flush=True)

        elif state == GS.CLOSE_GRIPPER:
            apply_gripper(ctrl, finger_idx, FINGER_GRASP)
            wait_cnt += 1
            if wait_cnt == 1:
                print(f"  [6] 그리퍼 닫는 중...", flush=True)
            if wait_cnt > 80:
                f1 = sim_js.positions[finger_idx[0]]
                f2 = sim_js.positions[finger_idx[1]]
                # 양 손가락 중 하나라도 목표보다 크면 접촉
                contact = max(f1, f2) > FINGER_GRASP + 0.003
                print(f"  [6] 닫힘 완료: f1={f1:.4f}, f2={f2:.4f}, 접촉={contact}", flush=True)

                # EE 위치: robot_prim_path/panda_hand 우선, 실패 시 grasp_world 사용
                ee_prim_path = robot_prim_path + "/panda_hand"
                ee_pos = get_ee_world_pos(stage, ee_prim_path)
                if ee_pos is None:
                    # 폴백: grasp_world 위치 사용
                    ee_pos = grasp_world[:3, 3]
                    print(f"  [6] EE prim 없음 → grasp_world 위치 사용: {ee_pos.round(3)}", flush=True)
                else:
                    print(f"  [6] EE 위치: {ee_pos.round(3)}", flush=True)

                cp, _ = target_cube.get_world_pose()
                if args.attach_mode == "kinematic":
                    # 데모용: 접촉 무관 강제 부착 (항상 들림)
                    attach_offset = cp - ee_pos
                    set_kinematic(stage, "/World/target_cube", True)
                    attached = True
                    print(f"  [6] 키네마틱 어태치(데모). offset={attach_offset.round(4)}", flush=True)
                else:
                    # 실제 물리 파지: 강제부착 안 함. 마찰·조임력으로만 들림 (안 잡히면 미끄러져 실패)
                    attached = False
                    attach_offset = None
                    if not contact:
                        print(f"  [6] ⚠️ 그리퍼 접촉 안 됨(contact=False) — 물리 파지 실패 예상", flush=True)
                    else:
                        print(f"  [6] 물리 파지 모드: 접촉 OK, 마찰·무게로 검증", flush=True)

                state    = GS.PLAN_LIFT
                wait_cnt = 0

        elif state == GS.PLAN_LIFT:
            lift_pos = grasp_world.copy()
            lift_pos[:3, 3] += grasp_world[:3, 2] * (-0.20)  # approach 반대로 20cm 후퇴 = 리프트
            # 실제로는 Z상방으로 들어올리는 것이 자연스러움
            lift_target = grasp_world.copy()
            lift_target[2, 3] = 0.28   # 절대 높이 28cm
            cpose = grasp_to_curobo_pose(lift_target, tensor_args)
            print(f"  [7] 리프트 플래닝 → z={lift_target[2,3]:.3f}m", flush=True)
            cmd = plan_to(motion_gen, cu_js, cpose, plan_config)
            if cmd is not None:
                print(f"     → 궤적 {len(cmd.position)} steps", flush=True)
                # 리프트 중 큐브 따라오게
                idx_list, common = [], []
                for x in sim_js_names:
                    if x in cmd.joint_names:
                        idx_list.append(robot.get_dof_index(x))
                        common.append(x)
                cmd_ordered = cmd.get_ordered_joint_state(common)
                for i in range(len(cmd_ordered.position)):
                    ctrl.apply_action(ArticulationAction(
                        cmd_ordered.position[i].cpu().numpy(),
                        cmd_ordered.velocity[i].cpu().numpy(),
                        joint_indices=idx_list,
                    ))
                    apply_gripper(ctrl, finger_idx, FINGER_GRASP)
                    for _ in range(2):
                        my_world.step(render=True)
                        if attached and attach_offset is not None:
                            ep = get_ee_world_pos(stage, robot_prim_path + "/panda_hand")
                            if ep is not None:
                                target_cube.set_world_pose(position=ep + attach_offset)
                state    = GS.MOVE_LIFT   # 리프트 실행 완료 → 직접 HOLD로
                wait_cnt = 0
            else:
                print("     → 리프트 실패, 닫힘 재시도", flush=True)
                state    = GS.CLOSE_GRIPPER
                wait_cnt = 0

        elif state == GS.MOVE_LIFT:
            # plan_lift에서 실행 완료
            cp, _ = target_cube.get_world_pose()
            print(f"  [8] 리프트 완료 ✅  큐브 z={cp[2]:.4f}m", flush=True)
            if cp[2] > obj_z + 0.02:
                print(f"  ✅✅ GraspGen→cuRobo 파지 성공! (Δz={cp[2]-obj_z:.4f}m)", flush=True)
            else:
                print(f"  ⚠️  큐브 미리프트 (z={cp[2]:.4f}m)", flush=True)
            state    = GS.HOLD
            wait_cnt = 0

        elif state == GS.HOLD:
            apply_gripper(ctrl, finger_idx, FINGER_GRASP)
            if attached and attach_offset is not None:
                ep = get_ee_world_pos(stage, robot_prim_path + "/panda_hand")
                if ep is not None:
                    target_cube.set_world_pose(position=ep + attach_offset)
            wait_cnt += 1
            if wait_cnt > 150:
                state    = GS.OPEN_DROP
                wait_cnt = 0

        elif state == GS.OPEN_DROP:
            apply_gripper(ctrl, finger_idx, FINGER_OPEN)
            wait_cnt += 1
            if wait_cnt == 1 and attached:
                set_kinematic(stage, "/World/target_cube", False)
                attached      = False
                attach_offset = None
                print("  [9] 키네마틱 해제", flush=True)
            if wait_cnt > 80:
                if cycle >= args.cycles:
                    print(f"\n{'='*55}", flush=True)
                    print(f"✅ {args.cycles}사이클 완료. 종료합니다.", flush=True)
                    simulation_app.close()
                    break
                new_x = np.random.uniform(0.35, 0.6)
                new_y = np.random.uniform(-0.2, 0.2)
                target_cube.set_world_pose(position=np.array([new_x, new_y, obj_z]),
                                           orientation=rand_yaw_quat())
                set_kinematic(stage, "/World/target_cube", False)
                print(f"  물체 재배치 → [{new_x:.3f}, {new_y:.3f}, {obj_z:.3f}]", flush=True)
                state    = GS.IDLE
                wait_cnt = 0

    simulation_app.close()


if __name__ == "__main__":
    main()
