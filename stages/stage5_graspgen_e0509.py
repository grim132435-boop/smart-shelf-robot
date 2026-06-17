#!/usr/bin/env python3
# Stage5 추종 정밀화: stage4 사본(2026-06-12). 중력보상 피드포워드 + 구체-장애물 간격 로깅 + 핸드오프 블렌딩.
"""
Stage 2: E0509 + RH-P12-RN 그리퍼 → cuRobo 모션 플래닝 GUI
Franka 검증이 완료된 동일 파이프라인을 E0509로 교체.

변경 사항 (Franka → E0509):
  - robot yml: franka.yml → e0509_gripper.yml (curobo_ws/robots/)
  - EE link: panda_hand → gripper_rh_p12_rn_base
  - 조인트: panda_joint1~7 → joint_1~6
  - 그리퍼: panda_finger → gripper_rh_r1/l1 (locked, 별도 제어)
  - USD: franka_panda_temp.usd → e0509_gripper_isaac.usd
"""

try:
    import isaacsim
except ImportError:
    pass

import torch
_ = torch.zeros(4, device="cuda:0")

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--cycles", type=int, default=3)
parser.add_argument("--port", type=int, default=5556, help="GraspGen ZMQ 서버 포트")
parser.add_argument("--obj-type", default="cylinder", choices=["box", "cylinder"])
parser.add_argument("--inspect-shelf", action="store_true",
                    help="매대(/World/Shelf) 하위 prim 월드위치+bbox 출력 후 종료 (좌표 확보용)")
parser.add_argument("--no-graspgen", action="store_true",
                    help="GraspGen 없이 고정 탑다운(기존 Step1 동작)으로 실행")
parser.add_argument("--place", action="store_true",
                    help="첫 grasp+lift 성공 시 캔을 3번(맨 위) 매대에 배치(cuRobo 충돌회피 시연) 후 정지")
parser.add_argument("--viz-spheres", action="store_true",
                    help="cuRobo 충돌구체를 /World/cspheres에 라이브 시각화(책상아래 빨강)")
parser.add_argument("--clutter", type=int, default=0,
                    help="타겟 양옆에 정적 클러터 캔 N개 배치(Phase2: 이웃=장애물 회피 검증). 기본 0")
parser.add_argument("--obj-dist", type=float, default=0.50,
                    help="캔 생성 거리(robot base로부터 +x, m). 0.58+이면 +x 정면 side 파지 IK 가능(도달맵 d≥0.45)")
parser.add_argument("--target-dy", type=float, default=0.0,
                    help="타겟 캔의 측면(y) 오프셋(m). 0이 아니면 옆쪽 캔을 픽 타겟으로(파란색 표시). 클러터 사이 측면 파지 데모")
parser.add_argument("--objects", type=int, default=1,
                    help="[Phase3 다물체] 픽 대상 캔 개수. 1=기존 단일(기본). N>1이면 y줄 스폰 → 순차 슬롯 적재(--place 필요)")
parser.add_argument("--obj-gap", type=float, default=0.15,
                    help="[Phase3] 다물체 캔 y 간격(m). 검증범위 [-0.2,+0.2] 안에서 배치")
args = parser.parse_args()
if args.objects > 1 and (not args.place or args.obj_type != "cylinder"):
    parser.error("--objects N>1은 --place + --obj-type cylinder 전용(다물체 슬롯 적재)")

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "width": "1920",
    "height": "1080",
})

import sys, os, time
import numpy as np
import trimesh
import trimesh.transformations as tra
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

# grasp_viz.py (이 파일과 같은 디렉터리) — GraspGen 파지 USD 시각화
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grasp_viz import draw_grasp_candidates_usd, clear_grasp_viz_usd

import carb
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.utils.stage import add_reference_to_stage

from curobo.geom.sdf.world import CollisionCheckerType, CollisionQueryBuffer
from curobo.geom.types import WorldConfig, Cuboid
from curobo.geom.sphere_fit import SphereFitType
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.util.logger import setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

# ── E0509 설정 경로 ──────────────────────────────────────────────────────────
ROBOT_DIR   = "/home/devuser/curobo_ws/robots/e0509_gripper"
ROBOT_YML   = f"{ROBOT_DIR}/e0509_gripper.yml"
ROBOT_USD   = "/home/devuser/e0509_gripper_isaac/e0509_gripper_isaac.usd"
# 사용자 제작 매대 씬 (로봇 /World/Robot + Table + Shelf + base 큐브 포함)
V2_USD      = "/home/devuser/CoWriteBotRL/models/shelf_workspace_v2.usd"
ROBOT_PRIM  = "/World/Robot"            # v2 안의 E0509 prim
# RH-P12 그리퍼: 1-DOF(gripper_rh_r1), open≈0.0 / close≈1.101 rad
GRIP_OPEN   = 0.0
GRIP_CLOSE  = 1.05
# RH-P12 EE(gripper_rh_p12_rn_base) → 파지점(TCP, 손가락 사이) 깊이 (approach +Z방향, m).
#   RH-P12 손가락 기하(base프레임): 손가락끝 z≈0.11, 그립면 z≈0.05~0.11, 중간≈0.08.
#   0.1034(robotiq depth)는 캔을 손가락 끝에 놓아 닫을때 캐밍으로 빠짐 → 손가락 중간(0.078)에
#   깊숙이 seat해야 측면 그립이 됨 (2026-06-05 물리파지 디버깅으로 확정).
#   ★간이 그리퍼오프셋(2026-06-10): 0.060은 너무 깊이 집어 캔이 기울어져 잡힘 → +0.05m(50mm 덜 깊이).
#     0.160(+100mm)은 그리퍼가 캔에 안 닿아 실패 → 0.110로 절충. 정밀 오프셋은 Stage6.
RHP12_TCP_DEPTH = 0.110
# robotiq(GraspGen) grasp프레임 → RH-P12 EE 깊이 보정. 두 그리퍼 TCP깊이 거의 같아 ≈0.
#   (기존 0.095는 근거 없는 추정으로 그리퍼가 캔에서 어긋났음 → 0으로 정정)
ROBOTIQ_TO_RHP12_Z = 0.0

# ── 3번(맨 위) 매대 배치 (월드, m. cuRobo 장애물 실측: 바닥top 1.14, 앞턱top 1.15, 천장 없음) ──
#   ★중요: 캔은 TCP(손가락 사이)보다 grip_z_offset(≈0.02m=0.3·half)만큼 아래에서 잡힘.
#     따라서 '캔 바닥'을 원하는 높이에 두려면 TCP center z = 캔바닥 + half + grip_z_offset.
#     이 오프셋을 무시하면(이전 버그) 캔 바닥이 바닥판보다 ~1.5cm 아래로 박혀 캔이 기울어 눕음.
#   동작: 매대 앞 충분히 띄운 곳(PRE)→ +y 진입 → -z 안착 → open → -y 후퇴 (캔이 매대에 안 닿게).
SHELF3_X         = 0.25
SHELF3_APPROACH  = [0.0, 1.0, 0.0]   # +y(매대 안쪽), 그리퍼 X=위 → 캔 직립
SHELF3_FLOOR_TOP = 1.14              # 3단 바닥판 윗면(world)
SHELF3_LIP_TOP   = 1.15              # 앞턱 윗면(world)
SHELF3_PRE_Y     = 0.22              # 앞접근 y: 앞턱(y≈0.37)에서 충분히 앞 → 진입 전 캔이 매대에 안 닿음
SHELF3_IN_Y      = 0.50              # 내부 y: 앞턱 뒤(안쪽)
SHELF3_ENTRY_CLR = 0.04             # 진입 시 캔바닥을 앞턱 위로 띄우는 여유(턱 간섭 방지)
SHELF3_REST_CLR  = 0.003            # 안착 시 캔바닥-바닥판 여유(살짝 띄워 박힘 방지)
# [Phase3] 다물체 슬롯 (x, in_y) — 2열 지그재그. 한 열 직선피치 0.11은 그리퍼 스윕폭(~0.07 half)이
#   이웃 캔/벽과 간섭(multiobj_run1: x0.14 하강 -3.5mm 침투)하고, x≥0.36은 도달한계(IK 실패).
#   → 뒷열 좌(0.165,0.56)·앞열 우(0.34,0.44)·앞열 중(0.21,0.44): 쌍별 거리 ≥0.128, 벽 여유 ≥0.088,
#   깊은 슬롯부터 적치(후속 하강 경로가 기적치 캔 옆을 0.125+ 띄워 지남). 단일 모드는 기존 (0.25,0.50).
SHELF3_SLOTS     = ([(SHELF3_X, SHELF3_IN_Y)] if args.objects <= 1 else
                    [(0.165, 0.56), (0.34, 0.44), (0.21, 0.44)])
SLOT_MIN_DIST    = 0.125            # 슬롯-기적치 캔 최소 중심거리(그리퍼 스윕 + 캔 반경 + 여유)
GRIP_Z_OFFSET_EST = 0.047           # 파지 전 슬롯 사전검사용 grip_z_offset 추정(실측값으로 본검사)

# ── 물리 상수 ─────────────────────────────────────────────────────────────────
CUBE_SIZE = 0.05
CUBE_MASS = 0.15
CUBE_Z    = CUBE_SIZE / 2     # 0.025m (테이블 위)

# ── E0509 retract 포즈 ────────────────────────────────────────────────────────
# 홈/retract 포즈: 파지하기 편한 자세 (사용자 지정 joint1~6 = -35,45,80,65,115,-40 deg)
RETRACT_CONFIG = [-0.610865, 0.785398, 1.396263, 1.134464, 2.007129, -0.698132]
# E0509 관절 한계 (URDF, rad, joint_1..6) — cu_js 시작상태 클램프용(INVALID_START 방지)
# j4·5는 URDF 한계수정(자세 정밀화)과 일치 — j4 ±180°, j5 [0,+135°](손목 위). 시작상태 클램프 일관(INVALID_START 방지)
ARM_JOINT_LOWER = np.array([-6.2832, -1.6581, -2.3562, -3.1416,  0.0000, -6.2832], dtype=np.float32)
ARM_JOINT_UPPER = np.array([ 6.2832,  1.6581,  2.3562,  3.1416,  2.3562,  6.2832], dtype=np.float32)

# ── 제어/관찰 파일 ─────────────────────────────────────────────────────────────
# stop-sentinel: 이 파일이 생기면 graceful 종료(kill 불필요 → CUDA UVM 오염 없음).
#   닫기: touch /tmp/stage5_stop  (stage4 창과 독립 종료)
STOP_FILE = "/tmp/stage5_stop"
# 뷰포트 스크린샷 저장 폴더 (창이 안 보여도 결과를 PNG로 확인 가능)
SHOT_DIR  = "/home/devuser/shelf_grasp_dev/logs/shots"

# ── GraspGen 파라미터 (stage3에서 이식) ──────────────────────────────────────
PREGRASP_STANDOFF    = 0.10        # pre-grasp 후퇴 거리(m). 0.15→0.10: 책상 앞모서리 회피
APPROACH_Z_MAX       = -0.90       # top 파지 수직도 임계(월드 Z성분, -1=완전수직)
APPROACH_Z_MAX_RELAX = -0.80       # fallback 완화 임계
SIDE_APPROACH_Z_MAX  = 0.85        # side 후보 채택 임계(approach z 절대값). 스냅이 수평 보장하므로
                                   #   필터는 '거의 top-down' 제외용(<0.85≈수평~58°). 출력은 항상 수평.
NUM_PC_POINTS        = 2048
# 물체별 사양: box=snack 근사, cylinder=can 근사. grasp_mode: top/side
OBJ_SPECS = {
    "box":      {"z": CUBE_SIZE / 2,                        "grasp_mode": "top"},
    "cylinder": {"z": 0.0675, "radius": 0.03, "height": 0.135, "grasp_mode": "side"},
}

class GS:
    IDLE          = "IDLE"
    QUERY_GRASP   = "QUERY_GRASP"   # GraspGen 추론+선택+시각화
    PLAN_PREGRASP = "PLAN_PREGRASP"
    MOVE_PREGRASP = "MOVE_PREGRASP"
    PLAN_GRASP    = "PLAN_GRASP"
    MOVE_GRASP    = "MOVE_GRASP"
    PLAN_LIFT     = "PLAN_LIFT"
    MOVE_LIFT     = "MOVE_LIFT"
    HOLD          = "HOLD"
    RELEASE       = "RELEASE"
    # 매대 배치(--place): 운반→삽입→하강→안착→후퇴
    PLAN_CARRY    = "PLAN_CARRY"     # 리프트 자세 → 매대 앞(충돌회피 plan_single)
    INSERT_SHELF  = "INSERT_SHELF"   # 매대 안으로 +y 진입(직접IK)
    LOWER_SHELF   = "LOWER_SHELF"    # 캔 바닥까지 하강(직접IK) + 그리퍼 open
    RETREAT_SHELF = "RETREAT_SHELF"  # -y 매대 밖 후퇴(직접IK)
    GO_HOME       = "GO_HOME"        # home 복귀(충돌회피 plan_single_js, 갱신된 cu_js로)
    HALT          = "HALT"   # 진단 후 정지(장면 유지, 재플래닝 안 함)


def get_ee_world_pos(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    T = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array([T[3][0], T[3][1], T[3][2]])


def set_kinematic(stage, path, enabled):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(enabled)


def set_gripper_friction(stage, material_prim_path="/World/Physics_Materials/cube_mat"):
    """RH-P12 손가락 collision prim에 고마찰 머티리얼 바인딩. 캔만 고마찰이면 두 물체 유효마찰
    (조합)이 부족해 미끄러짐 → 손가락도 고마찰로 그립 신뢰성↑. (r1/r2/l1/l2 collision)."""
    from pxr import UsdShade
    mat_prim = stage.GetPrimAtPath(material_prim_path)
    if not mat_prim.IsValid():
        print("  [그리퍼마찰] 머티리얼 prim 없음", flush=True); return
    mat = UsdShade.Material(mat_prim)
    n = 0
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if "gripper_rh_p12_rn_" in p and any(f in p for f in ("_r1", "_r2", "_l1", "_l2")):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                try:
                    UsdShade.MaterialBindingAPI.Apply(prim)
                    UsdShade.MaterialBindingAPI(prim).Bind(
                        mat, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                        materialPurpose="physics")
                    n += 1
                except Exception as e:
                    print(f"  [그리퍼마찰] {p} 실패: {e}", flush=True)
    print(f"  [그리퍼마찰] {n}개 손가락 collision에 고마찰 바인딩", flush=True)


def set_finger_sdf_collision(stage, resolution=256):
    """RH-P12 손가락 collision 근사를 convexHull→SDF로 교체 (Stage6 슬립 대책 B1).
    convexHull은 오목한 손가락 패드 안쪽을 메워 캔과 점접촉 → 미끄러짐·손끝 펴짐의 형상 원인.
    SDF는 오목면을 보존해 실물처럼 감싸쥐는 면접촉 재현. (USD 진단 2026-06-12: physics.usd
    /colliders/gripper_rh_*에 approx=convexHull 명시 확인. robotis_lab/Isaac 정석=SDF)"""
    from pxr import PhysxSchema
    fingers = ("gripper_rh_p12_rn_r1/", "gripper_rh_p12_rn_r2/",
               "gripper_rh_p12_rn_l1/", "gripper_rh_p12_rn_l2/")
    n = 0
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if "/collisions" not in p or not any(f in p for f in fingers):
            continue
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr().Set("sdf")
            sdf = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            sdf.CreateSdfResolutionAttr().Set(resolution)
            n += 1
    print(f"  [그리퍼SDF] 손가락 collision {n}개 convexHull→SDF(res={resolution})", flush=True)


def inspect_prim_tree(stage, root_path, max_depth=3):
    """root 하위 prim들의 월드위치+bbox(min/max) 출력 → 매대 선반 높이/포인트 좌표 확보용."""
    from pxr import UsdGeom
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        print(f"[매대검사] {root_path} 없음", flush=True); return
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    print(f"[매대검사] {root_path} 하위:", flush=True)
    for prim in Usd.PrimRange(root):
        depth = str(prim.GetPath()).count("/") - str(root_path).count("/")
        if depth > max_depth:
            continue
        try:
            w = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            pos = (round(w[3][0], 3), round(w[3][1], 3), round(w[3][2], 3))
            bb = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            mn = (round(bb.GetMin()[0], 3), round(bb.GetMin()[1], 3), round(bb.GetMin()[2], 3))
            mx = (round(bb.GetMax()[0], 3), round(bb.GetMax()[1], 3), round(bb.GetMax()[2], 3))
            print(f"  {'  '*depth}{prim.GetName():28s} pos={pos} bbox=({mn})~({mx})", flush=True)
        except Exception:
            pass


def stabilize_arm_drives(robot_art, arm_names, sim_js_names):
    """E0509 IsaacLab USD는 드라이브 게인이 placeholder(kp=100/kd=1)라 단독 로드 시 출렁임.
    프로젝트 검증값(CoWriteBotRL/e0509_ik_env_v7 ImplicitActuatorCfg, 실기 모사)으로 교체:
      arm(joint_1~6)  : stiffness=800,  damping=80,  effort=200
      gripper(rh_*)   : stiffness=2000, damping=100, effort=50   (물리 파지용 그립힘)
    """
    try:
        view = robot_art._articulation_view
        kp, kd = view.get_gains()
        kp = np.array(kp, dtype=np.float64).reshape(-1)
        kd = np.array(kd, dtype=np.float64).reshape(-1)
        print(f"[게인,전] kp={kp.round(1)}", flush=True)
        print(f"[게인,전] kd={kd.round(1)}", flush=True)
        arm_idx, grip_idx = [], []
        for nm in sim_js_names:
            i = robot_art.get_dof_index(nm)
            if nm in arm_names:
                kp[i], kd[i] = 800.0, 80.0;   arm_idx.append(i)
            elif nm.startswith("gripper_rh_"):
                kp[i], kd[i] = 600.0, 80.0; grip_idx.append(i)   # 컴플라이언트: 캔 들이받아 관통/발사 방지(2000→600)
        view.set_gains(kps=kp, kds=kd)
        if arm_idx:
            view.set_max_efforts(values=np.array([200.0]*len(arm_idx)),
                                 joint_indices=np.array(arm_idx))
        if grip_idx:
            view.set_max_efforts(values=np.array([25.0]*len(grip_idx)),
                                 joint_indices=np.array(grip_idx))
        print(f"[게인,후] arm kp=800 kd=80 eff=200 / gripper kp=600 kd=80 eff=25 "
              f"(arm {len(arm_idx)}축, gripper {len(grip_idx)}축)", flush=True)
    except Exception as e:
        print(f"[게인] 보정 실패: {e}", flush=True)


def enable_gravity_comp(robot_art, my_world, arm_names):
    """[Stage5] 중력보상 피드포워드: 매 물리스텝 PhysX 일반화 중력토크를 arm 6관절 effort로 인가.
    PD 게인(kp800/kd80)·드라이브 타겟은 유지한 채 effort 채널에 가산(set_joint_efforts→
    set_dof_actuation_forces, 드라이브와 독립) → joint_2 중력처짐(~4°) 제거 목적.
    그리퍼 관절은 0(파지력 간섭 방지). physics callback 1곳 등록으로 execute_plan/moveL/정착 루프 전부 커버."""
    try:
        view = robot_art._articulation_view
        nd = int(view.num_dof)
        arm_idx = np.array([robot_art.get_dof_index(n) for n in arm_names], dtype=np.int64)
        state = {"on": True, "logged": False}

        def _cb(step_size):
            if not state["on"]:
                return
            try:
                g = np.asarray(view.get_generalized_gravity_forces()).reshape(-1)
                eff = np.zeros((1, nd), dtype=np.float32)
                eff[0, arm_idx] = g[arm_idx]
                view.set_joint_efforts(eff)
                if not state["logged"]:
                    state["logged"] = True
                    print(f"  [중력보상] 활성 — arm 중력토크(Nm)={np.round(g[arm_idx], 2)}", flush=True)
            except Exception as e:
                state["on"] = False
                print(f"  [중력보상] 콜백 실패 → 비활성: {e}", flush=True)

        my_world.add_physics_callback("stage5_grav_comp", _cb)
        print("  [중력보상] physics callback 등록", flush=True)
        return state
    except Exception as e:
        print(f"  [중력보상] 등록 실패: {e}", flush=True)
        return None


# 로봇 베이스 받침 블록 (V1 /base Cube 스펙: center·size 그대로)
ROBOT_BASE_BLOCK_POS  = [-0.25, -0.04, 0.715]
ROBOT_BASE_BLOCK_SIZE = [0.18, 0.22, 0.03]
ROBOT_BASE_BLOCK_PATH = "/World/robot_base_block"


def fix_scene(stage):
    """씬 보정: (1) 로봇 받침 블록 추가(V2엔 없어 로봇이 떠 보임),
    (2) 책상/매대를 정적으로(움직임 방지), (3) 매대 공중부양 → 책상 윗면에 안착."""
    # (1) 로봇 받침 블록 (V1 /base와 동일) — 정적 큐브
    try:
        from omni.isaac.core.objects import cuboid as _cub
        _cub.FixedCuboid(
            prim_path=ROBOT_BASE_BLOCK_PATH, name="robot_base_block",
            position=np.array(ROBOT_BASE_BLOCK_POS, dtype=np.float32),
            scale=np.array(ROBOT_BASE_BLOCK_SIZE, dtype=np.float32),
            color=np.array([0.35, 0.35, 0.38]),
        )
        print(f"[씬] 로봇 받침 블록 추가 {ROBOT_BASE_BLOCK_POS} size={ROBOT_BASE_BLOCK_SIZE}", flush=True)
    except Exception as e:
        print(f"[씬] 받침 블록 실패: {e}", flush=True)

    # (2) 책상·매대 정적 고정 (움직임 방지)
    for p in ["/World/Table", "/World/Shelf"]:
        try:
            set_kinematic(stage, p, True)
        except Exception:
            pass

    # (2b) 매대 물리 collider 추가: v2 USD 매대 geom엔 CollisionAPI가 없어(책상엔 있음)
    #   그리퍼/캔이 매대를 물리적으로 통과 → 놓은 캔이 매대 바닥을 뚫고 떨어짐. 정적 collider로 만든다.
    try:
        from pxr import UsdGeom as _UG, UsdPhysics as _UP
        _nc = 0
        for _pr in Usd.PrimRange(stage.GetPrimAtPath("/World/Shelf")):
            if _pr.IsA(_UG.Mesh) or _pr.IsA(_UG.Cube):
                _UP.CollisionAPI.Apply(_pr)
                if _pr.IsA(_UG.Mesh):
                    _UP.MeshCollisionAPI.Apply(_pr).CreateApproximationAttr().Set("none")  # 정적=삼각메시 OK
                _nc += 1
        print(f"[씬] 매대 collider {_nc}개 적용(그리퍼/캔 통과 방지)", flush=True)
    except Exception as e:
        print(f"[씬] 매대 collider 실패: {e}", flush=True)

    # (3) 매대를 책상 윗면에 안착 (공중부양 수정): bbox로 gap 계산 후 z 평행이동
    try:
        from pxr import UsdGeom, Gf
        def _bb(path):
            pr = stage.GetPrimAtPath(path)
            if not pr.IsValid():
                return None
            r = UsdGeom.Imageable(pr).ComputeWorldBound(
                Usd.TimeCode.Default(), "default").ComputeAlignedRange()
            return r.GetMin(), r.GetMax()
        tbl, shf = _bb("/World/Table"), _bb("/World/Shelf")
        if tbl and shf:
            table_top  = tbl[1][2]
            shelf_bot  = shf[0][2]
            gap = shelf_bot - table_top           # >0 이면 공중부양
            if abs(gap) > 0.003:
                shelf = stage.GetPrimAtPath("/World/Shelf")
                xf = UsdGeom.Xformable(shelf)
                top = xf.GetOrderedXformOps()
                t_op = next((o for o in top
                             if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
                if t_op is None:
                    t_op = xf.AddTranslateOp()
                cur = t_op.Get() or Gf.Vec3d(0, 0, 0)
                t_op.Set(Gf.Vec3d(cur[0], cur[1], cur[2] - gap))
                print(f"[씬] 매대 안착: 책상top={table_top:.3f} 매대bot={shelf_bot:.3f} "
                      f"gap={gap:.3f} → z {-gap:+.3f}m 이동", flush=True)
            else:
                print(f"[씬] 매대 이미 책상에 안착(gap={gap:.3f})", flush=True)
    except Exception as e:
        print(f"[씬] 매대 안착 실패: {e}", flush=True)


def fix_robot_base(stage, robot_prim, base_link="base_link"):
    """로봇 base를 월드에 고정 (중력에 의한 흔들림/처짐 방지).
    stage2는 고정베이스 로봇 USD를 직접 참조 → 안정. v2 로봇은 IsaacLab floating-base라
    명시적으로 base_link↔월드 FixedJoint를 추가해 고정한다."""
    base_path = f"{robot_prim}/{base_link}"
    if not stage.GetPrimAtPath(base_path).IsValid():
        print(f"[고정] base_link 못 찾음: {base_path} → 고정 생략", flush=True)
        return False
    fj_path = f"{robot_prim}/world_fix_joint"
    if stage.GetPrimAtPath(fj_path).IsValid():
        return True
    from pxr import Gf
    # base_link 현재 월드 포즈를 조인트 앵커(body0=월드측)에 설정.
    #   앵커 미설정 시 rest pose가 월드원점이 되어 솔버가 base를 원점으로 당김 → X축 흔들림.
    bp = stage.GetPrimAtPath(base_path)
    xf = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(bp)
    trans = xf.ExtractTranslation()
    quat  = xf.ExtractRotationQuat()         # Gf.Quatd (w,xyz)
    fj = UsdPhysics.FixedJoint.Define(stage, fj_path)
    fj.CreateBody1Rel().SetTargets([Sdf.Path(base_path)])   # body0 비움 = 월드
    fj.CreateLocalPos0Attr().Set(Gf.Vec3f(trans))           # 월드측 앵커 = base 현재 위치
    fj.CreateLocalRot0Attr().Set(Gf.Quatf(quat))            # 월드측 앵커 = base 현재 회전
    fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))         # base_link측 앵커 = 자기 원점
    fj.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    print(f"[고정] 월드↔{base_path} FixedJoint 생성 (앵커={[round(float(t),3) for t in trans]}, 흔들림 방지)", flush=True)
    return True


# 뷰포트 카메라 프레이밍 (로봇 base[-0.25,-0.04,0.73]·캔[0.2,-0.04,0.78]·매대[0.25,0.5]를 한 화면에)
SHOT_CAM_EYE    = [1.15, -0.95, 1.25]   # 전체 팔+구체 보이게(구체 conform 확인용)
SHOT_CAM_TARGET = [0.05, -0.05, 0.85]
def set_scene_camera():
    """기본 원거리 뷰 대신 로봇·캔·매대가 크게 보이도록 persp 카메라 배치."""
    try:
        try:
            from omni.isaac.core.utils.viewports import set_camera_view
        except ImportError:
            from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=SHOT_CAM_EYE, target=SHOT_CAM_TARGET,
                        camera_prim_path="/OmniverseKit_Persp")
        print(f"  [CAM] persp 카메라 배치 eye={SHOT_CAM_EYE} target={SHOT_CAM_TARGET}", flush=True)
    except Exception as e:
        print(f"  [CAM] 배치 실패(무시): {e}", flush=True)


_shot_n = [0]
def save_shot(tag=""):
    """뷰포트를 PNG로 저장 (창이 안 보여도 결과 확인용). 실패해도 파이프라인 진행."""
    try:
        os.makedirs(SHOT_DIR, exist_ok=True)
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        _shot_n[0] += 1
        path = f"{SHOT_DIR}/shot_{_shot_n[0]:03d}_{tag}.png"
        capture_viewport_to_file(get_active_viewport(), path)
        print(f"  [SHOT] 저장: {path}", flush=True)
    except Exception as e:
        print(f"  [SHOT] 실패(무시): {e}", flush=True)


# ── GraspGen 헬퍼 (stage3에서 이식, E0509용으로 적응) ────────────────────────

def sample_object_pc(obj_type="box", n=NUM_PC_POINTS):
    """물체 표면 점구름 (오브젝트 중심 프레임). 실기체에선 RealSense+SAM 점구름으로 교체."""
    if obj_type == "cylinder":
        s = OBJ_SPECS["cylinder"]
        mesh = trimesh.creation.cylinder(radius=s["radius"], height=s["height"])
    else:
        mesh = trimesh.creation.box([CUBE_SIZE] * 3)
    pts, _ = trimesh.sample.sample_surface(mesh, n)
    return pts.astype(np.float32)


def robotiq_grasp_to_rhp12(grasp_4x4):
    """robotiq(GraspGen 학습) EE 프레임 → RH-P12-RN EE 프레임 Z 깊이 보정.
    GraspGen FAQ: new = grasp @ T([0,0,Z]). Z는 그리퍼 접촉깊이 차이(근사 0.095, 시각화로 보정)."""
    return grasp_4x4 @ tra.translation_matrix([0, 0, ROBOTIQ_TO_RHP12_Z])


def grasp_to_world(grasp_obj, obj_world_pos, obj_world_quat=None):
    """오브젝트 프레임 파지 → 월드 프레임 (물체 위치+회전 반영). quat=[w,x,y,z]."""
    T = np.eye(4)
    T[:3, 3] = obj_world_pos
    if obj_world_quat is not None:
        w, x, y, z = obj_world_quat
        T[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
    return T @ grasp_obj


def is_in_workspace(pos, base):
    """E0509 작업공간 필터 (로봇 base 기준 상대). base=[x,y,z]."""
    dx, dy, dz = pos[0] - base[0], pos[1] - base[1], pos[2] - base[2]
    r = np.sqrt(dx**2 + dy**2)
    return (0.15 < r < 0.85 and        # base로부터 수평 거리
            -0.35 < dz < 0.55 and      # base 기준 높이
            dx > 0.05)                 # 로봇 앞쪽(+x)


def pregrasp_from_grasp(grasp_world, standoff):
    pre = grasp_world.copy()
    approach = grasp_world[:3, 2]              # +Z 열 = approach
    pre[:3, 3] = grasp_world[:3, 3] - approach * standoff
    return pre


def _ik_ok_for_approach(approach, ref, pos, ik_solver, tensor_args, q_now):
    """approach 방향 + ref(닫힘 후보)로 EE 자세를 만들어 IK 성공 여부 반환."""
    approach = approach / (np.linalg.norm(approach) + 1e-9)
    closing  = ref - np.dot(ref, approach) * approach
    if np.linalg.norm(closing) < 1e-6:
        closing = np.array([1.0, 0, 0]) - approach[0] * approach
    closing /= (np.linalg.norm(closing) + 1e-9)
    y = np.cross(approach, closing)
    R = np.column_stack([closing, y, approach])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = pos
    try:
        ik = ik_solver.solve_single(mat4_to_curobo_pose(T, tensor_args),
                                    q_now.view(1, -1), q_now.view(1, 1, -1))
        return bool(ik.success.item())
    except Exception:
        return False


def probe_reachable_approaches(pos, ik_solver, tensor_args, cu_js_seed, base):
    """진단: 이 위치에서 E0509가 낼 수 있는 접근방향 확인 (GraspGen 규약 approach=+Z 사용).
      - top   : 수직 아래 [0,0,-1]
      - side  : 로봇→물체 수평 방향 및 좌우 (옆면 파지용)
    어떤 것도 안 되면 위치 도달불가 or RH-P12 프레임 규약 불일치(회전 오프셋 필요)."""
    q_now = cu_js_seed.position.view(-1)
    horiz = np.array([pos[0] - base[0], pos[1] - base[1], 0.0])   # 로봇→물체 수평
    horiz = horiz / (np.linalg.norm(horiz) + 1e-9)
    left  = np.array([-horiz[1], horiz[0], 0.0])
    tests = {
        "top(아래로)":  np.array([0.0, 0.0, -1.0]),
        "side(정면)":   horiz,
        "side(좌)":     left,
        "side(우)":     -left,
        "side+아래45":  (horiz + np.array([0, 0, -1.0])),
    }
    refs = [np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0])]
    ok = {}
    for name, appr in tests.items():
        ok[name] = any(_ik_ok_for_approach(appr, r, pos, ik_solver, tensor_args, q_now)
                       for r in refs)
    good = [k for k, v in ok.items() if v]
    if good:
        print(f"  [접근 탐침] E0509 가능 방향 ✅ {good} → 규약 OK, 후보 방향만 안 맞아 탈락", flush=True)
    else:
        print(f"  [접근 탐침] 모든 방향 IK 실패 ❌ → 위치 도달불가 or RH-P12 프레임 규약 불일치", flush=True)
    return ok


def snap_grasp_roll_90(grasp_4x4, obj_R=None):
    """닫힘축(X)을 물체 면(로컬 X/Y)에 90° 스냅 → 평행면 파지 보장 (모서리 파지 방지)."""
    R        = grasp_4x4[:3, :3]
    approach = R[:, 2]
    closing  = R[:, 0]
    grip_y   = R[:, 1]
    if obj_R is not None:
        ax, ay = obj_R[:3, 0], obj_R[:3, 1]
        candidates = [ax, -ax, ay, -ay]
    else:
        candidates = [np.array([1., 0, 0]), np.array([-1., 0, 0]),
                      np.array([0, 1., 0]), np.array([0, -1., 0])]
    dots = [np.dot(closing, c) for c in candidates]
    best = candidates[int(np.argmax(dots))]
    best_perp = best - np.dot(best, approach) * approach
    nrm = np.linalg.norm(best_perp)
    if nrm < 1e-6:
        return grasp_4x4
    best_perp /= nrm
    cos_a = np.clip(np.dot(closing, best_perp), -1, 1)
    sin_a = np.dot(np.cross(closing, best_perp), approach)
    angle = np.arctan2(sin_a, cos_a)
    if abs(angle) < 0.05:
        return grasp_4x4
    c, s = np.cos(angle), np.sin(angle)
    def rot(v):
        return c*v + s*np.cross(approach, v) + (1-c)*np.dot(approach, v)*approach
    new_R = np.column_stack([rot(closing), rot(grip_y), approach])
    out = grasp_4x4.copy()
    out[:3, :3] = new_R
    return out


def side_grasp_from_approach(a, obj_center, tcp_depth):
    """수평 approach 벡터로 RH-P12 side 파지 합성.
      - Z(approach)      = 캔 축을 향한 수평 방향 (옆면 수직 진입)
      - Y(손가락 분리축)  = 수평 접선  ← RH-P12는 base Y로 닫힘 (URDF: 손가락 ±y, x축 회전)
      - X(그리퍼 상하축) = 위(+z)로 강제 → 그리퍼 상단의 비전(RSD455)이 하늘 향함(책상 충돌 방지)
      - EE 위치 = 캔 중심 − tcp_depth·approach  → TCP(손가락 사이)가 캔 축 중앙에
    수평 성분 거의 없으면 None."""
    a = np.asarray(a, dtype=float).copy()
    a[2] = 0.0
    n = np.linalg.norm(a)
    if n < 1e-3:
        return None
    a /= n
    Y = np.array([-a[1], a[0], 0.0])
    X = np.cross(Y, a); X /= (np.linalg.norm(X) + 1e-9)
    if X[2] < 0:                       # X(그리퍼 상단/카메라축)가 하늘 향하도록
        X, Y = -X, -Y
    Y = np.cross(a, X); Y /= (np.linalg.norm(Y) + 1e-9)
    out = np.eye(4)
    out[:3, 0], out[:3, 1], out[:3, 2] = X, Y, a
    out[:3, 3] = np.asarray(obj_center, dtype=float) - tcp_depth * a
    return out


def synthesize_side_grasp_rhp12(grasp_4x4, obj_center, tcp_depth):
    """GraspGen 후보의 수평 방위만 취해 합성 (side_grasp_from_approach 래퍼)."""
    return side_grasp_from_approach(grasp_4x4[:3, 2], obj_center, tcp_depth)


def axis_load_cost(q_sol, q_now, retract):
    """파지 자세의 '로봇 축 부하' 근사 비용 (작을수록 좋음).
    동역학 토크 대신 실무 프록시 사용:
      - move : 현재 관절에서의 변화량 합 (이동량/과회전)
      - wrist: 손목(joint5,6) 변화 가중 (6번 과회전 억제 — 사용자 지적)
      - rest : 편안한 retract 자세 이탈 (extended/극단 자세일수록 토크↑)
    """
    dq = (q_sol - q_now).abs()
    move  = float(dq.sum().item())
    wrist = float(dq[4:].sum().item()) if dq.shape[0] >= 6 else 0.0
    rest  = float((q_sol - retract).abs().sum().item())
    return move + 1.5 * wrist + 0.5 * rest, move, wrist, rest


def select_best_reachable_grasp(grasps_world, scores, ik_solver, tensor_args, cu_js_seed,
                                base, retract, top_k=100, approach_z_max=APPROACH_Z_MAX,
                                obj_R=None, grasp_mode="top", max_candidates=12,
                                obj_center=None, obj_half_h=0.0):
    """방향필터(모드별) + IK 성공 후보 중 '축 부하 최소' 파지 선택.
    side 모드: 수평 스냅 후 (1)캔 몸통 높이 범위 내 (2)grasp+pre-grasp 모두 IK 가능 후보만 채택.
    반환: (grasp_4x4, pre_grasp_4x4) or (None, None)."""
    order = np.argsort(scores)[::-1][:top_k]
    print(f"  [IK 필터] mode={grasp_mode}, 상위 {top_k}개 탐색 (모션비용 최소 선택)...", flush=True)
    q_now  = cu_js_seed.position.view(-1)
    passed = []
    # 탈락 사유 카운터 (height=몸통높이밖, pregrasp=pre-grasp IK실패)
    rej = {"workspace": 0, "approach": 0, "closing": 0, "height": 0, "ik": 0, "pregrasp": 0}

    # ── side 모드: 대칭 캔이므로 GraspGen 임의 azimuth 대신 '정면(로봇→캔)±각도'를 합성·스윕 ──
    #    손목 부하 최소·도달 잘 되는 자세 선택 (카메라는 항상 하늘 향함). GraspGen은 top/box용.
    if grasp_mode == "side" and obj_center is not None:
        frontal = np.array([obj_center[0] - base[0], obj_center[1] - base[1], 0.0])
        if np.linalg.norm(frontal) < 1e-6:
            frontal = np.array([1.0, 0.0, 0.0])
        ang0 = np.arctan2(frontal[1], frontal[0])
        # 캔 윗부분을 잡도록 grasp 높이를 올림 → 그리퍼/손목이 책상면에서 더 떠서 책상 충돌 회피하며 파지.
        #   0.7·half = 중심서 +0.047m(상단 0.0675 바로 아래) = 상단부. RH-P12 적응형이라 상단 케이지 가능.
        grasp_center = np.array([obj_center[0], obj_center[1],
                                 obj_center[2] + 0.7 * obj_half_h])   # 캔 상단부(손목 책상클리어↑). 윗면(1.0)은 미끄러짐
        _dbg = []   # [진단] azimuth별 운명(+x deg=0 제외 원인 추적)
        for deg in [0, -20, 20, -40, 40, -60, 60, -80, 80, -100, 100]:
            ang = ang0 + np.radians(deg)
            a = np.array([np.cos(ang), np.sin(ang), 0.0])
            g_use = side_grasp_from_approach(a, grasp_center, RHP12_TCP_DEPTH)
            if g_use is None:
                _dbg.append((deg, "synth_none")); continue
            if not is_in_workspace(g_use[:3, 3], base):
                rej["workspace"] += 1; _dbg.append((deg, "ws_out")); continue
            ik = ik_solver.solve_single(mat4_to_curobo_pose(g_use, tensor_args),
                                        q_now.view(1, -1), q_now.view(1, 1, -1))
            if not ik.success.item():
                rej["ik"] += 1; _dbg.append((deg, "IK_FAIL")); continue
            pre = pregrasp_from_grasp(g_use, PREGRASP_STANDOFF)
            ikp = ik_solver.solve_single(mat4_to_curobo_pose(pre, tensor_args),
                                         q_now.view(1, -1), q_now.view(1, 1, -1))
            if not ikp.success.item():
                rej["pregrasp"] += 1; _dbg.append((deg, "PRE_FAIL")); continue
            q_sol = ik.solution.view(-1)[:q_now.shape[0]]
            jcost, _mv, _wr, _rs = axis_load_cost(q_sol, q_now, retract)
            # ★문제4: 이 자세에서 팔 최저 충돌구체가 책상 위로 유지되는지(손목 5,6이 책상에 안 박히게).
            clr = lowest_sphere_bottom_world(getattr(ik_solver, "kinematics", None), q_sol, base)
            _dbg.append((deg, f"OK clr={clr if clr is not None else -9:.3f} jc={jcost:.2f}"))
            passed.append((jcost, 1.0, g_use, pre, deg, g_use[:3, 2].round(2), _wr,
                           (clr if clr is not None else 1e9)))
        _tt = base[2] - 0.03
        print(f"  [진단:azimuth] table_top={_tt:.3f}, 컷={_tt-0.005:.3f}", flush=True)
        for _d, _s in sorted(_dbg, key=lambda x: x[0]):
            _safe_mark = "  ←책상침범" if _s.startswith("OK") and float(_s.split("clr=")[1].split()[0]) < _tt - 0.005 else ""
            print(f"     deg={_d:+4d}: {_s}{_safe_mark}", flush=True)
        if not passed:
            print(f"  [IK 필터] side 합성 도달 실패. 탈락: 작업공간밖={rej['workspace']}, "
                  f"IK={rej['ik']}, pre-grasp={rej['pregrasp']} (정면±각도 스윕)", flush=True)
            return None, None, []
        # ★문제4 선별: 팔 충돌구체가 책상(table_top) 위로 유지되는 후보만 우선 채택.
        #   그리퍼는 캔(책상 위) 높이라 정상이고, 손목(5,6)이 책상 아래로 내려가는 자세를 배제.
        table_top = base[2] - 0.03
        safe = [p for p in passed if p[7] >= table_top - 0.005]
        if safe:
            # 축부하(jcost)만 prior로 정렬. 장면 의존 편향(정면페널티·클리어런스 가산)은 제거 —
            #   최종 파지점은 plan_grasp(goalset)이 월드충돌·도달로 직접 선택하므로 휴리스틱 편향 불필요
            #   (Phase 2.5 정석). 책상침범은 위 safe 하드필터(p[7]≥table_top)가 이미 보장. (2026-06-12)
            safe.sort(key=lambda x: x[0])
            chosen = safe[0]
        else:
            print("  [IK 필터] ⚠ 모든 side 후보가 책상 침범 → 최대 클리어런스 차선 선택", flush=True)
            passed.sort(key=lambda x: -x[7])
            chosen = passed[0]
        jcost, sc, g_sel, pre_sel, deg, appr, wr, clr = chosen
        print(f"  [IK 필터] side 통과 {len(passed)}개(책상위 {len(safe)}개) → azimuth={deg:+d}°, "
              f"approach={appr}, 축부하={jcost:.2f}(손목={wr:.2f}rad), 최저구체월드z={clr:.3f}(책상{table_top:.2f})", flush=True)
        # ★Phase1/2: plan_grasp용 후보 묶음(책상위 안전후보, 축부하 낮은 순).
        #   Phase2 walk를 위해 상한 없이 전 azimuth 후보를 넘김 → 양옆 클러터로 ±y가 막혀도
        #   자유 방향(+x 정면=deg0 등)까지 순차로 도달해 무충돌 파지를 찾음. (스윕이 11개라 짧음)
        _pool = sorted((safe if safe else passed), key=lambda x: x[0])
        cands = [p[2] for p in _pool]
        return g_sel, pre_sel, cands
    for rank, idx in enumerate(order):
        g   = grasps_world[idx]
        approach = g[:3, 2]
        closing  = g[:3, 0]
        if grasp_mode == "side":
            if abs(approach[2]) > SIDE_APPROACH_Z_MAX:
                rej["approach"] += 1
                continue
            if obj_center is None:        # 합성에 캔 중심 필요
                rej["height"] += 1
                continue
            # GraspGen 방위만 취해 RH-P12용 side 파지 합성 (회전·깊이·위치 정확)
            g_use = synthesize_side_grasp_rhp12(g, obj_center, RHP12_TCP_DEPTH)
            if g_use is None:             # top-down → side 불가
                rej["approach"] += 1
                continue
            if not is_in_workspace(g_use[:3, 3], base):
                rej["workspace"] += 1
                continue
        else:  # top
            pos = g[:3, 3]
            if not is_in_workspace(pos, base):
                rej["workspace"] += 1
                continue
            if approach[2] > approach_z_max:
                rej["approach"] += 1
                continue
            if abs(closing[2]) > 0.40:
                rej["closing"] += 1
                continue
            g_use = snap_grasp_roll_90(g, obj_R=obj_R)
        cpose = mat4_to_curobo_pose(g_use, tensor_args)
        ik_result = ik_solver.solve_single(cpose, q_now.view(1, -1), q_now.view(1, 1, -1))
        if not ik_result.success.item():
            rej["ik"] += 1
            continue
        # pre-grasp(후퇴 자세)도 도달 가능해야 실제 접근 성공 → 미리 검증(plan 실패 예방)
        pre = pregrasp_from_grasp(g_use, PREGRASP_STANDOFF)
        ik_pre = ik_solver.solve_single(mat4_to_curobo_pose(pre, tensor_args),
                                        q_now.view(1, -1), q_now.view(1, 1, -1))
        if not ik_pre.success.item():
            rej["pregrasp"] += 1
            continue
        q_sol = ik_result.solution.view(-1)[:q_now.shape[0]]
        jcost, _mv, _wr, _rs = axis_load_cost(q_sol, q_now, retract)
        passed.append((jcost, float(scores[idx]), g_use, pre, rank,
                       g_use[:3, 2].round(2), _wr))   # 스냅 후 approach 기록(진단 정확화)
        if len(passed) >= max_candidates:
            break
    if not passed:
        print(f"  [IK 필터] 도달 가능 파지 없음. 탈락사유: "
              f"작업공간밖={rej['workspace']}, approach방향={rej['approach']}, "
              f"closing방향={rej['closing']}, 몸통높이밖={rej['height']}, "
              f"IK실패={rej['ik']}, pre-grasp실패={rej['pregrasp']} (총 {len(order)}개)", flush=True)
        return None, None, []
    print(f"  [IK 필터] 탈락: 작업공간밖={rej['workspace']}, approach={rej['approach']}, "
          f"closing={rej['closing']}, 몸통높이밖={rej['height']}, IK={rej['ik']}, "
          f"pre-grasp={rej['pregrasp']} / 통과={len(passed)}", flush=True)
    passed.sort(key=lambda x: x[0])            # 축 부하 비용 최소 우선
    jcost, sc, g_sel, pre_sel, rank, appr, wr = passed[0]
    print(f"  [IK 필터] 선택 rank={rank+1}, score={sc:.3f}, approach={appr}, "
          f"축부하비용={jcost:.2f} (손목변화={wr:.2f}rad), 후보 {len(passed)}개 중 최소", flush=True)
    return g_sel, pre_sel, []   # top 모드는 기존 plan_single 경로(plan_grasp 후보 없음)


# 월드 → cuRobo(로봇 base) 프레임 위치 보정. cuRobo는 base_link를 원점으로 계획하나
#   로봇은 월드 [-0.25,-0.04,0.73]에 놓여 있음(base 회전 0) → 모든 타겟 위치에서 이 값을 빼야
#   EE가 의도한 월드 위치에 옴. main에서 robot_base로 설정.
_ROBOT_BASE_OFFSET = np.zeros(3, dtype=np.float32)


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


def diag_pose_fail(tag, pos, quat_wxyz, ik_solver, result, tensor_args):
    """플래닝 실패 원인 진단: MotionGen status + IK 단독 도달 가능성.
    IK 성공인데 plan 실패면 → 충돌/궤적 문제. IK 실패면 → 자세 자체 도달 불가."""
    status = getattr(result, "status", None)
    goal = Pose(
        position=tensor_args.to_device((np.array(pos, dtype=np.float32) - _ROBOT_BASE_OFFSET).reshape(1, 3)),
        quaternion=tensor_args.to_device(np.array([quat_wxyz], dtype=np.float32)),
    )
    try:
        ik = ik_solver.solve_single(goal)
        ik_ok = bool(ik.success.item())
    except Exception as e:
        ik_ok = None
        status = f"{status} / IK예외:{e}"
    if ik_ok is True:
        cause = "IK 도달 가능 → 충돌/궤적(trajopt) 문제로 추정 (충돌객체·시작자세 확인)"
    elif ik_ok is False:
        cause = "IK 자체 실패 → 자세(pos/quat) 도달 불가 (좌표·접근방향·작업공간 확인)"
    else:
        cause = "IK 검사 예외"
    print(f"  [진단:{tag}] status={status}, IK_단독={ik_ok} → {cause}", flush=True)


def reachability_sanity(ik_solver, tensor_args, q_now):
    """E0509 도달 영역 맵 (base 프레임, offset 무관). 캔 높이가 닿는지/어디가 닿는지 진단.
    각 칸: T=top접근(아래로) 도달, S=side접근(수평 전방) 도달, .=불가."""
    from scipy.spatial.transform import Rotation as _Rt
    q = q_now.view(1, -1)
    def ik_ok(pos, appr):
        a = np.array(appr, dtype=float); a /= (np.linalg.norm(a) + 1e-9)
        tmp = np.array([0., 0., 1.]) if abs(a[2]) < 0.9 else np.array([1., 0., 0.])
        x = np.cross(tmp, a); x /= (np.linalg.norm(x) + 1e-9)
        y = np.cross(a, x)
        R = np.column_stack([x, y, a])
        qx = _Rt.from_matrix(R).as_quat()   # xyzw
        pose = Pose(position=tensor_args.to_device(np.array([pos], dtype=np.float32)),
                    quaternion=tensor_args.to_device(np.array([[qx[3], qx[0], qx[1], qx[2]]], dtype=np.float32)))
        try:   # select/probe와 동일 시그니처(3-인자) — cuda_graph 일관성 유지
            return bool(ik_solver.solve_single(pose, q, q.view(1, 1, -1)).success.item())
        except Exception:
            return False
    print("[reach] base프레임 IK 도달맵 (forward d=0.25/0.35/0.45/0.55, TS=top/side):", flush=True)
    for h in [0.6, 0.4, 0.2, 0.05, -0.1, -0.3, -0.5]:
        cells = []
        for d in [0.25, 0.35, 0.45, 0.55]:
            t = 'T' if ik_ok([d, 0, h], [0, 0, -1]) else '.'
            s = 'S' if ik_ok([d, 0, h], [1, 0, 0]) else '.'
            cells.append(t + s)
        print(f"   base_z={h:+.2f} :  " + "   ".join(cells), flush=True)


def draw_cspheres_usd(stage, motion_gen, q_tensor, base, table_top=0.70, quiet=True):
    """cuRobo 로봇 충돌구체를 월드 USD 구로 그림(시각화). 책상 top 아래 구체=빨강(처박음).
    매 호출마다 /World/cspheres를 갱신 → 로봇 따라다니며 책상/매대 간섭을 눈으로 확인."""
    from pxr import UsdGeom, Gf
    try:
        st  = motion_gen.kinematics.get_state(q_tensor.view(1, -1))
        sph = st.get_link_spheres().reshape(-1, 4).detach().cpu().numpy()
        root = "/World/cspheres"
        if stage.GetPrimAtPath(root).IsValid():
            stage.RemovePrim(root)
        UsdGeom.Scope.Define(stage, root)
        n = 0; below = 0; low = None
        for i, (x, y, z, r) in enumerate(sph):
            if r <= 0.0:
                continue
            wx, wy, wz = float(x) + base[0], float(y) + base[1], float(z) + base[2]
            bottom = wz - float(r)
            if low is None or bottom < low[2]:
                low = (wx, wy, bottom)
            sp = UsdGeom.Sphere.Define(stage, f"{root}/s{i}")
            sp.CreateRadiusAttr(float(r))
            UsdGeom.Xformable(sp).AddTranslateOp().Set(Gf.Vec3d(wx, wy, wz))
            if bottom < table_top:          # 책상 top 아래 = 빨강(처박음)
                sp.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)]); below += 1
            else:
                sp.CreateDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.9, 0.2)])  # 초록(정상)
            sp.CreateDisplayOpacityAttr().Set([0.45])
            n += 1
        if not quiet and low is not None:
            print(f"  [구체VIZ] {n}개(책상아래 빨강 {below}개) 최저구체 월드z={low[2]:.3f}", flush=True)
    except Exception as e:
        if not quiet:
            print(f"  [구체VIZ] 실패: {e}", flush=True)


def attach_cspheres_to_links(stage, robot_prim_path, spheres_yaml_path, opacity=0.35):
    """충돌구체를 각 로봇 링크 prim의 자식 USD Sphere로 '1회' 생성. cuRobo 구체는 링크 로컬
    좌표라 그대로 자식으로 붙이면 USD 트랜스폼 계층이 매 렌더 프레임 로봇과 강체로 이동 →
    시각화 lag 0(매 프레임 재계산 불필요). 순수 시각용 — 충돌 회피 로직과 무관(prim은 robot
    서브트리=ignore_substring이라 장애물로도 안 잡힘). 고객 제출용 깔끔 영상."""
    from pxr import UsdGeom, Gf
    data = load_yaml(spheres_yaml_path)["collision_spheres"]
    name2path = {}
    root = stage.GetPrimAtPath(robot_prim_path)
    if root.IsValid():
        for p in Usd.PrimRange(root):          # 깊이우선 → 링크 Xform이 자식 visuals보다 먼저
            name2path.setdefault(p.GetName(), p.GetPath())
    made = 0; miss = []
    for link, spheres in data.items():
        ppath = name2path.get(link)
        if ppath is None:
            miss.append(link); continue
        UsdGeom.Scope.Define(stage, f"{ppath}/cspheres")
        for i, s in enumerate(spheres):
            c = s.get("center"); r = float(s.get("radius", 0.0))
            if r <= 0.0 or c is None:
                continue
            sp = UsdGeom.Sphere.Define(stage, f"{ppath}/cspheres/s{i}")
            sp.CreateRadiusAttr(r)
            UsdGeom.Xformable(sp).AddTranslateOp().Set(Gf.Vec3d(float(c[0]), float(c[1]), float(c[2])))
            sp.CreateDisplayColorAttr().Set([Gf.Vec3f(0.15, 0.55, 0.95)])   # 옅은 파랑(통일)
            sp.CreateDisplayOpacityAttr().Set([float(opacity)])
            made += 1
    print(f"  [구체VIZ] 링크부착 {made}개 생성(자동 강체 추종). 미발견 링크={miss}", flush=True)
    return made


def make_jlim_urdf(src_urdf, dst_urdf, overrides):
    """원본 URDF를 건드리지 않고, 지정 관절의 position 한계만 바꾼 복사본을 생성.
    cuRobo는 URDF에서 위치한계를 읽고 BoundCost가 init에서 clone하므로, 비대칭 한계(예: j5≥0)는
    런타임 텐서 수정이 아니라 MotionGen 생성 전 URDF에서 줘야 반영됨. overrides={joint:(lower,upper)}."""
    import xml.etree.ElementTree as ET
    tree = ET.parse(src_urdf)
    root = tree.getroot()
    hit = []
    for j in root.findall("joint"):
        nm = j.get("name")
        if nm in overrides:
            lim = j.find("limit")
            lo, up = overrides[nm]
            lim.set("lower", repr(float(lo)))
            lim.set("upper", repr(float(up)))
            hit.append(nm)
    tree.write(dst_urdf, encoding="utf-8", xml_declaration=True)
    print(f"  [URDF한계] {dst_urdf} 생성 — 수정 관절 {hit}", flush=True)
    return dst_urdf


def log_arm_deg(robot, arm_joint_names, tag):
    """현재 arm 6관절 각도(deg) 로깅 — 자세 분기 진단(joint_4·6 180° 플립 추적)."""
    try:
        jp = robot.get_joint_positions()
        deg = [round(np.degrees(float(jp[robot.get_dof_index(n)])), 1) for n in arm_joint_names]
        print(f"  [관절deg:{tag}] j1~6 = {deg}", flush=True)
    except Exception as e:
        print(f"  [관절deg:{tag}] 실패: {e}", flush=True)


def execute_plan(cmd, sim_js_names, robot, ctrl, my_world, extra_steps=1, track_tag=None,
                 arm_only=False, viz=None):
    idx_list, common = [], []
    for x in sim_js_names:
        if x in cmd.joint_names:
            # arm_only: 그리퍼 관절 제외 → 캔 든 채 운반/복귀 시 그리퍼 열려 캔 떨구는 것 방지
            #   (cuRobo 계획은 lock_joints로 그리퍼=0(열림)이라 그대로 주면 그리퍼가 열림)
            if arm_only and str(x).startswith("gripper_rh"):
                continue
            idx_list.append(robot.get_dof_index(x))
            common.append(x)
    cmd_ord = cmd.get_ordered_joint_state(common)
    clr_min = None                             # [간격로깅] 세그먼트 최소간격 누적
    for i in range(len(cmd_ord.position)):
        ctrl.apply_action(ArticulationAction(
            cmd_ord.position[i].cpu().numpy(),
            cmd_ord.velocity[i].cpu().numpy(),
            joint_indices=idx_list,
        ))
        for _ in range(extra_steps):
            my_world.step(render=True)
        clr_min = _clr_probe(clr_min)
        if viz is not None and i % 10 == 0:    # 동작 중 충돌구체 실시간 갱신(팔 따라감)
            viz()
    # 추종오차(관절공간, 프레임 무관): 마지막 명령 관절각 vs 정착 후 실제 측정.
    #   게인이 궤적을 정확히 추종하는지 직접 지표. 큰 값이면 게인/effort 부족.
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
    """RH-P12 그리퍼 닫기/열기 (물리 제어, 점진 램프). r1/l1/r2/l2 모두 mimic(×1)이라 같은 각.
    목표를 한번에 주면 stiffness 2000으로 슬램해 캔을 쳐냄 → 현재각에서 목표로 점진 보간해
    손가락이 캔을 부드럽게 감싸게 함. angle: GRIP_OPEN(0)~GRIP_CLOSE(1.05)."""
    gidx = [robot_art.get_dof_index(n) for n in sim_js_names if str(n).startswith("gripper_rh")]
    if not gidx:
        return
    try:
        cur = float(robot_art.get_joint_positions()[gidx[0]])
        for t in np.linspace(0.0, 1.0, max(steps, 2)):
            tgt = (1.0 - t) * cur + t * float(angle)
            ctrl.apply_action(ArticulationAction(
                np.array([tgt] * len(gidx), dtype=np.float32), joint_indices=gidx))
            my_world.step(render=True)
    except Exception as e:
        print(f"  [그리퍼] 제어 실패: {e}", flush=True)


def move_direct_ik(target_world, ik_solver, tensor_args, cu_js_pos, arm_joint_names,
                   robot_art, ctrl, my_world, steps=50, settle=8, viz=None):
    """월드 4x4 목표로 직접 IK + 관절 보간 이동(짧은 동작, start-collision 검사 우회).
    리프트(PLAN_LIFT)와 동일 패턴. 성공 시 True. 매대 삽입/하강/후퇴에 사용."""
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
        if viz is not None and j % 10 == 0:    # 동작 중 충돌구체 실시간 갱신(팔 따라감)
            viz()
    for _ in range(settle):
        my_world.step(render=True)
    _clr_report(clr_min, "directIK")
    return True


def lowest_sphere_bottom_world(kin, q_vec, base):
    """주어진 관절각 q에서 로봇 충돌구체 중 가장 낮은 구체의 바닥 월드 z. 책상 침범 판정용.
    kin=cuRobo CudaRobotModel(get_state). 실패/미지원 시 None."""
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


# [간격로깅] Stage5: 실측 관절각 → 로봇 충돌구체(부착 캔 포함)-월드 장애물 최소간격(m).
#   세그먼트별 running-min을 [간격:tag]로 출력해 "무접촉"을 정착오차가 아닌 실거리로 증명.
_CLEARANCE = {"fn": None, "buf": None, "shape": None, "w": None, "act": None}


def min_world_clearance(motion_gen, q_dev):
    """측정 관절각(cuRobo 관절순서, device tensor)에서 구체-장애물 최소간격(m).
    world_coll_checker ESDF 질의(compute_esdf=True): 커널이 구체 반경을 빼고 반환하므로
    d>0=구체표면 침투깊이, d<0=간격 → 간격 = -d. motion_gen의 월드를 그대로 공유(이중관리 없음).
    유효반경(r>0) 구체만 집계 — 비활성 패딩구체(r<0)는 커널이 0을 써 min을 오염시킴.
    간격은 max_distance(0.1m) 근방서 포화 → 접촉 근방 값만 의미. 실패 시 None."""
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


def slot_feasible(slot_xy, entry_z, place_z, ik_solver, tensor_args, cu_js_pos,
                  motion_gen, used_xy):
    """[Phase3] 슬롯 실현성 사전검사. 좌표 하드코딩 신뢰 대신 매번 검사(multiobj_run1 교훈:
    x0.14=그리퍼-이웃 간섭, x0.36=도달한계). 검사 3종 — ① 기적치 캔과 중심거리 ≥ SLOT_MIN_DIST
    (하강 시 그리퍼 스윕 간섭, 순수 기하), ② 삽입·하강 pose IK(도달한계+관절한계 분기),
    ③ 삽입 pose 자세의 구체-장애물 간격>0 (벽 간섭, 부착 캔 포함). (ok, 사유) 반환."""
    sx, sy = slot_xy
    for ux, uy in used_xy:
        if np.hypot(sx - ux, sy - uy) < SLOT_MIN_DIST:
            return False, f"기적치근접({np.hypot(sx-ux, sy-uy):.3f}m)"
    q_ins = None
    for cz, tag in ((entry_z, "삽입"), (place_z, "하강")):
        Tw = side_grasp_from_approach(SHELF3_APPROACH, [sx, sy, cz], RHP12_TCP_DEPTH)
        ikr = ik_solver.solve_single(mat4_to_curobo_pose(Tw, tensor_args),
                                     cu_js_pos.view(1, -1), cu_js_pos.view(1, 1, -1))
        if not ikr.success.item():
            return False, f"IK불가({tag})"
        if tag == "삽입":
            q_ins = ikr.solution.view(-1)[:cu_js_pos.view(-1).shape[0]]
    c = min_world_clearance(motion_gen, q_ins)
    if c is not None and c <= 0.0:
        return False, f"간격침투(삽입 {c*1000:+.0f}mm)"
    return True, ""


def move_linear_ik(start_world, target_world, ik_solver, tensor_args, cu_js_pos,
                   arm_joint_names, robot_art, ctrl, my_world,
                   waypoints=40, substeps=2, settle=10, viz=None, tag=""):
    """TCP를 start→target 데카르트 직선으로 이동(moveL). 위치만 선형보간, 자세 고정.
    각 웨이포인트마다 IK를 풀어 따라가므로 단일축만 다르면 그 축으로만 직진(예: +y/-z/-y).
    IK 실패 웨이포인트 발생 시 그 지점에서 중단하고 False(부분 이동 상태)."""
    arm_idx = [robot_art.get_dof_index(n) for n in arm_joint_names]
    nA = len(arm_joint_names)
    p0 = np.asarray(start_world[:3, 3],  dtype=np.float64)
    p1 = np.asarray(target_world[:3, 3], dtype=np.float64)
    R  = target_world[:3, :3]                     # 자세 고정(직립 유지, start==target 회전)
    q_prev = cu_js_pos.view(-1).cpu().numpy()[:nA].astype(np.float32)
    seed   = cu_js_pos.view(1, 1, -1)
    clr_min = None                             # [간격로깅]
    for i in range(1, waypoints + 1):
        t  = i / waypoints
        Tw = np.eye(4, dtype=np.float32)
        Tw[:3, :3] = R
        Tw[:3, 3]  = (1.0 - t) * p0 + t * p1
        ikr = ik_solver.solve_single(mat4_to_curobo_pose(Tw, tensor_args),
                                     cu_js_pos.view(1, -1), seed)
        if not ikr.success.item():
            print(f"  [moveL{(' ' + tag) if tag else ''}] 웨이포인트 {i}/{waypoints} IK 실패 → 중단", flush=True)
            return False
        q_tgt = ikr.solution.view(-1)[:nA].cpu().numpy().astype(np.float32)
        for s in range(1, substeps + 1):          # 드라이브 추종용 미세 보간
            q = ((1.0 - s / substeps) * q_prev + (s / substeps) * q_tgt).astype(np.float32)
            ctrl.apply_action(ArticulationAction(q, joint_indices=arm_idx))
            my_world.step(render=True)
        q_prev = q_tgt
        seed   = ikr.solution.view(1, 1, -1)      # 다음 IK는 현 해로 시드 → 관절 연속성
        clr_min = _clr_probe(clr_min)
        if viz is not None and i % 5 == 0:
            viz()
    for _ in range(settle):
        my_world.step(render=True)
    _clr_report(clr_min, f"moveL{(' ' + tag) if tag else ''}")
    return True


def main():
    setup_curobo_logger("warn")

    # ── 로봇 설정 로드 ───────────────────────────────────────────────────────
    robot_cfg = load_yaml(ROBOT_YML)["robot_cfg"]
    # 홈 포즈를 파지 친화 자세로 교체 (init 텔레포트 + cuRobo retract 시드 일관)
    robot_cfg["kinematics"]["cspace"]["retract_config"] = list(RETRACT_CONFIG)
    # ★자세 정밀화(측정 기반): 운반 중 손목 플립(j5 +81°→−92°, j4 270° 회전 → 홈 204° 되감김)이 원인.
    #   대칭 position_limit_clip으론 비대칭(j5≥0=손목 위 유지)이 안 되고, 한계는 URDF에서 와 BoundCost가
    #   init에서 clone하므로 → MotionGen 생성 전 URDF 복사본에서 한계만 수정해 비대칭 적용.
    #   j5: [0, +135°]=손목 항상 위(플립 분기 차단, 파지 +81°는 안전). j4: ±180°(270° 과회전 차단).
    _JLIM = {"joint_4": (-3.141592653589793, 3.141592653589793),
             "joint_5": (0.0, 2.356194490192345)}
    _jlim_urdf = make_jlim_urdf(f"{ROBOT_DIR}/e0509_gripper_abs.urdf",
                                "/tmp/e0509_gripper_jlim.urdf", _JLIM)
    robot_cfg["kinematics"]["urdf_path"]          = _jlim_urdf
    robot_cfg["kinematics"]["asset_root_path"]    = ROBOT_DIR
    # 충돌구체: Isaac Sim Lula 에디터로 직접 만든 cuRobo 네이티브 YAML을 그대로 로드(런타임 생성/변환 없음).
    #   갱신 절차: 에디터 Export → normalize_lula_spheres.py 1회 → e0509_spheres.yml 덮어쓰기.
    robot_cfg["kinematics"]["collision_spheres"]  = f"{ROBOT_DIR}/e0509_spheres.yml"
    # cuRobo는 mimic joint(rh_r2/l1/l2)를 joint_data에 안 넣음 → lock_joints에 두면 KeyError.
    # 주 구동 gripper_rh_r1만 lock (mimic은 따라감). Isaac Sim에서 그리퍼는 별도 제어.
    robot_cfg["kinematics"]["lock_joints"]        = {"gripper_rh_r1": 0.0}
    # base_link은 책상 마운트 → 충돌검사 제외(구체 미생성). collision_link_names·self_collision에서도 제거.
    _kin = robot_cfg["kinematics"]
    if "base_link" in _kin.get("collision_link_names", []):
        _kin["collision_link_names"] = [l for l in _kin["collision_link_names"] if l != "base_link"]
    if isinstance(_kin.get("self_collision_ignore"), dict):
        _kin["self_collision_ignore"].pop("base_link", None)
        for _k in _kin["self_collision_ignore"]:
            _kin["self_collision_ignore"][_k] = [x for x in _kin["self_collision_ignore"][_k] if x != "base_link"]
    if isinstance(_kin.get("self_collision_buffer"), dict):
        _kin["self_collision_buffer"].pop("base_link", None)

    j_names       = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_cfg   = robot_cfg["kinematics"]["cspace"]["retract_config"]
    ee_link       = robot_cfg["kinematics"]["ee_link"]
    print(f"로봇: E0509, EE={ee_link}, 조인트={j_names}", flush=True)

    # ── Isaac Sim 월드 ───────────────────────────────────────────────────────
    my_world = World(stage_units_in_meters=1.0)
    stage    = my_world.stage

    # 사용자 제작 매대 씬(v2) 로드 — 로봇(/World/Robot) + Table + Shelf + base 포함
    # v2는 defaultPrim 메타가 없어 add_reference_to_stage(defaultPrim)가 실패 →
    # Sdf.Reference로 v2 내 "/World" prim을 현재 /World에 명시 참조
    robot_prim_path = ROBOT_PRIM   # /World/Robot
    # World()가 /World prim을 아직 안 만들었을 수 있어 DefinePrim으로 보장 후 참조
    world_prim = stage.DefinePrim("/World", "Xform")
    world_prim.GetReferences().AddReference(Sdf.Reference(V2_USD, "/World"))
    print(f"매대 씬(v2) 로드 → /World : {V2_USD}", flush=True)

    # v2 로봇은 floating-base(IsaacLab) → 중력에 흔들림. base_link를 월드에 고정.
    fix_robot_base(stage, ROBOT_PRIM, base_link="base_link")
    # 씬 보정: 로봇 받침 블록 추가 + 매대 책상 안착 + 책상/매대 정적화
    fix_scene(stage)

    # 매대 좌표 검사 모드: 선반 높이/포인트 출력 후 종료
    if args.inspect_shelf:
        inspect_prim_tree(stage, "/World/Shelf", max_depth=3)
        inspect_prim_tree(stage, "/World/Table", max_depth=2)
        print("[매대검사] 완료 → 종료", flush=True)
        simulation_app.close()
        return

    # v2 좌표 진단: 로봇 base / 테이블 위치 (큐브를 테이블 위에 놓기 위해)
    _robot_base = get_ee_world_pos(stage, ROBOT_PRIM)
    _table_pos  = get_ee_world_pos(stage, "/World/Table")
    _shelf_pos  = get_ee_world_pos(stage, "/World/Shelf")
    print(f"[진단] 로봇 base={None if _robot_base is None else _robot_base.round(3)}, "
          f"Table={None if _table_pos is None else _table_pos.round(3)}, "
          f"Shelf={None if _shelf_pos is None else _shelf_pos.round(3)}", flush=True)
    # 로봇 base 회전 진단: v2에서 로봇이 회전돼 있으면 월드 [1,0,0,0] 탑다운이 안 맞음
    _rp = stage.GetPrimAtPath(ROBOT_PRIM)
    if _rp.IsValid():
        _T = UsdGeom.Xformable(_rp).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        _R = np.array([[_T[0][0], _T[0][1], _T[0][2]],
                       [_T[1][0], _T[1][1], _T[1][2]],
                       [_T[2][0], _T[2][1], _T[2][2]]])
        _euler = Rotation.from_matrix(_R).as_euler("xyz", degrees=True)
        print(f"[진단] 로봇 base 회전(xyz deg)={_euler.round(1)}  "
              f"(0,0,0 이 아니면 탑다운 quat 보정 필요)", flush=True)
    # 물체를 로봇 앞(+x) 테이블 윗면에 배치 (로봇 base z ≈ 테이블 윗면)
    _obj_type = args.obj_type
    _half = (OBJ_SPECS["cylinder"]["height"] / 2 if _obj_type == "cylinder" else CUBE_SIZE / 2)
    # 캔: 책상 위, 로봇 앞 0.50m (사용자 지정 — base와 일정 거리 이상이어야 파지 자세 나옴)
    if _robot_base is not None:
        _cx, _cy = _robot_base[0] + args.obj_dist, _robot_base[1] + args.target_dy
        # 캔은 table1(윗면 월드 0.70 = robot_base 0.73 − 베이스블록 0.03)에 안착.
        #   기존 robot_base_z+half(0.785)는 0.035 띄워 떨어지며 튀고 기울어짐 → 들쭉날쭉(0.73/0.75).
        #   책상면 바로 위(+2mm)에 놓아 드롭 최소화 → 항상 직립 0.75 안착.
        _table_top = _robot_base[2] - 0.03
        _cz = _table_top + _half + 0.002
    else:
        _cx, _cy, _cz = 0.5, 0.0, _half
    print(f"[물체] type={_obj_type}, 생성 위치=[{_cx:.3f}, {_cy:.3f}, {_cz:.3f}]", flush=True)

    # 목표 물체: cylinder=캔(세워서 매대 진열 목표), box=스낵 근사
    targets = []   # [Phase3 다물체] [{"obj","path","status":pending|placed|skipped,"reason"}]
    if _obj_type == "cylinder" and args.objects > 1:
        # ── Phase3 다물체: 픽 대상 캔 N개를 y줄로 스폰(전부 동적=전부 픽 대상) ──
        from omni.isaac.core.objects import cylinder as _cyl
        s = OBJ_SPECS["cylinder"]
        _tgt_colors = [np.array([0.85, 0.1, 0.1]), np.array([0.1, 0.3, 0.9]),
                       np.array([0.1, 0.7, 0.2]), np.array([0.9, 0.6, 0.1])]
        for _i in range(args.objects):
            _dy_i = args.target_dy + (_i - (args.objects - 1) / 2.0) * args.obj_gap
            _op = f"/World/obj_{_i}"
            _ob = _cyl.DynamicCylinder(
                prim_path=_op, name=f"obj_{_i}",
                position=np.array([_cx, _robot_base[1] + _dy_i, _cz]),
                radius=s["radius"], height=s["height"],
                color=_tgt_colors[_i % len(_tgt_colors)], mass=0.30,
            )
            targets.append({"obj": _ob, "path": _op, "status": "pending", "reason": None})
            print(f"[다물체] {_op} @ dy={_dy_i:+.2f}", flush=True)
        target_cube = targets[0]["obj"]   # 이후 코드는 target_cube 별칭으로 현재 타겟 참조
    elif _obj_type == "cylinder":
        from omni.isaac.core.objects import cylinder as _cyl
        s = OBJ_SPECS["cylinder"]
        target_cube = _cyl.DynamicCylinder(
            prim_path="/World/target_cube", name="target_cube",
            position=np.array([_cx, _cy, _cz]),
            radius=s["radius"], height=s["height"],
            # 측면 타겟(--target-dy)은 파란색으로 표시(사용자: "파란색도 집어봐")
            color=(np.array([0.1, 0.3, 0.9]) if args.target_dy != 0 else np.array([0.85, 0.1, 0.1])),
            mass=0.30,   # 부분 캔 300g (그립 신뢰성↑; 만캔500g은 마진부족)
        )
    else:
        target_cube = cuboid.DynamicCuboid(
            prim_path="/World/target_cube", name="target_cube",
            position=np.array([_cx, _cy, _cz]),
            size=CUBE_SIZE, color=np.array([1.0, 0.4, 0.0]), mass=CUBE_MASS,
        )
    # 콜라캔(알루미늄)–그리퍼 고무패드 마찰. ★apply 해야 효력 발생 (stage3 교훈)
    cube_mat = PhysicsMaterial(
        prim_path="/World/Physics_Materials/cube_mat",
        static_friction=1.5, dynamic_friction=1.2, restitution=0.0,   # 그립 유지 위해↑(0.6/0.5→)
    )
    # Stage6-B2: 마찰 결합모드 max(두 표면 중 큰 쪽 채택) — 기본 average는 상대 표면이
    #   저마찰이면 유효마찰이 깎여 미끄러짐. (robotis_lab pick_place_env_cfg.py 정석)
    from pxr import PhysxSchema as _PhysxSchema
    _pmat = _PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath("/World/Physics_Materials/cube_mat"))
    _pmat.CreateFrictionCombineModeAttr().Set("max")
    _pmat.CreateRestitutionCombineModeAttr().Set("min")
    if targets:                       # Phase3 다물체: 전부 동일 재질
        for _t in targets:
            _t["obj"].apply_physics_material(cube_mat)
    else:                             # 단일 타겟도 targets 목록으로 통일(오케스트레이션 공용)
        target_cube.apply_physics_material(cube_mat)
        targets = [{"obj": target_cube, "path": "/World/target_cube", "status": "pending", "reason": None}]
    cur_tgt_i = 0                     # 현재 타겟 인덱스
    slot_used = [False] * len(SHELF3_SLOTS)     # [Phase3] 3단 슬롯 점유맵
    place_x, place_y = SHELF3_SLOTS[0]          # 현재 사이클 적치 (x, in_y) — PLAN_CARRY에서 갱신
    cur_slot  = 0

    # ── Phase2 클러터: 타겟 양옆 정적(kinematic) 캔 = cuRobo 장애물 ──────────
    #   타겟만 ignore_substring에 남고, 클러터는 ignore에 없으므로 자동으로 장애물.
    #   ±y로 띄워 옆 접근(±y azimuth)은 막히고 정면(-x→+x) 접근만 열려 회피 검증.
    CLUTTER_Y_OFF = 0.17   # center-to-center(m). 0.12(가장자리 0.06)는 그리퍼가 못 비집음(전 후보 거부)
                           #   → 0.17(가장자리 0.11)로 정면(+x) 진입 공간 확보 (사용자 확인 2026-06-10)
    if args.clutter > 0 and _obj_type == "cylinder":
        from omni.isaac.core.objects import cylinder as _cylc
        _sc = OBJ_SPECS["cylinder"]
        # 타겟이 파란색일 수 있으니(--target-dy) 클러터는 녹색/주황/보라/빨강로(파랑 회피)
        _cl_colors = [np.array([0.1, 0.7, 0.2]), np.array([0.9, 0.6, 0.1]),
                      np.array([0.6, 0.1, 0.7]), np.array([0.85, 0.1, 0.1])]
        for _i in range(args.clutter):
            if args.target_dy > 0:                    # 타겟=매대쪽(+y) → 클러터는 로봇쪽(-y)에 줄세움
                _clx, _cly = _cx, _cy - CLUTTER_Y_OFF * (_i + 1)
            elif args.target_dy < 0:
                _clx, _cly = _cx, _cy + CLUTTER_Y_OFF * (_i + 1)
            else:                                     # 중앙 타겟 → 양옆 번갈아
                _sign = 1 if _i % 2 == 0 else -1
                _ring = _i // 2 + 1
                _clx, _cly = _cx, _cy + _sign * CLUTTER_Y_OFF * _ring
            _clp = f"/World/clutter_{_i}"
            _cl = _cylc.DynamicCylinder(
                prim_path=_clp, name=f"clutter_{_i}",
                position=np.array([_clx, _cly, _cz]),
                radius=_sc["radius"], height=_sc["height"],
                color=_cl_colors[_i % len(_cl_colors)], mass=0.30,
            )
            _cl.apply_physics_material(cube_mat)
            set_kinematic(stage, _clp, True)          # 정적 장애물(로봇이 쳐도 안 움직임)
            print(f"[클러터] {_clp} @ [{_clx:.3f}, {_cly:.3f}, {_cz:.3f}]", flush=True)

    # ── cuRobo 설정 ─────────────────────────────────────────────────────────
    world_cfg_table = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
    )
    # 이 슬랩(5x5x0.2)은 cuRobo base 프레임 기준. 기본값이면 윗면이 로봇 마운트(월드 0.73)
    #   바로 아래(월드 0.71)에 깔려, 책상 위 캔을 잡으려는 grasp를 충돌로 거부함(거짓 데드존).
    #   → 슬랩 윗면을 실제 바닥(월드 0)에 맞춤. 실제 책상/매대 충돌은 update_world로 따로 반영.
    _floor_top = -float(_robot_base[2]) if _robot_base is not None else -0.73   # base프레임에서 월드0
    world_cfg_table.cuboid[0].pose[2] = _floor_top - world_cfg_table.cuboid[0].dims[2] / 2.0
    world_cfg = WorldConfig(cuboid=world_cfg_table.cuboid)

    tensor_args = TensorDeviceType()
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args,
        collision_checker_type=CollisionCheckerType.MESH,
        num_trajopt_seeds=12, num_graph_seeds=12,
        interpolation_dt=0.03,
        collision_cache={"obb": 60, "mesh": 60},   # v2 매대(Table/Shelf 등) 충돌객체 많음
        # 책상 위 캔을 잡으려면 그리퍼가 책상 가까이 가야 함 → 과한 안전여유가 모든 파지를 막음.
        #   5mm는 추종오차(joint_2 ~4°, 실경로 수cm 이탈)를 못 흡수해 클러터/캔을 실제로 스침 →
        #   15mm로 상향(파지 높이 0.7·half에서 손목-책상 여유 59mm라 파지는 안 막힘). 기본 ~0.025.
        collision_activation_distance=0.015,
        optimize_dt=True, trajopt_dt=None, trajopt_tsteps=32, trim_steps=[1, None],
        # ★Phase2.5: plan_grasp(goalset 선택)는 내부에서 goalset↔single을 섞어 호출 → cuda graph의
        #   고정 goal shape와 충돌("changing goal type", Phase0 확인). graph 끄면 계획 ~2s로 느려지나
        #   클러터 상황적합 선택이 우선. 속도는 Phase4서 재방문.
        use_cuda_graph=False,
    )
    motion_gen = MotionGen(motion_gen_config)
    print("warming up...", flush=True)
    motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
    print("cuRobo (E0509) Ready", flush=True)

    # ── 진단용 IK 솔버 (도달 가능성 단독 검사: IK 실패 vs 충돌/궤적 실패 구분) ──
    ik_config = IKSolverConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args,
        rotation_threshold=0.05, position_threshold=0.005,
        num_seeds=30, self_collision_check=True, self_collision_opt=True,
        use_cuda_graph=True,
    )
    ik_solver = IKSolver(ik_config)
    print("IK 솔버 Ready", flush=True)
    # 도달 영역 진단 (캔 높이 도달성/워크스페이스 판별)
    try:
        _q_seed = tensor_args.to_device(np.array(RETRACT_CONFIG, dtype=np.float32))
        reachability_sanity(ik_solver, tensor_args, _q_seed)
    except Exception as _e:
        print(f"[reach] 진단 실패: {_e}", flush=True)

    # ── GraspGen ZMQ 클라이언트 ───────────────────────────────────────────────
    grasp_client = None
    if not args.no_graspgen:
        sys.path.insert(0, "/home/devuser/graspgen_ws/GraspGen")
        from grasp_gen.serving.zmq_client import GraspGenClient
        print(f"[ZMQ] GraspGen 서버 연결 중 (port {args.port})...", flush=True)
        grasp_client = GraspGenClient("127.0.0.1", args.port, wait_for_server=True)
        print(f"[ZMQ] 연결 완료: {grasp_client._server_metadata}", flush=True)
    else:
        print("[GraspGen] --no-graspgen: 고정 탑다운 모드(Step1)", flush=True)

    obj_type   = args.obj_type
    grasp_mode = OBJ_SPECS[obj_type]["grasp_mode"]
    # 로봇 base (작업공간 필터 기준) — 진단에서 구한 _robot_base 재사용
    robot_base = _robot_base if _robot_base is not None else np.array([0.0, 0.0, 0.0])
    # ★cuRobo는 base_link 원점 기준 → 월드 타겟에서 base 위치를 빼도록 전역 offset 설정.
    #   (안 하면 모든 EE 타겟이 base 위치만큼 어긋나 공중으로 감 = joint1 폭주/그리퍼 캔서 멀어짐)
    global _ROBOT_BASE_OFFSET
    _ROBOT_BASE_OFFSET = np.array(robot_base, dtype=np.float32)
    print(f"[프레임] cuRobo base offset = {_ROBOT_BASE_OFFSET.round(3)} (월드 타겟에서 차감)", flush=True)

    # 점구름 시각화용 USD Points prim
    from pxr import Gf as _Gf, Vt as _Vt
    pc_prim = UsdGeom.Points.Define(stage, "/World/debug_pc")

    # 축 부하 비용의 기준이 되는 편안한 자세(retract) 텐서
    retract_t = tensor_args.to_device(np.array(RETRACT_CONFIG, dtype=np.float32))

    plan_config = MotionGenPlanConfig(
        enable_graph=False, enable_graph_attempt=4,
        max_attempts=8, enable_finetune_trajopt=True,
        time_dilation_factor=0.7,
    )
    # ★Phase2.5: 파지는 plan_grasp(2단계: offset까지 풀충돌인지 → 직선 최종진입+손가락 충돌면제)가
    #   접근 제약을 내장하므로 별도 접근 메트릭 불필요(plan_grasp는 pose_cost_metric=None을 요구).
    GRIPPER_COLL_LINKS = [l for l in robot_cfg["kinematics"].get("collision_link_names", [])
                          if "gripper" in l]   # 최종 진입 때 월드충돌 면제할 링크(손가락이 캔을 감싸야 함)
    usd_help = UsdHelper()
    usd_help.load_stage(my_world.stage)
    usd_help.add_world_to_stage(world_cfg, base_frame="/World")
    # v2에 defaultGroundPlane 포함 → 중복 생성 안 함
    set_gripper_friction(stage)   # 손가락 고마찰 바인딩 — ★play() 전에(후엔 physics view 무효화)
    set_finger_sdf_collision(stage)   # Stage6-B1: 손가락 SDF collision(오목 패드 접촉 복원) — play() 전
    my_world.play()
    set_scene_camera()        # PNG/뷰포트가 로봇·캔·매대를 크게 잡도록 카메라 배치

    # ── 루프 변수 ───────────────────────────────────────────────────────────
    ctrl         = None
    sim_js_names = None
    state        = GS.IDLE
    wait_cnt     = 0
    cycle        = 0
    attached     = False
    attach_off   = None
    cube_tgt_pos = None
    fail_cnt     = 0
    MAX_FAIL     = 5   # 무한 재시도 방지: 이 횟수 초과 시 진단 후 다음 사이클로
    regrasp_cnt  = 0
    MAX_REGRASP  = 3   # 물리 그립 실패 시 재파지 횟수 (마진 작아 ~67% → 재시도로 신뢰성↑)
    grasp_world  = None   # GraspGen 선택 파지 (4x4, 월드)
    pre_world    = None   # pre-grasp (4x4, 월드)
    grasp_cands  = []     # ★Phase1: plan_grasp용 side 파지후보 묶음(4x4 월드 리스트)
    grip_z_offset = 0.3 * (OBJ_SPECS["cylinder"]["height"] / 2.0)   # 캔중심이 TCP보다 아래인 양(파지 후 실측 갱신)

    # E0509 EE prim 경로 (panda_hand 대신 gripper_rh_p12_rn_base)
    ee_prim_path = f"{robot_prim_path}/{ee_link}"

    # 시작 시 stale stop-sentinel 제거
    if os.path.exists(STOP_FILE):
        os.remove(STOP_FILE)
    print(f"Stage 2 E0509 시작. {args.cycles}사이클. (종료: touch {STOP_FILE})", flush=True)

    # ★충돌구체 시각화: 링크 prim 자식으로 1회 부착 → USD 계층이 매 프레임 강체 추종(lag 0).
    #   (과거: 매 스텝 월드좌표 재계산 redraw라 렌더 프레임과 어긋나 뒤늦게 따라옴 → 고객영상 부적합)
    if args.viz_spheres:
        attach_cspheres_to_links(stage, robot_prim_path,
                                 f"{ROBOT_DIR}/e0509_spheres.yml", opacity=0.35)
    _viz_cb = None   # 부착형이라 동작 중 재그리기 불필요(자동 추종)

    while simulation_app.is_running():
        # ── graceful 종료 (kill 불필요): stop-sentinel 파일 감지 ──
        if os.path.exists(STOP_FILE):
            print(f"[종료] {STOP_FILE} 감지 → graceful 종료", flush=True)
            try:
                os.remove(STOP_FILE)
            except Exception:
                pass
            simulation_app.close()
            break

        my_world.step(render=True)
        step = my_world.current_time_step_index

        if ctrl is None:
            # E0509는 ArticulationView로 직접 제어
            from omni.isaac.core.articulations import Articulation
            try:
                robot_art = Articulation(prim_path=robot_prim_path)
                robot_art.initialize()
                ctrl = robot_art.get_articulation_controller()
                sim_js_names = robot_art.dof_names
                print(f"E0509 조인트: {sim_js_names}", flush=True)
                stabilize_arm_drives(robot_art, j_names, sim_js_names)  # 출렁임 억제 게인 보정
                enable_gravity_comp(robot_art, my_world, j_names)       # [Stage5] joint_2 중력처짐 보상
            except Exception as e:
                print(f"초기화 대기: {e}", flush=True)
                continue

        if step < 10:
            try:
                arm_idx = [robot_art.get_dof_index(x) for x in j_names if x in sim_js_names]
                _q_ret = np.array(default_cfg[:len(arm_idx)], dtype=np.float32)
                robot_art.set_joint_positions(_q_ret, arm_idx)   # home으로 텔레포트
                # ★드라이브 타겟도 home으로. 안 주면 PD가 기본값(≈0)으로 끌어 처짐/joint_2 넘어감.
                #   (게인 800/80은 댐핑 충분 → 1e5 때 같은 bounce 없음)
                ctrl.apply_action(ArticulationAction(_q_ret, joint_indices=arm_idx))
                # effort/게인은 stabilize_arm_drives에서 검증값(eff 200/50)으로 설정 → 여기서 덮지 않음
            except Exception:
                pass
            continue
        if step < 20:
            continue

        # 월드 동기화 ([Phase3] 현재 타겟만 ignore — 이웃/기적치 캔은 자동 장애물)
        if step == 50 or step % 1000 == 0:
            obs = usd_help.get_obstacles_from_stage(
                only_paths=["/World"],
                reference_prim_path=robot_prim_path,
                ignore_substring=[robot_prim_path, targets[cur_tgt_i]["path"],
                                   "/World/defaultGroundPlane", "/curobo",
                                   "/World/cspheres",   # ★시각화 구체는 장애물 아님(안 빼면 로봇이 자기 구체와 충돌→정지)
                                   ROBOT_BASE_BLOCK_PATH],   # 책상 충돌 복구(팔이 책상 회피)
            ).get_collision_check_world()
            motion_gen.update_world(obs)
            if step == 50:   # cuRobo가 든 장애물(책상/매대) 확인 — base 프레임 z 범위
                try:
                    _objs = obs.objects if hasattr(obs, "objects") else []
                    print(f"  [충돌월드] cuRobo 장애물 {len(_objs)}개:", flush=True)
                    for _o in _objs[:12]:
                        _p = getattr(_o, "pose", None); _d = getattr(_o, "dims", None)
                        print(f"     {_o.name}: pose={None if _p is None else [round(float(x),3) for x in _p[:3]]} dims={_d}", flush=True)
                except Exception as _e:
                    print(f"  [충돌월드] 로깅 실패: {_e}", flush=True)
            if _CLEARANCE["fn"] is None:   # [간격로깅] 월드 준비 후 1회 등록
                _CLEARANCE["fn"] = make_clearance_fn(robot_art, motion_gen, tensor_args)
                _c0 = _CLEARANCE["fn"]()
                print(f"  [간격로깅] 활성화 — 홈자세 최소간격 "
                      f"{'N/A' if _c0 is None else f'{_c0*1000:+.1f}mm'}", flush=True)

        # 현재 조인트 상태
        try:
            js_pos = robot_art.get_joint_positions()
            js_vel = robot_art.get_joint_velocities()
        except Exception:
            continue
        if js_pos is None:          # 물리뷰 미생성/HALT 등 → 이번 step 건너뜀(크래시 방지)
            continue

        # arm 조인트만 추출
        arm_joint_names = motion_gen.kinematics.joint_names
        arm_pos = []
        for jn in arm_joint_names:
            if jn in sim_js_names:
                idx = sim_js_names.index(jn)
                arm_pos.append(js_pos[idx])
            else:
                arm_pos.append(0.0)
        arm_pos = np.array(arm_pos, dtype=np.float32)
        # 시작상태를 관절 한계 안으로 클램프 (측정값 미세 초과 → INVALID_START_STATE 방지)
        if arm_pos.shape[0] == ARM_JOINT_LOWER.shape[0]:
            arm_pos = np.clip(arm_pos, ARM_JOINT_LOWER + 1e-3, ARM_JOINT_UPPER - 1e-3)

        cu_js = JointState(
            position=tensor_args.to_device(arm_pos),
            velocity=tensor_args.to_device(np.zeros_like(arm_pos)),
            acceleration=tensor_args.to_device(np.zeros_like(arm_pos)),
            jerk=tensor_args.to_device(np.zeros_like(arm_pos)),
            joint_names=list(arm_joint_names),
        )

        cube_pos, _ = target_cube.get_world_pose()

        # [구체VIZ] 링크부착형(시작 시 1회)이라 매 스텝 재그리기 불필요 — USD 계층이 자동 추종.

        # [프레임검증] cuRobo FK(측정관절)를 월드로 변환 vs 실제 EE 월드 — 좌표계 일치 1회 확인.
        #   차이≈0 → base offset/회전 정확(데드존은 실제). 차이 큼 → 프레임이 범인.
        if step == 45:
            try:
                _fk = ik_solver.fk(cu_js.position.view(1, -1))
                _ee_cur = _fk.ee_position.view(-1).cpu().numpy()
                _pred = _ee_cur + np.asarray(robot_base)
                _act = get_ee_world_pos(stage, ee_prim_path)
                print(f"[프레임검증] cuRobo FK EE(base)={_ee_cur.round(3)} → +base 월드예측={_pred.round(3)}", flush=True)
                if _act is not None:
                    print(f"[프레임검증] 실제 EE 월드={np.asarray(_act).round(3)} "
                          f"차이(실제-예측)={(np.asarray(_act)-_pred).round(3)}m", flush=True)
            except Exception as _e:
                print(f"[프레임검증] 실패: {_e}", flush=True)

        # ── 상태 머신 ─────────────────────────────────────────────────────────
        if state == GS.IDLE:
            if step > 60:
                # [Phase3 다물체] pending 타겟 순회. 실현 가능한 빈 슬롯(기하+IK+간격 사전검사)
                #   없으면 남은 타겟 일괄 no_slot 스킵 — 잡고 나서 갈 곳 없는 상황 차단.
                if args.objects > 1:
                    _half_s = OBJ_SPECS["cylinder"]["height"] / 2.0
                    _ez = SHELF3_LIP_TOP + SHELF3_ENTRY_CLR + _half_s + GRIP_Z_OFFSET_EST
                    _pz = SHELF3_FLOOR_TOP + SHELF3_REST_CLR + _half_s + GRIP_Z_OFFSET_EST
                    _uxy = [SHELF3_SLOTS[i] for i, u in enumerate(slot_used) if u]
                    _any_ok = any(
                        slot_feasible(SHELF3_SLOTS[i], _ez, _pz, ik_solver, tensor_args,
                                      cu_js.position, motion_gen, _uxy)[0]
                        for i, u in enumerate(slot_used) if not u)
                    if not _any_ok:
                        for _t in targets:
                            if _t["status"] == "pending":
                                _t["status"], _t["reason"] = "skipped", "no_slot"
                    _pend = [i for i, t in enumerate(targets) if t["status"] == "pending"]
                    _idle_done = not _pend
                else:
                    _idle_done = cycle >= args.cycles
                if _idle_done:
                    if args.objects > 1:
                        _pl = sum(1 for t in targets if t["status"] == "placed")
                        _sk = [(t["path"].split("/")[-1], t["reason"])
                               for t in targets if t["status"] == "skipped"]
                        print(f"\n✅ [다물체] 완료 — 적치 {_pl}/{len(targets)}"
                              f"{', 스킵 ' + str(_sk) if _sk else ''}. 장면 유지(HALT).", flush=True)
                    else:
                        print(f"\n✅ {args.cycles}사이클 종료. 장면 유지(HALT).", flush=True)
                    state = GS.HALT
                else:
                    if args.objects > 1:
                        # 매대(+y)에 가까운 캔부터(사용자 지시 2026-06-12): 운반 경로에 가장
                        # 걸리는 캔을 먼저 치워 후속 carry가 남은 캔 위를 안 지나게 함. 실측 y 기준.
                        def _tgt_y(_ti):
                            try:
                                return float(targets[_ti]["obj"].get_world_pose()[0][1])
                            except Exception:
                                return -1e9
                        cur_tgt_i  = max(_pend, key=_tgt_y)
                        target_cube = targets[cur_tgt_i]["obj"]   # 이후 코드는 별칭으로 현재 타겟 참조
                        tgt_retry  = 0
                        print(f"\n[다물체] 타겟 {cur_tgt_i+1}/{len(targets)} "
                              f"({targets[cur_tgt_i]['path']}, 매대측 우선) 시작", flush=True)
                    # 캔 안정화 대기 (재배치 후 떨어지며 흔들림/기울어짐 → 멈출 때까지).
                    #   안 기다리면 grasp 계산 위치와 실제가 어긋나 그리퍼가 캔 놓침(캔-EE≫TCP).
                    for _ in range(180):
                        my_world.step(render=True)
                        try:
                            if np.linalg.norm(target_cube.get_linear_velocity()) < 0.004:
                                break
                        except Exception:
                            break
                    cube_pos, _ = target_cube.get_world_pose()   # 안정 후 재측정
                    # ★넘어진 캔 감지: 직립이면 중심 z≈책상top+half. 그보다 낮으면(눕음 z≈top+r)
                    #   잡으려 들지 말고 직립 재배치(눕은 캔 파지는 전부 무의미한 실패).
                    _half_i = (OBJ_SPECS["cylinder"]["height"] / 2 if _obj_type == "cylinder"
                               else CUBE_SIZE / 2)
                    if cube_pos[2] < _table_top + _half_i - 0.012:
                        print(f"  [IDLE] 캔 넘어짐 감지(z={cube_pos[2]:.3f} < 직립 "
                              f"{_table_top+_half_i:.3f}) → 직립 재배치", flush=True)
                        target_cube.set_world_pose(
                            position=np.array([cube_pos[0], cube_pos[1], _table_top + _half_i + 0.002]),
                            orientation=np.array([1.0, 0.0, 0.0, 0.0]))
                        try:
                            target_cube.set_linear_velocity(np.zeros(3))
                            target_cube.set_angular_velocity(np.zeros(3))
                        except Exception:
                            pass
                        continue   # IDLE 유지 → 다음 iteration서 settle 후 재측정
                    cycle += 1
                    cube_tgt_pos = cube_pos.copy()
                    print(f"\n[사이클 {cycle}/{args.cycles}] 큐브(안정)={cube_tgt_pos.round(3)}", flush=True)
                    state    = GS.QUERY_GRASP if grasp_client is not None else GS.PLAN_PREGRASP
                    wait_cnt = 0
                    fail_cnt = 0   # ★사이클 시작에만 리셋(QUERY 재진입마다 리셋하면 무한 재선별)

        elif state == GS.QUERY_GRASP:
            # ★넘어짐 가드(사이클 중 재진입 포함): 접근/재파지 중 캔이 넘어가면 잡으려 들지 말고
            #   직립 재배치 후 사이클 재시작(눕은 캔 파지는 전부 무의미한 실패 + 무한루프 원인).
            _cz_now, _ = target_cube.get_world_pose()
            _half_q = (OBJ_SPECS["cylinder"]["height"] / 2 if _obj_type == "cylinder" else CUBE_SIZE / 2)
            if _cz_now[2] < _table_top + _half_q - 0.012:
                print(f"  [QUERY] 캔 넘어짐/이탈 감지(z={_cz_now[2]:.3f}) → 직립 재배치, 사이클 재시작", flush=True)
                target_cube.set_world_pose(
                    # 현재 타겟의 실측 x,y에 재배치(다물체: _cx,_cy는 중앙 캔 자리라 틀림)
                    position=np.array([_cz_now[0], _cz_now[1], _table_top + _half_q + 0.002]),
                    orientation=np.array([1.0, 0.0, 0.0, 0.0]))
                try:
                    target_cube.set_linear_velocity(np.zeros(3))
                    target_cube.set_angular_velocity(np.zeros(3))
                except Exception:
                    pass
                cycle -= 1
                state  = GS.IDLE
                continue
            # 접근 전 그리퍼 열기 (열린 손가락 사이로 캔이 들어와야 닫을 때 감쌈)
            set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=25)
            # ★Phase2: 타겟 캔을 "이번 사이클 한정" 장애물로 등록. ignore 상태로는 접근 경로가
            #   캔을 쓸고 지나가도 플래너가 모름(클러터로 좁아지면 실제로 캔을 쳐 넘어뜨림).
            #   그리퍼 구체는 손가락 표면뿐(사이 빔)이라 캔을 감싼 최종 파지 자세는 무충돌 → 목표 성립.
            #   파지(닫기) 후 enable_obstacle(False)로 해제(공식 attach_objects_to_robot 내부와 동일 메커니즘).
            try:
                _obs_t = usd_help.get_obstacles_from_stage(
                    only_paths=["/World"], reference_prim_path=robot_prim_path,
                    ignore_substring=[robot_prim_path, "/World/defaultGroundPlane", "/curobo",
                                      "/World/cspheres", "/World/grasp_viz", "/World/debug_pc",
                                      ROBOT_BASE_BLOCK_PATH],
                ).get_collision_check_world()
                motion_gen.update_world(_obs_t)
                print("  [충돌월드] 타겟 캔 포함 동기화(접근 스윕 방지)", flush=True)
            except Exception as _e:
                print(f"  [충돌월드] 타겟 포함 동기화 실패(기존 월드 유지): {_e}", flush=True)
            # ── GraspGen 추론 → 변환 → 선택 → 시각화 ──────────────────────────
            cube_quat = target_cube.get_world_pose()[1]      # [w,x,y,z]
            _w, _x, _y, _z = cube_quat
            obj_R = Rotation.from_quat([_x, _y, _z, _w]).as_matrix()

            print("  [GraspGen] 점구름 샘플링 & 추론...", flush=True)
            pc_obj   = sample_object_pc(obj_type)
            pc_world = (obj_R @ pc_obj.T).T + cube_tgt_pos
            # 점구름 시각화(초록)
            pc_prim.CreatePointsAttr().Set(
                _Vt.Vec3fArray([_Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pc_world]))
            pc_prim.CreateWidthsAttr().Set(_Vt.FloatArray([0.003] * len(pc_world)))
            pc_prim.CreateDisplayColorAttr().Set(
                _Vt.Vec3fArray([_Gf.Vec3f(0.2, 0.8, 0.2)] * len(pc_world)))

            t0 = time.time()
            grasps_obj, scores = grasp_client.infer(pc_obj, num_grasps=400)
            print(f"  [GraspGen] {len(grasps_obj)}개 파지 수신 ({time.time()-t0:.2f}s)", flush=True)
            if len(grasps_obj) == 0:
                if args.objects > 1:
                    tgt_retry += 1
                    if tgt_retry >= 2:   # [Phase3] 같은 타겟 2회 무파지 → 스킵하고 다음 타겟
                        targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "no_grasp"
                        print(f"  [다물체] {targets[cur_tgt_i]['path']} 스킵(no_grasp)", flush=True)
                        state = GS.IDLE
                    else:
                        print("  [GraspGen] 파지 없음 → 재시도", flush=True)
                        state = GS.IDLE
                else:
                    print("  [GraspGen] 파지 없음 → 재시도", flush=True)
                    state = GS.IDLE; cycle -= 1
            else:
                # robotiq → RH-P12 → 월드
                grasps_rhp12 = np.array([robotiq_grasp_to_rhp12(g) for g in grasps_obj])
                grasps_w = np.array([grasp_to_world(g, cube_tgt_pos, cube_quat) for g in grasps_rhp12])
                print(f"  [변환] robotiq→RH-P12 Z={ROBOTIQ_TO_RHP12_Z:.4f}m", flush=True)
                _obj_half_h = OBJ_SPECS[obj_type].get("height", 0.0) / 2.0   # 캔 반높이(side 게이트용)
                grasp_world, pre_world, grasp_cands = select_best_reachable_grasp(
                    grasps_w, scores, ik_solver, tensor_args, cu_js, robot_base, retract_t,
                    approach_z_max=APPROACH_Z_MAX, obj_R=obj_R, grasp_mode=grasp_mode,
                    obj_center=cube_tgt_pos, obj_half_h=_obj_half_h)
                if grasp_world is None and grasp_mode == "top":
                    print("  [IK 필터] 수직 파지 없음 → 사선 완화 재시도", flush=True)
                    grasp_world, pre_world, grasp_cands = select_best_reachable_grasp(
                        grasps_w, scores, ik_solver, tensor_args, cu_js, robot_base, retract_t,
                        approach_z_max=APPROACH_Z_MAX_RELAX, obj_R=obj_R, grasp_mode="top")
                # 파지 후보 시각화. ★side(캔)는 GraspGen 후보를 안 쓰고 azimuth 스윕으로 재합성하므로
                #   raw 후보(허공에 뜸)는 그리지 않고 실제 선택된 합성 파지만 표시(오해 방지).
                if grasp_mode == "side" and grasp_world is not None:
                    # ★전 후보 표시(approach 축). 실제 선택은 plan_grasp(goalset) 후 RGB축으로 갱신됨.
                    _cl = np.array(grasp_cands) if grasp_cands else np.array([grasp_world])
                    draw_grasp_candidates_usd(stage, _cl, np.ones(len(_cl)), selected_T=None)
                else:
                    _order = np.argsort(scores)[::-1]
                    draw_grasp_candidates_usd(stage, grasps_w[_order], scores[_order],
                                              selected_T=grasp_world)
                for _ in range(3):           # 시각화 렌더 반영
                    my_world.step(render=True)
                save_shot("grasps")          # 파지 후보+선택 PNG
                if grasp_world is None:
                    # 진단: 이 위치에서 E0509가 낼 수 있는 접근방향(top/side) 탐침
                    probe_reachable_approaches(cube_tgt_pos, ik_solver, tensor_args,
                                               cu_js, robot_base)
                    if args.objects > 1:   # [Phase3] feasible-first: 이 타겟 스킵, 다음으로
                        targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "unreachable"
                        print(f"  [다물체] {targets[cur_tgt_i]['path']} 스킵(unreachable)", flush=True)
                        state = GS.IDLE
                    else:
                        print("  [GraspGen] 도달 가능 파지 없음 → HALT(파지 후보는 화면 확인 가능)", flush=True)
                        state = GS.HALT
                else:
                    print(f"  [파지] 후보 {len(grasp_cands)}개 → plan_grasp(goalset)가 월드 기준 선택", flush=True)
                    # ★Phase2.5: side는 후보 "전체"를 plan_grasp에 넘겨 플래너가 상황적합 선택.
                    #   (룰베이스 가중치 선택 폐기 — 클러터/책상/캔을 아는 월드모델이 직접 고름)
                    state = GS.PLAN_GRASP if grasp_cands else GS.PLAN_PREGRASP

        elif state == GS.PLAN_PREGRASP:
            if grasp_client is not None:
                # GraspGen이 고른 pre-grasp (자세 포함)
                pre_pos  = pre_world[:3, 3]
                cpose    = mat4_to_curobo_pose(pre_world, tensor_args)
                _q = Rotation.from_matrix(pre_world[:3, :3]).as_quat()   # xyzw
                pre_quat = [float(_q[3]), float(_q[0]), float(_q[1]), float(_q[2])]
                pre_quat_dbg = "GraspGen"
            else:
                # 고정 탑다운(Step1): 큐브 위 18cm
                pre_pos  = np.array([cube_tgt_pos[0], cube_tgt_pos[1], cube_tgt_pos[2] + 0.18])
                cpose    = xyz_to_curobo_pose(pre_pos, [1,0,0,0], tensor_args)
                pre_quat = [1, 0, 0, 0]
                pre_quat_dbg = "[1,0,0,0]"
            print(f"  [1] Pre-grasp 플래닝 → {pre_pos.round(3)} ({pre_quat_dbg})", flush=True)
            result   = motion_gen.plan_single(cu_js.unsqueeze(0), cpose, plan_config)
            if result.success.item():
                cmd = motion_gen.get_full_js(result.get_interpolated_plan())
                execute_plan(cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                             track_tag="pre-grasp")
                print(f"  [2] Pre-grasp 도달", flush=True)
                state    = GS.PLAN_GRASP
                fail_cnt = 0
            else:
                fail_cnt += 1
                if fail_cnt == 1:
                    diag_pose_fail("pre", pre_pos, pre_quat, ik_solver, result, tensor_args)
                print(f"  [1] 플래닝 실패 ({fail_cnt}/{MAX_FAIL})", flush=True)
                if fail_cnt >= MAX_FAIL:
                    print("  [1] pre-grasp 도달 불가(이 위치) → 캔 재배치 후 다음 사이클", flush=True)
                    set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=20)
                    regrasp_cnt = 0
                    cycle -= 1
                    state    = GS.RELEASE
                    fail_cnt = 0

        elif state == GS.PLAN_GRASP:
            _grasp_reached = False
            _fail_status   = None
            if grasp_cands:
                # ★Phase2.5(공식 plan_grasp): 후보 묶음 → goalset이 월드(클러터·캔·책상) 기준 best 선택.
                #   2단계 내장: ①offset(standoff)까지 풀충돌인지 ②직선 최종진입(손가락 링크만 충돌면제,
                #   캔을 감싸야 하므로). 룰베이스 walk/메트릭 폐기.
                _N = len(grasp_cands)
                _pl = np.zeros((1, _N, 3), dtype=np.float32)
                _ql = np.zeros((1, _N, 4), dtype=np.float32)
                for _i, _c in enumerate(grasp_cands):
                    _pl[0, _i] = _c[:3, 3] - _ROBOT_BASE_OFFSET          # 월드→base
                    _q = Rotation.from_matrix(_c[:3, :3]).as_quat()      # xyzw
                    _ql[0, _i] = [_q[3], _q[0], _q[1], _q[2]]            # wxyz
                gposes = Pose(position=tensor_args.to_device(_pl),
                              quaternion=tensor_args.to_device(_ql))
                print(f"  [3] plan_grasp(goalset {_N}후보, 2단계 진입) ...", flush=True)
                gres = motion_gen.plan_grasp(
                    cu_js.unsqueeze(0), gposes, plan_config.clone(),
                    grasp_approach_offset=Pose.from_list([0, 0, -PREGRASP_STANDOFF, 1, 0, 0, 0]),
                    disable_collision_links=list(GRIPPER_COLL_LINKS),
                    plan_grasp_to_retract=False,   # 리프트는 닫은 뒤 plan_single로 별도 수행
                )
                if gres.success.item():
                    _gi = int(gres.goalset_index.item())
                    grasp_world = grasp_cands[_gi]
                    grasp_pos   = grasp_world[:3, 3]
                    print(f"  [3] goalset 선택 #{_gi+1}/{_N} → {grasp_pos.round(3)} "
                          f"approach={grasp_world[:3,2].round(2)}", flush=True)
                    # ★viz 갱신: 실제 로봇이 파지할 후보(goalset 선택)를 RGB 좌표축으로 표시
                    #   (이전엔 룰베이스 g_sel을 그려 실제 파지점과 어긋났음 — 사용자 지적 2026-06-11)
                    draw_grasp_candidates_usd(stage, np.array(grasp_cands),
                                              np.ones(len(grasp_cands)), selected_T=grasp_world)
                    for _ in range(2):
                        my_world.step(render=True)
                    # ★진단(2026-06-11): 접근 어느 구간이 캔을 치는지 분리 실행해 캔 이동 측정.
                    #   2단계(홈→pregrasp, 그리퍼 충돌 켜짐) vs 3단계(pregrasp→grasp 직선, 그리퍼 충돌 꺼짐).
                    _can_q0, _ = target_cube.get_world_pose()
                    cmd_a = motion_gen.get_full_js(gres.approach_result.get_interpolated_plan())
                    execute_plan(cmd_a, sim_js_names, robot_art, ctrl, my_world, extra_steps=1)
                    _can_q1, _ = target_cube.get_world_pose()
                    print(f"  [진단접근] 2단계(홈→pregrasp) 후 캔={np.round(_can_q1,3)} "
                          f"이동={np.round(_can_q1-_can_q0,3)} |Δ|={np.linalg.norm(_can_q1-_can_q0):.3f}", flush=True)
                    cmd_g = motion_gen.get_full_js(gres.grasp_result.get_interpolated_plan())
                    execute_plan(cmd_g, sim_js_names, robot_art, ctrl, my_world, extra_steps=1, track_tag="grasp")
                    _can_q2, _ = target_cube.get_world_pose()
                    print(f"  [진단접근] 3단계(pregrasp→grasp직선) 후 캔={np.round(_can_q2,3)} "
                          f"이동={np.round(_can_q2-_can_q1,3)} |Δ|={np.linalg.norm(_can_q2-_can_q1):.3f}", flush=True)
                    print(f"  [4] Grasp 위치 도달", flush=True)
                    log_arm_deg(robot_art, arm_joint_names, "grasp")
                    _grasp_reached = True
                else:
                    _fail_status = getattr(gres, "status", None)
            else:
                # top/box 경로(기존 plan_single 단일 목표)
                if grasp_client is not None:
                    grasp_pos = grasp_world[:3, 3]
                    cpose     = mat4_to_curobo_pose(grasp_world, tensor_args)
                else:
                    grasp_pos = np.array([cube_tgt_pos[0], cube_tgt_pos[1], cube_tgt_pos[2] + 0.13])
                    cpose     = xyz_to_curobo_pose(grasp_pos, [1, 0, 0, 0], tensor_args)
                print(f"  [3] Grasp 접근(plan_single, 충돌회피) → {grasp_pos.round(3)}", flush=True)
                result = motion_gen.plan_single(cu_js.unsqueeze(0), cpose, plan_config)
                if result.success.item():
                    cmd = motion_gen.get_full_js(result.get_interpolated_plan())
                    execute_plan(cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1, track_tag="grasp")
                    print(f"  [4] Grasp 위치 도달", flush=True)
                    _grasp_reached = True
                else:
                    _fail_status = getattr(result, "status", None)

            if _grasp_reached:
                _can_b, _ = target_cube.get_world_pose()
                _ee_b = get_ee_world_pos(stage, ee_prim_path)
                print(f"  [4-진단] 닫기前 캔={np.round(_can_b,3)} EE={None if _ee_b is None else np.round(_ee_b,3)} "
                      f"(캔-EE 수평거리={np.hypot(_can_b[0]-(_ee_b[0] if _ee_b is not None else 0), _can_b[1]-(_ee_b[1] if _ee_b is not None else 0)):.3f})", flush=True)
                save_shot("pre_close")   # 닫기 직전 그리퍼-캔
                set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_CLOSE, steps=70)
                _gjp = robot_art.get_joint_positions()
                _ga = float(_gjp[robot_art.get_dof_index("gripper_rh_r1")])
                # Stage6: 원위(r2/l2) 각도 — r2>r1이면 부족구동 curl(감싸쥠) 발생 증명
                _ga2 = float(_gjp[robot_art.get_dof_index("gripper_rh_r2")])
                _gl2 = float(_gjp[robot_art.get_dof_index("gripper_rh_l2")])
                for _ in range(45):      # 닫은 뒤 잠시 쥐고 안정
                    my_world.step(render=True)
                _can_a, _ = target_cube.get_world_pose()
                save_shot("post_close")
                print(f"  [4.5] 그리퍼 닫음 r1={_ga:.3f} r2={_ga2:.3f} l2={_gl2:.3f}rad"
                      f"{' (curl: 원위>근위 감싸쥠)' if min(_ga2,_gl2) > _ga + 0.05 else ''}. "
                      f"닫기後 캔={np.round(_can_a,3)} (이동={np.round(_can_a-_can_b,3)})", flush=True)
                for _ in range(3):
                    my_world.step(render=True)
                save_shot("grasp_reached")
                grip_z_offset = float(grasp_world[2, 3] - _can_a[2])
                print(f"  [4.6] grip z-offset={grip_z_offset:+.3f}m (캔중심이 TCP보다 이만큼 아래) "
                      f"→ 안착/진입 높이 보정에 반영", flush=True)
                # ★Phase2: 잡았으니 타겟 캔 장애물 해제 → lift/carry 시작상태가 충돌로 안 찍힘.
                #   (들고 있는 캔 부피는 attach 프록시가 담당. 다음 사이클 QUERY 동기화가 다시 등록)
                try:
                    _tkey = targets[cur_tgt_i]["path"].split("/")[-1]   # "obj_i" 또는 "target_cube"
                    _tnames = [o.name for o in motion_gen.world_model.objects
                               if _tkey in o.name]
                    for _tn in _tnames:
                        motion_gen.world_coll_checker.enable_obstacle(enable=False, name=_tn)
                    print(f"  [충돌월드] 타겟 캔 장애물 해제({len(_tnames)}개, key={_tkey})", flush=True)
                except Exception as _e:
                    print(f"  [충돌월드] 타겟 해제 실패: {_e}", flush=True)
                state    = GS.PLAN_LIFT
                wait_cnt = 0
                fail_cnt = 0
            else:
                # plan_grasp 전체 실패(goalset에 도달가능 후보 없음) 또는 top 경로 실패.
                #   직접IK 우회 안 함 → 재선별/재배치. (후보별 walk는 goalset이 내부에서 대체)
                fail_cnt += 1
                print(f"  [3] Grasp 접근 실패 ({fail_cnt}/{MAX_FAIL}) status={_fail_status} → 재선별/재배치", flush=True)
                if fail_cnt >= MAX_FAIL:
                    set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=20)
                    regrasp_cnt = 0
                    if args.objects > 1:   # [Phase3] 재배치 대신 스킵 → 다음 타겟
                        targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "unreachable"
                        print(f"  [다물체] {targets[cur_tgt_i]['path']} 스킵(unreachable, plan_grasp {MAX_FAIL}회 실패)", flush=True)
                        state = GS.IDLE
                    else:
                        print("  [3] 안전 접근 불가(이 위치) → 캔 재배치 후 다음 사이클", flush=True)
                        cycle -= 1                 # 이 사이클은 무효 처리(재배치만)
                        state    = GS.RELEASE
                    fail_cnt = 0
                else:
                    state = GS.QUERY_GRASP if grasp_client is not None else GS.PLAN_PREGRASP

        elif state == GS.PLAN_LIFT:
            # 물리 파지 리프트: kinematic attach 없이, 그리퍼 닫은 채 grasp 자세 유지하며 +z 상승.
            #   캔은 마찰로 딸려 올라와야 함(파지 성공 판정). 자세는 grasp 그대로(옆파지 유지).
            lift_world = grasp_world.copy()
            lift_world[2, 3] += 0.12    # 위로 올림(side 자세 유지)
            # ★리프트도 plan_single(충돌회피). grasp→+z 상승 동안 손목이 책상에 안 박히게 궤적 검사.
            #   그리퍼는 닫힌 채 유지(arm_only) → 캔 마찰로 딸려옴(cuRobo는 캔 미인지지만 위로만 가 안전).
            lift_cpose = mat4_to_curobo_pose(lift_world, tensor_args)
            res_lift   = motion_gen.plan_single(cu_js.unsqueeze(0), lift_cpose, plan_config)
            print(f"  [6] 리프트(plan_single, 회피) → z={lift_world[2,3]:.3f}m, "
                  f"{'OK' if res_lift.success.item() else '실패'}", flush=True)
            if res_lift.success.item():
                cmd = motion_gen.get_full_js(res_lift.get_interpolated_plan())
                execute_plan(cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                             track_tag="lift", arm_only=True)   # 그리퍼 닫힘 유지(캔 떨굼 방지)
                for _ in range(20):       # 정착
                    my_world.step(render=True)
                state = GS.MOVE_LIFT
            elif args.objects > 1:
                # [Phase3] 리프트 플랜 실패(이웃 근접 시작상태 등): 그리퍼 열고 같은 타겟
                #   재선별 1회(다른 파지 자세면 리프트 가능할 수 있음) → 재실패 시 스킵.
                set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=30)
                tgt_retry += 1
                if tgt_retry >= 2:
                    targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "lift_plan_fail"
                    print(f"  [다물체] {targets[cur_tgt_i]['path']} 스킵(lift_plan_fail)", flush=True)
                    _goal_h = JointState(position=retract_t.view(1, -1), joint_names=list(arm_joint_names))
                    _rh = motion_gen.plan_single_js(cu_js.unsqueeze(0), _goal_h, plan_config)
                    if _rh.success.item():
                        execute_plan(motion_gen.get_full_js(_rh.get_interpolated_plan()),
                                     sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                                     track_tag="reset_home")
                    state = GS.IDLE
                else:
                    print("  [6] 리프트 plan 실패 → 그리퍼 열고 같은 타겟 재선별", flush=True)
                    for _ in range(120):
                        my_world.step(render=True)
                        try:
                            if np.linalg.norm(target_cube.get_linear_velocity()) < 0.004:
                                break
                        except Exception:
                            break
                    cube_tgt_pos = target_cube.get_world_pose()[0].copy()
                    state = GS.QUERY_GRASP
            else:
                print("  [6] 리프트 plan_single 실패 → 잡은 채 정지(HOLD)", flush=True)
                state = GS.HOLD

        elif state == GS.MOVE_LIFT:
            cp, _ = target_cube.get_world_pose()
            lifted = cp[2] - cube_tgt_pos[2]
            success = lifted > 0.05   # 마찰로 5cm 이상 딸려 올라오면 물리 파지 성공
            print(f"  [7] 리프트 {'✅✅ 물리파지 성공' if success else '⚠️ 캔 안딸려옴(파지 실패/미끄러짐)'} "
                  f"물체z={cp[2]:.3f}(+{lifted:.3f}m)", flush=True)
            save_shot("lift")
            if success or regrasp_cnt >= MAX_REGRASP:
                if not success:
                    print(f"  [재파지] {MAX_REGRASP}회 초과 → 이번 사이클 포기", flush=True)
                regrasp_cnt = 0
                if args.place and success:
                    # Phase2: 잡은 캔을 robot에 attach → carry/home plan_single이 캔 부피까지
                    #   인지해 클러터·매대 회피. 공식 attach_external_objects_to_robot(simple_stacking).
                    #   캔은 ignore라 world_model에 없음 → 외부 Cuboid 프록시로 부착(현 grasp 설정 불변).
                    try:
                        _cp_w, _cq_w = target_cube.get_world_pose()          # 월드(pos, [w,x,y,z])
                        _cp_b = (np.asarray(_cp_w) - _ROBOT_BASE_OFFSET).astype(float)
                        _sc2 = OBJ_SPECS["cylinder"]
                        _held = Cuboid(
                            name="held_can",
                            pose=[float(_cp_b[0]), float(_cp_b[1]), float(_cp_b[2]),
                                  float(_cq_w[0]), float(_cq_w[1]), float(_cq_w[2]), float(_cq_w[3])],
                            dims=[2 * _sc2["radius"], 2 * _sc2["radius"], _sc2["height"]],
                        )
                        motion_gen.attach_external_objects_to_robot(
                            joint_state=cu_js,
                            external_objects=[_held],
                            surface_sphere_radius=0.005,
                            sphere_fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE,
                        )
                        attached = True
                        print("  [attach] 캔 부피 부착(carry/home 무충돌 인지)", flush=True)
                    except Exception as _e:
                        print(f"  [attach] 실패(무시하고 진행): {_e}", flush=True)
                    state = GS.PLAN_CARRY     # 매대 3단(맨 위) 배치 시연(충돌회피)
                elif args.objects > 1:
                    # [Phase3] 파지 미끄러짐 한도 초과 → 스킵, 그리퍼 열고 홈 복귀 후 다음 타겟
                    targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "grasp_slip"
                    print(f"  [다물체] {targets[cur_tgt_i]['path']} 스킵(grasp_slip)", flush=True)
                    set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=30)
                    _goal_h = JointState(position=retract_t.view(1, -1), joint_names=list(arm_joint_names))
                    _rh = motion_gen.plan_single_js(cu_js.unsqueeze(0), _goal_h, plan_config)
                    if _rh.success.item():
                        _cmd = motion_gen.get_full_js(_rh.get_interpolated_plan())
                        execute_plan(_cmd, sim_js_names, robot_art, ctrl, my_world,
                                     extra_steps=1, track_tag="reset_home")
                    state = GS.IDLE
                else:
                    state = GS.HOLD
                wait_cnt = 0
            else:
                # 그립 실패 → 그리퍼 열고 캔 재안정·재측정 후 다시 파지(물리 그립 마진 보완)
                regrasp_cnt += 1
                print(f"  [재파지] 그립 실패 → 재시도 {regrasp_cnt}/{MAX_REGRASP}", flush=True)
                set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=30)
                for _ in range(150):
                    my_world.step(render=True)
                    try:
                        if np.linalg.norm(target_cube.get_linear_velocity()) < 0.004:
                            break
                    except Exception:
                        break
                cp2, _ = target_cube.get_world_pose()
                cube_tgt_pos = cp2.copy()
                state    = GS.QUERY_GRASP if grasp_client is not None else GS.PLAN_PREGRASP
                wait_cnt = 0

        elif state == GS.HOLD:
            wait_cnt += 1                # 잡은 채 잠시 유지 (그리퍼 닫힘 타겟 persist)
            if wait_cnt > 90:
                state    = GS.RELEASE
                wait_cnt = 0

        elif state == GS.RELEASE:
            set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=40)
            print("  [8] 그리퍼 열기(해제)", flush=True)
            if args.objects > 1:
                # [Phase3] 다물체: 단일 모드의 캔 텔레포트 재배치 금지(남은 캔과 겹침 위험).
                #   홈 복귀만 하고 IDLE로 — 타겟 상태는 도달 경로(스킵/HOLD)에서 이미 기록됨.
                _goal_h = JointState(position=retract_t.view(1, -1), joint_names=list(arm_joint_names))
                _rh = motion_gen.plan_single_js(cu_js.unsqueeze(0), _goal_h, plan_config)
                if _rh.success.item():
                    execute_plan(motion_gen.get_full_js(_rh.get_interpolated_plan()),
                                 sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                                 track_tag="reset_home")
                print("  [다물체] RELEASE: 재배치 없이 홈 복귀 → 다음 타겟", flush=True)
                state    = GS.IDLE
                wait_cnt = 0
                continue   # 단일 모드 재배치(텔레포트) 코드로 안 내려가게 차단
            if cycle >= args.cycles:
                print(f"\n✅ {args.cycles}사이클 완료. 종료합니다.", flush=True)
                simulation_app.close()
                break
            # ★재배치 전 로봇 홈 복귀(사용자 요청): 캔을 새로 생성하기 전에 팔을 홈으로 빼
            #   → 새 캔 생성위치와 팔이 분리되고, 다음 파지를 깨끗한 홈에서 시작(접근 일관·캔 안 침).
            _goal_h = JointState(position=retract_t.view(1, -1), joint_names=list(arm_joint_names))
            _rh = motion_gen.plan_single_js(cu_js.unsqueeze(0), _goal_h, plan_config)
            if _rh.success.item():
                _cmd = motion_gen.get_full_js(_rh.get_interpolated_plan())
                execute_plan(_cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1, track_tag="reset_home")
                print("  [재배치] 로봇 홈 복귀 → 캔 재생성", flush=True)
            else:
                print("  [재배치] 홈 복귀 plan 실패(현 자세 유지) → 캔 재생성", flush=True)
            # 도달 가능 범위로 제한 (forward ≥ ~0.47m 유지: x는 -0.03~+0.08만 → 너무 가까워 HALT 방지)
            new_x = _cx + np.random.uniform(-0.03, 0.08)
            new_y = _cy + np.random.uniform(-0.10, 0.10)
            # 직립 방향 리셋 + 속도 0 (이전 사이클 기울기/속도가 남으면 캔이 기울어 안착=0.73 → grasp 실패)
            target_cube.set_world_pose(position=np.array([new_x, new_y, _cz]),
                                       orientation=np.array([1.0, 0.0, 0.0, 0.0]))
            try:
                target_cube.set_linear_velocity(np.zeros(3))
                target_cube.set_angular_velocity(np.zeros(3))
            except Exception:
                pass
            set_kinematic(stage, "/World/target_cube", False)
            print(f"  큐브 재배치(직립) → [{new_x:.2f}, {new_y:.2f}, {_cz:.3f}]", flush=True)
            state    = GS.IDLE
            wait_cnt = 0

        elif state == GS.PLAN_CARRY:
            # 매대 3단 앞 '진입 높이'까지 plan_single(매대/책상 회피). 진입높이=앞턱+여유+half+그립오프셋
            #   → 캔 바닥이 앞턱(1.15) 위로 떠서 매대에 안 닿음. PRE_Y는 앞턱(0.37)에서 충분히 앞.
            _half_c = OBJ_SPECS["cylinder"]["height"] / 2.0
            entry_z = SHELF3_LIP_TOP   + SHELF3_ENTRY_CLR + _half_c + grip_z_offset
            _pz_c   = SHELF3_FLOOR_TOP + SHELF3_REST_CLR  + _half_c + grip_z_offset
            # [Phase3] 첫 "실현 가능한" 빈 슬롯 선택(실측 grip_z_offset으로 본검사: 기하+IK+간격)
            _uxy_c = [SHELF3_SLOTS[i] for i, u in enumerate(slot_used) if u]
            cur_slot = -1
            for _si in range(len(SHELF3_SLOTS)):
                if slot_used[_si]:
                    continue
                _ok_s, _why_s = slot_feasible(SHELF3_SLOTS[_si], entry_z, _pz_c, ik_solver,
                                              tensor_args, cu_js.position, motion_gen, _uxy_c)
                if _ok_s:
                    cur_slot = _si
                    break
                print(f"  [슬롯] {_si+1} (x={SHELF3_SLOTS[_si][0]:.2f},y={SHELF3_SLOTS[_si][1]:.2f}) "
                      f"불가 — {_why_s}", flush=True)
            if cur_slot < 0:
                # 캔을 든 상태라 자동 복구 불가(되돌려 놓기 미구현) — 잡은 채 정지, 사용자 확인
                print("  [슬롯] 실현 가능한 슬롯 없음(파지 후 변동) → HOLD", flush=True)
                targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "no_slot_runtime"
                state = GS.HOLD
                continue
            place_x, place_y = SHELF3_SLOTS[cur_slot]
            print(f"  [슬롯] {cur_slot+1}/{len(SHELF3_SLOTS)} (x={place_x:.2f}, y={place_y:.2f}) 사용", flush=True)
            pre_center  = [place_x, SHELF3_PRE_Y, entry_z]
            carry_world = side_grasp_from_approach(SHELF3_APPROACH, pre_center, RHP12_TCP_DEPTH)
            cpose = mat4_to_curobo_pose(carry_world, tensor_args)
            print(f"  [P1] 매대 앞 운반(plan_single, 회피) → TCP {np.round(pre_center,3)} "
                  f"(캔바닥≈{SHELF3_LIP_TOP+SHELF3_ENTRY_CLR:.3f}, 앞턱 위)", flush=True)
            result = motion_gen.plan_single(cu_js.unsqueeze(0), cpose, plan_config)
            if result.success.item():
                cmd = motion_gen.get_full_js(result.get_interpolated_plan())
                execute_plan(cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                             track_tag="carry", arm_only=True, viz=_viz_cb)   # 그리퍼 닫힘 유지(캔 떨굼 방지)
                save_shot("carry_preshelf")
                print("  [P2] 매대 앞 도달", flush=True)
                log_arm_deg(robot_art, arm_joint_names, "carry/매대앞")
                state = GS.INSERT_SHELF
            else:
                print("  [P1] 운반 plan_single 실패 → 직접IK 폴백", flush=True)
                if move_direct_ik(carry_world, ik_solver, tensor_args, cu_js.position,
                                  arm_joint_names, robot_art, ctrl, my_world, viz=_viz_cb):
                    save_shot("carry_preshelf"); state = GS.INSERT_SHELF
                else:
                    print("  [P1] 운반 IK도 실패 → 정지(HALT)", flush=True); state = GS.HALT

        elif state == GS.INSERT_SHELF:
            # moveL: 매대 앞(PRE) → 안(IN), +y로만 직선 이동(진입높이 유지 → 캔이 앞턱 위로 통과).
            _half_c = OBJ_SPECS["cylinder"]["height"] / 2.0
            entry_z = SHELF3_LIP_TOP + SHELF3_ENTRY_CLR + _half_c + grip_z_offset
            ins_from  = side_grasp_from_approach(SHELF3_APPROACH, [place_x, SHELF3_PRE_Y, entry_z], RHP12_TCP_DEPTH)
            ins_world = side_grasp_from_approach(SHELF3_APPROACH, [place_x, place_y,      entry_z], RHP12_TCP_DEPTH)
            print(f"  [P3] 매대 안 +y 진입(moveL) → y {SHELF3_PRE_Y}→{place_y} @TCPz {entry_z:.3f}", flush=True)
            ok = move_linear_ik(ins_from, ins_world, ik_solver, tensor_args, cu_js.position,
                                arm_joint_names, robot_art, ctrl, my_world, viz=_viz_cb, tag="+y진입",
                                settle=0)   # [Stage5] 하강과 연속(중력보상 후 추종 0.2° → 정착 불요)
            save_shot("inserted")
            state = GS.LOWER_SHELF if ok else GS.HALT

        elif state == GS.LOWER_SHELF:
            # moveL: 매대 안(IN) → 안착(PLACE), -z로만 직선 하강. 안착높이=바닥+여유+half+그립오프셋
            #   → 캔 바닥이 바닥판(1.14) 바로 위에 놓임(박힘 없음). 앞턱 뒤(IN_Y)라 턱 간섭 없음.
            _half_c = OBJ_SPECS["cylinder"]["height"] / 2.0
            entry_z = SHELF3_LIP_TOP   + SHELF3_ENTRY_CLR + _half_c + grip_z_offset
            place_z = SHELF3_FLOOR_TOP + SHELF3_REST_CLR  + _half_c + grip_z_offset
            low_from  = side_grasp_from_approach(SHELF3_APPROACH, [place_x, place_y, entry_z], RHP12_TCP_DEPTH)
            low_world = side_grasp_from_approach(SHELF3_APPROACH, [place_x, place_y, place_z], RHP12_TCP_DEPTH)
            print(f"  [P4] 하강 안착(moveL -z) → @TCPz {place_z:.3f} (캔바닥≈{SHELF3_FLOOR_TOP+SHELF3_REST_CLR:.3f}, 바닥판 위)", flush=True)
            move_linear_ik(low_from, low_world, ik_solver, tensor_args, cu_js.position,
                           arm_joint_names, robot_art, ctrl, my_world, viz=_viz_cb, tag="-z하강",
                           settle=0)   # [Stage5] 그리퍼 램프 오픈(40스텝)이 사실상 정착 역할
            set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=40)   # 캔 안착
            for _ in range(120):       # 캔 안정 대기
                my_world.step(render=True)
                try:
                    if np.linalg.norm(target_cube.get_linear_velocity()) < 0.004:
                        break
                except Exception:
                    break
            _cp, _ = target_cube.get_world_pose()
            _upright = abs(_cp[2] - (SHELF3_FLOOR_TOP + _half_c)) < 0.03   # 캔중심≈바닥+half면 직립
            print(f"  [P5] 그리퍼 open. 캔 안착 월드={np.round(_cp,3)} "
                  f"({'직립✅' if _upright else '눕힘/기울어짐⚠'}, 직립기준 z≈{SHELF3_FLOOR_TOP+_half_c:.3f})", flush=True)
            log_arm_deg(robot_art, arm_joint_names, "place/안착")
            save_shot("placed")
            # [Phase3] 슬롯 점유 마크(직립 여부 무관 — 캔이 공간을 차지) + 타겟 상태 갱신
            slot_used[cur_slot] = True
            targets[cur_tgt_i]["status"] = "placed"
            if not _upright:
                targets[cur_tgt_i]["reason"] = "tilted"   # 적치는 됐으나 기울어짐(요약에 표시)
            if attached:   # Phase2: 캔이 매대에 놓였으니 분리 → 후퇴/home plan은 그리퍼만 인지
                try:
                    motion_gen.detach_object_from_robot()
                    attached = False
                    print("  [detach] 캔 분리(매대 안착 완료)", flush=True)
                except Exception as _e:
                    print(f"  [detach] 실패(무시): {_e}", flush=True)
            state = GS.RETREAT_SHELF

        elif state == GS.RETREAT_SHELF:
            # moveL: 안착 위치(PLACE) → 매대 앞(같은 높이), -y로만 직선 후퇴(그리퍼 오픈 상태).
            _half_c = OBJ_SPECS["cylinder"]["height"] / 2.0
            place_z = SHELF3_FLOOR_TOP + SHELF3_REST_CLR + _half_c + grip_z_offset
            ret_from  = side_grasp_from_approach(SHELF3_APPROACH, [place_x, place_y,      place_z], RHP12_TCP_DEPTH)
            ret_world = side_grasp_from_approach(SHELF3_APPROACH, [place_x, SHELF3_PRE_Y, place_z], RHP12_TCP_DEPTH)
            print(f"  [P6] 매대 밖 -y 후퇴(moveL) → y {place_y}→{SHELF3_PRE_Y} @TCPz {place_z:.3f}", flush=True)
            move_linear_ik(ret_from, ret_world, ik_solver, tensor_args, cu_js.position,
                           arm_joint_names, robot_art, ctrl, my_world, viz=_viz_cb, tag="-y후퇴",
                           settle=0)   # [Stage5] home 플랜과 연속
            state = GS.GO_HOME    # cu_js 갱신 위해 다음 iteration에서 home 복귀

        elif state == GS.GO_HOME:
            # ★충돌회피 시연: 매대 앞 → home(retract)까지 plan_single_js(관절공간 충돌회피).
            #   직접IK 후라 cu_js는 이번 iteration 상단에서 측정값으로 갱신됨 → start 정확.
            goal_js = JointState(position=retract_t.view(1, -1),
                                 joint_names=list(arm_joint_names))
            print("  [P7] home 복귀(plan_single_js, 충돌회피)", flush=True)
            res_home = motion_gen.plan_single_js(cu_js.unsqueeze(0), goal_js, plan_config)
            if res_home.success.item():
                cmd = motion_gen.get_full_js(res_home.get_interpolated_plan())
                execute_plan(cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                             track_tag="home", viz=_viz_cb)
                log_arm_deg(robot_art, arm_joint_names, "home/복귀")
                print("  [P8] ✅✅ 매대 3단(맨 위) 배치 + home 복귀 완료.", flush=True)
            else:
                print("  [P7] home 복귀 플래닝 실패 → 현 자세 유지(HALT).", flush=True)
            save_shot("place_done")
            # [Phase3] 다물체: 다음 pending 타겟으로 (IDLE이 잔여/슬롯 확인 후 HALT 결정)
            state = GS.IDLE if (args.objects > 1 and res_home.success.item()) else GS.HALT

        elif state == GS.HALT:
            # 진단 후 정지: 재플래닝 없이 장면만 유지 (사용자가 화면 확인)
            pass

    simulation_app.close()


if __name__ == "__main__":
    main()
