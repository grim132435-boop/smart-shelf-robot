# 모션팀용 — 비전 노드(grasp_candidates) → cuRobo plan_grasp(goalset) → 두산 궤적
#
# 실기 시스템의 "모션 측 책임"만 stage7_graspgen_e0509.py 에서 추출.
#   · GraspGen 추론 / 점구름 / 후보 자세 합성  →  비전 노드(webcam_seg_node)가 이미 담당.
#   · 이 코어는 비전이 발행한 후보(EE pose, base 프레임)를 받아 충돌없이 best 선택 + 궤적 생성.
#
# stage7 매핑(어느 함수가 여기로 오는가):
#   make_jlim_urdf()       ← stage7 동명 함수                 (L1017-1035)   ※리스크① 관절한계
#   build_motion_gen()     ← stage7 main robot_cfg 빌드        (L1328-1356)
#   plan_grasp_goalset()   ← stage7 main plan_grasp 블록        (L2174-2220)
#   plan_to_pose()         ← stage7 리프트/운반 plan_single     (L2337, L2516)
#   plan_to_joints()       ← stage7 home 복귀 plan_single_js     (L2604)
#
# ⚠ 모든 좌표는 robot BASE 프레임 · meter. 후보 quaternion 순서는 인터페이스 계약(wxyz/xyzw) 확인 필수.

from dataclasses import dataclass, field
from typing import Optional, List
import numpy as np

# ───────────────────────── 상수 (실기에서 재검증) ─────────────────────────
ROBOT_DIR        = "/home/devuser/curobo_ws/robots/e0509_gripper"   # 실기 경로로 교체
ROBOT_YML        = f"{ROBOT_DIR}/e0509_gripper.yml"
SRC_URDF         = f"{ROBOT_DIR}/e0509_gripper_abs.urdf"
SPHERES_YML      = f"{ROBOT_DIR}/e0509_spheres.yml"
JLIM_URDF_OUT    = "/tmp/e0509_gripper_jlim.urdf"

# 관절한계(팔 안 돌게) — stage7 _JLIM (L1335-1336). joint_4 ±180°, joint_5 0~135°.
JOINT_LIMITS     = {"joint_4": (-np.pi, np.pi), "joint_5": (0.0, 2.356194490192345)}
LOCK_JOINTS      = {"gripper_rh_r1": 0.0}        # 그리퍼 관절 잠금(플래닝은 팔만)
RETRACT_CONFIG   = [-0.610865, 0.785398, 1.396263, 1.134464, 2.007129, -0.698132]  # home 자세

PREGRASP_STANDOFF = 0.10        # plan_grasp 접근 오프셋(m) — stage7 L193
RHP12_TCP_DEPTH   = 0.110       # TCP(손가락 사이) 깊이 — ※리스크③ 오프셋 규약, 실측 보정 필요

# 클래스별 그리퍼 전류(mA) — ARCHITECTURE.md
GRIPPER_CURRENT_MA = {"snack_bag": 300, "bottle": 600, "can": 1000}


# ───────────────────────── 입출력 계약 ─────────────────────────
@dataclass
class GraspCandidates:
    """비전 노드 /dsr01/curobo/grasp_candidates(PoseArray)에서 변환.
    pos: (N,3) base·m, quat_wxyz: (N,4) — ROS xyzw로 받으면 노드에서 wxyz로 변환해 넣을 것."""
    pos: np.ndarray
    quat_wxyz: np.ndarray
    grasp_class: str = "can"          # /dsr01/curobo/grasp_class


@dataclass
class MotionInput:
    candidates: GraspCandidates
    joint_state: np.ndarray            # (6,) 현재 관절(rad) — 두산 /joint_states
    world_obstacles: object            # cuRobo WorldConfig (/dsr01/curobo/obstacles 파싱)


@dataclass
class MotionPlan:
    ok: bool
    grasp_idx: int = -1                # goalset가 고른 후보 인덱스
    grasp_pos: Optional[np.ndarray] = None
    approach_traj: object = None       # 두산에 보낼 관절궤적(홈→pregrasp)
    grasp_traj: object = None          # (pregrasp→grasp 직선)
    gripper_current_mA: int = 0
    reason: str = ""


@dataclass
class CoreDeps:
    motion_gen: object
    gripper_coll_links: list           # plan_grasp의 disable_collision_links
    tensor_args: object
    arm_joint_names: list


# ───────────── 1) 관절한계 URDF (리스크① — stage7 make_jlim_urdf 그대로) ─────────────
def make_jlim_urdf(src_urdf, dst_urdf, overrides):
    """원본 URDF를 안 건드리고 지정 관절 position 한계만 바꾼 복사본 생성.
    ★cuRobo BoundCost는 URDF를 init에서 clone → 런타임 텐서 수정은 무효. 반드시 URDF로 줘야 반영.
    overrides={joint:(lower,upper)}."""
    import xml.etree.ElementTree as ET
    tree = ET.parse(src_urdf); root = tree.getroot(); hit = []
    for j in root.findall("joint"):
        if j.get("name") in overrides:
            lim = j.find("limit"); lo, up = overrides[j.get("name")]
            lim.set("lower", repr(float(lo))); lim.set("upper", repr(float(up)))
            hit.append(j.get("name"))
    tree.write(dst_urdf, encoding="utf-8", xml_declaration=True)
    print(f"  [URDF한계] {dst_urdf} 생성 — 수정 관절 {hit}", flush=True)
    return dst_urdf


# ───────────── 2) MotionGen 빌드 (stage7 main L1328-1356) ─────────────
def build_motion_gen(tensor_args, world_obstacles=None):
    """관절한계 URDF + 충돌구체로 cuRobo MotionGen 빌드. 반환: (motion_gen, gripper_coll_links).
    ★이 빌드에서 관절한계를 안 주면 후보가 멀쩡해도 실기 팔이 플립함(리스크①)."""
    from curobo.util_file import load_yaml
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
    from curobo.geom.types import WorldConfig

    robot_cfg = load_yaml(ROBOT_YML)["robot_cfg"]
    robot_cfg["kinematics"]["cspace"]["retract_config"] = list(RETRACT_CONFIG)
    jlim_urdf = make_jlim_urdf(SRC_URDF, JLIM_URDF_OUT, JOINT_LIMITS)     # ← 리스크①
    robot_cfg["kinematics"]["urdf_path"]         = jlim_urdf
    robot_cfg["kinematics"]["asset_root_path"]   = ROBOT_DIR
    robot_cfg["kinematics"]["collision_spheres"] = SPHERES_YML
    robot_cfg["kinematics"]["lock_joints"]       = dict(LOCK_JOINTS)

    world = world_obstacles if world_obstacles is not None else WorldConfig()
    mg_cfg = MotionGenConfig.load_from_robot_config(robot_cfg, world, tensor_args)
    motion_gen = MotionGen(mg_cfg)
    motion_gen.warmup()

    # plan_grasp에서 손가락 링크만 충돌면제(캔을 감싸야 하므로) — stage7 L1648
    coll = robot_cfg["kinematics"].get("collision_link_names", [])
    gripper_coll_links = [l for l in coll if ("gripper" in l or "rh_p12" in l or "rh_r" in l)]
    return motion_gen, gripper_coll_links


# ───────────── 3) plan_grasp goalset 선택 (stage7 main L2174-2220) ─────────────
def plan_grasp_goalset(cands: GraspCandidates, joint_state: np.ndarray, deps: CoreDeps):
    """후보 N개를 묶어 plan_grasp에 넘기면, 플래너가 현재 월드(매대·이웃물체) 기준
    충돌없이 도달 가능한 best 후보를 직접 고른다(goalset_index). 2단계 진입 내장:
      ① offset(standoff)까지 풀충돌 검사 ② 직선 최종진입(손가락 링크만 충돌면제).
    반환: (idx, grasp_pos, approach_traj, grasp_traj) or (-1, None, None, None)."""
    from curobo.types.math import Pose
    ta = deps.tensor_args
    N = len(cands.pos)
    pl = cands.pos.reshape(1, N, 3).astype(np.float32)        # 이미 base 프레임(실기 offset=0)
    ql = cands.quat_wxyz.reshape(1, N, 4).astype(np.float32)
    gposes = Pose(position=ta.to_device(pl), quaternion=ta.to_device(ql))

    cu_js = _js_to_curobo(joint_state, deps)
    plan_config = _make_plan_config()
    gres = deps.motion_gen.plan_grasp(
        cu_js.unsqueeze(0), gposes, plan_config,
        grasp_approach_offset=Pose.from_list([0, 0, -PREGRASP_STANDOFF, 1, 0, 0, 0]),
        disable_collision_links=list(deps.gripper_coll_links),
        plan_grasp_to_retract=False,            # 리프트는 닫은 뒤 plan_single로 별도
    )
    if not gres.success.item():
        return -1, None, None, None
    gi = int(gres.goalset_index.item())
    appr  = deps.motion_gen.get_full_js(gres.approach_result.get_interpolated_plan())
    grasp = deps.motion_gen.get_full_js(gres.grasp_result.get_interpolated_plan())
    return gi, cands.pos[gi], appr, grasp


# ───────────── 4) 리프트/운반/홈 (plan_single / plan_single_js) ─────────────
def plan_to_pose(pos_base, quat_wxyz, joint_state, deps: CoreDeps):
    """충돌회피 단일 목표 궤적(리프트·운반). stage7 L2337/L2516 plan_single."""
    from curobo.types.math import Pose
    ta = deps.tensor_args
    cpose = Pose(position=ta.to_device(np.asarray(pos_base, np.float32).reshape(1, 3)),
                 quaternion=ta.to_device(np.asarray(quat_wxyz, np.float32).reshape(1, 4)))
    res = deps.motion_gen.plan_single(_js_to_curobo(joint_state, deps).unsqueeze(0),
                                      cpose, _make_plan_config())
    if not res.success.item():
        return None
    return deps.motion_gen.get_full_js(res.get_interpolated_plan())


def plan_to_joints(q_goal, joint_state, deps: CoreDeps):
    """관절공간 충돌회피(홈 복귀). stage7 L2604 plan_single_js."""
    ta = deps.tensor_args
    goal = _js_to_curobo(q_goal, deps)
    res = deps.motion_gen.plan_single_js(_js_to_curobo(joint_state, deps).unsqueeze(0),
                                         goal, _make_plan_config())
    if not res.success.item():
        return None
    return deps.motion_gen.get_full_js(res.get_interpolated_plan())


# ───────────── 오케스트레이션 (파지 1회) ─────────────
def plan_pick_motion(inp: MotionInput, deps: CoreDeps) -> MotionPlan:
    deps.motion_gen.update_world(inp.world_obstacles)        # 충돌월드 동기화(stage7 L2041)
    gi, pos, appr, grasp = plan_grasp_goalset(inp.candidates, inp.joint_state, deps)
    if gi < 0:
        return MotionPlan(ok=False, reason="plan_grasp_fail")
    return MotionPlan(
        ok=True, grasp_idx=gi, grasp_pos=pos, approach_traj=appr, grasp_traj=grasp,
        gripper_current_mA=GRIPPER_CURRENT_MA.get(inp.candidates.grasp_class, 600),
    )


# ───────────── 헬퍼 ─────────────
def _js_to_curobo(joint_state, deps: CoreDeps):
    from curobo.types.state import JointState
    ta = deps.tensor_args
    return JointState.from_position(
        ta.to_device(np.asarray(joint_state, np.float32).reshape(1, -1)),
        joint_names=deps.arm_joint_names)


def _make_plan_config():
    from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
    # stage7과 동일 옵션으로 맞출 것(time_dilation 등은 실기 속도정책에 따라).
    return MotionGenPlanConfig(enable_graph=False, max_attempts=4)
