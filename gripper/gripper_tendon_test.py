#!/usr/bin/env python3
# RH-P12 언더액추에이티드 재현: spatial tendon(케이블)으로 오른쪽 핑거 base→근위→원위 결합, 위치의존 curl 검증.
"""오른쪽 핑거에 spatial tendon을 걸어, 케이블을 당기면 근위가 먼저 닫히고 막히면 원위가 curl 하는지 검증.

구조: Root(base) → Mid(근위 r1) → Leaf(원위 r2). restLength를 줄여 케이블을 당김(닫기).
복귀 스프링: r1/r2에 약한 position drive(타겟=open) → 케이블 풀면 열림. 텐던이 이김.
좌측(l1/l2)은 열림 고정(이번엔 오른쪽만 검증). 팔=home stiff, 중력 OFF, fix_base.
부착점/강성은 상단 상수로 튜닝(반복). 텐던 디버그 표시 ON → PNG에 케이블 노란선.

[인자] --rl_close <닫힘 restLength 비율, 기본0.5>  --ts <tendon stiffness>
[캡처] logs/shots/tendon_{open,mid,closed}.png
[종료] touch /tmp/tendon_test_stop
"""
import os
import argparse
import math

_ap = argparse.ArgumentParser()
_ap.add_argument("--rl_close", type=float, default=0.5)
_ap.add_argument("--ts", type=float, default=5.0e4)
ARGS, _ = _ap.parse_known_args()

# ── 부착점(링크 로컬, meter). 튜닝 대상. (오른쪽 핑거: base→r1→r2)
ATT_ROOT_BASE = (0.0, -0.028, 0.048)   # base: 안쪽(-y) 피벗높이 → 근위 +닫힘 모멘트(수계산 +0.012)
ATT_MID_R1 = (0.0, 0.038, 0.018)       # r1: 원위조인트 약간 아래
ATT_LEAF_R2 = (0.0, 0.0, 0.038)        # r2: 원위 중간

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import carb
import omni.usd
import omni.physx
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

try:
    from isaacsim.asset.importer.urdf import _urdf
except ImportError:
    from omni.importer.urdf import _urdf

URDF_PATH = "/home/devuser/curobo_ws/robots/e0509_gripper/e0509_gripper_abs.urdf"
STOP_FILE = "/tmp/tendon_test_stop"
SHOT_DIR = "/home/devuser/shelf_grasp_dev/logs/shots"
os.makedirs(SHOT_DIR, exist_ok=True)

world = World(stage_units_in_meters=1.0)

cfg = _urdf.ImportConfig()
cfg.merge_fixed_joints = False
cfg.convex_decomp = False
cfg.fix_base = True
cfg.make_default_prim = True
cfg.self_collision = False
cfg.create_physics_scene = True
cfg.import_inertia_tensor = False
cfg.distance_scale = 1.0
cfg.density = 0.0
try:
    cfg.parse_mimic = False
except Exception:
    pass
try:
    cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
except Exception:
    pass
for _a in ("make_instanceable", "create_instanceable", "instanceable"):
    if hasattr(cfg, _a):
        try:
            setattr(cfg, _a, False)
        except Exception:
            pass

root_dir = os.path.dirname(URDF_PATH)
fname = os.path.basename(URDF_PATH)
urdf_if = _urdf.acquire_urdf_interface()
parsed = urdf_if.parse_urdf(root_dir, fname, cfg)
robot_prim = urdf_if.import_robot(root_dir, fname, parsed, cfg, "")

_stage = omni.usd.get_context().get_stage()
for _pass in range(12):
    _ch = 0
    for _p in _stage.Traverse():
        if _p.IsInstanceable():
            _p.SetInstanceable(False)
            _ch += 1
    if _ch == 0:
        break

from pxr import UsdGeom, Usd, Gf, UsdPhysics, PhysxSchema

L_BASE = f"{robot_prim}/gripper_rh_p12_rn_base"
L_R1 = f"{robot_prim}/gripper_rh_p12_rn_r1"
L_R2 = f"{robot_prim}/gripper_rh_p12_rn_r2"

# ── spatial tendon: Root(base) → Mid(r1) → Leaf(r2)
TN = "rhFlex"
_base_prim = _stage.GetPrimAtPath(L_BASE)
_r1_prim = _stage.GetPrimAtPath(L_R1)
_r2_prim = _stage.GetPrimAtPath(L_R2)

_rootApi = PhysxSchema.PhysxTendonAttachmentRootAPI.Apply(_base_prim, TN)
PhysxSchema.PhysxTendonAttachmentAPI(_rootApi, TN).CreateLocalPosAttr().Set(Gf.Vec3f(*ATT_ROOT_BASE))
_rootApi.CreateStiffnessAttr().Set(ARGS.ts)
_rootApi.CreateDampingAttr().Set(ARGS.ts * 0.3)

_midApi = PhysxSchema.PhysxTendonAttachmentAPI.Apply(_r1_prim, TN)
_midApi.CreateParentLinkRel().AddTarget(L_BASE)
_midApi.CreateParentAttachmentAttr().Set(TN)
_midApi.CreateLocalPosAttr().Set(Gf.Vec3f(*ATT_MID_R1))

_leafApi = PhysxSchema.PhysxTendonAttachmentLeafAPI.Apply(_r2_prim, TN)
_leafT = PhysxSchema.PhysxTendonAttachmentAPI(_leafApi, TN)
_leafT.CreateParentLinkRel().AddTarget(L_R1)
_leafT.CreateParentAttachmentAttr().Set(TN)
_leafT.CreateLocalPosAttr().Set(Gf.Vec3f(*ATT_LEAF_R2))
print(f"[tendon] spatial tendon Root(base)->Mid(r1)->Leaf(r2) 적용, stiffness={ARGS.ts}", flush=True)

# 텐던 디버그 표시 ON (PNG에 케이블 보이게)
try:
    _reg = carb.settings.acquire_settings_interface()
    _reg.set_int(omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS, 2)
except Exception as e:
    print(f"[tendon] 디버그표시 실패(무시): {e}", flush=True)

robot = world.scene.add(Robot(prim_path=robot_prim, name="e0509"))
world.reset()
try:
    robot.disable_gravity()
except Exception:
    pass

nd = robot.num_dof
q_home = np.asarray(robot.get_joint_positions(), dtype=np.float32)
iR1 = robot.get_dof_index("gripper_rh_r1")
iR2 = robot.get_dof_index("gripper_rh_r2")
iL1 = robot.get_dof_index("gripper_rh_l1")
iL2 = robot.get_dof_index("gripper_rh_l2")
# 팔=stiff home, 좌핑거=open 고정 stiff, 우핑거=약한 복귀스프링(open 타겟)
kp = np.full(nd, 1.0e5, dtype=np.float32)
kd = np.full(nd, 1.0e4, dtype=np.float32)
kp[iR1] = 8.0;   kd[iR1] = 1.0     # 근위: 약한 복귀 → 먼저 굽음
kp[iR2] = 200.0; kd[iR2] = 15.0    # 원위: 강한 복귀 → 근위 막힐 때까지 평행 유지(막히면 텐던이 curl)
ctrl = robot.get_articulation_controller()
ctrl.set_gains(kps=kp, kds=kd)
targ = q_home.copy()
targ[iR1] = 0.0; targ[iR2] = 0.0; targ[iL1] = 0.0; targ[iL2] = 0.0
ctrl.apply_action(ArticulationAction(joint_positions=targ))

# ── 카메라: 오른쪽 핑거 클로즈업
_xc = UsdGeom.XformCache(Usd.TimeCode.Default())
def _world_att(linkpath, localpos):
    M = _xc.GetLocalToWorldTransform(_stage.GetPrimAtPath(linkpath))
    return M.Transform(Gf.Vec3f(*localpos))
def _open_len():
    a = _world_att(L_BASE, ATT_ROOT_BASE)
    b = _world_att(L_R1, ATT_MID_R1)
    c = _world_att(L_R2, ATT_LEAF_R2)
    return (b - a).GetLength() + (c - b).GetLength()

for _ in range(20):
    world.step(render=True)
_xc = UsdGeom.XformCache(Usd.TimeCode.Default())
OPEN_LEN = _open_len()
_fc = _world_att(L_R2, ATT_LEAF_R2)
print(f"[tendon] open 텐던길이={OPEN_LEN*1000:.1f}mm, 우원위팁 z={_fc[2]:.3f}", flush=True)

try:
    from omni.isaac.core.utils.viewports import set_camera_view
except Exception:
    from isaacsim.core.utils.viewports import set_camera_view
_bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
_gc = _bb.ComputeWorldBound(_stage.GetPrimAtPath(L_BASE)).ComputeAlignedRange().GetMidpoint()
_gt = [_gc[0], _gc[1], _gc[2] + 0.06]                       # 핑거가 위로 뻗으므로 약간 위를 타겟
set_camera_view(eye=[_gt[0] + 0.30, _gt[1] - 0.30, _gt[2] + 0.10], target=_gt)

from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
_vp = get_active_viewport()
def _shot(tag):
    p = f"{SHOT_DIR}/tendon_{tag}.png"
    try:
        capture_viewport_to_file(_vp, p)
        for _ in range(8):
            world.step(render=True)
        print(f"[tendon] 캡처 → {p}", flush=True)
    except Exception as e:
        print(f"[tendon] 캡처 실패: {e}", flush=True)

if os.path.exists(STOP_FILE):
    os.remove(STOP_FILE)

# restLength 초기=open(무력) → 램프로 줄여 당김(닫기)
_leafApi.CreateRestLengthAttr().Set(float(OPEN_LEN))
for _ in range(20):
    world.step(render=True)
_shot("open")

RAMP = 240
rl_target = OPEN_LEN * ARGS.rl_close
i = 0
while simulation_app.is_running():
    a = min(1.0, i / RAMP)
    rl = OPEN_LEN + a * (rl_target - OPEN_LEN)
    _leafApi.GetRestLengthAttr().Set(float(rl))
    world.step(render=True)
    if i == RAMP // 2:
        _shot("mid")
    if i == RAMP + 80:
        jp = robot.get_joint_positions()
        print(f"[tendon] 닫힘 우핑거 r1={float(jp[iR1]):.3f} r2={float(jp[iR2]):.3f} (restLen {OPEN_LEN*1000:.0f}→{rl_target*1000:.0f}mm)", flush=True)
        print("[tendon] r1 먼저 커지고 r2도 따라 커지면 언더액추에이션 OK", flush=True)
        _shot("closed")
    i += 1
    if os.path.exists(STOP_FILE):
        break

simulation_app.close()
