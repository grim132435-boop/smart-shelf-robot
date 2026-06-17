#!/usr/bin/env python3
# 과자봉지 FEM volume deformable 스파이크 — 실제 7cm 박스를 soft body로(누르면 찌그러짐, 안정적).
"""
Stage7 과자봉지 1단계(de-risk) — 방향전환: particle cloth inflatable(평평→부풀림)은 작은 스케일서
불안정(풍선 진동) → **FEM volume deformable**(soft body)로. 실측 7cm 부푼 박스(0.16×0.23×0.07)를
soft body로 만들어 presser로 눌러 찌그러짐(스낵봉지 느낌) 확인. Young's ~2MPa(포일/비닐, IsaacLab 데모 0.7~3.3MPa).

근거: omni.physx FrankaDeformableDemo / deformableUtils.add_physx_deformable_body + add_deformable_body_material.
★GPU dynamics 필수(PhysxSceneAPI EnableGPUDynamics). FEM은 닫힌 박스 자동 사면체화 → Blender 메시 불요.
실행: /tmp/run_defspike.sh --youngs 2e6  / 종료: touch /tmp/defspike_stop
"""
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--youngs", type=float, default=2.0e6, help="영률(Pa). 낮을수록 물렁. 봉지 0.7~3e6")
parser.add_argument("--poisson", type=float, default=0.45)
parser.add_argument("--hexres", type=int, default=10, help="시뮬 사면체 해상도")
args = parser.parse_args()

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": "1280", "height": "720"})

import os
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema
from omni.isaac.core import World
from omni.physx.scripts import deformableUtils, physicsUtils

STOP = "/tmp/defspike_stop"
SHOT_DIR = "/home/devuser/shelf_grasp_dev/logs/shots"
HX, HY, HZ = 0.08, 0.115, 0.035   # 봉지 half-size (0.16×0.23×0.07)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

# ── 물리 씬 + GPU dynamics (deformable 필수) ──
scene = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
if scene is None:
    scene = UsdPhysics.Scene.Define(stage, "/physicsScene").GetPrim()
_s = PhysxSchema.PhysxSceneAPI.Apply(scene)
_s.CreateEnableGPUDynamicsAttr().Set(True)
_s.CreateBroadphaseTypeAttr().Set("GPU")
print("[DEFSPIKE] GPU dynamics 활성화", flush=True)

# ── 봉지 박스 메시(닫힌 박스, 실제 7cm) — FEM이 자동 사면체화 ──
bag_path = "/World/snack_bag"
bag = UsdGeom.Mesh.Define(stage, bag_path)
V = [(-HX,-HY,-HZ),(HX,-HY,-HZ),(HX,HY,-HZ),(-HX,HY,-HZ),
     (-HX,-HY, HZ),(HX,-HY, HZ),(HX,HY, HZ),(-HX,HY, HZ)]
F = [4,5,6, 4,6,7,  0,2,1, 0,3,2,  0,1,5, 0,5,4,
     2,3,7, 2,7,6,  0,4,7, 0,7,3,  1,2,6, 1,6,5]
bag.CreatePointsAttr([Gf.Vec3f(*v) for v in V])
bag.CreateFaceVertexCountsAttr([3]*12)
bag.CreateFaceVertexIndicesAttr(F)
UsdGeom.Xformable(bag).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, HZ + 0.002))   # 바닥 위
bag.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.27, 0.18)])

deformableUtils.add_physx_deformable_body(
    stage, bag_path, simulation_hexahedral_resolution=args.hexres,
    collision_simplification=True, self_collision=False,
    solver_position_iteration_count=20,
)
mat_path = "/World/Physics_Materials/bag_mat"
deformableUtils.add_deformable_body_material(
    stage, mat_path, youngs_modulus=args.youngs, poissons_ratio=args.poisson,
    dynamic_friction=0.6, elasticity_damping=0.01, damping_scale=1.0, density=120.0,
)
physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(bag_path), mat_path)
print(f"[DEFSPIKE] 봉지 FEM soft body: youngs={args.youngs:.1e} poisson={args.poisson} hexres={args.hexres}", flush=True)

# ── 두 손가락(kinematic 강체): 봉지 중앙을 양옆서 집어(close) 들어올림(lift) → 변형 확인 ──
from omni.isaac.core.objects import DynamicCuboid
def make_finger(name, x):
    f = DynamicCuboid(prim_path=f"/World/{name}", name=name,
                      position=np.array([x, 0.0, 0.045]), scale=np.array([0.02, 0.12, 0.09]),
                      color=np.array([0.1, 0.4, 0.9]), mass=1.0)
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(f"/World/{name}")).CreateKinematicEnabledAttr().Set(True)
    return f
fingerL = make_finger("fingerL", -0.11)
fingerR = make_finger("fingerR", +0.11)
X_OPEN, X_GRIP = 0.11, 0.066   # 손가락 x: 열림 / 쥠(봉지 half 0.08 안쪽으로 눌러 그립)

def topz():
    try:
        st = stage.GetPrimAtPath(bag_path)
        pts = UsdGeom.Mesh(st).GetPointsAttr().Get()
        zs = [p[2] for p in pts]
        return (max(zs) - min(zs))
    except Exception:
        return -1

def shot(tag):
    try:
        os.makedirs(SHOT_DIR, exist_ok=True)
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        capture_viewport_to_file(get_active_viewport(), f"{SHOT_DIR}/defbag_{tag}.png")
        print(f"[DEFSPIKE] shot {tag}", flush=True)
    except Exception as e:
        print(f"[DEFSPIKE] shot 실패: {e}", flush=True)

try:
    from omni.isaac.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.4, -0.4, 0.3], target=[0.0, 0.0, 0.05])
except Exception:
    pass

def set_fingers(x, z):
    fingerL.set_world_pose(position=np.array([-x, 0.0, z]))
    fingerR.set_world_pose(position=np.array([ x, 0.0, z]))

world.reset()
set_fingers(X_OPEN, 0.045)
if os.path.exists(STOP):
    os.remove(STOP)
print("[DEFSPIKE] 시작 — 손가락 close→lift로 봉지 중앙 파지·변형 관찰. 종료: touch /tmp/defspike_stop", flush=True)

_step = 0
while simulation_app.is_running():
    world.step(render=True)
    _step += 1
    # 50~150: 손가락 닫기(중앙 그립) / 200~330: 들어올리기 / 이후 유지
    if 50 <= _step < 150:
        t = (_step - 50) / 100.0
        set_fingers(X_OPEN + (X_GRIP - X_OPEN) * t, 0.045)
    elif 200 <= _step < 330:
        t = (_step - 200) / 130.0
        set_fingers(X_GRIP, 0.045 + (0.22 - 0.045) * t)
    if _step in (40, 150, 270, 360):
        _tag = {40: "01_open", 150: "02_gripped", 270: "03_lifting", 360: "04_lifted"}[_step]
        try:
            _bp = stage.GetPrimAtPath(bag_path)
            _pts = UsdGeom.Mesh(_bp).GetPointsAttr().Get()
            _xr = max(p[0] for p in _pts) - min(p[0] for p in _pts)
            _bz = sum(p[2] for p in _pts)/len(_pts)   # 봉지 평균 z(들렸나)
            print(f"[DEFSPIKE] {_tag}: 봉지 x폭={_xr*1000:.0f}mm 평균z={_bz*1000:.0f}mm", flush=True)
        except Exception:
            pass
        shot(f"grip_{_tag}")
    if _step == 380:
        print("[DEFSPIKE] 관찰 완료(open/gripped/lifting/lifted). HALT.", flush=True)
    if os.path.exists(STOP):
        print("[DEFSPIKE] stop 감지 → 종료", flush=True)
        simulation_app.close(); break
