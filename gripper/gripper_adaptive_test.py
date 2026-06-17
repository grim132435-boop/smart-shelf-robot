#!/usr/bin/env python3
# RH-P12 적응형 그립 컨트롤러 재현: 근위 닫다가 막히면 원위 curl(위치의존). 양쪽 핑거 동시.
"""컨트롤러로 언더액추에이션을 재현(물리 텐던 대신 결정론적 제어).

단계:
  1) approach: 근위(r1,l1)를 닫음, 원위(r2,l2)는 평행(0) 유지.
     - 근위 실제각이 지령보다 크게 뒤처지면(=물체에 막힘) curl 단계로.
     - 막힘 없이 근위가 끝까지 닫히면(=끝파지/물체없음) 평행 유지로 종료.
  2) curl: 근위 정지, 원위(r2,l2)를 닫아 물체를 감쌈.
결과: 안쪽 파지 → 근위 막힘 → curl 감쌈 / 끝 파지 → 근위 닫힘 → 평행.
양쪽 핑거가 함께 움직임. 캔=정적 collider, 중력 OFF.

[인자] --depth tip|deep  [캡처] logs/shots/adapt_<depth>_{open,close}.png  [종료] touch /tmp/adapt_stop
"""
import os
import argparse

_ap = argparse.ArgumentParser()
_ap.add_argument("--depth", choices=["tip", "deep", "none"], default="deep")
_ap.add_argument("--zoff", type=float, default=-1.0, help="캔 높이(base +z, m) 직접 지정. >0이면 depth 무시")
_ap.add_argument("--zworld", type=float, default=-1.0, help="캔 월드 z(m) 직접 지정. >0이면 최우선")
ARGS, _ = _ap.parse_known_args()

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
try:
    from isaacsim.asset.importer.urdf import _urdf
except ImportError:
    from omni.importer.urdf import _urdf

URDF_PATH = "/home/devuser/curobo_ws/robots/e0509_gripper/e0509_gripper_abs.urdf"
STOP_FILE = "/tmp/adapt_stop"
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

root_dir = os.path.dirname(URDF_PATH); fname = os.path.basename(URDF_PATH)
urdf_if = _urdf.acquire_urdf_interface()
parsed = urdf_if.parse_urdf(root_dir, fname, cfg)
robot_prim = urdf_if.import_robot(root_dir, fname, parsed, cfg, "")

_stage = omni.usd.get_context().get_stage()
for _pass in range(12):
    _ch = 0
    for _p in _stage.Traverse():
        if _p.IsInstanceable():
            _p.SetInstanceable(False); _ch += 1
    if _ch == 0:
        break

from pxr import UsdGeom, Usd, Gf, UsdPhysics, PhysxSchema, UsdLux
# 조명(없으면 캡처 PNG가 검정) — DomeLight 앰비언트 + DistantLight 키라이트
_dome = UsdLux.DomeLight.Define(_stage, "/World/domeLight")
_dome.CreateIntensityAttr(800.0)
_key = UsdLux.DistantLight.Define(_stage, "/World/keyLight")
_key.CreateIntensityAttr(2500.0)
UsdGeom.Xformable(_key).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))

# 접촉 안정화: 반발(restitution) 0 + 높은 마찰 물리재질 → 들이받아도 안 튕기고 감싸쥠
from pxr import UsdShade
_pmat = UsdShade.Material.Define(_stage, "/World/physMat")
_pmAPI = UsdPhysics.MaterialAPI.Apply(_pmat.GetPrim())
_pmAPI.CreateStaticFrictionAttr(1.3)
_pmAPI.CreateDynamicFrictionAttr(1.3)
_pmAPI.CreateRestitutionAttr(0.0)
def _bind_mat(prim):
    try:
        UsdShade.MaterialBindingAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(_pmat, bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
    except Exception:
        pass

# 핑거 SDF
_fingers = ("gripper_rh_p12_rn_r1/", "gripper_rh_p12_rn_r2/", "gripper_rh_p12_rn_l1/", "gripper_rh_p12_rn_l2/")
for _p in _stage.Traverse():
    _s = str(_p.GetPath())
    if "/collisions" in _s and any(f in _s for f in _fingers) and _p.HasAPI(UsdPhysics.MeshCollisionAPI):
        UsdPhysics.MeshCollisionAPI(_p).CreateApproximationAttr().Set("sdf")
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(_p).CreateSdfResolutionAttr().Set(256)
        _bind_mat(_p)

robot = world.scene.add(Robot(prim_path=robot_prim, name="e0509"))
world.reset()
try:
    robot.disable_gravity()
except Exception:
    pass

# 캔 배치(그리퍼 base 기준)
_xc = UsdGeom.XformCache(Usd.TimeCode.Default())
_M = _xc.GetLocalToWorldTransform(_stage.GetPrimAtPath(f"{robot_prim}/gripper_rh_p12_rn_base"))
_pos = _M.ExtractTranslation(); _rot = _M.ExtractRotationMatrix()
_zax = Gf.Vec3d(_rot[2][0], _rot[2][1], _rot[2][2]).GetNormalized()
if ARGS.depth != "none":
    _depth = ARGS.zoff if ARGS.zoff > 0 else (0.110 if ARGS.depth == "tip" else 0.085)
    if ARGS.zworld > 0:
        _can_c = Gf.Vec3d(_pos[0], _pos[1], ARGS.zworld)   # 월드 z 직접 지정(x,y는 그리퍼 중심)
    else:
        _can_c = _pos + _zax * _depth
    _cyl = UsdGeom.Cylinder.Define(_stage, "/World/test_can")
    _cyl.GetRadiusAttr().Set(0.025); _cyl.GetHeightAttr().Set(0.115); _cyl.GetAxisAttr().Set("X")
    _cyl.GetDisplayColorAttr().Set([Gf.Vec3f(0.85, 0.1, 0.1)])
    _Mc = Gf.Matrix4d(_M); _Mc.SetTranslateOnly(_can_c)
    _xf = UsdGeom.Xformable(_cyl); _xf.ClearXformOpOrder(); _xf.AddTransformOp().Set(_Mc)
    UsdPhysics.CollisionAPI.Apply(_cyl.GetPrim())
    _rb = UsdPhysics.RigidBodyAPI.Apply(_cyl.GetPrim())   # kinematic → Play 중 드래그로 위치 조정 가능
    _rb.CreateKinematicEnabledAttr(True)
    _bind_mat(_cyl.GetPrim())
    _cam_c = _can_c
else:
    _cam_c = _pos + _zax * 0.10
print(f"[adapt] depth={ARGS.depth}", flush=True)

nd = robot.num_dof
q_home = np.asarray(robot.get_joint_positions(), dtype=np.float32)
iR1 = robot.get_dof_index("gripper_rh_r1"); iR2 = robot.get_dof_index("gripper_rh_r2")
iL1 = robot.get_dof_index("gripper_rh_l1"); iL2 = robot.get_dof_index("gripper_rh_l2")
PROX = [iR1, iL1]; DIST = [iR2, iL2]
kp = np.full(nd, 1.0e5, dtype=np.float32); kd = np.full(nd, 1.0e4, dtype=np.float32)
for i in PROX + DIST:
    kp[i] = 8.0e2; kd[i] = 8.0e1     # 부드러운 드라이브: 캔에 닿으면 안 튕기고 그 자리서 멈춤
ctrl = robot.get_articulation_controller()
ctrl.set_gains(kps=kp, kds=kd)

# 카메라
try:
    from omni.isaac.core.utils.viewports import set_camera_view
except Exception:
    from isaacsim.core.utils.viewports import set_camera_view
_bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
_bc = _bbc.ComputeWorldBound(_stage.GetPrimAtPath(f"{robot_prim}/gripper_rh_p12_rn_base")).ComputeAlignedRange().GetMidpoint()
set_camera_view(eye=[_bc[0] + 0.02, _bc[1] - 0.40, _bc[2] + 0.10],   # 정면(캔 원형면) 뷰
                target=[_bc[0], _bc[1], _bc[2] + 0.06])
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
_vp = get_active_viewport()
def _shot(tag):
    p = f"{SHOT_DIR}/adapt_{ARGS.depth}_{tag}.png"
    try:
        capture_viewport_to_file(_vp, p)
        for _ in range(8):
            world.step(render=True)
        print(f"[adapt] 캡처 → {p}", flush=True)
    except Exception as e:
        print(f"[adapt] 캡처 실패: {e}", flush=True)

if os.path.exists(STOP_FILE):
    os.remove(STOP_FILE)
for _ in range(30):
    world.step(render=True)
_shot("open")

# ── 컨트롤러(사이클, 녹화용 고정 자세): 근위 0.52rad(30°)로 캔 물기, 원위 펴짐. 열기↔닫기 반복.
CLOSE = 0.52          # 근위 닫힘각(사용자 확정 자세, 30°)
g = 0.0
opening = False
holdc = 0; openc = 0; cyc = 1; shot_done = False
print(f"[adapt] === cycle 1 닫기 ({ARGS.depth}) ===", flush=True)
i = 0
while simulation_app.is_running():
    if opening:
        g = max(0.0, g - 0.012)
        if g <= 0.0:
            openc += 1
            if openc > 60:
                opening = False; openc = 0; cyc += 1; shot_done = False
                print(f"[adapt] === cycle {cyc} 닫기 ===", flush=True)
    else:
        g = min(CLOSE, g + 0.003)        # 천천히 닫아 부드럽게 접촉(바운스 방지)
        if g >= CLOSE:
            holdc += 1
            if holdc == 50 and not shot_done:
                jp = robot.get_joint_positions()
                print(f"[adapt] 닫힘 근위=[{float(jp[iR1]):.2f},{float(jp[iL1]):.2f}] 원위=[{float(jp[iR2]):.2f},{float(jp[iL2]):.2f}] (캔이 막는 곳까지)", flush=True)
                _shot("close"); shot_done = True
            if holdc > 140:
                opening = True; holdc = 0
    targ = q_home.copy()
    targ[iR1] = g; targ[iL1] = g                           # 근위: 0→0.52(30°) 캔 물기
    targ[iR2] = 0.0; targ[iL2] = 0.0                       # 원위: 펴짐 유지
    ctrl.apply_action(ArticulationAction(joint_positions=targ))
    world.step(render=True)
    i += 1
    if os.path.exists(STOP_FILE):
        break

simulation_app.close()
