#!/usr/bin/env python3
# 그리퍼+캔 인터랙티브 도구: 사용자가 캔을 드래그하고 관절(r1/l1/r2/l2)을 직접 jog해 grip 자세를 찾는다.
"""캔을 끌어다 놓고 그리퍼 관절을 직접 움직여 원하는 grip을 잡는 도구.

- 그리퍼: URDF 임포트, 4관절 독립(parse_mimic=False) → 각 관절 따로 jog 가능.
- 캔: kinematic 실린더(지름 5cm) → 선택 후 W키로 끌어서 위치 조정(Play 중에도 됨).
- 조명: DomeLight+DistantLight(캡처/뷰 검정 방지).
- 관절 jog: Window ▸ Physics ▸ Physics Inspector → Articulation=/e0509_with_gripper → 슬라이더.
- 찾은 값 알려주기: 각 관절각은 Physics Inspector에, 캔 위치는 캔 prim의 Transform(Translate)에 표시됨.
[종료] 창 닫기 또는 touch /tmp/can_jog_stop
"""
import os
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.kit.app
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

# Physics Inspector(관절 슬라이더) 확장 ON
try:
    _ext = omni.kit.app.get_app().get_extension_manager()
    for _e in ("omni.physx.supportui", "omni.physx.ui"):
        _ext.set_extension_enabled_immediate(_e, True)
    print("[canjog] Physics Inspector 확장 ON", flush=True)
except Exception as e:
    print(f"[canjog] 확장 ON 실패(무시): {e}", flush=True)

try:
    from isaacsim.asset.importer.urdf import _urdf
except ImportError:
    from omni.importer.urdf import _urdf

URDF_PATH = "/home/devuser/curobo_ws/robots/e0509_gripper/e0509_gripper_abs.urdf"
STOP_FILE = "/tmp/can_jog_stop"

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

from pxr import UsdGeom, Usd, Gf, UsdPhysics, PhysxSchema, UsdLux, UsdShade
# 조명
_dome = UsdLux.DomeLight.Define(_stage, "/World/domeLight"); _dome.CreateIntensityAttr(800.0)
_key = UsdLux.DistantLight.Define(_stage, "/World/keyLight"); _key.CreateIntensityAttr(2500.0)
UsdGeom.Xformable(_key).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))

# 핑거 SDF + 마찰/반발0 재질
_pmat = UsdShade.Material.Define(_stage, "/World/physMat")
_pm = UsdPhysics.MaterialAPI.Apply(_pmat.GetPrim())
_pm.CreateStaticFrictionAttr(1.3); _pm.CreateDynamicFrictionAttr(1.3); _pm.CreateRestitutionAttr(0.0)
def _bind(prim):
    try:
        UsdShade.MaterialBindingAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(_pmat, bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
    except Exception:
        pass
_fingers = ("gripper_rh_p12_rn_r1/", "gripper_rh_p12_rn_r2/", "gripper_rh_p12_rn_l1/", "gripper_rh_p12_rn_l2/")
for _p in _stage.Traverse():
    _s = str(_p.GetPath())
    if "/collisions" in _s and any(f in _s for f in _fingers) and _p.HasAPI(UsdPhysics.MeshCollisionAPI):
        UsdPhysics.MeshCollisionAPI(_p).CreateApproximationAttr().Set("sdf")
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(_p).CreateSdfResolutionAttr().Set(256)
        _bind(_p)

robot = world.scene.add(Robot(prim_path=robot_prim, name="e0509"))
world.reset()
try:
    robot.disable_gravity()
except Exception:
    pass

# 캔: kinematic 실린더(지름 5cm). 그리퍼 base 위쪽 z=1.22에 둠 — 선택 후 W로 드래그.
_xc = UsdGeom.XformCache(Usd.TimeCode.Default())
_M = _xc.GetLocalToWorldTransform(_stage.GetPrimAtPath(f"{robot_prim}/gripper_rh_p12_rn_base"))
_pos = _M.ExtractTranslation()
_cyl = UsdGeom.Cylinder.Define(_stage, "/World/test_can")
_cyl.GetRadiusAttr().Set(0.025); _cyl.GetHeightAttr().Set(0.115); _cyl.GetAxisAttr().Set("X")
_cyl.GetDisplayColorAttr().Set([Gf.Vec3f(0.85, 0.1, 0.1)])
_Mc = Gf.Matrix4d(_M); _Mc.SetTranslateOnly(Gf.Vec3d(_pos[0], _pos[1], 1.22))
_xf = UsdGeom.Xformable(_cyl); _xf.ClearXformOpOrder(); _xf.AddTransformOp().Set(_Mc)
UsdPhysics.CollisionAPI.Apply(_cyl.GetPrim())
_rb = UsdPhysics.RigidBodyAPI.Apply(_cyl.GetPrim()); _rb.CreateKinematicEnabledAttr(True)
_bind(_cyl.GetPrim())
print(f"[canjog] 캔 배치 z=1.22 (드래그로 조정). base z={_pos[2]:.3f}", flush=True)

# 관절 home 유지(stiff). 사용자가 Physics Inspector로 jog.
import numpy as np
nd = robot.num_dof
q_home = np.asarray(robot.get_joint_positions(), dtype=np.float32)
ctrl = robot.get_articulation_controller()
ctrl.set_gains(kps=np.full(nd, 3.0e4, dtype=np.float32), kds=np.full(nd, 3.0e3, dtype=np.float32))
ctrl.apply_action(ArticulationAction(joint_positions=q_home))
try:
    print(f"[canjog] DOF: {list(robot.dof_names)}", flush=True)
except Exception:
    pass

# 카메라: 그리퍼 정면
_bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
_bc = _bb.ComputeWorldBound(_stage.GetPrimAtPath(f"{robot_prim}/gripper_rh_p12_rn_base")).ComputeAlignedRange().GetMidpoint()
try:
    from omni.isaac.core.utils.viewports import set_camera_view
except Exception:
    from isaacsim.core.utils.viewports import set_camera_view
set_camera_view(eye=[_bc[0] + 0.02, _bc[1] - 0.40, _bc[2] + 0.10], target=[_bc[0], _bc[1], _bc[2] + 0.06])

world.play()
print("[canjog] 준비됨. 캔 선택→W로 드래그 / Window▸Physics▸Physics Inspector로 관절 jog", flush=True)
print(f"[canjog] 종료: 창 닫기 또는 touch {STOP_FILE}", flush=True)

if os.path.exists(STOP_FILE):
    os.remove(STOP_FILE)
while simulation_app.is_running():
    world.step(render=True)
    if os.path.exists(STOP_FILE):
        break
simulation_app.close()
