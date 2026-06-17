#!/usr/bin/env python3
# RH-P12 언더액추에이티드(적응형) 파지 검증: 근위 stiff + 원위 soft spring로 파지점별 평행/curl 재현 테스트.
"""그리퍼 사이에 테스트 캔을 놓고 닫아, 파지 깊이에 따라 평행/curl이 갈리는지 물리로 확인.

모델(언더액추에이티드 근사):
  - 근위 r1/l1 = stiff position drive(액추에이티드 모터), 0→1.05 램프로 닫음.
  - 원위 r2/l2 = soft spring drive(낮은 kp), 같은 타겟이지만 약하게 → 접촉에 밀려 curl/평행 결정.
  - 핑거 collision = SDF(오목 패드 보존). 캔 = 정적 collider(중력 OFF, 제자리 고정).
가설: 캔이 핑거팁 끝이면 원위가 막혀 평행, 안쪽이면 원위가 자유로워 curl 감쌈.

[인자] --depth tip|deep (캔을 핑거팁쪽/안쪽 어디에 둘지), --dk <원위kp>, --df <원위 max effort>
[캡처] ~/shelf_grasp_dev/logs/shots/grasp_<depth>_{open,closing,closed}.png
[종료] 창 닫기 또는 touch /tmp/grasp_test_stop
"""
import os
import argparse

_ap = argparse.ArgumentParser()
_ap.add_argument("--depth", choices=["tip", "deep"], default="deep")
_ap.add_argument("--dk", type=float, default=120.0, help="원위(r2/l2) 스프링 강성 kp")
_ap.add_argument("--df", type=float, default=3.0, help="원위 max effort(Nm) — 낮을수록 잘 휨")
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
STOP_FILE = "/tmp/grasp_test_stop"
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
    cfg.parse_mimic = False          # 원위를 독립 제어(언더액추에이티드 모델용)
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

# ── 핑거 collision: convexHull → SDF(오목 패드면 보존, 접촉 정확).
_fingers = ("gripper_rh_p12_rn_r1/", "gripper_rh_p12_rn_r2/",
            "gripper_rh_p12_rn_l1/", "gripper_rh_p12_rn_l2/")
_nsdf = 0
for _p in _stage.Traverse():
    _s = str(_p.GetPath())
    if "/collisions" in _s and any(f in _s for f in _fingers):
        if _p.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI(_p).CreateApproximationAttr().Set("sdf")
            _sdf = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(_p)
            _sdf.CreateSdfResolutionAttr().Set(256)
            _nsdf += 1
print(f"[grasp] 핑거 SDF collision {_nsdf}개", flush=True)

robot = world.scene.add(Robot(prim_path=robot_prim, name="e0509"))
world.reset()
try:
    robot.disable_gravity()
except Exception:
    pass

# ── 그리퍼 base 월드 변환 → 캔을 핑거 사이 grasp 깊이에 배치(핑거 방향 = base local +z).
_xc = UsdGeom.XformCache(Usd.TimeCode.Default())
_M = _xc.GetLocalToWorldTransform(_stage.GetPrimAtPath(f"{robot_prim}/gripper_rh_p12_rn_base"))
_pos = _M.ExtractTranslation()
_rot = _M.ExtractRotationMatrix()
_zax = Gf.Vec3d(_rot[2][0], _rot[2][1], _rot[2][2]).GetNormalized()   # 핑거 길이 방향
_depth = 0.130 if ARGS.depth == "tip" else 0.085                       # 팁쪽 vs 안쪽
_can_c = _pos + _zax * _depth
# 캔: 정적 collider 실린더(축 X = base local x, 핑거 닫힘면에 가로로 눕힘)
_can_path = "/World/test_can"
_cyl = UsdGeom.Cylinder.Define(_stage, _can_path)
_cyl.GetRadiusAttr().Set(0.025)   # 지름 5cm
_cyl.GetHeightAttr().Set(0.115)
_cyl.GetAxisAttr().Set("X")
_cyl.GetDisplayColorAttr().Set([Gf.Vec3f(0.85, 0.1, 0.1)])
_Mcan = Gf.Matrix4d(_M)
_Mcan.SetTranslateOnly(_can_c)
_xf = UsdGeom.Xformable(_cyl)
_xf.ClearXformOpOrder()
_xf.AddTransformOp().Set(_Mcan)
UsdPhysics.CollisionAPI.Apply(_cyl.GetPrim())     # RigidBody 없음 → 정적 고정
print(f"[grasp] 테스트 캔({ARGS.depth}, depth={_depth}m) @ {tuple(round(v,3) for v in _can_c)}", flush=True)

# ── 드라이브: 팔=stiff home, 근위=stiff, 원위=soft spring(낮은 kp + 낮은 max effort).
nd = robot.num_dof
names = list(robot.dof_names)
q_home = np.asarray(robot.get_joint_positions(), dtype=np.float32)
_prox = [robot.get_dof_index(n) for n in ("gripper_rh_r1", "gripper_rh_l1")]
_dist = [robot.get_dof_index(n) for n in ("gripper_rh_r2", "gripper_rh_l2")]
kp = np.full(nd, 1.0e5, dtype=np.float32)
kd = np.full(nd, 1.0e4, dtype=np.float32)
for i in _prox:
    kp[i] = 1.0e4; kd[i] = 1.0e3
for i in _dist:
    kp[i] = ARGS.dk; kd[i] = max(2.0, ARGS.dk * 0.1)
ctrl = robot.get_articulation_controller()
ctrl.set_gains(kps=kp, kds=kd)
# 원위 max effort 낮춰 잘 휘게(가능하면).
try:
    _me = np.full(nd, 1.0e4, dtype=np.float32)
    for i in _dist:
        _me[i] = ARGS.df
    robot._articulation_view.set_max_efforts(_me.reshape(1, -1))
    print(f"[grasp] 원위 max effort={ARGS.df}Nm, kp={ARGS.dk}", flush=True)
except Exception as e:
    print(f"[grasp] max_effort 설정 실패(무시, kp만으로 컴플라이언스): {e}", flush=True)

# ── 카메라 프레이밍(그리퍼+캔).
try:
    try:
        from omni.isaac.core.utils.viewports import set_camera_view
    except Exception:
        from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[_can_c[0] + 0.3, _can_c[1] - 0.3, _can_c[2] + 0.12],
                    target=[_can_c[0], _can_c[1], _can_c[2]])
except Exception:
    pass

from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
_vp = get_active_viewport()


def _shot(tag):
    path = f"{SHOT_DIR}/grasp_{ARGS.depth}_{tag}.png"
    try:
        capture_viewport_to_file(_vp, path)
        for _ in range(8):
            world.step(render=True)
        print(f"[grasp] 캡처 → {path}", flush=True)
    except Exception as e:
        print(f"[grasp] 캡처 실패: {e}", flush=True)


if os.path.exists(STOP_FILE):
    os.remove(STOP_FILE)

# 안정화
for _ in range(30):
    world.step(render=True)
_shot("open")

# ── 닫기 램프: 근위/원위 타겟 0→1.05.
RAMP = 240
GRIP_CLOSE = 1.05
shot_mid = False
i = 0
while simulation_app.is_running():
    a = min(1.0, i / RAMP)
    targ = q_home.copy()
    val = a * GRIP_CLOSE
    for j in _prox + _dist:
        targ[j] = val
    ctrl.apply_action(ArticulationAction(joint_positions=targ))
    world.step(render=True)
    if (not shot_mid) and i == RAMP // 2:
        _shot("closing"); shot_mid = True
    if i == RAMP + 60:
        # 닫힘 안정화 후 캡처 + 실제 관절각 로깅
        jp = robot.get_joint_positions()
        pr = [float(jp[k]) for k in _prox]
        ds = [float(jp[k]) for k in _dist]
        print(f"[grasp] 닫힘 관절각 근위(r1,l1)={[round(x,3) for x in pr]} 원위(r2,l2)={[round(x,3) for x in ds]} (타겟 {GRIP_CLOSE})", flush=True)
        print("[grasp] → 원위가 타겟보다 작으면 캔에 막혀 curl/평행 결정된 것", flush=True)
        _shot("closed")
    i += 1
    if os.path.exists(STOP_FILE):
        print("[grasp] stop 감지 → 종료", flush=True)
        break

simulation_app.close()
