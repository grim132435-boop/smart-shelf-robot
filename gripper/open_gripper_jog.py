#!/usr/bin/env python3
# E0509+RH-P12 그리퍼를 Isaac Sim에 띄워 관절을 수동 jog하는 진단 도구(핑거팁 curl 거동 확인용).
"""핑거팁(l2/r2) curl 거동 진단용 그리퍼 뷰어 — 관절을 손으로 움직여본다.

open_robot_for_spheres.py 기반. 차이점:
  1) cfg.parse_mimic=False → mimic 관절(r2/l1/l2)을 독립 관절로 임포트.
     (mimic은 PhysX에서 'finite limit' 에러로 안 걸려 핑거가 안 따라옴 → 독립으로 풀어
      각 관절을 따로 돌려 실물처럼 안쪽 curl 되는 각도를 탐색)
  2) omni.physx.supportui(Physics Inspector) 확장 활성화 → 관절 슬라이더 UI 사용.
  3) 렌더만 하던 루프를 world.step(render=True)로 → 물리 스텝 ON, jog 즉시 반영.
  중력 OFF + fix_base + 드라이브 타겟=home → 팔은 정지, 그리퍼만 jog.

[관절 움직이는 법]
  A) Window ▸ Physics ▸ Physics Inspector → Articulation=/e0509_with_gripper → 관절 슬라이더 드래그.
  B) Stage트리 joints/gripper_rh_r1 선택 → Property ▸ Drive ▸ Target Position 0→1.0.
[종료] 창 닫기 또는 touch /tmp/gripper_jog_stop
"""
import os

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.kit.app
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

# Physics Inspector(관절 슬라이더 UI) 확장 활성화 — standalone 런처는 기본 비활성이라 강제 ON.
try:
    _ext = omni.kit.app.get_app().get_extension_manager()
    for _e in ("omni.physx.supportui", "omni.physx.ui"):
        _ext.set_extension_enabled_immediate(_e, True)
    print("[jog] Physics Inspector(omni.physx.supportui) 확장 활성화", flush=True)
except Exception as e:
    print(f"[jog] 확장 활성화 실패(무시): {e}", flush=True)

# URDF Importer 인터페이스 (버전별 네임스페이스 폴백)
try:
    from isaacsim.asset.importer.urdf import _urdf
except ImportError:
    try:
        from omni.importer.urdf import _urdf
    except ImportError:
        from omni.isaac.urdf import _urdf

URDF_PATH = "/home/devuser/curobo_ws/robots/e0509_gripper/e0509_gripper_abs.urdf"
STOP_FILE = "/tmp/gripper_jog_stop"

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
# ★핵심: mimic 관절을 독립 관절로 임포트(PhysX mimic 에러 회피 + 각 관절 따로 jog).
try:
    cfg.parse_mimic = False
    print("[jog] cfg.parse_mimic=False → 그리퍼 4관절 독립", flush=True)
except Exception as e:
    print(f"[jog] parse_mimic 설정 실패(무시): {e}", flush=True)
try:
    cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    cfg.default_drive_strength = 10000.0
    cfg.default_position_drive_damping = 100.0
except Exception:
    pass

# instanceable 메시는 메시 선택/렌더와 비호환 → 끌 수 있으면 끈다.
for _attr in ("make_instanceable", "create_instanceable", "instanceable"):
    if hasattr(cfg, _attr):
        try:
            setattr(cfg, _attr, False)
        except Exception:
            pass

root_dir = os.path.dirname(URDF_PATH)
fname = os.path.basename(URDF_PATH)
urdf_if = _urdf.acquire_urdf_interface()
parsed = urdf_if.parse_urdf(root_dir, fname, cfg)
robot_prim = urdf_if.import_robot(root_dir, fname, parsed, cfg, "")

# ── de-instance: instanceable 프림 펼쳐 메시가 실제 프림으로 보이게(안 바뀔 때까지 반복).
import omni.usd

_stage = omni.usd.get_context().get_stage()
_total = 0
for _pass in range(12):
    _changed = 0
    for _prim in _stage.Traverse():
        if _prim.IsInstanceable():
            _prim.SetInstanceable(False)
            _changed += 1
    _total += _changed
    if _changed == 0:
        break
print(f"[jog] de-instance: {_total}개 프림 펼침", flush=True)

# 스케일 bake 제거: 0.001 Xform 스케일을 1로 리셋하는 처리가 leaf 메시뿐 아니라 링크
#   Xform의 조인트 오프셋까지 1000배 부풀려 링크를 폭발시켰음(그리퍼가 z~30m로 날아감).
#   임포터 기본 스케일이 이미 정상 크기를 내므로 jog/검사에는 bake 불필요. (Lula 구체저작 전용 hack)
from pxr import UsdGeom, Usd, Gf  # noqa: F401  (카메라 프레이밍 BBoxCache용)

robot = world.scene.add(Robot(prim_path=robot_prim, name="e0509"))
world.reset()

# Play 눌러도/물리 스텝해도 정지하도록: 중력 OFF + 드라이브 타겟=home + kp/kd.
import numpy as np

try:
    robot.disable_gravity()
    print("[jog] 중력 OFF", flush=True)
except Exception as e:
    print(f"[jog] disable_gravity 실패(무시): {e}", flush=True)
try:
    nd = robot.num_dof
    q_home = np.asarray(robot.get_joint_positions(), dtype=np.float32)
    ctrl = robot.get_articulation_controller()
    ctrl.set_gains(kps=np.full(nd, 1.0e5, dtype=np.float32),
                   kds=np.full(nd, 1.0e4, dtype=np.float32))
    ctrl.apply_action(ArticulationAction(joint_positions=q_home))
    print(f"[jog] 드라이브 타겟=home, kp1e5/kd1e4 (딱딱, DOF {nd})", flush=True)
    try:
        names = robot.dof_names
        print(f"[jog] DOF 이름: {list(names)}", flush=True)
    except Exception:
        pass
except Exception as e:
    print(f"[jog] 드라이브 안정화 실패(무시): {e}", flush=True)

world.play()

# ── 카메라를 그리퍼에 맞춰 자동 프레이밍(선택/F 없이 바로 보이게).
try:
    try:
        from omni.isaac.core.utils.viewports import set_camera_view
    except Exception:
        from isaacsim.core.utils.viewports import set_camera_view
    _gb = _stage.GetPrimAtPath(f"{robot_prim}/gripper_rh_p12_rn_base")
    _target_prim = _gb if (_gb and _gb.IsValid()) else _stage.GetPrimAtPath(robot_prim)
    _bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    _rng = _bb.ComputeWorldBound(_target_prim).ComputeAlignedRange()
    _c = _rng.GetMidpoint()
    _d = max(0.25, _rng.GetSize().GetLength() * 0.9)   # 대상 크기에 비례한 거리
    set_camera_view(eye=[_c[0] + _d, _c[1] - _d, _c[2] + _d * 0.5],
                    target=[_c[0], _c[1], _c[2]])
    print(f"[jog] 카메라 프레이밍 → 대상중심({_c[0]:.3f},{_c[1]:.3f},{_c[2]:.3f}) 거리~{_d:.2f}m", flush=True)
except Exception as e:
    print(f"[jog] 카메라 프레이밍 실패(무시): {e}", flush=True)

print(f"[jog] 임포트 완료 → {robot_prim}", flush=True)
print("[jog] 관절 jog: Window▸Physics▸Physics Inspector → Articulation=/e0509_with_gripper → 슬라이더 드래그", flush=True)
print("[jog] 또는 Stage트리 joints/gripper_rh_r1 → Property▸Drive▸Target Position 0→1.0", flush=True)
print(f"[jog] 종료: 창 닫기 또는 touch {STOP_FILE}", flush=True)

if os.path.exists(STOP_FILE):
    os.remove(STOP_FILE)

while simulation_app.is_running():
    world.step(render=True)          # 물리 스텝 ON → jog 즉시 반영
    if os.path.exists(STOP_FILE):
        print("[jog] stop 파일 감지 → 종료", flush=True)
        break

simulation_app.close()
