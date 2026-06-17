# 과자봉지 변형체 독립 테스트 하니스 — snack_bag_module을 임포트해 스퀴즈/소성 검증(stage7 안 건드림).
"""
stage7과 완전 분리(다른 창이 stage7 그리퍼 작업 중). 전용 sentinel/log/shot 사용.
- 실행: bash /tmp/run_snackbag.sh --mode cloth   (종료: touch /tmp/snackbag_stop)
- 검증: 부풀림 → 2cm 손가락으로 양옆 스퀴즈(실측 4.3cm 침투 → 두께 7→9.5cm) → 바닥에서 손가락 열기 → 유지 관찰.
- 결과 샷: logs/shots/snackbag_*.png, 로그: logs/snackbag*.log
"""
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=["cloth", "fem_beta"], default="cloth")
parser.add_argument("--pressure", type=float, default=None)
parser.add_argument("--stretch", type=float, default=None)
parser.add_argument("--bend", type=float, default=None)
parser.add_argument("--damping", type=float, default=None)
parser.add_argument("--idle", action="store_true", help="봉지만 띄우고 손가락·스퀴즈 없음(GUI 라이브 튜닝용)")
parser.add_argument("--mesh", choices=["pillow", "flat", "pillow_fem"], default="pillow")
args = parser.parse_args()
_MESH = {"pillow": "/home/devuser/shelf_grasp_dev/assets/snack_bag_pillow.usd",
         "flat": "/home/devuser/shelf_grasp_dev/assets/snack_bag_flat.usd",
         "pillow_fem": "/home/devuser/shelf_grasp_dev/assets/snack_bag_pillow_fem.usd"}[args.mesh]

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": "1280", "height": "720"})

import os, sys
import numpy as np
import omni.usd
from pxr import UsdGeom, UsdPhysics
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

sys.path.insert(0, "/home/devuser/shelf_grasp_dev/snack_bag")
from snack_bag_module import (enable_gpu_dynamics, enable_deformable_beta,
                              spawn_snack_bag, apply_plastic_yield)

STOP = "/tmp/snackbag_stop"
SHOT_DIR = "/home/devuser/shelf_grasp_dev/logs/shots"
REST_Z = 0.02 if args.mesh == "flat" else 0.045   # 평평=바닥근처, 베개=7cm 중심
BAG_ROOT = "/World/snack_bag"

if args.mode == "fem_beta":
    enable_deformable_beta()   # ★beta deformable API 사용 필수(SimulationApp 생성 후)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

scene_path = enable_gpu_dynamics(stage)
print(f"[SNACKBAG] GPU dynamics 활성화, mode={args.mode}", flush=True)

# 파라미터 오버라이드(튜닝용) + 메시 선택
ov = {k: v for k, v in dict(pressure=args.pressure, stretch=args.stretch,
                            bend=args.bend, damping=args.damping).items() if v is not None}
ov["mesh_usd"] = _MESH
bag_path = spawn_snack_bag(stage, scene_path, (0.0, 0.0), REST_Z, mode=args.mode, params=ov)
print(f"[SNACKBAG] 봉지 생성 @ {bag_path} mesh={args.mesh} (override={ov})", flush=True)

# 2cm 폭 손가락(실측). idle 모드면 생략(봉지만 — GUI 라이브 튜닝용)
if not args.idle:
    from omni.isaac.core.materials import PhysicsMaterial
    _finger_mat = PhysicsMaterial(prim_path="/World/Physics_Materials/finger_mat",
                                  static_friction=2.0, dynamic_friction=1.8, restitution=0.0)
    def make_finger(name, x):
        f = DynamicCuboid(prim_path=f"/World/{name}", name=name,
                          position=np.array([x, 0.0, 0.04]), scale=np.array([0.02, 0.025, 0.025]),
                          color=np.array([0.1, 0.4, 0.9]), mass=1.0)
        f.apply_physics_material(_finger_mat)
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(f"/World/{name}")).CreateKinematicEnabledAttr().Set(True)
        return f
    fingerL = make_finger("fingerL", -0.10)
    fingerR = make_finger("fingerR", +0.10)
X_OPEN, X_GRIP = 0.10, 0.045   # 실물 4.3cm 침투. ★더 깊게 금지(실물선 칩 다 부서짐). 그립은 마찰+팽팽함으로

def set_fingers(x, z):
    fingerL.set_world_pose(position=np.array([-x, 0.0, z]))
    fingerR.set_world_pose(position=np.array([ x, 0.0, z]))

def bag_metrics():
    pts = UsdGeom.Mesh(stage.GetPrimAtPath(bag_path)).GetPointsAttr().Get()
    if not pts: return (0, 0, 0)
    zs = [p[2] for p in pts]
    return (max(zs) - min(zs), sum(zs) / len(zs), max(zs))

def shot(tag):
    try:
        os.makedirs(SHOT_DIR, exist_ok=True)
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        capture_viewport_to_file(get_active_viewport(), f"{SHOT_DIR}/snackbag_{tag}.png")
        th, az, tz = bag_metrics()
        print(f"[SNACKBAG] {tag}: 두께={th*1000:.0f}mm 평균z={az*1000:.0f}mm 최고z={tz*1000:.0f}mm", flush=True)
    except Exception as e:
        print(f"[SNACKBAG] shot 실패: {e}", flush=True)

try:
    from omni.isaac.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.35, -0.35, 0.25], target=[0.0, 0.0, 0.05])
except Exception:
    pass

world.reset()
if not args.idle:
    set_fingers(X_OPEN, 0.04)
if os.path.exists(STOP):
    os.remove(STOP)
if args.idle:
    print("[SNACKBAG] IDLE — 봉지만. GUI Property 패널서 라이브 튜닝하세요. 종료: touch /tmp/snackbag_stop", flush=True)
else:
    print("[SNACKBAG] 시작 — 부풀림→스퀴즈→리프트. 종료: touch /tmp/snackbag_stop", flush=True)

_step = 0
while simulation_app.is_running():
    world.step(render=True)
    _step += 1
    if args.idle:
        if _step == 200:
            shot("idle")
            print("[SNACKBAG] idle 안정화 완료 — 이제 GUI서 튜닝. (종료: touch /tmp/snackbag_stop)", flush=True)
        if os.path.exists(STOP):
            print("[SNACKBAG] stop 감지 → 종료", flush=True); simulation_app.close(); break
        continue
    GZ = 0.04
    # 실물 방식 검증: 부풀림 → 2cm손가락 4.3cm침투 스퀴즈 → 리프트(들리나)
    if 150 <= _step < 250:
        t = (_step - 150) / 100.0
        set_fingers(X_OPEN + (X_GRIP - X_OPEN) * t, GZ)        # 스퀴즈(중앙 쥠)
    elif 250 <= _step < 290:
        set_fingers(X_GRIP, GZ)                                 # 그립 안정
    elif 290 <= _step < 430:
        t = (_step - 290) / 140.0
        set_fingers(X_GRIP, GZ + (0.25 - GZ) * t)              # ★리프트(쥔 채 들어올림)
    if _step in (140, 250, 430, 560):
        _tag = {140: "01_inflated", 250: "02_squeezed", 430: "03_lifted", 560: "04_held"}[_step]
        shot(_tag)
    if _step == 600:
        print("[SNACKBAG] 관찰 완료. HALT.", flush=True)
    if os.path.exists(STOP):
        print("[SNACKBAG] stop 감지 → 종료", flush=True)
        simulation_app.close(); break
