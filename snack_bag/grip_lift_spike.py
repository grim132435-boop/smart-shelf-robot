# 과자봉지 cloth 옆-스퀴즈 그립-리프트 검증 — 실측 조건(2cm 손가락 4.3cm 침투→두께 7→9.5cm) 재현 + 리프트.
"""
실측: 16×23×7cm 봉지를 2cm 손가락으로 양옆 각 4.3cm 침투해 잡으면 두께 7→9.5cm(부피보존 불룩). 끝까지 안 닫음.
시퀀스: 부풀림 → 옆 스퀴즈(x, 4.3cm 침투) → 유지 → 리프트(쥔 채 z↑) → 두께·들림 측정.
실행: bash snack_bag/run_griplift.sh [--pressure P --stretch S --friction F --grip G]  종료: touch /tmp/snackbag_stop
"""
import os, sys, math, argparse
parser = argparse.ArgumentParser()
parser.add_argument("--pressure", type=float, default=10.0)
parser.add_argument("--stretch", type=float, default=30000.0, help="비신축↑(공기압에 단단해짐, 표면 안 늘어남)")
parser.add_argument("--solver", type=int, default=48, help="고stretch 안정용 solver 반복")
parser.add_argument("--grip", type=float, default=0.045, help="스퀴즈 시 손가락 중심 반간격(0.045≈4.3cm 침투, 끝까지 안 닫음)")
parser.add_argument("--friction", type=float, default=25.0, help="입자-강체 마찰(DexGarmentLab 25)")
parser.add_argument("--adhesion", type=float, default=0.0, help="입자-강체 들러붙음(과하면 표면붙어 늘어남/바닥붙음)")
parser.add_argument("--fscale", type=float, default=0.5, help="particle_friction_scale")
parser.add_argument("--ascale", type=float, default=0.5, help="particle_adhesion_scale")
args = parser.parse_args()

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": "1280", "height": "720"})

import numpy as np
import omni.usd
from pxr import UsdGeom, UsdPhysics
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.materials import PhysicsMaterial

sys.path.insert(0, "/home/devuser/shelf_grasp_dev/snack_bag")
from snack_bag_module import enable_gpu_dynamics, spawn_snack_bag

STOP = "/tmp/snackbag_stop"
SHOT_DIR = "/home/devuser/shelf_grasp_dev/logs/shots"
MESH = "/home/devuser/shelf_grasp_dev/assets/snack_bag_pillow.usd"
REST_Z = 0.045
X_OPEN = 0.10          # 벌림(봉지 밖)
FINGER_Z = 0.055       # 손가락 중심 z(10cm 높이 → 봉지 옆면 0~0.10 전체 덮음)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
scene_path = enable_gpu_dynamics(stage)
_fmat = PhysicsMaterial(prim_path="/World/Physics_Materials/finger_mat",
                        static_friction=2.2, dynamic_friction=2.0, restitution=0.0)

spawn_snack_bag(stage, scene_path, (0.0, 0.0), REST_Z, mode="cloth", prim_path="/World/snack_bag",
                params={"pressure": args.pressure, "stretch": args.stretch, "bend": 150.0, "shear": 50.0,
                        "friction": args.friction, "adhesion": args.adhesion,
                        "friction_scale": args.fscale, "adhesion_scale": args.ascale,
                        "mesh_usd": MESH, "pco": 0.005, "sro": 0.0025, "solver": args.solver,
                        "pbd_damping": 14.0, "max_velocity": 1.0, "gravity_scale": 1.0})

def mk(name, x):   # 2cm 폭, z 10cm(옆면 전체 덮음), 위에서 연결된 프롱
    f = DynamicCuboid(prim_path=f"/World/{name}", name=name,
                      position=np.array([x, 0.0, FINGER_Z]), scale=np.array([0.02, 0.05, 0.10]),
                      color=np.array([0.1, 0.4, 0.9]), mass=1.0)
    f.apply_physics_material(_fmat)
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(f"/World/{name}")).CreateKinematicEnabledAttr().Set(True)
    return f
fL = mk("fingerL", -X_OPEN); fR = mk("fingerR", +X_OPEN)

def set_fingers(x, z=FINGER_Z):
    fL.set_world_pose(position=np.array([-x, 0.0, z]))
    fR.set_world_pose(position=np.array([x, 0.0, z]))

def avgz():
    pts = UsdGeom.Mesh(stage.GetPrimAtPath("/World/snack_bag")).GetPointsAttr().Get()
    if not pts: return None
    z = sum(p[2] for p in pts) / len(pts) * 1000
    return None if math.isnan(z) else z

def thick():
    pts = UsdGeom.Mesh(stage.GetPrimAtPath("/World/snack_bag")).GetPointsAttr().Get()
    zs = [p[2] for p in pts] if pts else [0]
    t = (max(zs) - min(zs)) * 1000
    return None if (math.isnan(t) or t > 400) else int(t)

def shot(tag):
    try:
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        capture_viewport_to_file(get_active_viewport(), f"{SHOT_DIR}/griplift_{tag}.png")
    except Exception as e:
        print(f"shot실패 {e}", flush=True)

try:
    from omni.isaac.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.5, -0.6, 0.4], target=[0.0, 0.0, 0.08])
except Exception:
    pass

world.reset()
set_fingers(X_OPEN)
if os.path.exists(STOP):
    os.remove(STOP)
print(f"[GRIPLIFT] 옆-스퀴즈. pressure={args.pressure} stretch={args.stretch} grip={args.grip} "
      f"friction={args.friction} adhesion={args.adhesion}. 종료: touch {STOP}", flush=True)

_t0 = None; _z0 = None
_step = 0
while simulation_app.is_running():
    world.step(render=True)
    _step += 1
    # 100~200 옆 스퀴즈(x: 0.10→grip, 4.3cm 침투) / 200~280 유지 / 280~430 리프트(쥔 채 z↑)
    if _step < 100:
        set_fingers(X_OPEN)
    elif 100 <= _step < 200:
        t = (_step - 100) / 100.0
        set_fingers(X_OPEN + (args.grip - X_OPEN) * t)
    elif 200 <= _step < 280:
        set_fingers(args.grip)
    elif 280 <= _step < 430:
        t = (_step - 280) / 150.0
        set_fingers(args.grip, FINGER_Z + (0.30 - FINGER_Z) * t)
    if _step == 95:
        _t0 = thick(); _z0 = avgz()
        print(f"[GRIPLIFT] 부풀림 두께={_t0}mm 평균z={_z0:.0f}mm (목표 두께≈70)", flush=True); shot("01_inflated")
    if _step == 275:
        _t1 = thick()
        _b = (_t1 - _t0) if (_t1 is not None and _t0 is not None) else None
        print(f"[GRIPLIFT] 스퀴즈 두께={_t1}mm (불룩 {_b:+}mm, 목표 7→9.5cm=+25)", flush=True); shot("02_squeeze")
    if _step == 440:
        _z1 = avgz()
        _d = (_z1 - _z0) if (_z1 is not None and _z0 is not None) else None
        shot("03_lift")
        print(f"[GRIPLIFT] 리프트 후 평균z={_z1}mm (Δ={_d:+.0f}mm). "
              f"{'들림✅' if (_d is not None and _d > 30) else '안들림/폭발❌'}", flush=True)
        print("[GRIPLIFT] 완료. 종료 touch /tmp/snackbag_stop", flush=True)
    if os.path.exists(STOP):
        simulation_app.close(); break
