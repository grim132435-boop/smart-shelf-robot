# 과자봉지 그립 파라미터 병렬 스윕 — 봉지 격자(stretch×bend)를 각각 손가락으로 스퀴즈해 불룩량·안정성 동시 측정.
"""
한 씬에 봉지 N개(stretch×bend 격자), 봉지마다 2cm 손가락 2개가 4.3cm 침투 스퀴즈.
측정: 부풀림 두께 → 스퀴즈 두께 → 불룩량(목표 +25mm = 실측 7→9.5cm) + 안정성(NaN/과대=스파이크/폭발).
촘촘 메시(3mm)로 고stretch 안정성 확인. 실행: bash /tmp/run_sweepgrip.sh  종료: touch /tmp/snackbag_stop
"""
import os, sys, math
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": "1600", "height": "1000"})

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
MESH = "/home/devuser/shelf_grasp_dev/assets/snack_bag_pillow.usd"   # 코스 5mm(잘 부풂)
PCO, SRO = 0.005, 0.0025
REST_Z = 0.045
# 격자: stretch(열) × pressure(행) — 부피·불룩 튜닝(실측 7→9.5cm 불룩 +25mm, 안정, 들림)
STRETCHES = [6000.0, 8000.0, 12000.0]
PRESSURES = [9.0, 10.0, 11.0]
FRICTION = 2.2   # 그립 미끄럼 방지(사용자 설정값)
DX, DY = 0.45, 0.60
X_OPEN, X_GRIP = 0.10, 0.045   # 4.3cm 침투
GZ = 0.04

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
scene_path = enable_gpu_dynamics(stage)
_fmat = PhysicsMaterial(prim_path="/World/Physics_Materials/finger_mat",
                        static_friction=2.0, dynamic_friction=1.8, restitution=0.0)

bags = []   # (idx, gx, gy, P info)
fingers = []  # (fingerL, fingerR, gx, gy)
idx = 0
for r, P in enumerate(PRESSURES):
    gy = (r - 1) * DY
    for c, S in enumerate(STRETCHES):
        gx = (c - 1) * DX
        spawn_snack_bag(stage, scene_path, (gx, gy), REST_Z, mode="cloth",
                        prim_path=f"/World/bag_{idx}",
                        params={"pressure": P, "stretch": S, "bend": 150.0, "shear": 50.0,
                                "friction": FRICTION, "mesh_usd": MESH, "pco": PCO, "sro": SRO,
                                "pbd_damping": 14.0, "max_velocity": 1.0})
        def mk(name, x, y):
            f = DynamicCuboid(prim_path=f"/World/{name}", name=name,
                              position=np.array([x, y, GZ]), scale=np.array([0.02, 0.025, 0.025]),
                              color=np.array([0.1, 0.4, 0.9]), mass=1.0)
            f.apply_physics_material(_fmat)
            UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(f"/World/{name}")).CreateKinematicEnabledAttr().Set(True)
            return f
        fL = mk(f"fL_{idx}", gx - X_OPEN, gy); fR = mk(f"fR_{idx}", gx + X_OPEN, gy)
        fingers.append((fL, fR, gx, gy))
        bags.append((idx, S, P))
        print(f"  bag_{idx} @({gx:+.2f},{gy:+.2f}) stretch={S} pressure={P}", flush=True)
        idx += 1
print(f"[SWEEPGRIP] {idx}개. 열=stretch{STRETCHES} 행=pressure{PRESSURES}", flush=True)

def set_grip(x, z=GZ):
    for (fL, fR, gx, gy) in fingers:
        fL.set_world_pose(position=np.array([gx - x, gy, z]))
        fR.set_world_pose(position=np.array([gx + x, gy, z]))

def thick(i):
    pts = UsdGeom.Mesh(stage.GetPrimAtPath(f"/World/bag_{i}")).GetPointsAttr().Get()
    zs = [p[2] for p in pts] if pts else [0]
    t = (max(zs) - min(zs)) * 1000
    return None if (math.isnan(t) or t > 300) else int(t)

def avgz(i):
    pts = UsdGeom.Mesh(stage.GetPrimAtPath(f"/World/bag_{i}")).GetPointsAttr().Get()
    if not pts: return None
    z = sum(p[2] for p in pts) / len(pts) * 1000
    return None if math.isnan(z) else int(z)

def shot(tag):
    try:
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        capture_viewport_to_file(get_active_viewport(), f"{SHOT_DIR}/{tag}.png")
    except Exception as e:
        print(f"shot실패 {e}", flush=True)

try:
    from omni.isaac.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.0, -1.1, 1.9], target=[0.0, 0.0, 0.05])
except Exception:
    pass

world.reset()
set_grip(X_OPEN)
if os.path.exists(STOP):
    os.remove(STOP)
print("[SWEEPGRIP] 시작 — 부풀림→스퀴즈(4.3cm). 종료: touch /tmp/snackbag_stop", flush=True)

_infl_t = None; _infl_z = None
_step = 0
while simulation_app.is_running():
    world.step(render=True)
    _step += 1
    if 150 <= _step < 280:
        t = (_step - 150) / 130.0
        set_grip(X_OPEN + (X_GRIP - X_OPEN) * t)              # 스퀴즈(4.3cm)
    elif 300 <= _step < 450:
        t = (_step - 300) / 150.0
        set_grip(X_GRIP, GZ + (0.25 - GZ) * t)               # ★리프트(쥔 채 올림)
    if _step == 145:
        _infl_t = [thick(i) for i in range(idx)]; _infl_z = [avgz(i) for i in range(idx)]
        print(f"[SWEEPGRIP] 부풀림 두께(mm): {_infl_t} 평균z(mm): {_infl_z}", flush=True); shot("sweepgrip_inflated")
    if _step == 290:
        sq = [thick(i) for i in range(idx)]
        bulge = [(None if (sq[i] is None or _infl_t[i] is None) else sq[i]-_infl_t[i]) for i in range(idx)]
        print(f"[SWEEPGRIP] 스퀴즈 두께(mm): {sq}  불룩량(목표+25): {bulge}", flush=True); shot("sweepgrip_squeeze")
    if _step == 460:
        lz = [avgz(i) for i in range(idx)]
        lifted = [(None if (lz[i] is None or _infl_z[i] is None) else lz[i]-_infl_z[i]) for i in range(idx)]
        print(f"[SWEEPGRIP] 리프트 평균z(mm): {lz}", flush=True)
        print(f"[SWEEPGRIP] ★들림량(mm, +클수록 잘들림): {lifted}  (열=stretch{STRETCHES} 행=pressure{PRESSURES})", flush=True)
        shot("sweepgrip")
        print("[SWEEPGRIP] 완료(sweepgrip.png). None=폭발. 종료 touch /tmp/snackbag_stop", flush=True)
    if os.path.exists(STOP):
        simulation_app.close(); break
