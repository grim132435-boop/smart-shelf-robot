# 과자봉지 파라미터 스윕 — 한 씬에 봉지 격자(pressure×damping)를 동시에 띄워 한 번에 비교(병렬 탐색).
"""
순차 배치(조합당 4~5분) 대신, 한 Isaac Sim 런에서 N개 봉지를 격자로 띄워 동시에 관찰.
같은 particle system 공유, cloth별 파라미터만 다르게. 위에서 내려다보는 샷 1장으로 9개 비교.
실행: bash /tmp/run_sweep.sh   (종료: touch /tmp/snackbag_stop)
결과: logs/shots/sweep.png + 로그에 격자 배치(어느 칸이 어떤 값) 출력.
"""
import os, sys
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": "1600", "height": "1000"})

import omni.usd
from pxr import UsdGeom
from omni.isaac.core import World

sys.path.insert(0, "/home/devuser/shelf_grasp_dev/snack_bag")
from snack_bag_module import enable_gpu_dynamics, spawn_snack_bag

STOP = "/tmp/snackbag_stop"
SHOT_DIR = "/home/devuser/shelf_grasp_dev/logs/shots"
MESH = "/home/devuser/shelf_grasp_dev/assets/snack_bag_pillow.usd"   # 2cm 최소 베개

# 스윕 격자: pressure(행) × spring_damping(열)
PRESSURES = [4.0, 8.0, 14.0]
DAMPINGS  = [4.0, 12.0, 25.0]
DX, DY = 0.40, 0.55   # 봉지 간격(겹침 방지)
REST_Z = 0.045

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
scene_path = enable_gpu_dynamics(stage)
print("[SWEEP] GPU dynamics. 격자 배치:", flush=True)

idx = 0
for r, P in enumerate(PRESSURES):
    gy = (r - 1) * DY
    for c, D in enumerate(DAMPINGS):
        gx = (c - 1) * DX
        spawn_snack_bag(stage, scene_path, (gx, gy), REST_Z, mode="cloth",
                        prim_path=f"/World/bag_{idx}",
                        params={"pressure": P, "damping": D, "stretch": 80000.0,
                                "bend": 800.0, "shear": 50.0, "mesh_usd": MESH})
        print(f"  bag_{idx} @({gx:+.2f},{gy:+.2f})  pressure={P}  damping={D}", flush=True)
        idx += 1
print(f"[SWEEP] {idx}개 봉지 생성. 행=pressure{PRESSURES}(앞→뒤), 열=damping{DAMPINGS}(좌→우)", flush=True)

def shot(tag):
    try:
        os.makedirs(SHOT_DIR, exist_ok=True)
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        capture_viewport_to_file(get_active_viewport(), f"{SHOT_DIR}/{tag}.png")
        print(f"[SWEEP] shot {tag}", flush=True)
    except Exception as e:
        print(f"[SWEEP] shot 실패: {e}", flush=True)

try:
    from omni.isaac.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.0, -1.05, 2.0], target=[0.0, 0.0, 0.05])   # 격자 전체 위에서 내려다봄
except Exception:
    pass

world.reset()
if os.path.exists(STOP):
    os.remove(STOP)
print("[SWEEP] 시작 — 부풀려 안정화 후 샷. 종료: touch /tmp/snackbag_stop", flush=True)

import math
def thicknesses():
    out = []
    for i in range(9):
        pts = UsdGeom.Mesh(stage.GetPrimAtPath(f"/World/bag_{i}")).GetPointsAttr().Get()
        zs = [p[2] for p in pts] if pts else [0]
        th = (max(zs) - min(zs)) * 1000
        out.append("폭발" if (math.isnan(th) or th > 300) else int(th))   # NaN/과대 = 폭발
    return out

_step = 0
while simulation_app.is_running():
    world.step(render=True)
    _step += 1
    if _step in (300, 600):
        shot("sweep")
        print(f"[SWEEP] step{_step} 두께(mm) 행=P{PRESSURES} 열=D{DAMPINGS}: {thicknesses()}", flush=True)
    if _step == 600:
        print("[SWEEP] 안정화 샷 완료(sweep.png). 종료 touch /tmp/snackbag_stop", flush=True)
    if os.path.exists(STOP):
        print("[SWEEP] stop → 종료", flush=True); simulation_app.close(); break
