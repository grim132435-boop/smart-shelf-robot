# FEM 소성 격리 스파이크 — FEM 봉지를 2cm presser로 눌렀다 떼서 "눌린 자국 유지(소성)" 검증.
"""
hijimasa 기법(plastic_deformation.PlasticDeformation) de-risk. stage7 안 건드림.
시퀀스: 안착 → 누르기(2cm 손가락 양옆) → 떼기 → 자국 유지 관찰(노드 평균변위 잔류 + shot).
실행: bash snack_bag/run_fem_plastic_spike.sh --yield 5000   종료: touch /tmp/fem_plastic_stop
"""
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--yield", dest="yld", type=float, default=2000.0, help="항복 응력(낮을수록 쉽게 영구변형)")
parser.add_argument("--press", type=float, default=0.045, help="손가락 침투 x(작을수록 깊게)")
parser.add_argument("--youngs", type=float, default=5.0e4, help="FEM 영률(낮을수록 부드러움)")
parser.add_argument("--poisson", type=float, default=0.30)
parser.add_argument("--prim", default="bag", help="FEM 형상 bag(둥근박스)/Cube/Sphere")
args = parser.parse_args()

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": "1280", "height": "720"})

import os, sys
import numpy as np
import omni.usd
from pxr import UsdGeom, UsdPhysics
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

sys.path.insert(0, "/home/devuser/shelf_grasp_dev/snack_bag")
from snack_bag_module import enable_gpu_dynamics, enable_deformable_beta, spawn_snack_bag
from plastic_deformation import PlasticDeformation

STOP = "/tmp/fem_plastic_stop"
SHOT_DIR = "/home/devuser/shelf_grasp_dev/logs/shots"
REST_Z = 0.04
X_OPEN = 0.10

enable_deformable_beta()      # ★FEM beta 스키마 활성화(SimulationApp 후)
# ★GPU 일치 — deformable 노드 텐서 API는 GPU(suppressReadback)에서만 구현. GPU pipeline은 코어 유틸을
#   torch로 고르므로, World도 torch+cuda로 통일해야 reset서 numpy/torch 불일치가 안 남.
from isaacsim.core.simulation_manager import SimulationManager
SimulationManager.set_physics_sim_device("cuda:0")
world = World(stage_units_in_meters=1.0, backend="torch", device="cuda:0")
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
scene_path = enable_gpu_dynamics(stage)

bag = spawn_snack_bag(stage, scene_path, (0.0, 0.0), REST_Z, mode="fem_beta",
                      params={"youngs": args.youngs, "poisson": args.poisson, "fem_prim": args.prim})
print(f"[FEMPLASTIC] FEM 봉지 spawn @ {bag}", flush=True)

FINGER_Z = 0.055   # 프롱 중심 z(위에서 연결된 손가락). 10cm 높이 → 베개(0.005~0.075) 옆면 전체 덮음
def make_finger(name, x):
    f = DynamicCuboid(prim_path=f"/World/{name}", name=name,
                      position=np.array([x, 0.0, FINGER_Z]), scale=np.array([0.02, 0.025, 0.10]),
                      color=np.array([0.1, 0.4, 0.9]), mass=1.0)
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(f"/World/{name}")).CreateKinematicEnabledAttr().Set(True)
    return f
fingerL = make_finger("fingerL", -X_OPEN)
fingerR = make_finger("fingerR", +X_OPEN)
_finger_x = X_OPEN
def set_fingers(x):
    global _finger_x
    _finger_x = x
    fingerL.set_world_pose(position=np.array([-x, 0.0, FINGER_Z]))
    fingerR.set_world_pose(position=np.array([x, 0.0, FINGER_Z]))

def shot(tag):
    try:
        os.makedirs(SHOT_DIR, exist_ok=True)
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        capture_viewport_to_file(get_active_viewport(), f"{SHOT_DIR}/fem_plastic_{tag}.png")
        print(f"[FEMPLASTIC] shot {tag} — stats(항복,고정,평균변위mm)={pd.stats()}", flush=True)
    except Exception as e:
        print(f"[FEMPLASTIC] shot 실패: {e}", flush=True)

try:
    from omni.isaac.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.35, -0.35, 0.25], target=[0.0, 0.0, 0.04])
except Exception:
    pass

pd = PlasticDeformation(bag, youngs=args.youngs, poisson=args.poisson, yield_stress=args.yld)
world.reset()
set_fingers(X_OPEN)
if os.path.exists(STOP):
    os.remove(STOP)
print(f"[FEMPLASTIC] 시작 — 안착→누르기→떼기. yield={args.yld} press={args.press}. 종료: touch {STOP}", flush=True)

_step = 0
while simulation_app.is_running():
    world.step(render=True)
    _step += 1
    # ★rest 기준을 안착 후(step 130)에 캡처 → 낙하/안착 transient는 항복 제외, 그립 변형만 소성.
    if not pd._ready and _step in (130, 140, 150):
        pd.initialize()
    if pd._ready:
        pd.step()                                   # post(상태갱신)+pre(freeze)
    # 시퀀스: 160~260 누르기 / 260~340 유지 / 340~430 떼기 / 이후 관찰
    if 160 <= _step < 260:
        t = (_step - 160) / 100.0
        set_fingers(X_OPEN + (args.press - X_OPEN) * t)
    elif 260 <= _step < 340:
        set_fingers(args.press)
    elif 340 <= _step < 430:
        t = (_step - 340) / 90.0
        set_fingers(args.press + (X_OPEN - args.press) * t)
    if pd._ready and _step % 10 == 0:               # 크래시 지점 추적용 주기 로깅
        print(f"[FEMPLASTIC] step={_step} finger_x={_finger_x:.4f} stats={pd.stats()}", flush=True)
    if _step in (155, 300, 480, 600):
        shot({155: "01_settle", 300: "02_pressed", 480: "03_released", 600: "04_final"}[_step])
    if _step == 620:
        print("[FEMPLASTIC] 관찰 완료. (자국 유지=평균변위 잔류) HALT.", flush=True)
    if os.path.exists(STOP):
        print("[FEMPLASTIC] stop → 종료", flush=True); simulation_app.close(); break
