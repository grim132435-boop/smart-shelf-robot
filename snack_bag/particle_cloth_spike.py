# 과자봉지 particle cloth 인플레이터블 스파이크 — 얇은 닫힌 봉지를 공기압으로 부풀려 실제 봉지처럼 변형 검증.
"""
Stage7 과자봉지 2단계 — FEM volume(속찬 폼)은 실제 봉지 거동 불가 → **particle cloth + pressure(공기압)**.
사용자 블렌더 메시(snack_bag.usd, 0.16×0.23×0.015 얇은 닫힌 박스)를 천으로 만들어 공기압으로 부풀림
(중심 ~7cm 목표). 가운데를 손가락으로 쥐면 스퀴즈→중앙 솟음, 놓으면 형상 거의 복원 안 됨(소성 느낌).

★cm 스케일 핵심: particle_contact_offset/solid_rest_offset를 메시 격자(~0.7cm)에 맞춰 작게.
  공식 데모(ParticleInflatableDemo) 기본 offset=5cm는 봉지(16cm)보다 커서 폭발 → 이전 불안정 원인.
★GPU dynamics 필수(파티클은 GPU 전용).
실행: /tmp/run_clothspike.sh --pressure 2.0  / 종료: touch /tmp/clothspike_stop
"""
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--pressure", type=float, default=2.0, help="공기압 배수(rest volume×). 높을수록 빵빵")
parser.add_argument("--stretch", type=float, default=10000.0, help="stretch 강성(낮을수록 물렁)")
parser.add_argument("--bend", type=float, default=20.0)
parser.add_argument("--shear", type=float, default=20.0)
parser.add_argument("--damping", type=float, default=2.0, help="높을수록 덜 튕김(소성 느낌)")
parser.add_argument("--pco", type=float, default=0.008, help="particle_contact_offset(cm 스케일)")
parser.add_argument("--sro", type=float, default=0.004, help="solid_rest_offset")
args = parser.parse_args()

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False, "width": "1280", "height": "720"})

import os
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema
from omni.isaac.core import World
from omni.physx.scripts import particleUtils, physicsUtils

STOP = "/tmp/clothspike_stop"
SHOT_DIR = "/home/devuser/shelf_grasp_dev/logs/shots"
BAG_USD = "/home/devuser/shelf_grasp_dev/assets/snack_bag_pillow.usd"   # ★이미 7cm로 부푼 베개형(공기압 인플레이션 불요)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

# ── 물리 씬 + GPU dynamics (파티클 필수) ──
scene = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
if scene is None:
    scene = UsdPhysics.Scene.Define(stage, "/physicsScene").GetPrim()
_s = PhysxSchema.PhysxSceneAPI.Apply(scene)
_s.CreateEnableGPUDynamicsAttr().Set(True)
_s.CreateBroadphaseTypeAttr().Set("GPU")
print("[CLOTHSPIKE] GPU dynamics 활성화", flush=True)

# ── 파티클 시스템 (cm 스케일 offset — 봉지 격자에 맞춤) ──
psys_path = "/World/particleSystem"
particleUtils.add_physx_particle_system(
    stage, psys_path,
    simulation_owner=scene.GetPath(),
    contact_offset=args.pco, rest_offset=args.pco * 0.8,
    particle_contact_offset=args.pco, solid_rest_offset=args.sro,
    solver_position_iterations=20,
    enable_ccd=True,
    non_particle_collision_enabled=True,   # ★바닥·손가락 등 강체와 충돌(안 하면 통과해 가라앉음)
)
# PBD 재질(마찰 — 그리퍼 그립 위해 ↑)
pmat_path = "/World/Physics_Materials/bag_pbd"
particleUtils.add_pbd_particle_material(stage, pmat_path, friction=1.0)
physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(psys_path), pmat_path)

# ── 봉지 메시(사용자 블렌더) 로드 → 용접+삼각화 → 천 prim에 직접 올림 ──
#   ★레퍼런스(dynamic_mesh_path) 대신 직접 복사: USD 기본 prim이 Xform이면 천 prim에 점이 안 올라와
#     "failed to setup cooking params" 발생. 또 particle cloth는 삼각형+용접된 정점 필요.
def _load_weld_triangulate(usd_path, tol=1e-4):
    src = Usd.Stage.Open(usd_path)
    sm = next(UsdGeom.Mesh(p) for p in src.Traverse() if p.IsA(UsdGeom.Mesh))
    pts = sm.GetPointsAttr().Get(); fvc = sm.GetFaceVertexCountsAttr().Get(); fvi = sm.GetFaceVertexIndicesAttr().Get()
    # 용접: 근접 정점 병합
    remap = {}; newpts = []; keymap = {}
    inv = 1.0 / tol
    for i, p in enumerate(pts):
        k = (round(p[0]*inv), round(p[1]*inv), round(p[2]*inv))
        if k in keymap:
            remap[i] = keymap[k]
        else:
            keymap[k] = len(newpts); remap[i] = len(newpts); newpts.append(p)
    # 삼각화(팬) + 용접 인덱스 적용 + 퇴화 제거
    tris = []; idx = 0
    for c in fvc:
        f = [remap[fvi[idx + k]] for k in range(c)]; idx += c
        for k in range(1, c - 1):
            a, b, cc = f[0], f[k], f[k + 1]
            if a != b and b != cc and a != cc:
                tris += [a, b, cc]
    return [Gf.Vec3f(*p) for p in newpts], tris

bag_path = "/World/snack_bag"
_pts, _tris = _load_weld_triangulate(BAG_USD)
_bag_def = UsdGeom.Mesh.Define(stage, bag_path)
_bag_def.CreatePointsAttr(_pts)
_bag_def.CreateFaceVertexCountsAttr([3] * (len(_tris) // 3))
_bag_def.CreateFaceVertexIndicesAttr(_tris)
print(f"[CLOTHSPIKE] 봉지메시 용접+삼각화: 정점={len(_pts)} 삼각면={len(_tris)//3}", flush=True)
particleUtils.add_physx_particle_cloth(
    stage=stage, path=bag_path, dynamic_mesh_path=None,
    particle_system_path=psys_path,
    spring_stretch_stiffness=args.stretch,
    spring_bend_stiffness=args.bend,
    spring_shear_stiffness=args.shear,
    spring_damping=args.damping,
    self_collision=True, self_collision_filter=True, particle_group=0,
    pressure=args.pressure,   # >1 이면 공기 들어가 부풂
)
bag = UsdGeom.Mesh(stage.GetPrimAtPath(bag_path))
# 봉지를 책상 위(여기선 ground 위)로, 중심 z 약간 띄움
physicsUtils.set_or_add_translate_op(bag, Gf.Vec3f(0.0, 0.0, 0.045))
# 52g
massApi = UsdPhysics.MassAPI.Apply(bag.GetPrim())
massApi.GetMassAttr().Set(0.052)
print(f"[CLOTHSPIKE] cloth: pressure={args.pressure} stretch={args.stretch} damping={args.damping} "
      f"pco={args.pco} sro={args.sro}", flush=True)

# ── 손가락 2개(kinematic): 부풀린 뒤 중앙 쥐기→들기→놓기 ──
from omni.isaac.core.objects import DynamicCuboid
def make_finger(name, x):
    f = DynamicCuboid(prim_path=f"/World/{name}", name=name,
                      position=np.array([x, 0.0, 0.06]), scale=np.array([0.02, 0.02, 0.05]),  # 실측 너비 2cm 손가락
                      color=np.array([0.1, 0.4, 0.9]), mass=1.0)
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(f"/World/{name}")).CreateKinematicEnabledAttr().Set(True)
    return f
fingerL = make_finger("fingerL", -0.10)
fingerR = make_finger("fingerR", +0.10)
X_OPEN, X_GRIP = 0.10, 0.045   # 실측 매칭: 2cm폭 손가락 양옆 각 4.3cm 침투(0.08-0.043≈0.037 안쪽 가장자리)

def set_fingers(x, z):
    fingerL.set_world_pose(position=np.array([-x, 0.0, z]))
    fingerR.set_world_pose(position=np.array([ x, 0.0, z]))

def bag_metrics():
    pts = UsdGeom.Mesh(stage.GetPrimAtPath(bag_path)).GetPointsAttr().Get()
    if not pts: return (0, 0, 0)
    zs = [p[2] for p in pts]
    return (max(zs) - min(zs), sum(zs)/len(zs), max(zs))   # 두께, 평균z, 최고z

def shot(tag):
    try:
        os.makedirs(SHOT_DIR, exist_ok=True)
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        capture_viewport_to_file(get_active_viewport(), f"{SHOT_DIR}/cloth_{tag}.png")
        thick, avgz, topz = bag_metrics()
        print(f"[CLOTHSPIKE] {tag}: 봉지두께={thick*1000:.0f}mm 평균z={avgz*1000:.0f}mm 최고z={topz*1000:.0f}mm", flush=True)
    except Exception as e:
        print(f"[CLOTHSPIKE] shot 실패: {e}", flush=True)

try:
    from omni.isaac.core.utils.viewports import set_camera_view
    set_camera_view(eye=[0.35, -0.35, 0.25], target=[0.0, 0.0, 0.05])
except Exception:
    pass

world.reset()
set_fingers(X_OPEN, 0.025)
if os.path.exists(STOP):
    os.remove(STOP)
print("[CLOTHSPIKE] 시작 — 부풀림→중앙쥐기→들기→놓기. 종료: touch /tmp/clothspike_stop", flush=True)

_step = 0
while simulation_app.is_running():
    world.step(render=True)
    _step += 1
    # 0~150: 부풀림 관찰(손가락 열림) / 150~250: 중앙 쥐기 / 300~430: 들기 / 480~560: 놓기(복원 관찰)
    GZ = 0.04   # 봉지 중심부(베개 7cm, 중심 z≈0.045)
    # 소성 검증: 부풀림 → 바닥에서 스퀴즈 → 그대로 손가락만 열기 → 두께 유지 관찰(리프트 없음)
    if 150 <= _step < 250:
        t = (_step - 150) / 100.0
        set_fingers(X_OPEN + (X_GRIP - X_OPEN) * t, GZ)   # 스퀴즈
    elif 250 <= _step < 380:
        set_fingers(X_GRIP, GZ)                            # 유지
    elif 380 <= _step < 460:
        t = (_step - 380) / 80.0
        set_fingers(X_GRIP + (X_OPEN - X_GRIP) * t, GZ)    # 바닥에서 손가락 열기(복원/유지 관찰)
    if _step in (140, 250, 460, 560):
        _tag = {140: "01_inflated", 250: "02_gripped", 460: "03_released", 560: "04_settled"}[_step]
        shot(_tag)
    if _step == 600:
        print("[CLOTHSPIKE] 관찰 완료. HALT.", flush=True)
    if os.path.exists(STOP):
        print("[CLOTHSPIKE] stop 감지 → 종료", flush=True)
        simulation_app.close(); break
