# 과자봉지 cloth를 1회 시뮬해 안착·변형된 메시를 정적 USD로 베이크(B안: CPU 씬용 봉지 비주얼)
#   particle cloth는 GPU 전용이라 CPU 혼합씬과 라이브 공존 불가 → 자연스러운 변형 모양만 떠서(bake)
#   정적 메시로 재사용. 결과: assets/snack_bag_baked.usd (bottom z=0, xy 중심 정렬).
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

import sys
import numpy as np
sys.path.insert(0, "/home/devuser/shelf_grasp_dev/snack_bag")
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
from omni.isaac.core import World

OUT = "/home/devuser/shelf_grasp_dev/assets/snack_bag_baked.usd"
SETTLE_STEPS = 400

world = World(stage_units_in_meters=1.0)
stage = world.stage
world.scene.add_default_ground_plane()

# GPU dynamics 켜기(particle cloth 필수) — play 전
scn = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
sa = PhysxSchema.PhysxSceneAPI.Apply(scn)
sa.CreateEnableGPUDynamicsAttr().Set(True)
sa.CreateBroadphaseTypeAttr().Set("GPU")
scn_path = str(scn.GetPath())

from snack_bag_module import spawn_snack_bag
spawn_snack_bag(stage, scn_path, (0.0, 0.0), 0.05, mode="cloth")   # 지면(z=0) 위 5cm 중심에서 안착

world.reset()
print(f"[bake] {SETTLE_STEPS}스텝 안착 시뮬 시작...", flush=True)
for i in range(SETTLE_STEPS):
    world.step(render=False)
    if i % 100 == 0:
        print(f"[bake] step {i}", flush=True)

MESH = "/World/snack_bag"
mesh = UsdGeom.Mesh(stage.GetPrimAtPath(MESH))

# 변형 점 읽기: Fabric(usdrt, 라이브 변형) 우선 → 실패 시 USD authored fallback
pts = None
try:
    import omni.usd
    import usdrt
    sid = omni.usd.get_context().get_stage_id()
    rt = usdrt.Usd.Stage.Attach(sid)
    rt_mesh = rt.GetPrimAtPath(MESH)
    rt_attr = rt_mesh.GetAttribute("points")
    if rt_attr and rt_attr.HasValue():
        pts = [(float(p[0]), float(p[1]), float(p[2])) for p in rt_attr.Get()]
        print(f"[bake] Fabric(usdrt)서 변형 점 {len(pts)}개 읽음", flush=True)
except Exception as e:
    print(f"[bake] usdrt 읽기 실패({e}) → USD fallback", flush=True)

if pts is None:
    _p = mesh.GetPointsAttr().Get()
    pts = [(float(p[0]), float(p[1]), float(p[2])) for p in _p]
    print(f"[bake] USD authored 점 {len(pts)}개 읽음(주의: rest일 수 있음)", flush=True)

# 메시 월드변환 반영(점이 로컬프레임일 수 있음) → 월드로
M = UsdGeom.Xformable(mesh.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
wpts = [M.Transform(Gf.Vec3d(p[0], p[1], p[2])) for p in pts]
arr = np.array([[w[0], w[1], w[2]] for w in wpts], dtype=np.float64)

# bottom z=0, xy 중심 정렬(spawn이 (x,y,table_top)에 놓도록)
cx = (arr[:, 0].min() + arr[:, 0].max()) / 2.0
cy = (arr[:, 1].min() + arr[:, 1].max()) / 2.0
zmin = arr[:, 2].min()
arr[:, 0] -= cx
arr[:, 1] -= cy
arr[:, 2] -= zmin
dx = arr[:, 0].max() - arr[:, 0].min()
dy = arr[:, 1].max() - arr[:, 1].min()
dz = arr[:, 2].max() - arr[:, 2].min()
print(f"[bake] 정렬 후 봉지 치수 폭(X)={dx*100:.1f} 길이(Y)={dy*100:.1f} 두께(Z)={dz*100:.1f} cm", flush=True)

fvc = mesh.GetFaceVertexCountsAttr().Get()
fvi = mesh.GetFaceVertexIndicesAttr().Get()

out = Usd.Stage.CreateNew(OUT)
om = UsdGeom.Mesh.Define(out, "/snack_bag_baked")
om.CreatePointsAttr([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in arr])
om.CreateFaceVertexCountsAttr([int(c) for c in fvc])
om.CreateFaceVertexIndicesAttr([int(i) for i in fvi])
om.CreateDisplayColorAttr([Gf.Vec3f(0.82, 0.78, 0.25)])
UsdGeom.Xformable(om.GetPrim()).AddTranslateOp()   # spawn이 translate 채움
out.GetRootLayer().Save()
print(f"[bake] ✅ 저장: {OUT} (점 {len(arr)}개, 면 {len(fvc)}개)", flush=True)

simulation_app.close()
