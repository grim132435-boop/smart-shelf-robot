# 환경(로봇·책상·매대=V2_USD) + 거치대를 하나의 에셋으로 결합 — 거치대 매 런 절차생성 제거(사용자).
#   결과: assets/shelf_workspace_v2_stand.usd. stage8은 이 에셋만 로드하면 환경+거치대 전부 들어옴.
#   거치대 위치는 이 에셋에서 편집(GUI로 옮김) → stage8이 런타임에 /World/snack_stand 위치를 읽어 봉지 적치에 사용.
#   ★거치대를 '로컬중심 지오메트리 + translate op'로 만들어 gizmo가 객체 위에 뜨고 GUI 이동 가능. Z-up. width=0.12(기존과 동일).
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

import sys
sys.path.insert(0, "/home/devuser/shelf_grasp_dev/snack_bag")
from pxr import Usd, Sdf, UsdGeom, Gf
from snack_bag_module import add_snack_stand

OUT = "/home/devuser/shelf_grasp_dev/assets/shelf_workspace_v2_stand.usd"
V2  = "/home/devuser/CoWriteBotRL/models/shelf_workspace_v2.usd"
STAND_POS = (0.32328, 0.48668, 1.14)   # ★사용자 GUI 확정 위치(2026-06-22)

stage = Usd.Stage.CreateNew(OUT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)        # ★Z-up(env와 일치 — 카메라 뒤집힘 방지)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = stage.DefinePrim("/World", "Xform")
world.GetReferences().AddReference(Sdf.Reference(V2, "/World"))   # 환경(로봇/책상/매대) 참조

# 거치대: 로컬중심(원점 기준) 지오메트리로 굽고, translate op로 위치 지정 → gizmo가 객체 위·이동 가능.
add_snack_stand(stage, "/World/snack_stand", (0.0, 0.0), 0.0, width=0.12)
_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/snack_stand"))
_xf.AddTranslateOp().Set(Gf.Vec3d(*STAND_POS))

# 로봇 받침 블록(V2엔 없어 로봇이 떠 보임) — 환경 한곳 취지로 에셋에 포함. 정적 콜라이더 큐브.
#   stage8 fix_scene은 이미 있으면 생성 안 함(중복방지). pos/size = ROBOT_BASE_BLOCK 상수와 동일.
from pxr import UsdPhysics as _UP
_base = UsdGeom.Cube.Define(stage, "/World/robot_base_block")
_base.CreateSizeAttr(1.0)
_base.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.35, 0.38)])
_bxf = UsdGeom.Xformable(_base.GetPrim())
_bxf.AddTranslateOp().Set(Gf.Vec3d(-0.25, -0.04, 0.715))
_bxf.AddScaleOp().Set(Gf.Vec3f(0.18, 0.22, 0.03))
_UP.CollisionAPI.Apply(_base.GetPrim())

stage.SetDefaultPrim(world)
stage.GetRootLayer().Save()
print(f"[bake-env] ✅ 저장: {OUT} (Z-up, 거치대 width=0.12 @ {STAND_POS}, translate op)", flush=True)

simulation_app.close()
