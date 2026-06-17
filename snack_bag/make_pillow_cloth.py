# particle cloth용 촘촘한 베개 메시 — 고운 주름 + facet 스파이크 감소(얇은 가장자리, ~5mm 격자).
"""
얇은 가장자리(0.8cm) 베개 7cm. 촘촘(NU×NV)하게 해서 주름이 곱고 입자가 튀어 생기는 뾰족함을 줄임.
삼각면, watertight. 출력: snack_bag_pillow.usd (cloth 모드 공용).
★촘촘해지면 particle_contact_offset도 격자간격에 맞춰 줄여야 함(모듈 CLOTH_PARAMS pco/sro).
"""
import sys
from pxr import Usd, UsdGeom, Gf

# 사용법: make_pillow_cloth.py [center_cm] [out_suffix] [density_factor]  (기본 7cm,1배 → snack_bag_pillow.usd)
_center_cm = float(sys.argv[1]) if len(sys.argv) > 1 else 7.0
_suffix = sys.argv[2] if len(sys.argv) > 2 else ""
_dens = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
OUT = f"/home/devuser/shelf_grasp_dev/assets/snack_bag_pillow{_suffix}.usd"
HX, HY = 0.08, 0.115
CENTER_HALF = _center_cm / 2 / 100.0   # cm → half(m)
EDGE_HALF = 0.0005        # ★가장자리 ~0(두 시트가 만남, 벽 없음). 0.0=좌표중복(degenerate) 방지용 0.5mm
NU, NV = int(32 * _dens), int(46 * _dens)   # 기본 ~5mm 격자, density_factor로 촘촘하게(0.16/32=5mm, 0.23/46=5mm)

def half_thick(u, v):
    # ★모든 가장자리 두께 0(두 시트가 만남, 벽 없음): 모든 변에서 0으로 매끄럽게 테이퍼.
    return EDGE_HALF + (CENTER_HALF - EDGE_HALF) * (1 - u * u) * (1 - v * v)

pts = []; idx = {}
def add(i, j, top):
    key = (i, j, top)
    if key in idx: return idx[key]
    u = -1 + 2 * i / NU; v = -1 + 2 * j / NV
    z = half_thick(u, v) * (1 if top else -1)
    idx[key] = len(pts); pts.append((u * HX, v * HY, z)); return idx[key]

tris = []
def quad(a, b, c, d): tris.extend([a, b, c, a, c, d])

for i in range(NU):
    for j in range(NV):
        quad(add(i, j, True), add(i + 1, j, True), add(i + 1, j + 1, True), add(i, j + 1, True))
for i in range(NU):
    for j in range(NV):
        quad(add(i, j, False), add(i, j + 1, False), add(i + 1, j + 1, False), add(i + 1, j, False))
loop = []
for i in range(NU): loop.append((i, 0))
for j in range(NV): loop.append((NU, j))
for i in range(NU, 0, -1): loop.append((i, NV))
for j in range(NV, 0, -1): loop.append((0, j))
for k in range(len(loop)):
    i0, j0 = loop[k]; i1, j1 = loop[(k + 1) % len(loop)]
    quad(add(i0, j0, True), add(i1, j1, True), add(i1, j1, False), add(i0, j0, False))

stage = Usd.Stage.CreateNew(OUT)
m = UsdGeom.Mesh.Define(stage, "/snack_bag_pillow")
stage.SetDefaultPrim(m.GetPrim())
m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
m.CreateFaceVertexCountsAttr([3] * (len(tris) // 3))
m.CreateFaceVertexIndicesAttr(tris)
stage.GetRootLayer().Save()

edges = {}
for t in range(0, len(tris), 3):
    f = [tris[t], tris[t + 1], tris[t + 2]]
    for k in range(3):
        a, b = f[k], f[(k + 1) % 3]; e = (min(a, b), max(a, b)); edges[e] = edges.get(e, 0) + 1
bnd = sum(1 for v in edges.values() if v == 1); nm = sum(1 for v in edges.values() if v > 2)
print(f"정점={len(pts)} 삼각={len(tris)//3} 격자~{0.16/NU*1000:.0f}mm 경계엣지={bnd} 비매니폴드={nm} "
      f"→ {'닫힘✓' if bnd==0 and nm==0 else '열림✗'}")
print(f"저장: {OUT}")
