# FEM용 두꺼운 가장자리 베개 메시 생성 — tet 쿠킹 sliver 회피(가장자리 2.4cm, 중심 7cm).
"""
얇은 가장자리(0.8cm) 베개는 tet 쿠킹이 sliver로 거부 → FEM 전용으로 가장자리를 두껍게.
삼각면, watertight, 0.16×0.23, 중심두께 7cm. 출력: snack_bag_pillow_fem.usd
"""
from pxr import Usd, UsdGeom, Gf

OUT = "/home/devuser/shelf_grasp_dev/assets/snack_bag_pillow_fem.usd"
HX, HY = 0.08, 0.115          # 0.16×0.23 half
CENTER_HALF = 0.035           # 중심 반두께(7cm)
EDGE_HALF = 0.012             # 가장자리 반두께(2.4cm) — sliver 회피
NU, NV = 24, 34               # 격자

def half_thick(u, v):
    # u,v ∈ [-1,1]; 중심서 두껍고 가장자리서 EDGE_HALF
    return EDGE_HALF + (CENTER_HALF - EDGE_HALF) * (1 - u * u) * (1 - v * v)

pts = []
idx = {}
def add(i, j, top):
    key = (i, j, top)
    if key in idx:
        return idx[key]
    u = -1 + 2 * i / NU
    v = -1 + 2 * j / NV
    z = half_thick(u, v) * (1 if top else -1)
    p = (u * HX, v * HY, z)
    idx[key] = len(pts); pts.append(p); return idx[key]

tris = []
def quad(a, b, c, d):
    tris.extend([a, b, c, a, c, d])

# 윗면(법선 +z): CCW from above
for i in range(NU):
    for j in range(NV):
        quad(add(i, j, True), add(i + 1, j, True), add(i + 1, j + 1, True), add(i, j + 1, True))
# 아랫면(법선 -z): 반대 winding
for i in range(NU):
    for j in range(NV):
        quad(add(i, j, False), add(i, j + 1, False), add(i + 1, j + 1, False), add(i + 1, j, False))
# 가장자리 솔기(top↔bottom 경계 잇기), 바깥 향하도록
def edge_loop():
    loop = []
    for i in range(NU): loop.append((i, 0))
    for j in range(NV): loop.append((NU, j))
    for i in range(NU, 0, -1): loop.append((i, NV))
    for j in range(NV, 0, -1): loop.append((0, j))
    return loop
loop = edge_loop()
for k in range(len(loop)):
    i0, j0 = loop[k]; i1, j1 = loop[(k + 1) % len(loop)]
    t0 = add(i0, j0, True); t1 = add(i1, j1, True)
    b0 = add(i0, j0, False); b1 = add(i1, j1, False)
    quad(t0, t1, b1, b0)

mesh = UsdGeom.Mesh.Define(Usd.Stage.CreateNew(OUT) if False else None, "/tmp") if False else None
stage = Usd.Stage.CreateNew(OUT)
m = UsdGeom.Mesh.Define(stage, "/snack_bag_pillow_fem")
stage.SetDefaultPrim(m.GetPrim())
m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
m.CreateFaceVertexCountsAttr([3] * (len(tris) // 3))
m.CreateFaceVertexIndicesAttr(tris)
stage.GetRootLayer().Save()

# watertight 체크
edges = {}
for t in range(0, len(tris), 3):
    f = [tris[t], tris[t + 1], tris[t + 2]]
    for k in range(3):
        a, b = f[k], f[(k + 1) % 3]; e = (min(a, b), max(a, b)); edges[e] = edges.get(e, 0) + 1
bnd = sum(1 for v in edges.values() if v == 1); nm = sum(1 for v in edges.values() if v > 2)
zs = [p[2] for p in pts]
print(f"정점={len(pts)} 삼각={len(tris)//3} 두께={max(zs)-min(zs):.3f} 가장자리={EDGE_HALF*2:.3f} "
      f"경계엣지={bnd} 비매니폴드={nm} → {'닫힘✓' if bnd==0 and nm==0 else '열림✗'}")
print(f"저장: {OUT}")
