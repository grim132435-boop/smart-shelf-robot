# 평평한 두-시트 봉지 메시 — 실제처럼 거의 두께없는 시트 2장(공기로 부풀릴 것). 7cm 사전두께 안 줌.
"""
사용자 지적: 실제 봉지=거의 두께없는 시트 2장 사이에 공기. 7cm 미리 주면 pressure가 과팽창→필름 늘어남.
→ 평평(±0.005=1cm 갭) 두 시트 + 둘레 솔기. pressure가 ~7cm로 부풀림(공기가 부피 담당).
삼각면, watertight. 출력: snack_bag_flat.usd
"""
from pxr import Usd, UsdGeom, Gf

OUT = "/home/devuser/shelf_grasp_dev/assets/snack_bag_flat.usd"
HX, HY = 0.08, 0.115      # 0.16×0.23
HZ = 0.005                # 평평(시트 간격 ±0.5cm=1cm). 공기가 부풀림
NU, NV = 32, 46           # ~5mm 격자

pts = []; idx = {}
def add(i, j, top):
    key = (i, j, top)
    if key in idx: return idx[key]
    u = -1 + 2 * i / NU; v = -1 + 2 * j / NV
    idx[key] = len(pts); pts.append((u * HX, v * HY, HZ if top else -HZ)); return idx[key]

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
m = UsdGeom.Mesh.Define(stage, "/snack_bag_flat")
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
print(f"정점={len(pts)} 삼각={len(tris)//3} 두께={2*HZ:.3f}(평평) 경계엣지={bnd} 비매니폴드={nm} "
      f"→ {'닫힘✓' if bnd==0 and nm==0 else '열림✗'}")
print(f"저장: {OUT}")
