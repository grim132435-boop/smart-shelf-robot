# 과자봉지 변형체 에셋 모듈 — stage7과 독립적으로 개발/실행하고 나중에 머지하는 단위(particle cloth / FEM beta).
"""
독립 캡슐화 목적.
- stage7_graspgen_e0509.py(다른 창: 실물 그리퍼 충돌구체·오프셋 작업)와 **파일 충돌 없이** 봉지 변형체를
  개발하기 위해, 봉지 에셋/물리 코드를 이 모듈 하나로 분리. stage7은 나중에 `from snack_bag_module import ...`.
- 모드: "cloth"(particle cloth 인플레이터블, 검증됨 — 빵빵+그립, 탄성) / "fem_beta"(소성 추구, 개발중).

머지 규약(나중에 stage7에서).
  from multiobj_pipeline.snack_bag_module import enable_gpu_dynamics, spawn_snack_bag
  enable_gpu_dynamics(stage)                       # my_world.play() 전
  bag_path = spawn_snack_bag(stage, scene_prim_path, (cx, cy), rest_center_z, mode="cloth")
  → 기존 stage7 인라인 snack 코드(박스 FEM 등)는 제거하고 위 호출로 대체. 상세는 MERGE_snack_bag.md.

좌표 단위 meter. snack_bag.usd(0.16×0.23×0.015 얇은 닫힌 박스) / snack_bag_pillow.usd(7cm 부푼 베개) 공용.
"""
import os
from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema, Sdf
from omni.physx.scripts import particleUtils, physicsUtils, deformableUtils

# FEM beta deformable 파라미터(소성 추구). youngs는 필름 강성 근사.
FEM_PARAMS = dict(
    youngs=5.0e4,        # 부드럽게(눌리게). 3e5는 너무 단단해 손가락이 못 누름
    poisson=0.30,        # 비압축성↓(0.45→0.3) → 덜 단단, settle 응력↓
    dynamic_friction=0.8,
    fem_resolution=10,   # hex sim mesh 해상도
    mass=0.052,
    fem_prim="bag",          # bag=베개(빵빵 몸통+평평 밀봉 가장자리, 실제 봉지 룩) / Cube(각진 슬랩) / Sphere(타원)
    fem_center_half=0.035,   # 중앙 반두께(m) → 몸통 7cm 빵빵
    fem_edge_half=0.0012,    # 가장자리 밀봉 립 반두께(m) → 2.4mm 평평 실링(0이면 쿠킹거부라 비-0)
    fem_density=46.0,        # kg/m³ → 부피 ~1140cm³ × 46 ≈ 52g(실제 봉지). 기본 자동(≈1000)은 20배 과중
)

# 에셋 경로(읽기 전용 공유 — 두 창 모두 읽기만)
_ASSET_DIR = "/home/devuser/shelf_grasp_dev/assets"
PILLOW_USD = os.path.join(_ASSET_DIR, "snack_bag_pillow.usd")          # (구) 7cm 사전부푼 베개 — pressure가 과팽창시켜 폐기
BAG_FLAT_USD = os.path.join(_ASSET_DIR, "snack_bag_flat.usd")          # ★평평 두-시트(1cm) — 공기로 부풀림(실제 봉지 모델)
PILLOW_FEM_USD = os.path.join(_ASSET_DIR, "snack_bag_pillow_fem.usd")  # 가장자리 2.4cm(FEM tet 쿠킹 sliver 회피)
FLAT_USD   = os.path.join(_ASSET_DIR, "snack_bag.usd")                 # 얇은 닫힌 박스

# 검증된 particle cloth 파라미터(cloth6: 촘촘 베개 + 빳빳한 호일 느낌, 84mm 빵빵, 안정). 실측 BOPP 필름 반영.
CLOTH_PARAMS = dict(
    pressure=11.0,       # 질소 공기압(빵빵↑). 앞선 11 폭발은 friction 0 탓 → friction 2면 안정. 12+는 PBD 한계(폭발)
    stretch=6000.0,      # ★안정(8000은 그립 스퀴즈서 폭발 확인). 1e4↑는 꿀렁/스파이크
    bend=150.0,          # 과대 bend는 불안정 → 적당히
    shear=50.0,
    damping=4.0,         # 출렁임 제거(8↑은 폭발)
    pco=0.005,           # particle_contact_offset(촘촘 5mm 격자에 맞춤 — 격자보다 크면 입자 과중첩→폭발)
    sro=0.0025,          # solid_rest_offset
    # ★내용물(contents)/소성/B는 포기(2026-06-16) — PBD cm스케일서 cloth+내용물 반복 폭발, 탄성 수용.
    contents=False,
    friction=2.0,        # ★안정 하한(2→4 했다 복귀). 카테리-보조라 그립 힘 불요 → 안정 우선
    pbd_damping=18.0,    # ★전역 입자 속도 감쇠(14→18, 단일 입자 스파이크 에너지 흡수)
    max_velocity=0.2,    # ★입자 최대속도 제한(0.3→0.2, 그리퍼 끼임 튕김 스파이크 클램프)
    # max_depen_velocity 제거 — 0.5는 부풀림 단계서 입자분리 막아 spawn전 폭발(역효과)
    adhesion=0.0,        # B(접착 구김고정)는 접음 — C(알맹이)가 소성 담당
    adhesion_scale=1.0,
    friction_scale=0.5,  # ★입자간 마찰(폴드 유지·내부 안정, DexGarmentLab)
    gravity_scale=1.0,   # ★중력 명시(파티클 시스템이 중력 적용 — RigidBody API 아님)
    mass=0.052,          # 52g
    pillow_density=1.0,  # ★절차적 베개 사용(폭 조절 위해). 1.0=5mm격자(USD와 동일)
    pillow_hx=0.06,      # ★폭 12cm(16→12, 0.75배 — 사용자). 그리퍼 span 11cm 근처
    pillow_hy=0.086,     # ★길이도 비율 축소(23→17.25cm, 0.75배 — 사용자)
)


def enable_gpu_dynamics(stage, scene_path="/physicsScene"):
    """변형체(파티클/FEM)는 GPU 전용 → my_world.play() 전에 호출. 기존 PhysicsScene 있으면 재사용."""
    scene = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
    if scene is None:
        scene = UsdPhysics.Scene.Define(stage, scene_path).GetPrim()
    s = PhysxSchema.PhysxSceneAPI.Apply(scene)
    s.CreateEnableGPUDynamicsAttr().Set(True)
    s.CreateBroadphaseTypeAttr().Set("GPU")
    return scene.GetPath()


def enable_deformable_beta():
    """새 deformable(beta) API(UsdPhysics.DeformableBodyAPI) 사용 필수 설정. SimulationApp 생성 후 호출."""
    import carb
    import omni.physx.bindings._physx as _pb
    carb.settings.get_settings().set_bool(_pb.SETTING_ENABLE_DEFORMABLE_BETA, True)


def _load_weld_triangulate(usd_path, tol=1e-4):
    """USD 메시 로드 → 근접 정점 용접 + 삼각화(particle cloth는 삼각형+용접 정점 필요). 반환 (points, tri_indices)."""
    src = Usd.Stage.Open(usd_path)
    sm = next(UsdGeom.Mesh(p) for p in src.Traverse() if p.IsA(UsdGeom.Mesh))
    pts = sm.GetPointsAttr().Get(); fvc = sm.GetFaceVertexCountsAttr().Get(); fvi = sm.GetFaceVertexIndicesAttr().Get()
    remap = {}; newpts = []; keymap = {}; inv = 1.0 / tol
    for i, p in enumerate(pts):
        k = (round(p[0] * inv), round(p[1] * inv), round(p[2] * inv))
        if k in keymap:
            remap[i] = keymap[k]
        else:
            keymap[k] = len(newpts); remap[i] = len(newpts); newpts.append(p)
    tris = []; idx = 0
    for c in fvc:
        f = [remap[fvi[idx + k]] for k in range(c)]; idx += c
        for k in range(1, c - 1):
            a, b, cc = f[0], f[k], f[k + 1]
            if a != b and b != cc and a != cc:
                tris += [a, b, cc]
    return [Gf.Vec3f(*p) for p in newpts], tris


def add_snack_stand(stage, prim_path, center_xy, base_z,
                    width=0.164, depth=0.14, height=0.127):
    """과자봉지 거치대 = 삼각 프리즘(직육면체 대각선 컷, 옆에서 직각삼각형). 봉지를 비스듬히 기대 세움.
    가로 width(x), 밑변 depth(y), 높이 height(z). 직각=뒤-아래, 빗변=앞-아래→뒤-위(봉지가 빗변에 기댐).
    정적 콜라이더(convexHull)로 봉지가 위에 안착. center_xy=(cx,cy) 밑면 중심, base_z=바닥판 윗면 z."""
    cx, cy = center_xy
    hw, D, H = width / 2.0, depth, height
    y0, y1 = cy - D / 2.0, cy + D / 2.0   # 앞(y0)·뒤(y1)
    # 단면(y,z): A=앞아래(y0,base), B=뒤아래(y1,base), C=뒤위(y1,base+H). 직각@B, 빗변 A-C
    pts = [
        Gf.Vec3f(cx - hw, y0, base_z), Gf.Vec3f(cx - hw, y1, base_z), Gf.Vec3f(cx - hw, y1, base_z + H),  # 좌단 A0,B0,C0
        Gf.Vec3f(cx + hw, y0, base_z), Gf.Vec3f(cx + hw, y1, base_z), Gf.Vec3f(cx + hw, y1, base_z + H),  # 우단 A1,B1,C1
    ]
    # 면: 양끝 삼각 2 + 옆 쿼드 3(밑/뒤/빗변)
    tris = [
        0, 1, 2,  3, 5, 4,                 # 좌단(ABC) / 우단(ACB 역향)
        0, 3, 4, 0, 4, 1,                  # 밑면 A0A1B1B0
        1, 4, 5, 1, 5, 2,                  # 뒷면 B0B1C1C0
        0, 2, 5, 0, 5, 3,                  # 빗변 A0C0C1A1
    ]
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([3] * (len(tris) // 3))
    mesh.CreateFaceVertexIndicesAttr(tris)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.22)])
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    _mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
    _mc.CreateApproximationAttr().Set("convexHull")   # 정적 콜라이더(쐐기=볼록)
    # ★봉지 받침 턱: 빗변 앞-아래 끝(y0)에 얇은 판을 빗변과 평행히(x축 37.4°) 세워 봉지 흘러내림 방지.
    #   사용자 Isaac 에디터 배치값 재현 — pos≈(cx, y0, base+6mm), rotX 37.427°, dims 폭×5.35mm×28.9mm.
    #   ★받침턱 높이 키움(0.01181→0.02892, 2026-06-19 사용자 에디터값 — 봉지 걸림 안정).
    lip = UsdGeom.Cube.Define(stage, prim_path + "/lip")
    lip.CreateSizeAttr(1.0)
    lip.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.22)])
    lipx = UsdGeom.Xformable(lip)
    lipx.AddTranslateOp().Set(Gf.Vec3d(cx, y0, base_z + 0.006))
    lipx.AddRotateXOp().Set(37.427)
    lipx.AddScaleOp().Set(Gf.Vec3f(width, 0.00535, 0.02892))
    UsdPhysics.CollisionAPI.Apply(lip.GetPrim())   # box 정적 콜라이더
    return prim_path


def _make_pillow_mesh(center_half=0.035, density=1.0, edge_half=0.0005, hx=0.08, hy=0.115):
    """베개 메시를 절차적으로 생성(USD 외부파일 불요 — pxr가 SimApp 후에만 되므로 런타임 생성).
    density↑ = 촘촘(작은 유체 입자 담으려면 천 격자도 촘촘해야 누수 방지). 반환 (points[Gf.Vec3f], tri_indices).
    가장자리 두께≈0(두 시트 만남), 중앙 center_half, (1-u²)(1-v²) 테이퍼. make_pillow_cloth.py 포팅."""
    NU, NV = int(32 * density), int(46 * density)

    def half_thick(u, v):
        return edge_half + (center_half - edge_half) * (1 - u * u) * (1 - v * v)

    pts = []
    idx = {}

    def add(i, j, top):
        key = (i, j, top)
        if key in idx:
            return idx[key]
        u = -1 + 2 * i / NU
        v = -1 + 2 * j / NV
        z = half_thick(u, v) * (1 if top else -1)
        idx[key] = len(pts)
        pts.append(Gf.Vec3f(u * hx, v * hy, z))
        return idx[key]

    tris = []

    def quad(a, b, c, d):
        tris.extend([a, b, c, a, c, d])

    for i in range(NU):
        for j in range(NV):
            quad(add(i, j, True), add(i + 1, j, True), add(i + 1, j + 1, True), add(i, j + 1, True))
    for i in range(NU):
        for j in range(NV):
            quad(add(i, j, False), add(i, j + 1, False), add(i + 1, j + 1, False), add(i + 1, j, False))
    loop = []
    for i in range(NU):
        loop.append((i, 0))
    for j in range(NV):
        loop.append((NU, j))
    for i in range(NU, 0, -1):
        loop.append((i, NV))
    for j in range(NV, 0, -1):
        loop.append((0, j))
    for k in range(len(loop)):
        i0, j0 = loop[k]
        i1, j1 = loop[(k + 1) % len(loop)]
        quad(add(i0, j0, True), add(i1, j1, True), add(i1, j1, False), add(i0, j0, False))
    return pts, tris


def add_snack_contents(stage, center_xy, rest_center_z, particle_system_path=None,
                       path="/World/snack_contents", total_mass=0.04,
                       spacing=0.035, group=1, half=(0.025, 0.06, 0.0)):
    """봉지 안 알맹이(꼬깔콘) = 작은 강체 구 몇 개(A). 묵직함+비탄성(저반발)+형상유지(서로 밀려 자리잡음).
    PBD granular는 cm 스케일서 cloth와 같이 터져서 강체로. cloth 입자가 강체와 충돌(non_particle_collision)."""
    from pxr import UsdShade
    cx, cy = center_xy
    hx, hy, _ = half
    zc = rest_center_z - 0.006        # 봉지 중심 약간 아래, ±0.035 안쪽
    radius = spacing * 0.45           # 인접 구 안 겹침
    # 저반발·마찰 재질(비탄성)
    mat_path = "/World/Physics_Materials/chip_mat"
    if not stage.GetPrimAtPath(mat_path):
        UsdShade.Material.Define(stage, mat_path)
        mp = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(mat_path))
        mp.CreateRestitutionAttr().Set(0.0)        # 안 튕김
        mp.CreateDynamicFrictionAttr().Set(0.8)
        mp.CreateStaticFrictionAttr().Set(0.9)
    xs = [cx - hx + i * spacing for i in range(int(2 * hx / spacing) + 1)]
    ys = [cy - hy + j * spacing for j in range(int(2 * hy / spacing) + 1)]
    n = len(xs) * len(ys)
    pmass = total_mass / max(n, 1)
    idx = 0
    for x in xs:
        for y in ys:
            sp = UsdGeom.Sphere.Define(stage, f"{path}/chip_{idx}")
            sp.CreateRadiusAttr(radius)
            sp.CreateDisplayColorAttr([Gf.Vec3f(0.8, 0.55, 0.2)])
            physicsUtils.set_or_add_translate_op(sp, Gf.Vec3f(x, y, zc))
            prim = sp.GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(pmass)
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rb.CreateLinearDampingAttr().Set(0.8)   # 빨리 안정(비탄성 느낌)
            rb.CreateAngularDampingAttr().Set(0.8)
            physicsUtils.add_physics_material_to_prim(stage, prim, mat_path)
            idx += 1
    return path, n


def add_snack_fluid(stage, system_path, center_xy, rest_center_z, fluid_rest_offset,
                    path="/World/snack_fluid", half=(0.065, 0.095, 0.024),
                    total_mass=0.01, particle_group=0):
    """봉지 안 '공기'를 PBD 유체 입자로 채움(비압축 → 스퀴즈시 단단·물풍선식, 압력구속보다 안정).
    봉지 내부 부피를 격자로 샘플. 같은 파티클 시스템(천과 상호작용 → 유체가 천을 밀어 부풀림)."""
    cx, cy = center_xy
    hx, hy, hz = half
    sp = 2.0 * fluid_rest_offset           # 유체 입자 간격
    xs = [cx - hx + i * sp for i in range(int(2 * hx / sp) + 1)]
    ys = [cy - hy + j * sp for j in range(int(2 * hy / sp) + 1)]
    zs = [rest_center_z - hz + k * sp for k in range(int(2 * hz / sp) + 1)]
    pos = [(x, y, z) for x in xs for y in ys for z in zs]
    n = len(pos)
    vel = [(0.0, 0.0, 0.0)] * n
    widths = [2.0 * fluid_rest_offset] * n
    particleUtils.add_physx_particleset_points(
        stage, path=path, positions_list=pos, velocities_list=vel, widths_list=widths,
        particle_system_path=system_path, self_collision=True, fluid=True,
        particle_group=particle_group, particle_mass=total_mass / max(n, 1), density=0.0,
    )
    return path, n


def _spawn_cloth(stage, scene_path, center_xy, rest_center_z, prim_path, params):
    """particle cloth 인플레이터블 봉지(베개 메시). 반환 bag prim path."""
    cx, cy = center_xy
    psys_path = "/World/snackParticleSystem"
    if not stage.GetPrimAtPath(psys_path):
        particleUtils.add_physx_particle_system(
            stage, psys_path, simulation_owner=scene_path,
            contact_offset=params["pco"], rest_offset=params["pco"] * 0.8,
            particle_contact_offset=params["pco"], solid_rest_offset=params["sro"],
            fluid_rest_offset=params.get("fluid_rest_offset", None),   # ★유체 충전 시 필요(봉지 속 공기=유체 입자)
            solver_position_iterations=params.get("solver", 32), enable_ccd=True,   # 비신축(고stretch)엔 ↑ 필요(안정)
            non_particle_collision_enabled=True,   # 바닥/그리퍼 등 강체와 충돌
            max_velocity=params.get("max_velocity", 2.0),     # ★입자 속도 상한(출렁임 억제)
            max_depenetration_velocity=params.get("max_depen_velocity", None),  # ★겹침후 튕김속도 클램프(뾰족 스파이크 억제)
        )
    pmat_path = "/World/Physics_Materials/snack_pbd"
    if not stage.GetPrimAtPath(pmat_path):
        particleUtils.add_pbd_particle_material(
            stage, pmat_path, friction=params["friction"],
            damping=params.get("pbd_damping", 0.0),                    # ★전역 속도 감쇠(출렁임 제거)
            adhesion=params.get("adhesion", 0.0),                      # ★입자-강체 들러붙음(그리퍼 그립·리프트, DexGarmentLab)
            particle_adhesion_scale=params.get("adhesion_scale", 1.0), # 입자간 들러붙음(폴드 안정)
            particle_friction_scale=params.get("friction_scale", 1.0),# ★입자간 마찰(폴드 유지·내부 안정, DexGarmentLab 0.5)
            adhesion_offset_scale=params.get("adhesion_offset_scale", 0.0),  # adhesion fall-off 거리(×rest offset)
            gravity_scale=params.get("gravity_scale", 1.0),           # ★중력 명시(1.0=정상 중력). 입자는 시스템이 중력 적용
            cohesion=params.get("cohesion", None),                    # 유체 입자 응집(공기=낮게)
            viscosity=params.get("viscosity", None),                  # 유체 점성(안정화)
            surface_tension=params.get("surface_tension", None),      # 유체 표면장력
        )
    physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(psys_path), pmat_path)

    if params.get("pillow_density"):   # 절차적 베개(폭/촘촘도 조절 — USD standalone 생성 불가 우회)
        pts, tris = _make_pillow_mesh(center_half=params.get("pillow_center_half", 0.035),
                                      density=params["pillow_density"],
                                      hx=params.get("pillow_hx", 0.08), hy=params.get("pillow_hy", 0.115))
    else:
        pts, tris = _load_weld_triangulate(params.get("mesh_usd", PILLOW_USD))   # 기본=부푸는 베개(평평은 PBD서 안 부풂)
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([3] * (len(tris) // 3))
    mesh.CreateFaceVertexIndicesAttr(tris)
    particleUtils.add_physx_particle_cloth(
        stage=stage, path=prim_path, dynamic_mesh_path=None, particle_system_path=psys_path,
        spring_stretch_stiffness=params["stretch"], spring_bend_stiffness=params["bend"],
        spring_shear_stiffness=params["shear"], spring_damping=params["damping"],
        self_collision=True, self_collision_filter=True, particle_group=0,
        pressure=params["pressure"],
    )
    physicsUtils.set_or_add_translate_op(mesh, Gf.Vec3f(cx, cy, rest_center_z))
    UsdPhysics.MassAPI.Apply(mesh.GetPrim()).GetMassAttr().Set(params["mass"])

    # C: 내부 알맹이(꼬깔콘) — 충격흡수(비탄성)+형상유지(소성)+묵직함
    if params.get("contents", False):
        _, _n = add_snack_contents(stage, center_xy, rest_center_z, psys_path,
                                   total_mass=params.get("contents_mass", 0.04),
                                   spacing=params.get("contents_spacing", 0.012))
        print(f"[snack_bag_module] 알맹이 {_n}개 생성(총 {params.get('contents_mass',0.04)*1000:.0f}g)", flush=True)

    # 유체 충전(봉지 속 '공기'=PBD 유체 입자) — 비압축이라 스퀴즈시 단단·안정(압력구속 대체/보완)
    if params.get("fluid_fill", False):
        _fro = params.get("fluid_rest_offset", params["pco"] * 0.6)
        _fp, _fn = add_snack_fluid(stage, psys_path, center_xy, rest_center_z, _fro,
                                   total_mass=params.get("fluid_mass", 0.01),
                                   half=params.get("fluid_half", (0.065, 0.095, 0.024)),
                                   particle_group=1)
        print(f"[snack_bag_module] 유체 입자 {_fn}개 충전(공기, fluid_rest_offset={_fro:.4f})", flush=True)
    return prim_path


def _reshape_to_pillow(skin, hx, hy, center_half, edge_half):
    """쿠킹되는 Sphere 프리미티브의 '점만' 베개(빵빵 몸통+평평 밀봉 가장자리)로 리매핑.
    토폴로지는 검증된 Sphere 것을 그대로 둠(손으로 짠 메시는 쿠커가 invalid 거부) → 모양만 봉지화.
    북/남극→중앙 위/아래(두께 center_half), 적도→직사각 가장자리(밀봉 립 edge_half).
    실제 과자봉지처럼: 중앙 빵빵, 둘레 평평 밀봉. 솔리드 watertight 볼륨이라 tet 쿠킹됨."""
    import math
    sp = skin.GetPointsAttr().Get()
    out = []
    for v in sp:
        n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) or 1.0
        nx, ny, nz = v[0] / n, v[1] / n, v[2] / n
        nz = max(-1.0, min(1.0, nz))
        lat = math.acos(nz)                   # 0=북극(중앙 위), π=남극(중앙 아래)
        s = math.sin(lat)                     # 수평 반경비 0(극)~1(적도=가장자리)
        lon = math.atan2(ny, nx)
        dx, dy = math.cos(lon), math.sin(lon)
        denom = max(abs(dx) / hx, abs(dy) / hy, 1e-9)   # 직사각 경계까지 스케일
        x, y = s * dx / denom, s * dy / denom
        u, w = x / hx, y / hy
        th = edge_half + (center_half - edge_half) * (1 - u * u) * (1 - w * w)
        z = (1.0 if math.cos(lat) >= 0 else -1.0) * th
        out.append(Gf.Vec3f(float(x), float(y), float(z)))
    skin.GetPointsAttr().Set(out)


def _spawn_fem_beta(stage, scene_path, center_xy, rest_center_z, prim_path, params):
    """FEM beta volume deformable 봉지(거동 우선 — 출렁임 없음 + 소성 형상유지). VolumeDeformableDemo 패턴.
    ★enable_deformable_beta()를 먼저 호출. ★소성은 apply_plastic_yield()로 변형 후 rest 갱신.
    ★쿠킹: 절차적 베개(솔기)는 tet 쿠킹 거부 → 데모처럼 클린 sphere 프리미티브를 봉지 비율로 납작 스케일(쿠킹 OK)."""
    import omni.kit.commands
    cx, cy = center_xy
    p = params
    root = UsdGeom.Xform.Define(stage, prim_path)
    physicsUtils.set_or_add_translate_op(root, Gf.Vec3f(cx, cy, rest_center_z))

    # 스킨 = 클린 프리미티브(쿠킹 성공)를 봉지 비율(0.16×0.23×0.07)로 스케일.
    #   Cube=사각 슬랩(봉지에 가까움), Sphere=타원. 베개 셸은 tet 쿠킹 거부라 프리미티브 사용.
    skin_path = prim_path + "/skin"
    _ptype = params.get("fem_prim", "bag")
    HX, HY, HZ = 0.08, 0.115, 0.035
    # 쿠킹되는 클린 프리미티브를 봉지 비율로 변형. bag=Sphere 토폴로지에 superellipsoid 리매핑(둥근 박스).
    _base = "Sphere" if _ptype == "bag" else _ptype
    _, _tmp = omni.kit.commands.execute("CreateMeshPrim", prim_type=_base, select_new_prim=False)
    omni.kit.commands.execute("MovePrim", path_from=_tmp, path_to=skin_path)
    skin = UsdGeom.Mesh.Get(stage, skin_path)
    if _ptype == "bag":
        _reshape_to_pillow(skin, HX, HY,
                           center_half=params.get("fem_center_half", 0.035),
                           edge_half=params.get("fem_edge_half", 0.0012))
    else:
        sp = skin.GetPointsAttr().Get()
        R = max(max(abs(v[0]), abs(v[1]), abs(v[2])) for v in sp) or 1.0   # 프리미티브 로컬 반치수
        skin.GetPointsAttr().Set([Gf.Vec3f(v[0] / R * HX, v[1] / R * HY, v[2] / R * HZ) for v in sp])
    UsdGeom.Xformable(skin.GetPrim()).SetXformOpOrder([])   # 로컬 변환 제거(translate는 root가)
    skin.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.27, 0.18)])

    deformableUtils.create_auto_volume_deformable_hierarchy(
        stage,
        root_prim_path=prim_path,
        simulation_tetmesh_path=prim_path + "/simMesh",
        collision_tetmesh_path=prim_path + "/collMesh",
        cooking_src_mesh_path=skin_path,
        simulation_hex_mesh_enabled=True,
        cooking_src_simplification_enabled=True,
        set_visibility_with_guide_purpose=True,
    )
    root.GetPrim().GetAttribute("physxDeformableBody:resolution").Set(p["fem_resolution"])
    root.GetPrim().ApplyAPI("PhysxBaseDeformableBodyAPI")
    root.GetPrim().GetAttribute("physxDeformableBody:selfCollision").Set(False)

    mat_path = prim_path + "/deformableMaterial"
    deformableUtils.add_deformable_material(
        stage, mat_path, youngs_modulus=p["youngs"], poissons_ratio=p["poisson"],
        dynamic_friction=p["dynamic_friction"], density=p.get("fem_density", 46.0),
    )
    physicsUtils.add_physics_material_to_prim(stage, root.GetPrim(), mat_path)
    return prim_path


def apply_plastic_yield(stage, root_prim_path, sim_mesh_subpath="/simMesh"):
    """소성 근사 — 현재 시뮬 tet 점들을 rest 형상으로 갱신해 변형을 영구화(놓아도 복원 안 됨).
    파지로 변형된 상태에서 호출(예: 그립 유지 중 매 N스텝, 또는 적치 직전 1회).
    ★native plasticity가 없어(Isaac 5.1 전 변형체 탄성) 쓰는 우회. restShapePoints 런타임 갱신 효력은 GPU 검증 필요.
    반환: 갱신 성공 여부."""
    sim = stage.GetPrimAtPath(root_prim_path + sim_mesh_subpath)
    if not sim:
        return False
    cur = UsdGeom.PointBased(sim).GetPointsAttr().Get()   # 현재(변형된) 시뮬 점
    rest_attr = sim.GetAttribute("omniphysics:restShapePoints")
    if not rest_attr or cur is None:
        return False
    rest_attr.Set(cur)
    return True


def rigidify_bag(stage, bag_path="/World/snack_bag",
                 particle_system_path="/World/snackParticleSystem"):
    """봉지 강체화(파지 운반용) — 사용자 아이디어 '집으면 강체 변경'.
    파티클 시스템을 정지(cloth 시뮬 동결 → 봉지가 정적 메시=강체처럼 거동)시키고, 현재 월드 AABB
    중심·치수를 반환한다. 반환값으로 cuRobo Cuboid 프록시를 만들어 attach하면 캔/병과 동일한
    carry/place plan_single이 봉지 부피를 무충돌 인지한다(매대 회피). 물성 복귀는 soften_bag().
    반환: (center_world (x,y,z), dims (dx,dy,dz)) — meter 튜플."""
    ps = stage.GetPrimAtPath(particle_system_path)
    if ps and ps.IsValid():
        PhysxSchema.PhysxParticleSystem(ps).CreateParticleSystemEnabledAttr().Set(False)
    bag = stage.GetPrimAtPath(bag_path)
    bb = UsdGeom.Imageable(bag).ComputeWorldBound(
        Usd.TimeCode.Default(), UsdGeom.Tokens.default_).ComputeAlignedRange()
    mn, mx = bb.GetMin(), bb.GetMax()
    center = ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0)
    dims = (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])
    return center, dims


def soften_bag(stage, particle_system_path="/World/snackParticleSystem"):
    """봉지 물성 복귀 — 파티클 시스템 재활성(적치 직전 호출). cloth가 다시 시뮬되어 거치대 빗면에 안착."""
    ps = stage.GetPrimAtPath(particle_system_path)
    if ps and ps.IsValid():
        PhysxSchema.PhysxParticleSystem(ps).CreateParticleSystemEnabledAttr().Set(True)


def spawn_snack_bag(stage, scene_path, center_xy, rest_center_z,
                    mode="cloth", prim_path="/World/snack_bag", params=None):
    """과자봉지 변형체 생성(머지 단위). enable_gpu_dynamics()를 play() 전에 먼저 호출할 것.
    center_xy=(x,y) m, rest_center_z=봉지 중심 z(world m). 반환: bag prim path."""
    if mode == "cloth":
        p = dict(CLOTH_PARAMS)
        if params:
            p.update(params)
        return _spawn_cloth(stage, scene_path, center_xy, rest_center_z, prim_path, p)
    elif mode == "fem_beta":
        p = dict(FEM_PARAMS)
        if params:
            p.update(params)
        return _spawn_fem_beta(stage, scene_path, center_xy, rest_center_z, prim_path, p)
    raise ValueError(f"알 수 없는 mode: {mode}")
