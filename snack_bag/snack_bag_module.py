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
    pressure=11.0,       # 질소 공기압. 10→11: 옆스퀴즈 진입시 납작 방지·두께 유지(12 근처 폭발 주의)
    stretch=8000.0,      # ★비신축↑(5000→8000): 옆 누르면 부피가 두께로(불룩 강화). 1e4↑는 꿀렁/스파이크
    bend=150.0,          # 과대 bend는 불안정 → 적당히
    shear=50.0,
    damping=4.0,         # 출렁임 제거(8↑은 폭발)
    pco=0.005,           # particle_contact_offset(촘촘 5mm 격자에 맞춤 — 격자보다 크면 입자 과중첩→폭발)
    sro=0.0025,          # solid_rest_offset
    # ★내용물(contents)/소성/B는 포기(2026-06-16) — PBD cm스케일서 cloth+내용물 반복 폭발, 탄성 수용.
    contents=False,
    friction=2.2,        # 그리퍼 그립 마찰(1.6→2.2: 옆스퀴즈 그립 유지력↑, 리프트시 안빠짐)
    pbd_damping=14.0,    # ★전역 입자 속도 감쇠(핀치 에너지 흡수 강화 10→14 — spring_damping↑은 폭발하므로 이걸로)
    max_velocity=1.0,    # ★입자 최대속도 제한(크러시시 폭발 속도 클램프 2→1)
    adhesion=0.0,        # B(접착 구김고정)는 접음 — C(알맹이)가 소성 담당
    adhesion_scale=1.0,
    friction_scale=0.5,  # ★입자간 마찰(폴드 유지·내부 안정, DexGarmentLab)
    gravity_scale=1.0,   # ★중력 명시(파티클 시스템이 중력 적용 — RigidBody API 아님)
    mass=0.052,          # 52g
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


def _spawn_cloth(stage, scene_path, center_xy, rest_center_z, prim_path, params):
    """particle cloth 인플레이터블 봉지(베개 메시). 반환 bag prim path."""
    cx, cy = center_xy
    psys_path = "/World/snackParticleSystem"
    if not stage.GetPrimAtPath(psys_path):
        particleUtils.add_physx_particle_system(
            stage, psys_path, simulation_owner=scene_path,
            contact_offset=params["pco"], rest_offset=params["pco"] * 0.8,
            particle_contact_offset=params["pco"], solid_rest_offset=params["sro"],
            solver_position_iterations=params.get("solver", 32), enable_ccd=True,   # 비신축(고stretch)엔 ↑ 필요(안정)
            non_particle_collision_enabled=True,   # 바닥/그리퍼 등 강체와 충돌
            max_velocity=params.get("max_velocity", 2.0),     # ★입자 속도 상한(출렁임 억제)
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
        )
    physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath(psys_path), pmat_path)

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
