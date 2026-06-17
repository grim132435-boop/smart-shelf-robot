#!/usr/bin/env python3
"""Isaac Sim GUI에 E0509 + RH-P12 로봇을 URDF에서 직접 임포트 — 충돌구체 저작용.

기존 e0509_gripper_isaac.usd는 다른 USD 체인을 참조하는 얇은 래퍼라 단독 로드 시
메시가 안 보였다. cuRobo 문서대로 메시 포함 URDF(e0509_gripper_abs.urdf, 절대경로 .dae)를
URDF Importer로 직접 임포트해 메시가 뷰포트에 확실히 표시되게 한다.

fix_base=True + 물리 스텝 없음(렌더만) → 로봇이 정지 자세 유지, 구체 저작에 안정적.

[구체 에디터 여는 법 — Isaac Sim 5.1]
  1) 메뉴 Window → Extensions → 검색창에 'robot description' →
     'omni.isaac.robot_description_editor'(deprecated 표시) 토글 ON (+ AUTOLOAD 체크)
  2) 메뉴 Isaac Utils (또는 Tools) → Lula Robot Description Editor 등장
  3) 패널에서 Select Articulation = /World/e0509 (임포트된 로봇), Robot URDF 칸에
     /home/devuser/curobo_ws/robots/e0509_gripper/e0509_gripper_abs.urdf 지정
  4) Link Sphere Editor에서 링크별(link_4/5/6·그리퍼) 구체 저작 → Export Robot Description File
  ※ 신규 XRDF Editor(Tools→Robotics→XRDF Editor)도 구체 저작 가능(포맷만 다름).

[Export 후] python ~/shelf_grasp_dev/normalize_lula_spheres.py <export파일>
            → ~/curobo_ws/robots/e0509_gripper/e0509_spheres.yml 갱신.
[종료] 창 닫기 또는: touch /tmp/sphere_editor_stop
"""
import os
import argparse
_ap = argparse.ArgumentParser()
# 기본=URDF 임포트(카메라 없음). --from-usd=실행 USD(v2 씬, RealSense 카메라 포함) 로드 — 런타임에 메시 해석.
_ap.add_argument("--from-usd", action="store_true",
                 help="실행하던 v2 씬 USD를 로드(RealSense 카메라 포함)해 구체 저작. 기본은 URDF 임포트.")
ARGS, _ = _ap.parse_known_args()
FROM_USD = ARGS.from_usd

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.kit.app
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction

# Lula Robot Description Editor(=구체 저작기) 확장을 강제 활성화 → Tools>Robotics 메뉴에 등장.
#   기본 비활성이라 안 켜면 메뉴에 'Lula Test Widget'만 보임.
try:
    _ext = omni.kit.app.get_app().get_extension_manager()
    for _e in ("isaacsim.robot_setup.xrdf_editor",):
        _ext.set_extension_enabled_immediate(_e, True)
    print("[구체에디터] Lula Robot Description Editor 확장 활성화", flush=True)
except Exception as e:
    print(f"[구체에디터] 확장 활성화 실패(무시): {e}", flush=True)

# URDF Importer 인터페이스 (Isaac Sim 버전별 네임스페이스 폴백)
try:
    from isaacsim.asset.importer.urdf import _urdf          # Isaac Sim 2024-10+ / 5.x
except ImportError:
    try:
        from omni.importer.urdf import _urdf                # 2023.1
    except ImportError:
        from omni.isaac.urdf import _urdf                   # 2022.2

URDF_PATH  = "/home/devuser/curobo_ws/robots/e0509_gripper/e0509_gripper_abs.urdf"
STOP_FILE  = "/tmp/sphere_editor_stop"

world = World(stage_units_in_meters=1.0)

cfg = _urdf.ImportConfig()
cfg.merge_fixed_joints      = False
cfg.convex_decomp           = False
cfg.fix_base                = True       # 베이스 고정 → 처짐/추락 없음
cfg.make_default_prim       = True
cfg.self_collision          = False
cfg.create_physics_scene    = True
cfg.import_inertia_tensor   = False
cfg.distance_scale          = 1.0
cfg.density                 = 0.0
try:
    cfg.default_drive_type      = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    cfg.default_drive_strength  = 10000.0
    cfg.default_position_drive_damping = 100.0
except Exception:
    pass

# instanceable 메시는 Lula Robot Description Editor와 비호환(Select Mesh 비어/Generate 실패).
#   임포트 설정에서 끌 수 있으면 끄고, 그래도 남으면 임포트 후 de-instance로 펼친다.
for _attr in ("make_instanceable", "create_instanceable", "instanceable"):
    if hasattr(cfg, _attr):
        try:
            setattr(cfg, _attr, False)
        except Exception:
            pass

V2_USD = "/home/devuser/CoWriteBotRL/models/shelf_workspace_v2.usd"
if FROM_USD:
    # 실행하던 v2 씬을 그대로 참조 → robot(/World/Robot)에 RealSense 카메라 prim 포함.
    #   카메라 메시(rsd455)는 런타임에 Isaac이 해석(오프라인 pxr만 실패, GUI 런타임은 로드됨 — stage7 로그 확인).
    from pxr import Sdf
    _stg = omni.usd.get_context().get_stage()
    _wp = _stg.DefinePrim("/World", "Xform")
    _wp.GetReferences().AddReference(Sdf.Reference(V2_USD, "/World"))
    robot_prim = "/World/Robot"
    print(f"[구체에디터] v2 씬 로드(카메라 포함) → {V2_USD}, robot={robot_prim}", flush=True)
else:
    root_dir = os.path.dirname(URDF_PATH)
    fname    = os.path.basename(URDF_PATH)
    urdf_if  = _urdf.acquire_urdf_interface()
    parsed   = urdf_if.parse_urdf(root_dir, fname, cfg)
    robot_prim = urdf_if.import_robot(root_dir, fname, parsed, cfg, "")

# ── de-instance: 모든 instanceable 프림을 펼쳐 메시가 링크 밑 실제 프림으로 보이게 함.
#    중첩(인스턴스 안 인스턴스)까지 잡도록 더 이상 안 바뀔 때까지 반복.
import omni.usd
_stage = omni.usd.get_context().get_stage()
_total = 0
for _pass in range(12):
    _changed = 0
    for _prim in _stage.Traverse():
        if _prim.IsInstanceable():
            _prim.SetInstanceable(False)
            _changed += 1
    _total += _changed
    if _changed == 0:
        break
print(f"[구체에디터] de-instance 완료: {_total}개 프림 펼침(메시 선택 가능해짐)", flush=True)

# ── 스케일 bake: URDF 메시는 점이 raw mm(~120)이고 0.001 스케일이 Xform에만 걸려 있음.
#    Lula Generate Spheres는 점을 raw로 읽고 get_world_pose의 스케일을 무시 → 1000배 큰 구체 생성.
#    점좌표에 0.001을 곱해 미터화하고, 0.001 스케일 Xform을 1로 바꿔 렌더 크기는 그대로 유지.
from pxr import UsdGeom, Usd, Gf
# ★URDF·v2 USD 둘 다 메시 점이 raw mm(~100)이고 0.001 스케일이 Xform에만 걸려 있음(2026-06-15 v2 확인).
#   Lula Generate Spheres는 점을 raw로 읽고 스케일 무시 → 1000배 큰 구체. 점을 미터화하고 스케일→1로 bake.
if True:
    _SCL = 0.001
    # ★RealSense(rsd455) 에셋은 이미 미터라 bake 대상서 제외 — 안 그러면 점×0.001로 1000배 작아져 안 보임
    #   (2026-06-15 사용자: bake 후 카메라 사라짐). 로봇 STL(raw mm·0.001 스케일)만 미터화.
    def _skip(_p):
        s = str(_p.GetPath())
        return "Realsense" in s or "RSD455" in s
    _nm = 0
    for _p in Usd.PrimRange(_stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if _p.IsA(UsdGeom.Mesh) and not _skip(_p):
            _pa = UsdGeom.Mesh(_p).GetPointsAttr(); _v = _pa.Get()
            if _v:
                _pa.Set([(x * _SCL, y * _SCL, z * _SCL) for (x, y, z) in _v]); _nm += 1
    _nx = 0
    for _p in Usd.PrimRange(_stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if _p.IsA(UsdGeom.Xformable) and not _skip(_p):
            _xf = UsdGeom.Xformable(_p); _tr = Gf.Transform(_xf.GetLocalTransformation())
            _s = _tr.GetScale()
            if any(abs(abs(float(c)) - _SCL) < _SCL * 0.5 for c in _s):
                _tr.SetScale(Gf.Vec3d(1, 1, 1))
                _xf.ClearXformOpOrder(); _xf.AddTransformOp().Set(_tr.GetMatrix()); _nx += 1
    print(f"[구체에디터] 스케일 bake: 메시 {_nm}개 미터화, Xform {_nx}개 스케일→1 (Realsense 제외, 렌더크기 유지)", flush=True)

robot = world.scene.add(Robot(prim_path=robot_prim, name="e0509"))
world.reset()

# ── Play를 눌러도 정지하도록: 중력 OFF + 드라이브 타겟=home(현재자세) + 적당한 kp/kd.
#    게인 0(이전 시도)은 댐핑이 없어 미세속도가 안 멈추고 로봇이 영원히 회전함 → 금지.
#    home을 타겟으로 잡고 댐핑을 줘야 속도가 감쇠해 정지함. 타겟0이면 home에서 스냅해 폭발.
import numpy as np
try:
    robot.disable_gravity()
    print("[구체에디터] 중력 OFF", flush=True)
except Exception as e:
    print(f"[구체에디터] disable_gravity 실패(무시): {e}", flush=True)
try:
    nd = robot.num_dof
    q_home = np.asarray(robot.get_joint_positions(), dtype=np.float32)
    ctrl = robot.get_articulation_controller()
    ctrl.set_gains(kps=np.full(nd, 2000.0, dtype=np.float32),
                   kds=np.full(nd, 200.0, dtype=np.float32))
    ctrl.apply_action(ArticulationAction(joint_positions=q_home))   # 타겟=home → 그 자세 유지
    print(f"[구체에디터] 드라이브 타겟=home, kp2000/kd200 (DOF {nd}) → Play 눌러도 정지", flush=True)
except Exception as e:
    print(f"[구체에디터] 드라이브 안정화 실패(무시): {e}", flush=True)

# ── RealSense RSD455 placeholder: 카메라 USD 에셋(rsd455)이 S3 전용이라 인터넷 없으면 메시가
#    렌더 안 됨(Realsense prim은 있으나 빈 Xform). 보이는 프록시 박스를 띄워 구체 저작 기준으로 삼는다.
#    from-usd: 기존 Realsense prim의 자식으로 붙여 그 prim의 실제 변환(translate 0.047,0,0.04·orient -90Y)을
#              그대로 상속 → 정확한 카메라 위치·방향에 렌더. URDF 모드: base-local 추정치(0.06,0,0.03).
try:
    if FROM_USD:
        raise RuntimeError("from-usd: 실제 RealSense 카메라 메시 렌더(bake 제외) → 프록시 박스 불필요")
    from pxr import UsdGeom, Gf, Sdf
    _cam_path = f"{robot_prim}/gripper_rh_p12_rn_base/RSD455_placeholder"
    _local_t  = Gf.Vec3d(0.06, 0.0, 0.03)   # base-local 추정 (URDF 모드, 카메라 prim 없음)
    _where    = "base-local 0.06,0,0.03(추정)"
    _cube = UsdGeom.Cube.Define(world.stage, _cam_path)
    _cube.GetSizeAttr().Set(1.0)
    _xf = UsdGeom.Xformable(_cube)
    _xf.ClearXformOpOrder()
    _xf.AddTranslateOp().Set(_local_t)
    _xf.AddScaleOp().Set(Gf.Vec3f(0.124, 0.029, 0.026))    # RSD455 W×H×D
    _cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.95, 0.15, 0.15)])
    _cube.GetDisplayOpacityAttr().Set([0.45])
    print(f"[구체에디터] RSD455 프록시 박스 생성 → {_cam_path} ({_where}, 124x29x26mm 반투명)", flush=True)
    print("[구체에디터] ★이 빨간 박스가 카메라 — 위에 구체 저작.", flush=True)
except Exception as e:
    print(f"[구체에디터] placeholder 생성 실패(무시): {e}", flush=True)

print(f"[구체에디터] URDF 임포트 완료 → 프림 {robot_prim} (메시 포함)", flush=True)
print(f"[구체에디터] 구체 저작: Play(▶) → Tools→Robotics→Lula Robot Description Editor → Select Articulation={robot_prim}", flush=True)
print(f"[구체에디터] 종료: 창 닫기 또는 touch {STOP_FILE}", flush=True)

if os.path.exists(STOP_FILE):
    os.remove(STOP_FILE)

while simulation_app.is_running():
    simulation_app.update()          # 렌더/UI만 (물리 스텝 X → 정지 유지)
    if os.path.exists(STOP_FILE):
        print("[구체에디터] stop 파일 감지 → 종료", flush=True)
        break

simulation_app.close()
