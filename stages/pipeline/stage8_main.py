#!/usr/bin/env python3
# Stage7: Stage5(추종 정밀화·다물체) + Stage6(그리퍼 정밀화) 병합본 — 2026-06-12. 도메인 랜덤화 베이스.
"""
Stage 7: E0509 + RH-P12-RN GraspGen→cuRobo 다물체 픽앤플레이스 (Isaac Sim GUI) — 통합 베이스

병합(stage6_merge_notes.md): stage5를 베이스로 stage6 고유분 2개만 단방향 이식.
  · stage5 승계: 중력보상 피드포워드(j2 추종 4.55°→0.17°) + 구체-장애물 간격 로깅(ESDF) +
                moveL settle=0 핸드오프 블렌딩 + 다물체 오케스트레이션(--objects/--obj-gap, 2열 슬롯)
  · stage6 이식: ① 매대 안 부분열림 릴리즈(GRIP_RELEASE, 측벽·이웃캔 간섭 회피)
                ② --gripper-test 진단모드(빈손 형상 판별, 운영 무관 게이트)
  · 공유분(손가락 SDF collision·마찰 combine=max·curl 로깅)은 양 파일 동일 → stage5분 그대로 유지.
이후 Stage7(도메인 랜덤화)은 이 파일에서 진행. 실행: ./run_stage7.sh / 종료: touch /tmp/stage7_stop
"""

try:
    import isaacsim
except ImportError:
    pass

import torch
_ = torch.zeros(4, device="cuda:0")

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--cycles", type=int, default=3)
parser.add_argument("--port", type=int, default=5556, help="GraspGen ZMQ 서버 포트")
parser.add_argument("--obj-type", default="cylinder", choices=["box", "cylinder", "bottle", "snack"])
parser.add_argument("--mixed", action="store_true",
                    help="혼합 씬: 캔1(→2층)+페트병1(→3층) 동시 배치·정리(객체별 타입/층 전환). --place 필요")
parser.add_argument("--force-gpu", action="store_true",
                    help="[진단] 봉지 없이도 GPU dynamics 강제 ON — 캔 단독에서 GPU+SDF 관통/회전 버그 재현용")
parser.add_argument("--rigid-bag", action="store_true",
                    help="[B안] 봉지를 particle cloth 대신 강체 베개메시로 → 씬 전체 CPU 물리(캔/병 CCD로 깨끗한 파지). cloth 변형 없음")
parser.add_argument("--dr-pos", action="store_true",
                    help="혼합 3물체(캔·병·봉지) 책상 spawn 위치를 로봇 도달영역 내 비겹침 무작위(매 실행 DR)")
parser.add_argument("--inspect-shelf", action="store_true",
                    help="매대(/World/Shelf) 하위 prim 월드위치+bbox 출력 후 종료 (좌표 확보용)")
parser.add_argument("--no-graspgen", action="store_true",
                    help="GraspGen 없이 고정 탑다운(기존 Step1 동작)으로 실행")
parser.add_argument("--place", action="store_true",
                    help="첫 grasp+lift 성공 시 캔을 3번(맨 위) 매대에 배치(cuRobo 충돌회피 시연) 후 정지")
parser.add_argument("--gripper-test", action="store_true",
                    help="빈손 그리퍼 open/풀클로즈 형상·각도 판별 실험(팔 home 고정, 실물 사진 대조용) 후 HALT")
parser.add_argument("--shelf-level", type=int, default=3, choices=[2, 3],
                    help="적치 매대 층(Stage7). 3=맨위 개방(기본), 2=중간(천장1.11·개구16cm, 캔 진입 스파이크)")
parser.add_argument("--grasp-frac", type=float, default=0.7,
                    help="side 파지 높이(캔중심 기준 half 배율). 0.7=상단부(기본,책상클리어), 0=중앙(천장클리어). 2층용 하향 스파이크")
parser.add_argument("--can-pose", default="upright", choices=["upright", "lying"],
                    help="캔 초기 자세(단일물체). upright=직립(기본), lying=눕힘(축 y, z=반경). 누운캔 파지 견고성 테스트")
parser.add_argument("--viz-spheres", action="store_true",
                    help="cuRobo 충돌구체를 /World/cspheres에 라이브 시각화(책상아래 빨강)")
parser.add_argument("--clutter", type=int, default=0,
                    help="타겟 양옆에 정적 클러터 캔 N개 배치(Phase2: 이웃=장애물 회피 검증). 기본 0")
parser.add_argument("--obj-dist", type=float, default=0.50,
                    help="캔 생성 거리(robot base로부터 +x, m). 0.58+이면 +x 정면 side 파지 IK 가능(도달맵 d≥0.45)")
parser.add_argument("--target-dy", type=float, default=0.0,
                    help="타겟 캔의 측면(y) 오프셋(m). 0이 아니면 옆쪽 캔을 픽 타겟으로(파란색 표시). 클러터 사이 측면 파지 데모")
parser.add_argument("--snack-only", action="store_true",
                    help="--mixed에서 캔·병 스킵, 봉지만 실행(빠른 적치 튜닝용)")
parser.add_argument("--objects", type=int, default=1,
                    help="[Phase3 다물체] 픽 대상 캔 개수. 1=기존 단일(기본). N>1이면 y줄 스폰 → 순차 슬롯 적재(--place 필요)")
parser.add_argument("--obj-gap", type=float, default=0.15,
                    help="[Phase3] 다물체 캔 y 간격(m). 검증범위 [-0.2,+0.2] 안에서 배치")
parser.add_argument("--dr", action="store_true",
                    help="[Phase6] 도메인 랜덤화: 물체 위치+yaw 자세를 매 사이클 무작위(타입→층 라우팅과 함께). 조명/텍스처는 제외")
parser.add_argument("--lying-yaw", type=float, default=0.0,
                    help="누운 캔(--can-pose lying)을 수평면에서 추가 yaw(도) 회전. 예 45 → 축이 45°로 누움(누운픽 강건성 테스트)")
parser.add_argument("--all-lying", action="store_true",
                    help="--can-pose lying에서 짝수=직립/홀수=눕힘 대신 '전부 눕힘'으로 spawn")
parser.add_argument("--spread-axis", default="y", choices=["x", "y"],
                    help="다물체(--objects N) 생성 위치 변화 축. y=좌우(기본), x=전후(로봇 거리방향). gap=--obj-gap")
args = parser.parse_args()
if args.objects > 1 and (not args.place or args.obj_type not in ("cylinder", "bottle")):
    parser.error("--objects N>1은 --place + --obj-type cylinder 전용(다물체 슬롯 적재)")
if args.mixed and not args.place:
    parser.error("--mixed(캔+병 혼합)은 --place 필요")

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "width": "1920",
    "height": "1080",
})

import sys, os, time
import numpy as np
import trimesh
import trimesh.transformations as tra
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, PhysxSchema, Gf
from omni.physx.scripts import deformableUtils, physicsUtils   # 과자봉지 FEM 변형체용

# [모듈화] 이 파일은 stages/pipeline/. 같은 폴더의 pp_* 모듈 + grasp_viz(shelf_grasp_dev/gripper/) 경로 추가.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))           # stages/pipeline/
_PARENT   = os.path.dirname(_THIS_DIR)                            # stages/
_ROOT     = os.path.dirname(_PARENT)                              # shelf_grasp_dev/
for _p in (_THIS_DIR, _PARENT, _ROOT, os.path.join(_ROOT, "gripper")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
from grasp_viz import draw_grasp_candidates_usd, clear_grasp_viz_usd
# [1단계 모듈화] 순수 파지기하·누운픽 → pp_geometry. (런타임 전역 의존 없는 함수만 분리)
from pp_geometry import (make_bottle_mesh, robotiq_grasp_to_rhp12, grasp_to_world,
                         is_in_workspace, pregrasp_from_grasp, snap_grasp_roll_90,
                         side_grasp_from_approach, synthesize_side_grasp_rhp12,
                         lying_grasp_from_axis, can_is_lying, ROBOTIQ_TO_RHP12_Z)
# [2단계 모듈화] 단계별 로직(실행순) → pp_phases. 현재: 그랩젠 생성(query_graspgen).
from pp_phases import query_graspgen

import carb
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.utils.stage import add_reference_to_stage

from curobo.geom.sdf.world import CollisionCheckerType, CollisionQueryBuffer
from curobo.geom.types import WorldConfig, Cuboid
from curobo.geom.sphere_fit import SphereFitType
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.util.logger import setup_curobo_logger
from curobo.util.usd_helper import UsdHelper
from curobo.util_file import get_world_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
# [1단계 모듈화] 모션 실행·간격코어 → pp_motion (omni/curobo 필요 → SimApp·curobo import 이후에 로드).
from pp_motion import (mat4_to_curobo_pose, xyz_to_curobo_pose, log_arm_deg, execute_plan,
                       set_gripper, move_direct_ik, lowest_sphere_bottom_world,
                       highest_sphere_top_world, min_world_clearance, make_clearance_fn,
                       _clr_probe, _clr_report, _CLEARANCE, move_linear_ik,
                       _ROBOT_BASE_OFFSET, set_base_offset)

# ── E0509 설정 경로 ──────────────────────────────────────────────────────────
ROBOT_DIR   = "/home/devuser/curobo_ws/robots/e0509_gripper"
ROBOT_YML   = f"{ROBOT_DIR}/e0509_gripper.yml"
ROBOT_USD   = "/home/devuser/e0509_gripper_isaac/e0509_gripper_isaac.usd"
# 사용자 제작 매대 씬 (로봇 /World/Robot + Table + Shelf + base 큐브 포함)
V2_USD      = "/home/devuser/CoWriteBotRL/models/shelf_workspace_v2.usd"
# ★환경 한곳에서만 로드(사용자): V2 환경 + 거치대(snack_stand)를 결합한 에셋. 거치대 매 런 절차생성 폐기.
#   거치대 위치는 이 에셋에서 편집(GUI) → 런타임에 /World/snack_stand 위치를 읽어 봉지 적치에 사용.
ENV_USD     = "/home/devuser/shelf_grasp_dev/assets/shelf_workspace_v2_stand.usd"
ROBOT_PRIM  = "/World/Robot"            # v2 안의 E0509 prim
# RH-P12 그리퍼: 1-DOF(gripper_rh_r1), open≈0.0 / close≈1.101 rad
GRIP_OPEN   = 0.0
GRIP_CLOSE  = 1.05
# 매대 안 릴리즈용 부분 열림(Stage6 이식): 풀오픈(0.0)은 패드 간격 107mm + 근위바 좌우 스윙으로
#   매대 측벽/이웃캔 간섭 위험 → 캔(66mm)만 놓을 만큼만 연다.
#   내면 간격 = 2*(0.008 + 0.0494cos q − 0.0285sin q − 0.0039) → q=0.35에서 81.5mm(캔+양측 7.7mm).
GRIP_RELEASE = 0.35


def grip_angle_for_gap(gap_m):
    """RH-P12 내면 간격(gap_m)을 만드는 닫힘각 q(rad) 근사. gap = 2*(0.0041 + 0.0494cos q − 0.0285sin q).
    ★GRIP_CLOSE=1.05는 간격 ~8mm(드라이브 타겟) → 60mm 캔을 계속 뚫고 들어가 과침투·회전.
    물체 지름에서 약간만 압축한 간격을 만드는 각으로 닫아야 접촉 직전 정지(과침투 방지)."""
    import math as _m
    def _gap(q):
        return 2.0 * (0.0041 + 0.0494 * _m.cos(q) - 0.0285 * _m.sin(q))
    lo, hi = 0.0, 1.10          # 단조감소 구간 이분탐색
    for _ in range(40):
        mid = (lo + hi) / 2.0
        if _gap(mid) > gap_m:   # q↑ → gap↓
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


# side 파지 높이 배율(캔중심 기준 half). 0.7=상단(책상클리어)/0=중앙(천장클리어). [Stage7 2층 스파이크]
GRASP_HEIGHT_FRAC = args.grasp_frac
# 누운 캔 픽: 타겟별 can_is_lying()로 자동 감지(혼합 다물체 지원) — 전역 플래그 폐기.
# RH-P12 EE(gripper_rh_p12_rn_base) → 파지점(TCP, 손가락 사이) 깊이 (approach +Z방향, m).
#   RH-P12 손가락 기하(base프레임): 손가락끝 z≈0.11, 그립면 z≈0.05~0.11, 중간≈0.08.
#   0.1034(robotiq depth)는 캔을 손가락 끝에 놓아 닫을때 캐밍으로 빠짐 → 손가락 중간(0.078)에
#   깊숙이 seat해야 측면 그립이 됨 (2026-06-05 물리파지 디버깅으로 확정).
#   ★간이 그리퍼오프셋(2026-06-10): 0.060은 너무 깊이 집어 캔이 기울어져 잡힘 → +0.05m(50mm 덜 깊이).
#     0.160(+100mm)은 그리퍼가 캔에 안 닿아 실패 → 0.110로 절충. 정밀 오프셋은 Stage6.
RHP12_TCP_DEPTH = 0.110
# ROBOTIQ_TO_RHP12_Z 는 pp_geometry로 이동(상단 import). 두 그리퍼 TCP깊이 거의 같아 ≈0.

# ── 3번(맨 위) 매대 배치 (월드, m. cuRobo 장애물 실측: 바닥top 1.14, 앞턱top 1.15, 천장 없음) ──
#   ★중요: 캔은 TCP(손가락 사이)보다 grip_z_offset(≈0.02m=0.3·half)만큼 아래에서 잡힘.
#     따라서 '캔 바닥'을 원하는 높이에 두려면 TCP center z = 캔바닥 + half + grip_z_offset.
#     이 오프셋을 무시하면(이전 버그) 캔 바닥이 바닥판보다 ~1.5cm 아래로 박혀 캔이 기울어 눕음.
#   동작: 매대 앞 충분히 띄운 곳(PRE)→ +y 진입 → -z 안착 → open → -y 후퇴 (캔이 매대에 안 닿게).
SHELF3_X         = 0.25
SHELF3_APPROACH  = [0.0, 1.0, 0.0]   # +y(매대 안쪽), 그리퍼 X=위 → 캔 직립
# ── Stage7 매대 층 라우팅: 상태머신은 SHELF3_* 상수만 참조하므로 값만 바꾸면 place 전체가 해당 층으로.
#    [Phase6] 타입→층 자동 라우팅(사용자 정책): 캔(cylinder)→2층, 페트병(bottle)·과자봉지(snack)→3층.
#    실측(--inspect-shelf 2026-06-12): 3층=바닥1.14/앞턱1.15/천장없음, 2층=바닥0.95/앞턱0.96/천장1.11(개구0.16m).
_TYPE_LEVEL = {"cylinder": 2, "bottle": 3, "snack": 3, "box": 3}
SHELF_LEVEL = _TYPE_LEVEL.get(args.obj_type, args.shelf_level)
print(f"[층 라우팅] obj_type={args.obj_type} → 매대 {SHELF_LEVEL}층 (캔→2, 병/봉지→3)", flush=True)
if SHELF_LEVEL == 2:
    SHELF3_FLOOR_TOP = 0.95
    SHELF3_LIP_TOP   = 0.96
    SHELF_CEIL       = 1.11          # 2층 천장 아랫면(그리퍼 최고점 간섭 판정용)
    SHELF3_ENTRY_CLR = 0.005         # 2층은 앞턱이 1cm뿐 → 띄움 최소(많이 띄우면 캔 윗면이 천장 침범)
else:
    SHELF3_FLOOR_TOP = 1.14          # 3단 바닥판 윗면(world)
    SHELF3_LIP_TOP   = 1.15          # 앞턱 윗면(world)
    SHELF_CEIL       = None          # 3층 개방(천장 없음)
    SHELF3_ENTRY_CLR = 0.04          # 진입 시 캔바닥을 앞턱 위로 띄우는 여유(턱 간섭 방지)
# [Phase6] 천장 있는 층의 자동 파지높이(grasp_frac)는 CYLSPEC 정의 후 계산(아래 '자동 파지높이' 참조).
SHELF3_PRE_Y     = 0.22              # 앞접근 y: 앞턱(y≈0.37)에서 충분히 앞 → 진입 전 캔이 매대에 안 닿음
SHELF3_IN_Y      = 0.50              # 내부 y: 앞턱 뒤(안쪽)
SHELF3_REST_CLR  = 0.003            # 안착 시 캔바닥-바닥판 여유(살짝 띄워 박힘 방지)
# [Phase3] 다물체 슬롯 (x, in_y) — 2열 지그재그. 한 열 직선피치 0.11은 그리퍼 스윕폭(~0.07 half)이
#   이웃 캔/벽과 간섭(multiobj_run1: x0.14 하강 -3.5mm 침투)하고, x≥0.36은 도달한계(IK 실패).
#   → 뒷열 좌(0.165,0.56)·앞열 우(0.34,0.44)·앞열 중(0.21,0.44): 쌍별 거리 ≥0.128, 벽 여유 ≥0.088,
#   깊은 슬롯부터 적치(후속 하강 경로가 기적치 캔 옆을 0.125+ 띄워 지남). 단일 모드는 기존 (0.25,0.50).
SHELF3_SLOTS     = ([(SHELF3_X, SHELF3_IN_Y)] if args.objects <= 1 else
                    [(0.165, 0.56), (0.34, 0.44), (0.21, 0.44)])
SLOT_MIN_DIST    = 0.125            # 슬롯-기적치 캔 최소 중심거리(그리퍼 스윕 + 캔 반경 + 여유)
GRIP_Z_OFFSET_EST = 0.047           # 파지 전 슬롯 사전검사용 grip_z_offset 추정(실측값으로 본검사)

# ── 물리 상수 ─────────────────────────────────────────────────────────────────
CUBE_SIZE = 0.05
CUBE_MASS = 0.15
CUBE_Z    = CUBE_SIZE / 2     # 0.025m (테이블 위)

# ── E0509 retract 포즈 ────────────────────────────────────────────────────────
# 홈/retract 포즈: 파지하기 편한 자세 (사용자 지정 joint1~6 = -35,45,80,65,115,-40 deg)
RETRACT_CONFIG = [-0.610865, 0.785398, 1.396263, 1.134464, 2.007129, -0.698132]
# 비전 탐지 자세: 물체 파지 전 카메라가 테이블을 바라보는 중간 포즈 (deg→rad)
import math as _math_init
PRODUCT_VIEW_CONFIG = [_math_init.radians(v) for v in [0.0, -36.0, 56.0, 5.0, 110.0, 0.0]]
# E0509 관절 한계 (URDF, rad, joint_1..6) — cu_js 시작상태 클램프용(INVALID_START 방지)
# j4·5는 URDF 한계수정(자세 정밀화)과 일치 — j4 ±180°, j5 [0,+135°](손목 위). 시작상태 클램프 일관(INVALID_START 방지)
ARM_JOINT_LOWER = np.array([-6.2832, -1.6581, -2.3562, -3.1416,  0.0000, -6.2832], dtype=np.float32)
ARM_JOINT_UPPER = np.array([ 6.2832,  1.6581,  2.3562,  3.1416,  2.3562,  6.2832], dtype=np.float32)

# ── 제어/관찰 파일 ─────────────────────────────────────────────────────────────
# stop-sentinel: 이 파일이 생기면 graceful 종료(kill 불필요 → CUDA UVM 오염 없음).
#   닫기: touch /tmp/stage7_stop  (다른 stage 창과 독립 종료)
STOP_FILE = "/tmp/stage7_stop"
# 뷰포트 스크린샷 저장 폴더 (창이 안 보여도 결과를 PNG로 확인 가능)
SHOT_DIR  = "/home/devuser/shelf_grasp_dev/logs/shots"

# ── GraspGen 파라미터 (stage3에서 이식) ──────────────────────────────────────
PREGRASP_STANDOFF    = 0.10        # ★사용자: 접근이 너무 뒤에서 시작(로봇 자세 불합리) → 0.15→0.10(파지점 가까이).
                                   #   0.04(직행)은 메시 그리퍼가 톨 보틀에 닿아 밀어냄+IK_FAIL이라 과도, 0.10이 접근·도달 균형.
# RENDER_EVERY 는 pp_motion으로 이동(모션 렌더 솎기에서 사용).
APPROACH_Z_MAX       = -0.90       # top 파지 수직도 임계(월드 Z성분, -1=완전수직)
APPROACH_Z_MAX_RELAX = -0.80       # fallback 완화 임계
SIDE_APPROACH_Z_MAX  = 0.85        # side 후보 채택 임계(approach z 절대값). 스냅이 수평 보장하므로
                                   #   필터는 '거의 top-down' 제외용(<0.85≈수평~58°). 출력은 항상 수평.
NUM_PC_POINTS        = 2048
# 물체별 사양: box=snack 근사, cylinder=can 근사. grasp_mode: top/side
OBJ_SPECS = {
    "box":      {"z": CUBE_SIZE / 2,                        "grasp_mode": "top"},
    "cylinder": {"z": 0.0675, "radius": 0.03,  "height": 0.135, "grasp_mode": "side"},   # 캔
    "bottle":   {"z": 0.1125, "radius": 0.035, "height": 0.225, "grasp_mode": "side"},   # 페트병(파워에이드600ml)
    "snack":    {"z": 0.035, "height": 0.07, "grasp_mode": "top"},   # 과자봉지 FEM(베개 0.16×0.23×0.07), squish top
}
# 실린더 계열(캔/페트병)의 현재 활성 spec — 하드코딩 대체(args 모듈최상위라 사용가능).
CYLSPEC = OBJ_SPECS["bottle"] if args.obj_type == "bottle" else OBJ_SPECS["cylinder"]
# [Phase6] 자동 파지높이: 중앙 고정 대신, 천장 있는 층이면 '천장 클리어되는 최대 grasp_frac'을 계산.
#   grasp_frac↑ → grip_z_offset↑ → 진입 시 그리퍼가 높아짐 → 천장여유↓.
#   2층 실측 캘리브(can): 천장여유(mm) ≈ 24.5 − 53.6·frac (frac0=+24.5, frac0.7=−13).
#   여유 margin 이상 되는 최대 frac을 선택(파지는 가능한 높게=책상 클리어↑, 천장은 안전).
#   3층(SHELF_CEIL=None)은 0.7 유지. --grasp-frac 명시 시 사용자값 존중.
if SHELF_CEIL is not None and abs(args.grasp_frac - 0.7) < 1e-6:
    # ★2층은 내부가 낮고(≈16cm), 그리퍼 위 카메라(RSD455)가 캔 윗면보다 높아 천장에 닿음.
    #   moveL 진입은 충돌검사가 없으니 진입높이로 클리어해야 함 → 최저 진입(중앙 파지, frac=0)으로
    #   고정해 카메라 여유 최대화(캔윗면 여유 +24.5mm, 캘리브 24.5−53.6·frac).
    _CEIL_MARGIN_MM = 24.5
    GRASP_HEIGHT_FRAC = 0.0
    print(f"[자동 파지높이] 천장 z={SHELF_CEIL} → grasp_frac={GRASP_HEIGHT_FRAC:.2f} "
          f"(중앙 파지=최저 진입; 카메라 천장간섭 최소화)", flush=True)
# [페트병] 키가 커서(22.5cm) 상단(0.7)은 넥/어깨를 잡음 → 몸통 중앙 파지로 낮춤(사용자 요청).
#   --grasp-frac 명시 시 사용자값 존중.
elif args.obj_type == "bottle" and abs(args.grasp_frac - 0.7) < 1e-6:
    GRASP_HEIGHT_FRAC = 0.0
    print(f"[파지높이] 페트병 → 몸통 중앙(grasp_frac=0.0; 키 커서 상단=넥)", flush=True)

class GS:
    IDLE          = "IDLE"
    QUERY_GRASP   = "QUERY_GRASP"   # GraspGen 추론+선택+시각화
    PLAN_PREGRASP = "PLAN_PREGRASP"
    MOVE_PREGRASP = "MOVE_PREGRASP"
    PLAN_GRASP    = "PLAN_GRASP"
    MOVE_GRASP    = "MOVE_GRASP"
    PLAN_LIFT     = "PLAN_LIFT"
    MOVE_LIFT     = "MOVE_LIFT"
    HOLD          = "HOLD"
    RELEASE       = "RELEASE"
    # 매대 배치(--place): 운반→삽입→하강→안착→후퇴
    PLAN_CARRY    = "PLAN_CARRY"     # 리프트 자세 → 매대 앞(충돌회피 plan_single)
    INSERT_SHELF  = "INSERT_SHELF"   # 매대 안으로 +y 진입(직접IK)
    LOWER_SHELF   = "LOWER_SHELF"    # 캔 바닥까지 하강(직접IK) + 그리퍼 open
    RETREAT_SHELF = "RETREAT_SHELF"  # -y 매대 밖 직선 이탈(moveL, +z 생략) → home은 매대 밖서 시작
    GO_HOME       = "GO_HOME"        # home 복귀(충돌회피 plan_single_js, 갱신된 cu_js로)
    HALT          = "HALT"   # 진단 후 정지(장면 유지, 재플래닝 안 함)


# ── 텍타임 계측: 물체별 grasp/carry/place/home 구간 측정 (3종 적치 비교, Phase0) ──
class TactTimer:
    """물체별 단계 경계 타임스탬프로 구간시간을 낸다. mark(key,event)로 경계 기록.
    이벤트 순서: grasp(시작)→carry(운반 시작)→place(하강 시작)→home(복귀 시작)→done(완료).
    구간: grasp=grasp→carry, carry=carry→place, place=place→home, home=home→done, total=grasp→done."""
    def __init__(self):
        self.marks = {}      # key -> {event: perf_counter}
        self.order = []      # key 등장 순서
        self.reported = False

    def mark(self, key, event):
        if key not in self.marks:
            self.marks[key] = {}; self.order.append(key)
        self.marks[key][event] = time.perf_counter()

    def _dur(self, key, a, b):
        d = self.marks.get(key, {})
        return (d[b] - d[a]) if (a in d and b in d) else None

    def report(self):
        if self.reported or not self.order:
            return
        self.reported = True
        def _f(v):
            return f"{v:8.2f}" if v is not None else f"{'-':>8}"
        print("\n===== 텍타임 요약 (초) =====", flush=True)
        print(f"{'물체':<16}{'grasp':>8}{'carry':>8}{'place':>8}{'home':>8}{'total':>8}", flush=True)
        lines = ["object,grasp,carry,place,home,total"]
        for key in self.order:
            g = self._dur(key, "grasp", "carry"); c = self._dur(key, "carry", "place")
            p = self._dur(key, "place", "home");  h = self._dur(key, "home", "done")
            t = self._dur(key, "grasp", "done")
            print(f"{key:<16}{_f(g)}{_f(c)}{_f(p)}{_f(h)}{_f(t)}", flush=True)
            lines.append(",".join([key] + [(f"{v:.3f}" if v is not None else "") for v in (g, c, p, h, t)]))
        try:
            _logs = os.path.dirname(SHOT_DIR)
            os.makedirs(_logs, exist_ok=True)
            _path = os.path.join(_logs, f"tact_{int(time.time())}.csv")
            with open(_path, "w") as _fp:
                _fp.write("\n".join(lines) + "\n")
            print(f"  [텍타임] CSV 저장: {_path}", flush=True)
        except Exception as _e:
            print(f"  [텍타임] CSV 실패: {_e}", flush=True)


_TACT = TactTimer()


def get_ee_world_pos(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    T = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array([T[3][0], T[3][1], T[3][2]])


def set_kinematic(stage, path, enabled):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(enabled)


def set_gripper_friction(stage, material_prim_path="/World/Physics_Materials/cube_mat"):
    """RH-P12 손가락 collision prim에 고마찰 머티리얼 바인딩. 캔만 고마찰이면 두 물체 유효마찰
    (조합)이 부족해 미끄러짐 → 손가락도 고마찰로 그립 신뢰성↑. (r1/r2/l1/l2 collision)."""
    from pxr import UsdShade
    mat_prim = stage.GetPrimAtPath(material_prim_path)
    if not mat_prim.IsValid():
        print("  [그리퍼마찰] 머티리얼 prim 없음", flush=True); return
    mat = UsdShade.Material(mat_prim)
    n = 0
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if "gripper_rh_p12_rn_" in p and any(f in p for f in ("_r1", "_r2", "_l1", "_l2")):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                try:
                    UsdShade.MaterialBindingAPI.Apply(prim)
                    UsdShade.MaterialBindingAPI(prim).Bind(
                        mat, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                        materialPurpose="physics")
                    n += 1
                except Exception as e:
                    print(f"  [그리퍼마찰] {p} 실패: {e}", flush=True)
    print(f"  [그리퍼마찰] {n}개 손가락 collision에 고마찰 바인딩", flush=True)


def set_finger_sdf_collision(stage, resolution=256):
    """RH-P12 손가락 collision 근사를 convexHull→SDF로 교체 (Stage6 슬립 대책 B1).
    convexHull은 오목한 손가락 패드 안쪽을 메워 캔과 점접촉 → 미끄러짐·손끝 펴짐의 형상 원인.
    SDF는 오목면을 보존해 실물처럼 감싸쥐는 면접촉 재현. (USD 진단 2026-06-12: physics.usd
    /colliders/gripper_rh_*에 approx=convexHull 명시 확인. robotis_lab/Isaac 정석=SDF)"""
    from pxr import PhysxSchema
    fingers = ("gripper_rh_p12_rn_r1/", "gripper_rh_p12_rn_r2/",
               "gripper_rh_p12_rn_l1/", "gripper_rh_p12_rn_l2/")
    n = 0
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if "/collisions" not in p or not any(f in p for f in fingers):
            continue
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr().Set("sdf")
            sdf = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            sdf.CreateSdfResolutionAttr().Set(resolution)
            # ★contact/rest offset: 손가락도 조기 접촉(2cm)·rest(0.5cm) → 캔 표면 못 파고듦
            try:
                pc = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                pc.CreateContactOffsetAttr().Set(0.02)
                pc.CreateRestOffsetAttr().Set(0.0)
            except Exception:
                pass
            n += 1
    # ★CCD: 손가락 링크 바디에 연속충돌검출 — 닫는 손가락이 캔을 한 스텝에 뚫는 터널링 방지
    _cc = 0
    for _lk in ("gripper_rh_p12_rn_r1", "gripper_rh_p12_rn_r2",
                "gripper_rh_p12_rn_l1", "gripper_rh_p12_rn_l2"):
        _lp = stage.GetPrimAtPath(f"/World/Robot/{_lk}")
        if _lp and _lp.IsValid():
            try:
                PhysxSchema.PhysxRigidBodyAPI.Apply(_lp).CreateEnableCCDAttr().Set(True)
                _cc += 1
            except Exception:
                pass
    print(f"  [그리퍼SDF] 손가락 collision {n}개 convexHull→SDF(res={resolution}) + offset + CCD {_cc}개", flush=True)


def disable_visual_collision(stage, name_filter="gripper_rh_p12_rn"):
    """렌더 전용 /visuals 메시에 잘못 붙은 CollisionAPI 비활성화 (URDF 임포트 아티팩트).
    /visuals는 동적바디라 triangle→convexHull로 폴백되어, 손가락 SDF /collisions 위에 '오목 틈을
    메우는 convex 덩어리'가 덧씌워짐 → 강체(캔)가 그 보이지 않는 덩어리에 박혀 끼임(GPU 솔버서
    깊은 관통 복구 불가 → 열어도 안 빠짐). /visuals 콜리전을 끄면 SDF /collisions만 남아 정상 감싸쥠.
    (NVIDIA 권고: 시각 전용 메시는 콜리전 미부착. IsaacLab Discussion #2651 동일 증상.)
    name_filter=None이면 전체 /visuals 대상. 기본은 그리퍼만(외과적)."""
    from pxr import UsdPhysics
    n = 0
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if not p.endswith("/visuals"):
            continue
        if name_filter and name_filter not in p:
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
            n += 1
    print(f"  [콜리전정리] /visuals 콜리전 {n}개 비활성화(중복 convexHull 덩어리 제거 — 끼임 방지)", flush=True)


def toggle_table_collision(stage, enabled, table_root="/World/Table"):
    """책상 콜리전 on/off — 강체 봉지(kinematic 비주얼 그립) top-down 파지 시 그리퍼 손가락이 납작한 봉지를
    잡으러 책상에 박혀 물리 충돌→로봇 흔들림. 봉지 파지~리프트 동안만 책상 콜리전을 꺼 흔들림 제거.
    안전: 이 시점 캔/병은 매대 적치 완료(책상 무관), 봉지는 kinematic(책상 안착 불요)이라 끌 물체 없음.
    cuRobo 플래닝 장애물(별도 월드)은 영향 없음 — 물리 접촉만 토글."""
    from pxr import Usd, UsdPhysics
    root = stage.GetPrimAtPath(table_root)
    if not root or not root.IsValid():
        return
    n = 0
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(bool(enabled))
            n += 1
    print(f"  [책상콜리전] {'ON' if enabled else 'OFF'} ({n}개) — 봉지 파지 흔들림 방지", flush=True)


def toggle_bag_collision(stage, enabled, bag_path="/World/snack_bag"):
    """봉지 콜리전 on/off — kinematic 봉지(불변 강체)에 그리퍼가 close하면 손가락이 convexHull에 깊이 침투→
    솔버가 강하게 밀어내 팔 폭발. 봉지는 kinematic 비주얼 그립(추종)+cuRobo attach 프록시라 물리 콜리전 불요.
    파지 직전 OFF → 폭발 제거. (캔/병 처리 중엔 ON=cuRobo 장애물)."""
    from pxr import UsdPhysics
    p = stage.GetPrimAtPath(bag_path)
    if p and p.IsValid() and p.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr().Set(bool(enabled))
        print(f"  [봉지콜리전] {'ON' if enabled else 'OFF'} — 그리퍼 close 시 kinematic 봉지 관통 폭발 방지", flush=True)


def filter_bag_gripper_collision(stage, robot_prim_path, bag_path="/World/snack_bag"):
    """봉지↔그리퍼 충돌 쌍만 제외(FilteredPairsAPI). 봉지 콜리전은 항상 ON으로 유지(시작 시 쿠킹) →
    파지/운반 시 그리퍼와 안 부딪혀 폭발 없음 + 적치 시 봉지↔거치대/매대 충돌은 살아있어
    그리퍼 오픈+dynamic 전환 시 실제 중력으로 자연 안착(런타임 콜리전 재활성 불필요)."""
    from pxr import UsdPhysics
    bag = stage.GetPrimAtPath(bag_path)
    if not (bag and bag.IsValid()):
        return
    links = ["link_5", "link_6", "gripper_rh_p12_rn_base",
             "gripper_rh_p12_rn_l1", "gripper_rh_p12_rn_l2",
             "gripper_rh_p12_rn_r1", "gripper_rh_p12_rn_r2"]
    fp = UsdPhysics.FilteredPairsAPI.Apply(bag)
    rel = fp.CreateFilteredPairsRel()
    n = 0
    for lk in links:
        p = f"{robot_prim_path}/{lk}"
        if stage.GetPrimAtPath(p).IsValid():
            rel.AddTarget(p); n += 1
    print(f"  [봉지필터] 봉지↔그리퍼 {n}개 링크 충돌 제외(FilteredPairs) — 콜리전 항상 ON 유지", flush=True)


def tune_rigid_grasp_iters(stage, prim_path, pos_iter=192, vel_iter=1, max_depen=5.0):
    """강체 파지 안정(NVIDIA Factory 레시피): 솔버 position iteration↑(관통방지 핵심), velocity
    iteration=1(↑면 접촉이 물러져 수렴 악화 — 기존 vel=8이 오히려 해로웠음), max depenetration 속도.
    GPU TGS 솔버서 그리퍼-강체 깊은 관통/끼임 방지. 출처: IsaacLab factory_env_cfg(pos192/vel1,
    소스 주석 'Important to avoid interpenetration')."""
    from pxr import PhysxSchema, UsdPhysics
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False
    try:
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr().Set(pos_iter)
        rb.CreateSolverVelocityIterationCountAttr().Set(vel_iter)
        rb.CreateMaxDepenetrationVelocityAttr().Set(max_depen)
        rb.CreateEnableCCDAttr().Set(True)   # ★CCD: 닫는 손가락이 캔을 뚫는 터널링 방지(CPU에서 유효)
        # ※각/선 댐핑(50/5)은 240Hz와 함께 캔 5cm 밀림 유발 → 제거. 회전은 다른 레버로.
    except Exception as _e:
        print(f"  [파지튜닝] {prim_path} rigidbody 실패(무시): {_e}", flush=True)
        return False
    # ★contact/rest offset: 캔 collision을 살짝 부풀려(rest 0.5cm) 그리퍼가 기하적으로 끝까지 못 파고들게,
    #   contact 2cm로 조기 접촉 검출(velog 권고값, contact>rest≥0). prim 자체+자식 collision 프림에.
    for cp in [prim] + list(prim.GetChildren()):
        if cp.HasAPI(UsdPhysics.CollisionAPI):
            try:
                pc = PhysxSchema.PhysxCollisionAPI.Apply(cp)
                pc.CreateContactOffsetAttr().Set(0.02)
                pc.CreateRestOffsetAttr().Set(0.005)
            except Exception:
                pass
    return True


_BAKED_BAG = "/home/devuser/shelf_grasp_dev/assets/snack_bag_baked.usd"


def spawn_rigid_bag(stage, prim_path, center_xy, table_top, target_width=0.09):
    """[B안] 봉지를 particle cloth 대신 강체 메시로 생성 → 씬 전체 CPU 가능(캔/병 CCD 파지).
    베이크된 cloth 변형 메시(assets/snack_bag_baked.usd, bottom z=0·중심정렬)를 로드해 자연스러운
    봉지 모양 사용. 그리퍼 span(~10.6cm)이 감싸게 폭을 target_width(9cm)로 스케일. 없으면 절차적 pillow 폴백."""
    from pxr import Gf as _Gf, UsdGeom as _UG, UsdPhysics as _UP
    if os.path.exists(_BAKED_BAG):
        _src = Usd.Stage.Open(_BAKED_BAG)
        _sm = _UG.Mesh(_src.GetPrimAtPath("/snack_bag_baked"))
        _p0 = _sm.GetPointsAttr().Get()
        _ax = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in _p0])
        _w0 = _ax[:, 0].max() - _ax[:, 0].min()
        _sc = float(target_width / _w0) if _w0 > 1e-6 else 1.0   # 폭을 9cm로 맞추는 균일 스케일
        _ax *= _sc
        _ax[:, 2] -= _ax[:, 2].min()   # 스케일 후 bottom 다시 z=0
        m = _UG.Mesh.Define(stage, prim_path)
        m.CreatePointsAttr([_Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in _ax])
        m.CreateFaceVertexCountsAttr([int(c) for c in _sm.GetFaceVertexCountsAttr().Get()])
        m.CreateFaceVertexIndicesAttr([int(i) for i in _sm.GetFaceVertexIndicesAttr().Get()])
        m.CreateDisplayColorAttr([_Gf.Vec3f(0.82, 0.78, 0.25)])
        _dx = _ax[:, 0].max() - _ax[:, 0].min(); _dy = _ax[:, 1].max() - _ax[:, 1].min(); _dz = _ax[:, 2].max() - _ax[:, 2].min()
        _src_desc = f"베이크메시 ×{_sc:.2f} 폭{_dx*100:.0f}×길이{_dy*100:.0f}×두께{_dz*100:.0f}cm"
    else:
        import sys as _s
        if "/home/devuser/shelf_grasp_dev/snack_bag" not in _s.path:
            _s.path.insert(0, "/home/devuser/shelf_grasp_dev/snack_bag")
        from snack_bag_module import _make_pillow_mesh
        pts, tris = _make_pillow_mesh(center_half=0.03, hx=0.045, hy=0.08)
        m = _UG.Mesh.Define(stage, prim_path)
        m.CreatePointsAttr(pts)
        m.CreateFaceVertexCountsAttr([3] * (len(tris) // 3))
        m.CreateFaceVertexIndicesAttr([int(i) for i in tris])
        m.CreateDisplayColorAttr([_Gf.Vec3f(0.85, 0.80, 0.20)])
        _src_desc = "절차적 pillow(베이크 없음 폴백)"
    _UG.Xformable(m.GetPrim()).AddTranslateOp().Set(
        _Gf.Vec3d(float(center_xy[0]), float(center_xy[1]), float(table_top + 0.002)))
    _rbapi = _UP.RigidBodyAPI.Apply(m.GetPrim())
    _rbapi.CreateKinematicEnabledAttr(True)   # ★kinematic 복원(기존 방식): 파지 닫을 때 강체 봉지 튕김/손가락 spin 방지, EE 추종으로 들림
    _UP.CollisionAPI.Apply(m.GetPrim())
    _UP.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr().Set("convexHull")
    _UP.MassAPI.Apply(m.GetPrim()).CreateMassAttr().Set(0.05)
    print(f"  [B안] 강체 봉지 spawn @[{center_xy[0]:.2f},{center_xy[1]:.2f}] — {_src_desc}(convexHull)", flush=True)
    return prim_path


def read_stand_center(stage, path="/World/snack_stand"):
    """환경 에셋(결합본)에 포함된 거치대의 월드 중심(cx,cy)·바닥z를 읽음 — 봉지 적치 타겟에 사용.
    거치대를 에셋에서 GUI로 옮기면 봉지 적치도 자동 추종(하드코딩 좌표 제거). 없으면 None."""
    p = stage.GetPrimAtPath(path)
    if not p or not p.IsValid():
        return None
    bb = UsdGeom.Imageable(p).ComputeWorldBound(
        Usd.TimeCode.Default(), UsdGeom.Tokens.default_).ComputeAlignedRange()
    mn, mx = bb.GetMin(), bb.GetMax()
    return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, mn[2])   # cx, cy, base_z(바닥)


def inspect_prim_tree(stage, root_path, max_depth=3):
    """root 하위 prim들의 월드위치+bbox(min/max) 출력 → 매대 선반 높이/포인트 좌표 확보용."""
    from pxr import UsdGeom
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        print(f"[매대검사] {root_path} 없음", flush=True); return
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    print(f"[매대검사] {root_path} 하위:", flush=True)
    for prim in Usd.PrimRange(root):
        depth = str(prim.GetPath()).count("/") - str(root_path).count("/")
        if depth > max_depth:
            continue
        try:
            w = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            pos = (round(w[3][0], 3), round(w[3][1], 3), round(w[3][2], 3))
            bb = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            mn = (round(bb.GetMin()[0], 3), round(bb.GetMin()[1], 3), round(bb.GetMin()[2], 3))
            mx = (round(bb.GetMax()[0], 3), round(bb.GetMax()[1], 3), round(bb.GetMax()[2], 3))
            print(f"  {'  '*depth}{prim.GetName():28s} pos={pos} bbox=({mn})~({mx})", flush=True)
        except Exception:
            pass


def stabilize_arm_drives(robot_art, arm_names, sim_js_names):
    """E0509 IsaacLab USD는 드라이브 게인이 placeholder(kp=100/kd=1)라 단독 로드 시 출렁임.
    프로젝트 검증값(CoWriteBotRL/e0509_ik_env_v7 ImplicitActuatorCfg, 실기 모사)으로 교체:
      arm(joint_1~6)  : stiffness=800,  damping=80,  effort=200
      gripper(rh_*)   : stiffness=2000, damping=100, effort=50   (물리 파지용 그립힘)
    """
    try:
        view = robot_art._articulation_view
        kp, kd = view.get_gains()
        kp = np.array(kp, dtype=np.float64).reshape(-1)
        kd = np.array(kd, dtype=np.float64).reshape(-1)
        print(f"[게인,전] kp={kp.round(1)}", flush=True)
        print(f"[게인,전] kd={kd.round(1)}", flush=True)
        arm_idx, grip_idx = [], []
        for nm in sim_js_names:
            i = robot_art.get_dof_index(nm)
            if nm in arm_names:
                kp[i], kd[i] = 800.0, 80.0;   arm_idx.append(i)
            elif nm.startswith("gripper_rh_"):
                kp[i], kd[i] = 600.0, 80.0; grip_idx.append(i)   # 컴플라이언트: 캔 들이받아 관통/발사 방지(2000→600)
        view.set_gains(kps=kp, kds=kd)
        if arm_idx:
            view.set_max_efforts(values=np.array([200.0]*len(arm_idx)),
                                 joint_indices=np.array(arm_idx))
        if grip_idx:
            view.set_max_efforts(values=np.array([25.0]*len(grip_idx)),
                                 joint_indices=np.array(grip_idx))
        print(f"[게인,후] arm kp=800 kd=80 eff=200 / gripper kp=600 kd=80 eff=25 "
              f"(arm {len(arm_idx)}축, gripper {len(grip_idx)}축)", flush=True)
    except Exception as e:
        print(f"[게인] 보정 실패: {e}", flush=True)


def enable_gravity_comp(robot_art, my_world, arm_names):
    """[Stage5] 중력보상 피드포워드: 매 물리스텝 PhysX 일반화 중력토크를 arm 6관절 effort로 인가.
    PD 게인(kp800/kd80)·드라이브 타겟은 유지한 채 effort 채널에 가산(set_joint_efforts→
    set_dof_actuation_forces, 드라이브와 독립) → joint_2 중력처짐(~4°) 제거 목적.
    그리퍼 관절은 0(파지력 간섭 방지). physics callback 1곳 등록으로 execute_plan/moveL/정착 루프 전부 커버."""
    try:
        view = robot_art._articulation_view
        nd = int(view.num_dof)
        arm_idx = np.array([robot_art.get_dof_index(n) for n in arm_names], dtype=np.int64)
        state = {"on": True, "logged": False}

        def _cb(step_size):
            if not state["on"]:
                return
            try:
                g = np.asarray(view.get_generalized_gravity_forces()).reshape(-1)
                eff = np.zeros((1, nd), dtype=np.float32)
                eff[0, arm_idx] = g[arm_idx]
                view.set_joint_efforts(eff)
                if not state["logged"]:
                    state["logged"] = True
                    print(f"  [중력보상] 활성 — arm 중력토크(Nm)={np.round(g[arm_idx], 2)}", flush=True)
            except Exception as e:
                state["on"] = False
                print(f"  [중력보상] 콜백 실패 → 비활성: {e}", flush=True)

        my_world.add_physics_callback("stage5_grav_comp", _cb)
        print("  [중력보상] physics callback 등록", flush=True)
        return state
    except Exception as e:
        print(f"  [중력보상] 등록 실패: {e}", flush=True)
        return None


# 로봇 베이스 받침 블록 (V1 /base Cube 스펙: center·size 그대로)
ROBOT_BASE_BLOCK_POS  = [-0.25, -0.04, 0.715]
ROBOT_BASE_BLOCK_SIZE = [0.18, 0.22, 0.03]
ROBOT_BASE_BLOCK_PATH = "/World/robot_base_block"


def fix_scene(stage):
    """씬 보정: (1) 로봇 받침 블록 추가(V2엔 없어 로봇이 떠 보임),
    (2) 책상/매대를 정적으로(움직임 방지), (3) 매대 공중부양 → 책상 윗면에 안착."""
    # (1) 로봇 받침 블록 (V1 /base와 동일) — 정적 큐브. ★결합 에셋에 이미 있으면 건너뜀(중복방지).
    if stage.GetPrimAtPath(ROBOT_BASE_BLOCK_PATH).IsValid():
        print(f"[씬] 받침 블록 이미 존재(환경 에셋) → 생성 건너뜀", flush=True)
    else:
        try:
            from omni.isaac.core.objects import cuboid as _cub
            _cub.FixedCuboid(
                prim_path=ROBOT_BASE_BLOCK_PATH, name="robot_base_block",
                position=np.array(ROBOT_BASE_BLOCK_POS, dtype=np.float32),
                scale=np.array(ROBOT_BASE_BLOCK_SIZE, dtype=np.float32),
                color=np.array([0.35, 0.35, 0.38]),
            )
            print(f"[씬] 로봇 받침 블록 추가 {ROBOT_BASE_BLOCK_POS} size={ROBOT_BASE_BLOCK_SIZE}", flush=True)
        except Exception as e:
            print(f"[씬] 받침 블록 실패: {e}", flush=True)

    # (2) 책상·매대 정적 고정 (움직임 방지)
    for p in ["/World/Table", "/World/Shelf"]:
        try:
            set_kinematic(stage, p, True)
        except Exception:
            pass

    # (2b) 매대 물리 collider 추가: v2 USD 매대 geom엔 CollisionAPI가 없어(책상엔 있음)
    #   그리퍼/캔이 매대를 물리적으로 통과 → 놓은 캔이 매대 바닥을 뚫고 떨어짐. 정적 collider로 만든다.
    try:
        from pxr import UsdGeom as _UG, UsdPhysics as _UP
        _nc = 0
        for _pr in Usd.PrimRange(stage.GetPrimAtPath("/World/Shelf")):
            if _pr.IsA(_UG.Mesh) or _pr.IsA(_UG.Cube):
                _UP.CollisionAPI.Apply(_pr)
                if _pr.IsA(_UG.Mesh):
                    _UP.MeshCollisionAPI.Apply(_pr).CreateApproximationAttr().Set("none")  # 정적=삼각메시 OK
                _nc += 1
        print(f"[씬] 매대 collider {_nc}개 적용(그리퍼/캔 통과 방지)", flush=True)
    except Exception as e:
        print(f"[씬] 매대 collider 실패: {e}", flush=True)

    # (3) 매대를 책상 윗면에 안착 (공중부양 수정): bbox로 gap 계산 후 z 평행이동
    try:
        from pxr import UsdGeom, Gf
        def _bb(path):
            pr = stage.GetPrimAtPath(path)
            if not pr.IsValid():
                return None
            r = UsdGeom.Imageable(pr).ComputeWorldBound(
                Usd.TimeCode.Default(), "default").ComputeAlignedRange()
            return r.GetMin(), r.GetMax()
        tbl, shf = _bb("/World/Table"), _bb("/World/Shelf")
        if tbl and shf:
            table_top  = tbl[1][2]
            shelf_bot  = shf[0][2]
            gap = shelf_bot - table_top           # >0 이면 공중부양
            if abs(gap) > 0.003:
                shelf = stage.GetPrimAtPath("/World/Shelf")
                xf = UsdGeom.Xformable(shelf)
                top = xf.GetOrderedXformOps()
                t_op = next((o for o in top
                             if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
                if t_op is None:
                    t_op = xf.AddTranslateOp()
                cur = t_op.Get() or Gf.Vec3d(0, 0, 0)
                t_op.Set(Gf.Vec3d(cur[0], cur[1], cur[2] - gap))
                print(f"[씬] 매대 안착: 책상top={table_top:.3f} 매대bot={shelf_bot:.3f} "
                      f"gap={gap:.3f} → z {-gap:+.3f}m 이동", flush=True)
            else:
                print(f"[씬] 매대 이미 책상에 안착(gap={gap:.3f})", flush=True)
    except Exception as e:
        print(f"[씬] 매대 안착 실패: {e}", flush=True)


def fix_robot_base(stage, robot_prim, base_link="base_link"):
    """로봇 base를 월드에 고정 (중력에 의한 흔들림/처짐 방지).
    stage2는 고정베이스 로봇 USD를 직접 참조 → 안정. v2 로봇은 IsaacLab floating-base라
    명시적으로 base_link↔월드 FixedJoint를 추가해 고정한다."""
    base_path = f"{robot_prim}/{base_link}"
    if not stage.GetPrimAtPath(base_path).IsValid():
        print(f"[고정] base_link 못 찾음: {base_path} → 고정 생략", flush=True)
        return False
    fj_path = f"{robot_prim}/world_fix_joint"
    if stage.GetPrimAtPath(fj_path).IsValid():
        return True
    from pxr import Gf
    # base_link 현재 월드 포즈를 조인트 앵커(body0=월드측)에 설정.
    #   앵커 미설정 시 rest pose가 월드원점이 되어 솔버가 base를 원점으로 당김 → X축 흔들림.
    bp = stage.GetPrimAtPath(base_path)
    xf = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(bp)
    trans = xf.ExtractTranslation()
    quat  = xf.ExtractRotationQuat()         # Gf.Quatd (w,xyz)
    fj = UsdPhysics.FixedJoint.Define(stage, fj_path)
    fj.CreateBody1Rel().SetTargets([Sdf.Path(base_path)])   # body0 비움 = 월드
    fj.CreateLocalPos0Attr().Set(Gf.Vec3f(trans))           # 월드측 앵커 = base 현재 위치
    fj.CreateLocalRot0Attr().Set(Gf.Quatf(quat))            # 월드측 앵커 = base 현재 회전
    fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))         # base_link측 앵커 = 자기 원점
    fj.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    print(f"[고정] 월드↔{base_path} FixedJoint 생성 (앵커={[round(float(t),3) for t in trans]}, 흔들림 방지)", flush=True)
    return True


# 뷰포트 카메라 프레이밍 (로봇 base[-0.25,-0.04,0.73]·캔[0.2,-0.04,0.78]·매대[0.25,0.5]를 한 화면에)
SHOT_CAM_EYE    = [1.15, -0.95, 1.25]   # 전체 팔+구체 보이게(구체 conform 확인용)
SHOT_CAM_TARGET = [0.05, -0.05, 0.85]
def set_scene_camera():
    """기본 원거리 뷰 대신 로봇·캔·매대가 크게 보이도록 persp 카메라 배치."""
    try:
        try:
            from omni.isaac.core.utils.viewports import set_camera_view
        except ImportError:
            from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=SHOT_CAM_EYE, target=SHOT_CAM_TARGET,
                        camera_prim_path="/OmniverseKit_Persp")
        print(f"  [CAM] persp 카메라 배치 eye={SHOT_CAM_EYE} target={SHOT_CAM_TARGET}", flush=True)
    except Exception as e:
        print(f"  [CAM] 배치 실패(무시): {e}", flush=True)


# ── 손목 리얼센스 카메라(eye-in-hand) ────────────────────────────────────────
# 로봇 모델에 이미 내장된 RealSense D455의 RGB 카메라를 사용(합성 카메라 생성 안 함).
#   경로: /World/Robot/gripper_rh_p12_rn_base/Realsense/RSD455/Camera_OmniVision_OV9782_Color
WRIST_CAM_REL = "Realsense/RSD455/Camera_OmniVision_OV9782_Color"   # EE 하위 내장 RGB 카메라
def setup_wrist_camera(stage, ee_prim_path):
    """EE(그리퍼)에 내장된 RealSense RGB 카메라 경로 반환. 없으면 None."""
    def _flip_upright(_cam_prim):
        # 뷰가 180° 뒤집힘 → 광축(로컬 Z) 기준 180° 롤 추가로 바로 세움
        from pxr import UsdGeom as _UG2, Gf as _Gf2
        _xf = _UG2.Xformable(_cam_prim)
        _xf.AddRotateZOp(opSuffix="flip").Set(180.0)
    cam_path = f"{ee_prim_path}/{WRIST_CAM_REL}"
    _cp = stage.GetPrimAtPath(cam_path)
    if _cp.IsValid():
        _flip_upright(_cp)
        print(f"  [CAM] 내장 RealSense 카메라 사용(180° 롤 보정): {cam_path}", flush=True)
        return cam_path
    # 폴백: 내장 카메라 못 찾으면 후보 탐색(이름 변형 대비)
    from pxr import UsdGeom as _UG
    _rs = stage.GetPrimAtPath(f"{ee_prim_path}/Realsense")
    if _rs and _rs.IsValid():
        for p in Usd.PrimRange(_rs):
            if p.GetTypeName() == "Camera" and "Color" in p.GetName():
                print(f"  [CAM] 내장 RealSense 카메라(탐색): {p.GetPath()}", flush=True)
                return str(p.GetPath())
    print(f"  [CAM] 내장 RealSense 카메라 못 찾음: {cam_path}", flush=True)
    return None


def open_wrist_viewport(cam_path):
    """두 번째 뷰포트 창 생성 → 손목 카메라 바인딩. floating 창(사용자가 직접 배치)."""
    try:
        from omni.kit.viewport.utility import create_viewport_window
        win = create_viewport_window(name="RealSense (wrist)", width=640, height=480,
                                     position_x=40, position_y=40, camera_path=cam_path)
        if win:
            win.viewport_api.camera_path = cam_path
            print(f"  [CAM] RealSense 뷰포트 생성 → {cam_path}", flush=True)
        return win
    except Exception as e:
        print(f"  [CAM] RealSense 뷰포트 생성 실패(무시): {e}", flush=True)
        return None


_shot_n = [0]
def save_shot(tag=""):
    """뷰포트를 PNG로 저장 (창이 안 보여도 결과 확인용). 실패해도 파이프라인 진행."""
    try:
        os.makedirs(SHOT_DIR, exist_ok=True)
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        _shot_n[0] += 1
        path = f"{SHOT_DIR}/shot_{_shot_n[0]:03d}_{tag}.png"
        capture_viewport_to_file(get_active_viewport(), path)
        print(f"  [SHOT] 저장: {path}", flush=True)
    except Exception as e:
        print(f"  [SHOT] 실패(무시): {e}", flush=True)


# ── GraspGen 헬퍼 (stage3에서 이식, E0509용으로 적응) ────────────────────────

# [1단계 모듈화] _BOTTLE_PROFILE, make_bottle_mesh → pp_geometry (상단 import).


def sample_object_pc(obj_type="box", n=NUM_PC_POINTS):
    """물체 표면 점구름 (오브젝트 중심 프레임). 실기체에선 RealSense+SAM 점구름으로 교체."""
    if obj_type == "bottle":
        s = CYLSPEC
        mesh = make_bottle_mesh(s["radius"], s["height"])          # 컨투어 페트병
    elif obj_type == "cylinder":
        s = CYLSPEC
        mesh = trimesh.creation.cylinder(radius=s["radius"], height=s["height"])
    else:
        mesh = trimesh.creation.box([CUBE_SIZE] * 3)
    pts, _ = trimesh.sample.sample_surface(mesh, n)
    return pts.astype(np.float32)


# [1단계 모듈화] robotiq_grasp_to_rhp12, grasp_to_world, is_in_workspace, pregrasp_from_grasp → pp_geometry.


def _ik_ok_for_approach(approach, ref, pos, ik_solver, tensor_args, q_now):
    """approach 방향 + ref(닫힘 후보)로 EE 자세를 만들어 IK 성공 여부 반환."""
    approach = approach / (np.linalg.norm(approach) + 1e-9)
    closing  = ref - np.dot(ref, approach) * approach
    if np.linalg.norm(closing) < 1e-6:
        closing = np.array([1.0, 0, 0]) - approach[0] * approach
    closing /= (np.linalg.norm(closing) + 1e-9)
    y = np.cross(approach, closing)
    R = np.column_stack([closing, y, approach])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = pos
    try:
        ik = ik_solver.solve_single(mat4_to_curobo_pose(T, tensor_args),
                                    q_now.view(1, -1), q_now.view(1, 1, -1))
        return bool(ik.success.item())
    except Exception:
        return False


def probe_reachable_approaches(pos, ik_solver, tensor_args, cu_js_seed, base):
    """진단: 이 위치에서 E0509가 낼 수 있는 접근방향 확인 (GraspGen 규약 approach=+Z 사용).
      - top   : 수직 아래 [0,0,-1]
      - side  : 로봇→물체 수평 방향 및 좌우 (옆면 파지용)
    어떤 것도 안 되면 위치 도달불가 or RH-P12 프레임 규약 불일치(회전 오프셋 필요)."""
    q_now = cu_js_seed.position.view(-1)
    horiz = np.array([pos[0] - base[0], pos[1] - base[1], 0.0])   # 로봇→물체 수평
    horiz = horiz / (np.linalg.norm(horiz) + 1e-9)
    left  = np.array([-horiz[1], horiz[0], 0.0])
    tests = {
        "top(아래로)":  np.array([0.0, 0.0, -1.0]),
        "side(정면)":   horiz,
        "side(좌)":     left,
        "side(우)":     -left,
        "side+아래45":  (horiz + np.array([0, 0, -1.0])),
    }
    refs = [np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0])]
    ok = {}
    for name, appr in tests.items():
        ok[name] = any(_ik_ok_for_approach(appr, r, pos, ik_solver, tensor_args, q_now)
                       for r in refs)
    good = [k for k, v in ok.items() if v]
    if good:
        print(f"  [접근 탐침] E0509 가능 방향 ✅ {good} → 규약 OK, 후보 방향만 안 맞아 탈락", flush=True)
    else:
        print(f"  [접근 탐침] 모든 방향 IK 실패 ❌ → 위치 도달불가 or RH-P12 프레임 규약 불일치", flush=True)
    return ok


# [1단계 모듈화] snap_grasp_roll_90, side_grasp_from_approach, synthesize_side_grasp_rhp12,
#   lying_grasp_from_axis, can_is_lying → pp_geometry (상단 import). 누운픽 머지 단위.


def axis_load_cost(q_sol, q_now, retract):
    """파지 자세의 '로봇 축 부하' 근사 비용 (작을수록 좋음).
    동역학 토크 대신 실무 프록시 사용:
      - move : 현재 관절에서의 변화량 합 (이동량/과회전)
      - wrist: 손목(joint5,6) 변화 가중 (6번 과회전 억제 — 사용자 지적)
      - rest : 편안한 retract 자세 이탈 (extended/극단 자세일수록 토크↑)
    """
    dq = (q_sol - q_now).abs()
    move  = float(dq.sum().item())
    wrist = float(dq[4:].sum().item()) if dq.shape[0] >= 6 else 0.0
    rest  = float((q_sol - retract).abs().sum().item())
    return move + 1.5 * wrist + 0.5 * rest, move, wrist, rest


def select_best_reachable_grasp(grasps_world, scores, ik_solver, tensor_args, cu_js_seed,
                                base, retract, top_k=100, approach_z_max=APPROACH_Z_MAX,
                                obj_R=None, grasp_mode="top", max_candidates=12,
                                obj_center=None, obj_half_h=0.0):
    """방향필터(모드별) + IK 성공 후보 중 '축 부하 최소' 파지 선택.
    side 모드: 수평 스냅 후 (1)캔 몸통 높이 범위 내 (2)grasp+pre-grasp 모두 IK 가능 후보만 채택.
    반환: (grasp_4x4, pre_grasp_4x4) or (None, None)."""
    order = np.argsort(scores)[::-1][:top_k]
    print(f"  [IK 필터] mode={grasp_mode}, 상위 {top_k}개 탐색 (모션비용 최소 선택)...", flush=True)
    q_now  = cu_js_seed.position.view(-1)
    passed = []
    # 탈락 사유 카운터 (height=몸통높이밖, pregrasp=pre-grasp IK실패)
    rej = {"workspace": 0, "approach": 0, "closing": 0, "height": 0, "ik": 0, "pregrasp": 0}

    # ── side 모드: 대칭 캔이므로 GraspGen 임의 azimuth 대신 '정면(로봇→캔)±각도'를 합성·스윕 ──
    #    손목 부하 최소·도달 잘 되는 자세 선택 (카메라는 항상 하늘 향함). GraspGen은 top/box용.
    if grasp_mode == "side" and obj_center is not None:
        frontal = np.array([obj_center[0] - base[0], obj_center[1] - base[1], 0.0])
        if np.linalg.norm(frontal) < 1e-6:
            frontal = np.array([1.0, 0.0, 0.0])
        ang0 = np.arctan2(frontal[1], frontal[0])
        # 파지 높이 = 중심 + frac·half. 기본 frac=GRASP_HEIGHT_FRAC(병=0 중앙).
        #   [사용자] 중간 우선이되 모션플래닝상 간섭 시 다른 높이로 → 중간±단계 높이를 모두 후보로 두고
        #   plan_grasp(충돌인지)이 무간섭 후보 선택. 중간 선호는 높이편차 페널티로 부여(중간=페널티0).
        _HEIGHT_PEN = 3.0
        # ★2층(천장층): 중심보다 위 파지는 TCP↑ → 손목이 천장(SHELF_CEIL)에 박힘(사용자 관찰).
        #   → 천장 있으면 위쪽 후보(+) 제외, 중심·아래만 제공해 파지점을 낮춤. 개방 3층은 전부.
        if SHELF_CEIL is not None:
            _hfracs = [GRASP_HEIGHT_FRAC + d for d in (0.0, -0.20, -0.40)]
        else:
            _hfracs = [GRASP_HEIGHT_FRAC + d for d in (0.0, -0.20, 0.20, -0.40, 0.40)]
        _dbg = []   # [진단] (deg, 상태)
        for hfrac in _hfracs:
            grasp_center = np.array([obj_center[0], obj_center[1],
                                     obj_center[2] + hfrac * obj_half_h])
            for deg in [0, -20, 20, -40, 40, -60, 60, -80, 80, -100, 100]:
                ang = ang0 + np.radians(deg)
                a = np.array([np.cos(ang), np.sin(ang), 0.0])
                g_use = side_grasp_from_approach(a, grasp_center, RHP12_TCP_DEPTH)
                if g_use is None:
                    continue
                if not is_in_workspace(g_use[:3, 3], base):
                    rej["workspace"] += 1; continue
                ik = ik_solver.solve_single(mat4_to_curobo_pose(g_use, tensor_args),
                                            q_now.view(1, -1), q_now.view(1, 1, -1))
                if not ik.success.item():
                    rej["ik"] += 1; continue
                pre = pregrasp_from_grasp(g_use, PREGRASP_STANDOFF)
                ikp = ik_solver.solve_single(mat4_to_curobo_pose(pre, tensor_args),
                                             q_now.view(1, -1), q_now.view(1, 1, -1))
                if not ikp.success.item():
                    rej["pregrasp"] += 1; continue
                q_sol = ik.solution.view(-1)[:q_now.shape[0]]
                jcost, _mv, _wr, _rs = axis_load_cost(q_sol, q_now, retract)
                jcost += _HEIGHT_PEN * abs(hfrac - GRASP_HEIGHT_FRAC)   # ★중간 높이 우선(편차 페널티)
                # ★문제4: 이 자세에서 팔 최저 충돌구체가 책상 위로 유지되는지(손목 5,6이 책상에 안 박히게).
                clr = lowest_sphere_bottom_world(getattr(ik_solver, "kinematics", None), q_sol, base)
                _dbg.append((deg, f"h{hfrac:+.2f} OK clr={clr if clr is not None else -9:.3f} jc={jcost:.2f}"))
                passed.append((jcost, 1.0, g_use, pre, deg, g_use[:3, 2].round(2), _wr,
                               (clr if clr is not None else 1e9), hfrac))
        _tt = base[2] - 0.03
        print(f"  [진단:azimuth] table_top={_tt:.3f}, 컷={_tt-0.005:.3f}", flush=True)
        for _d, _s in sorted(_dbg, key=lambda x: x[0]):
            _safe_mark = "  ←책상침범" if _s.startswith("OK") and float(_s.split("clr=")[1].split()[0]) < _tt - 0.005 else ""
            print(f"     deg={_d:+4d}: {_s}{_safe_mark}", flush=True)
        if not passed:
            print(f"  [IK 필터] side 합성 도달 실패. 탈락: 작업공간밖={rej['workspace']}, "
                  f"IK={rej['ik']}, pre-grasp={rej['pregrasp']} (정면±각도 스윕)", flush=True)
            return None, None, []
        # ★문제4 선별: 팔 충돌구체가 책상(table_top) 위로 유지되는 후보만 우선 채택.
        #   그리퍼는 캔(책상 위) 높이라 정상이고, 손목(5,6)이 책상 아래로 내려가는 자세를 배제.
        table_top = base[2] - 0.03
        safe = [p for p in passed if p[7] >= table_top - 0.005]
        if safe:
            # 축부하(jcost)만 prior로 정렬. 장면 의존 편향(정면페널티·클리어런스 가산)은 제거 —
            #   최종 파지점은 plan_grasp(goalset)이 월드충돌·도달로 직접 선택하므로 휴리스틱 편향 불필요
            #   (Phase 2.5 정석). 책상침범은 위 safe 하드필터(p[7]≥table_top)가 이미 보장. (2026-06-12)
            safe.sort(key=lambda x: x[0])
            chosen = safe[0]
        else:
            print("  [IK 필터] ⚠ 모든 side 후보가 책상 침범 → 최대 클리어런스 차선 선택", flush=True)
            passed.sort(key=lambda x: -x[7])
            chosen = passed[0]
        jcost, sc, g_sel, pre_sel, deg, appr, wr, clr, hfrac = chosen
        print(f"  [IK 필터] side 통과 {len(passed)}개(책상위 {len(safe)}개) → azimuth={deg:+d}°, "
              f"높이frac={hfrac:+.2f}(0=중앙), approach={appr}, 축부하={jcost:.2f}(손목={wr:.2f}rad), "
              f"최저구체월드z={clr:.3f}(책상{table_top:.2f})", flush=True)
        # ★Phase1/2: plan_grasp용 후보 묶음(책상위 안전후보, 축부하 낮은 순).
        #   Phase2 walk를 위해 상한 없이 전 azimuth 후보를 넘김 → 양옆 클러터로 ±y가 막혀도
        #   자유 방향(+x 정면=deg0 등)까지 순차로 도달해 무충돌 파지를 찾음. (스윕이 11개라 짧음)
        _pool = sorted((safe if safe else passed), key=lambda x: x[0])
        cands = [p[2] for p in _pool]
        return g_sel, pre_sel, cands
    for rank, idx in enumerate(order):
        g   = grasps_world[idx]
        approach = g[:3, 2]
        closing  = g[:3, 0]
        if grasp_mode == "side":
            if abs(approach[2]) > SIDE_APPROACH_Z_MAX:
                rej["approach"] += 1
                continue
            if obj_center is None:        # 합성에 캔 중심 필요
                rej["height"] += 1
                continue
            # GraspGen 방위만 취해 RH-P12용 side 파지 합성 (회전·깊이·위치 정확)
            g_use = synthesize_side_grasp_rhp12(g, obj_center, RHP12_TCP_DEPTH)
            if g_use is None:             # top-down → side 불가
                rej["approach"] += 1
                continue
            if not is_in_workspace(g_use[:3, 3], base):
                rej["workspace"] += 1
                continue
        else:  # top
            pos = g[:3, 3]
            if not is_in_workspace(pos, base):
                rej["workspace"] += 1
                continue
            if approach[2] > approach_z_max:
                rej["approach"] += 1
                continue
            if abs(closing[2]) > 0.40:
                rej["closing"] += 1
                continue
            g_use = snap_grasp_roll_90(g, obj_R=obj_R)
        cpose = mat4_to_curobo_pose(g_use, tensor_args)
        ik_result = ik_solver.solve_single(cpose, q_now.view(1, -1), q_now.view(1, 1, -1))
        if not ik_result.success.item():
            rej["ik"] += 1
            continue
        # pre-grasp(후퇴 자세)도 도달 가능해야 실제 접근 성공 → 미리 검증(plan 실패 예방)
        pre = pregrasp_from_grasp(g_use, PREGRASP_STANDOFF)
        ik_pre = ik_solver.solve_single(mat4_to_curobo_pose(pre, tensor_args),
                                        q_now.view(1, -1), q_now.view(1, 1, -1))
        if not ik_pre.success.item():
            rej["pregrasp"] += 1
            continue
        q_sol = ik_result.solution.view(-1)[:q_now.shape[0]]
        jcost, _mv, _wr, _rs = axis_load_cost(q_sol, q_now, retract)
        passed.append((jcost, float(scores[idx]), g_use, pre, rank,
                       g_use[:3, 2].round(2), _wr))   # 스냅 후 approach 기록(진단 정확화)
        if len(passed) >= max_candidates:
            break
    if not passed:
        print(f"  [IK 필터] 도달 가능 파지 없음. 탈락사유: "
              f"작업공간밖={rej['workspace']}, approach방향={rej['approach']}, "
              f"closing방향={rej['closing']}, 몸통높이밖={rej['height']}, "
              f"IK실패={rej['ik']}, pre-grasp실패={rej['pregrasp']} (총 {len(order)}개)", flush=True)
        return None, None, []
    print(f"  [IK 필터] 탈락: 작업공간밖={rej['workspace']}, approach={rej['approach']}, "
          f"closing={rej['closing']}, 몸통높이밖={rej['height']}, IK={rej['ik']}, "
          f"pre-grasp={rej['pregrasp']} / 통과={len(passed)}", flush=True)
    passed.sort(key=lambda x: x[0])            # 축 부하 비용 최소 우선
    jcost, sc, g_sel, pre_sel, rank, appr, wr = passed[0]
    print(f"  [IK 필터] 선택 rank={rank+1}, score={sc:.3f}, approach={appr}, "
          f"축부하비용={jcost:.2f} (손목변화={wr:.2f}rad), 후보 {len(passed)}개 중 최소", flush=True)
    return g_sel, pre_sel, []   # top 모드는 기존 plan_single 경로(plan_grasp 후보 없음)


# [1단계 모듈화] _ROBOT_BASE_OFFSET, mat4_to_curobo_pose, xyz_to_curobo_pose → pp_motion (상단 import).
#   _ROBOT_BASE_OFFSET는 main에서 set_base_offset(robot_base)로 in-place 설정.


def diag_pose_fail(tag, pos, quat_wxyz, ik_solver, result, tensor_args):
    """플래닝 실패 원인 진단: MotionGen status + IK 단독 도달 가능성.
    IK 성공인데 plan 실패면 → 충돌/궤적 문제. IK 실패면 → 자세 자체 도달 불가."""
    status = getattr(result, "status", None)
    goal = Pose(
        position=tensor_args.to_device((np.array(pos, dtype=np.float32) - _ROBOT_BASE_OFFSET).reshape(1, 3)),
        quaternion=tensor_args.to_device(np.array([quat_wxyz], dtype=np.float32)),
    )
    try:
        ik = ik_solver.solve_single(goal)
        ik_ok = bool(ik.success.item())
    except Exception as e:
        ik_ok = None
        status = f"{status} / IK예외:{e}"
    if ik_ok is True:
        cause = "IK 도달 가능 → 충돌/궤적(trajopt) 문제로 추정 (충돌객체·시작자세 확인)"
    elif ik_ok is False:
        cause = "IK 자체 실패 → 자세(pos/quat) 도달 불가 (좌표·접근방향·작업공간 확인)"
    else:
        cause = "IK 검사 예외"
    print(f"  [진단:{tag}] status={status}, IK_단독={ik_ok} → {cause}", flush=True)


def reachability_sanity(ik_solver, tensor_args, q_now):
    """E0509 도달 영역 맵 (base 프레임, offset 무관). 캔 높이가 닿는지/어디가 닿는지 진단.
    각 칸: T=top접근(아래로) 도달, S=side접근(수평 전방) 도달, .=불가."""
    from scipy.spatial.transform import Rotation as _Rt
    q = q_now.view(1, -1)
    def ik_ok(pos, appr):
        a = np.array(appr, dtype=float); a /= (np.linalg.norm(a) + 1e-9)
        tmp = np.array([0., 0., 1.]) if abs(a[2]) < 0.9 else np.array([1., 0., 0.])
        x = np.cross(tmp, a); x /= (np.linalg.norm(x) + 1e-9)
        y = np.cross(a, x)
        R = np.column_stack([x, y, a])
        qx = _Rt.from_matrix(R).as_quat()   # xyzw
        pose = Pose(position=tensor_args.to_device(np.array([pos], dtype=np.float32)),
                    quaternion=tensor_args.to_device(np.array([[qx[3], qx[0], qx[1], qx[2]]], dtype=np.float32)))
        try:   # select/probe와 동일 시그니처(3-인자) — cuda_graph 일관성 유지
            return bool(ik_solver.solve_single(pose, q, q.view(1, 1, -1)).success.item())
        except Exception:
            return False
    print("[reach] base프레임 IK 도달맵 (forward d=0.25/0.35/0.45/0.55, TS=top/side):", flush=True)
    for h in [0.6, 0.4, 0.2, 0.05, -0.1, -0.3, -0.5]:
        cells = []
        for d in [0.25, 0.35, 0.45, 0.55]:
            t = 'T' if ik_ok([d, 0, h], [0, 0, -1]) else '.'
            s = 'S' if ik_ok([d, 0, h], [1, 0, 0]) else '.'
            cells.append(t + s)
        print(f"   base_z={h:+.2f} :  " + "   ".join(cells), flush=True)


def draw_cspheres_usd(stage, motion_gen, q_tensor, base, table_top=0.70, quiet=True):
    """cuRobo 로봇 충돌구체를 월드 USD 구로 그림(시각화). 책상 top 아래 구체=빨강(처박음).
    매 호출마다 /World/cspheres를 갱신 → 로봇 따라다니며 책상/매대 간섭을 눈으로 확인."""
    from pxr import UsdGeom, Gf
    try:
        st  = motion_gen.kinematics.get_state(q_tensor.view(1, -1))
        sph = st.get_link_spheres().reshape(-1, 4).detach().cpu().numpy()
        root = "/World/cspheres"
        if stage.GetPrimAtPath(root).IsValid():
            stage.RemovePrim(root)
        UsdGeom.Scope.Define(stage, root)
        n = 0; below = 0; low = None
        for i, (x, y, z, r) in enumerate(sph):
            if r <= 0.0:
                continue
            wx, wy, wz = float(x) + base[0], float(y) + base[1], float(z) + base[2]
            bottom = wz - float(r)
            if low is None or bottom < low[2]:
                low = (wx, wy, bottom)
            sp = UsdGeom.Sphere.Define(stage, f"{root}/s{i}")
            sp.CreateRadiusAttr(float(r))
            UsdGeom.Xformable(sp).AddTranslateOp().Set(Gf.Vec3d(wx, wy, wz))
            if bottom < table_top:          # 책상 top 아래 = 빨강(처박음)
                sp.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)]); below += 1
            else:
                sp.CreateDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.9, 0.2)])  # 초록(정상)
            sp.CreateDisplayOpacityAttr().Set([0.45])
            n += 1
        if not quiet and low is not None:
            print(f"  [구체VIZ] {n}개(책상아래 빨강 {below}개) 최저구체 월드z={low[2]:.3f}", flush=True)
    except Exception as e:
        if not quiet:
            print(f"  [구체VIZ] 실패: {e}", flush=True)


def attach_cspheres_to_links(stage, robot_prim_path, spheres_yaml_path, opacity=0.35):
    """충돌구체를 각 로봇 링크 prim의 자식 USD Sphere로 '1회' 생성. cuRobo 구체는 링크 로컬
    좌표라 그대로 자식으로 붙이면 USD 트랜스폼 계층이 매 렌더 프레임 로봇과 강체로 이동 →
    시각화 lag 0(매 프레임 재계산 불필요). 순수 시각용 — 충돌 회피 로직과 무관(prim은 robot
    서브트리=ignore_substring이라 장애물로도 안 잡힘). 고객 제출용 깔끔 영상."""
    from pxr import UsdGeom, Gf
    data = load_yaml(spheres_yaml_path)["collision_spheres"]
    name2path = {}
    root = stage.GetPrimAtPath(robot_prim_path)
    if root.IsValid():
        for p in Usd.PrimRange(root):          # 깊이우선 → 링크 Xform이 자식 visuals보다 먼저
            name2path.setdefault(p.GetName(), p.GetPath())
    made = 0; miss = []
    for link, spheres in data.items():
        ppath = name2path.get(link)
        if ppath is None:
            miss.append(link); continue
        UsdGeom.Scope.Define(stage, f"{ppath}/cspheres")
        for i, s in enumerate(spheres):
            c = s.get("center"); r = float(s.get("radius", 0.0))
            if r <= 0.0 or c is None:
                continue
            sp = UsdGeom.Sphere.Define(stage, f"{ppath}/cspheres/s{i}")
            sp.CreateRadiusAttr(r)
            UsdGeom.Xformable(sp).AddTranslateOp().Set(Gf.Vec3d(float(c[0]), float(c[1]), float(c[2])))
            sp.CreateDisplayColorAttr().Set([Gf.Vec3f(0.15, 0.55, 0.95)])   # 옅은 파랑(통일)
            sp.CreateDisplayOpacityAttr().Set([float(opacity)])
            made += 1
    print(f"  [구체VIZ] 링크부착 {made}개 생성(자동 강체 추종). 미발견 링크={miss}", flush=True)
    return made


def make_jlim_urdf(src_urdf, dst_urdf, overrides):
    """원본 URDF를 건드리지 않고, 지정 관절의 position 한계만 바꾼 복사본을 생성.
    cuRobo는 URDF에서 위치한계를 읽고 BoundCost가 init에서 clone하므로, 비대칭 한계(예: j5≥0)는
    런타임 텐서 수정이 아니라 MotionGen 생성 전 URDF에서 줘야 반영됨. overrides={joint:(lower,upper)}."""
    import xml.etree.ElementTree as ET
    tree = ET.parse(src_urdf)
    root = tree.getroot()
    hit = []
    for j in root.findall("joint"):
        nm = j.get("name")
        if nm in overrides:
            lim = j.find("limit")
            lo, up = overrides[nm]
            lim.set("lower", repr(float(lo)))
            lim.set("upper", repr(float(up)))
            hit.append(nm)
    tree.write(dst_urdf, encoding="utf-8", xml_declaration=True)
    print(f"  [URDF한계] {dst_urdf} 생성 — 수정 관절 {hit}", flush=True)
    return dst_urdf


# [1단계 모듈화] log_arm_deg, execute_plan → pp_motion (상단 import).


# [1단계 모듈화] set_gripper, move_direct_ik → pp_motion (상단 import).


# [1단계 모듈화] lowest_sphere_bottom_world, highest_sphere_top_world → pp_motion (상단 import).


def log_ceil_headroom(tag, tcp_world, ik_solver, tensor_args, cu_js, base, half_c, grip_off):
    """Stage7 2층 스파이크: 주어진 TCP pose의 IK 해에서 그리퍼 최고 구체 z와 캔 윗면 z를
    매대 천장(SHELF_CEIL)과 비교해 여유(mm)를 출력. SHELF_CEIL=None(3층 개방)이면 건너뜀."""
    if SHELF_CEIL is None:
        return
    try:
        ikr = ik_solver.solve_single(mat4_to_curobo_pose(tcp_world, tensor_args),
                                     cu_js.position.view(1, -1), cu_js.position.view(1, 1, -1))
        if not ikr.success.item():
            print(f"  [천장간격:{tag}] IK 실패 — 이 자세 도달 불가", flush=True); return
        can_top = float(tcp_world[2, 3]) - grip_off + half_c   # 캔중심=TCP−grip_off, 윗면=+half
        c_head = (SHELF_CEIL - can_top) * 1000
        # ★캔 윗면 vs 천장만 본다(기하 정확). 그리퍼-매대 실제 간격은 [간격:moveL](min_world_clearance)이
        #   매대 메시 기준으로 측정하므로 그게 권위값 — 전역 최고구체는 매대 밖 팔이라 오탐.
        print(f"  [캔천장간격:{tag}] 천장={SHELF_CEIL:.3f} 캔윗면={can_top:.3f} 여유 {c_head:+.0f}mm "
              f"{'← 캔이 천장 침범!' if c_head < 0 else 'OK'} (그리퍼-매대 실간격은 [간격:moveL] 참조)", flush=True)
    except Exception as e:
        print(f"  [천장간격:{tag}] 계산 실패: {e}", flush=True)


# [1단계 모듈화] _CLEARANCE, min_world_clearance, make_clearance_fn, _clr_probe, _clr_report → pp_motion.


def slot_feasible(slot_xy, entry_z, place_z, ik_solver, tensor_args, cu_js_pos,
                  motion_gen, used_xy):
    """[Phase3] 슬롯 실현성 사전검사. 좌표 하드코딩 신뢰 대신 매번 검사(multiobj_run1 교훈:
    x0.14=그리퍼-이웃 간섭, x0.36=도달한계). 검사 3종 — ① 기적치 캔과 중심거리 ≥ SLOT_MIN_DIST
    (하강 시 그리퍼 스윕 간섭, 순수 기하), ② 삽입·하강 pose IK(도달한계+관절한계 분기),
    ③ 삽입 pose 자세의 구체-장애물 간격>0 (벽 간섭, 부착 캔 포함). (ok, 사유) 반환."""
    sx, sy = slot_xy
    for ux, uy in used_xy:
        if np.hypot(sx - ux, sy - uy) < SLOT_MIN_DIST:
            return False, f"기적치근접({np.hypot(sx-ux, sy-uy):.3f}m)"
    q_ins = None
    for cz, tag in ((entry_z, "삽입"), (place_z, "하강")):
        Tw = side_grasp_from_approach(SHELF3_APPROACH, [sx, sy, cz], RHP12_TCP_DEPTH)
        ikr = ik_solver.solve_single(mat4_to_curobo_pose(Tw, tensor_args),
                                     cu_js_pos.view(1, -1), cu_js_pos.view(1, 1, -1))
        if not ikr.success.item():
            return False, f"IK불가({tag})"
        if tag == "삽입":
            q_ins = ikr.solution.view(-1)[:cu_js_pos.view(-1).shape[0]]
    c = min_world_clearance(motion_gen, q_ins)
    if c is not None and c <= 0.0:
        return False, f"간격침투(삽입 {c*1000:+.0f}mm)"
    return True, ""


# [1단계 모듈화] move_linear_ik → pp_motion (상단 import).


def main():
    setup_curobo_logger("warn")
    # [혼합 씬] 객체별(캔→2층/병→3층) 타입·spec·층 지오메트리·파지높이를 타겟 선택 시 전환하기 위해
    #   모듈 전역을 main에서 재할당. (단일/다물체 모드는 영향 없음 — args.mixed일 때만 재할당.)
    global CYLSPEC, GRASP_HEIGHT_FRAC, SHELF3_FLOOR_TOP, SHELF3_LIP_TOP, SHELF_CEIL, SHELF3_ENTRY_CLR, SHELF3_SLOTS

    # ── 로봇 설정 로드 ───────────────────────────────────────────────────────
    robot_cfg = load_yaml(ROBOT_YML)["robot_cfg"]
    # 홈 포즈를 파지 친화 자세로 교체 (init 텔레포트 + cuRobo retract 시드 일관)
    robot_cfg["kinematics"]["cspace"]["retract_config"] = list(RETRACT_CONFIG)
    # ★자세 정밀화(측정 기반): 운반 중 손목 플립(j5 +81°→−92°, j4 270° 회전 → 홈 204° 되감김)이 원인.
    #   대칭 position_limit_clip으론 비대칭(j5≥0=손목 위 유지)이 안 되고, 한계는 URDF에서 와 BoundCost가
    #   init에서 clone하므로 → MotionGen 생성 전 URDF 복사본에서 한계만 수정해 비대칭 적용.
    #   j5: [0, +135°]=손목 항상 위(플립 분기 차단, 파지 +81°는 안전). j4: ±180°(270° 과회전 차단).
    _JLIM = {"joint_4": (-3.141592653589793, 3.141592653589793),
             "joint_5": (0.0, 2.356194490192345)}
    _jlim_urdf = make_jlim_urdf(f"{ROBOT_DIR}/e0509_gripper_abs.urdf",
                                "/tmp/e0509_gripper_jlim.urdf", _JLIM)
    robot_cfg["kinematics"]["urdf_path"]          = _jlim_urdf
    robot_cfg["kinematics"]["asset_root_path"]    = ROBOT_DIR
    # 충돌구체: Isaac Sim Lula 에디터로 직접 만든 cuRobo 네이티브 YAML을 그대로 로드(런타임 생성/변환 없음).
    #   갱신 절차: 에디터 Export → normalize_lula_spheres.py 1회 → e0509_spheres.yml 덮어쓰기.
    robot_cfg["kinematics"]["collision_spheres"]  = f"{ROBOT_DIR}/e0509_spheres.yml"
    # cuRobo는 mimic joint(rh_r2/l1/l2)를 joint_data에 안 넣음 → lock_joints에 두면 KeyError.
    # 주 구동 gripper_rh_r1만 lock (mimic은 따라감). Isaac Sim에서 그리퍼는 별도 제어.
    robot_cfg["kinematics"]["lock_joints"]        = {"gripper_rh_r1": 0.0}
    # base_link은 책상 마운트 → 충돌검사 제외(구체 미생성). collision_link_names·self_collision에서도 제거.
    _kin = robot_cfg["kinematics"]
    if "base_link" in _kin.get("collision_link_names", []):
        _kin["collision_link_names"] = [l for l in _kin["collision_link_names"] if l != "base_link"]
    if isinstance(_kin.get("self_collision_ignore"), dict):
        _kin["self_collision_ignore"].pop("base_link", None)
        for _k in _kin["self_collision_ignore"]:
            _kin["self_collision_ignore"][_k] = [x for x in _kin["self_collision_ignore"][_k] if x != "base_link"]
    if isinstance(_kin.get("self_collision_buffer"), dict):
        _kin["self_collision_buffer"].pop("base_link", None)
    # ★[버그수정] 든 물체 attach가 작동하려면 'attached_object' 링크에 사전할당 구체 슬롯이 필요.
    #   (없으면 attach_external_objects_to_robot이 max_spheres=0 → 조용히 False → 든 물체가 cuRobo 충돌모델에
    #    안 들어가 운반/적치 중 매대에 박음). cuRobo 정석: extra_collision_spheres + collision_link_names + self_ignore.
    _kin["extra_collision_spheres"] = {"attached_object": 100}   # 톨 보틀 부피 표현용 충분히
    if "attached_object" not in _kin.get("collision_link_names", []):
        _kin["collision_link_names"] = list(_kin.get("collision_link_names", [])) + ["attached_object"]
    if isinstance(_kin.get("self_collision_ignore"), dict):
        _all_links = [l for l in _kin["collision_link_names"] if l != "attached_object"]
        _kin["self_collision_ignore"]["attached_object"] = _all_links      # 든 물체는 로봇 자기충돌 무시(월드만 회피)
        for _k in _all_links:                                              # 양방향 무시(그리퍼/손목이 든 물체와 닿는 건 정상)
            if "attached_object" not in _kin["self_collision_ignore"].get(_k, []):
                _kin["self_collision_ignore"][_k] = list(_kin["self_collision_ignore"].get(_k, [])) + ["attached_object"]
    if isinstance(_kin.get("self_collision_buffer"), dict):
        _kin["self_collision_buffer"]["attached_object"] = 0.0

    j_names       = robot_cfg["kinematics"]["cspace"]["joint_names"]
    default_cfg   = robot_cfg["kinematics"]["cspace"]["retract_config"]
    ee_link       = robot_cfg["kinematics"]["ee_link"]
    print(f"로봇: E0509, EE={ee_link}, 조인트={j_names}", flush=True)

    # ── Isaac Sim 월드 ───────────────────────────────────────────────────────
    # ※240Hz 실험 결과: 캔이 5cm 밀려나고 그리퍼 과도폐쇄 → 악화. 60Hz 기본으로 복귀(CCD가 터널링 담당).
    my_world = World(stage_units_in_meters=1.0)
    stage    = my_world.stage

    # 사용자 제작 매대 씬(v2) 로드 — 로봇(/World/Robot) + Table + Shelf + base 포함
    # v2는 defaultPrim 메타가 없어 add_reference_to_stage(defaultPrim)가 실패 →
    # Sdf.Reference로 v2 내 "/World" prim을 현재 /World에 명시 참조
    robot_prim_path = ROBOT_PRIM   # /World/Robot
    # World()가 /World prim을 아직 안 만들었을 수 있어 DefinePrim으로 보장 후 참조
    world_prim = stage.DefinePrim("/World", "Xform")
    world_prim.GetReferences().AddReference(Sdf.Reference(ENV_USD, "/World"))   # ★환경+거치대 결합 에셋 한곳 로드
    print(f"환경 씬(v2+거치대) 로드 → /World : {ENV_USD}", flush=True)

    # v2 로봇은 floating-base(IsaacLab) → 중력에 흔들림. base_link를 월드에 고정.
    fix_robot_base(stage, ROBOT_PRIM, base_link="base_link")
    # 씬 보정: 로봇 받침 블록 추가 + 매대 책상 안착 + 책상/매대 정적화
    fix_scene(stage)

    # 매대 좌표 검사 모드: 선반 높이/포인트 출력 후 종료
    if args.inspect_shelf:
        inspect_prim_tree(stage, "/World/Shelf", max_depth=3)
        inspect_prim_tree(stage, "/World/Table", max_depth=2)
        print("[매대검사] 완료 → 종료", flush=True)
        simulation_app.close()
        return

    # v2 좌표 진단: 로봇 base / 테이블 위치 (큐브를 테이블 위에 놓기 위해)
    _robot_base = get_ee_world_pos(stage, ROBOT_PRIM)
    _table_pos  = get_ee_world_pos(stage, "/World/Table")
    _shelf_pos  = get_ee_world_pos(stage, "/World/Shelf")
    print(f"[진단] 로봇 base={None if _robot_base is None else _robot_base.round(3)}, "
          f"Table={None if _table_pos is None else _table_pos.round(3)}, "
          f"Shelf={None if _shelf_pos is None else _shelf_pos.round(3)}", flush=True)
    # 로봇 base 회전 진단: v2에서 로봇이 회전돼 있으면 월드 [1,0,0,0] 탑다운이 안 맞음
    _rp = stage.GetPrimAtPath(ROBOT_PRIM)
    if _rp.IsValid():
        _T = UsdGeom.Xformable(_rp).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        _R = np.array([[_T[0][0], _T[0][1], _T[0][2]],
                       [_T[1][0], _T[1][1], _T[1][2]],
                       [_T[2][0], _T[2][1], _T[2][2]]])
        _euler = Rotation.from_matrix(_R).as_euler("xyz", degrees=True)
        print(f"[진단] 로봇 base 회전(xyz deg)={_euler.round(1)}  "
              f"(0,0,0 이 아니면 탑다운 quat 보정 필요)", flush=True)
    # 물체를 로봇 앞(+x) 테이블 윗면에 배치 (로봇 base z ≈ 테이블 윗면)
    _obj_type = args.obj_type
    _half = (CYLSPEC["height"] / 2 if _obj_type in ("cylinder", "bottle") else CUBE_SIZE / 2)
    # 캔: 책상 위, 로봇 앞 0.50m (사용자 지정 — base와 일정 거리 이상이어야 파지 자세 나옴)
    if _robot_base is not None:
        _cx, _cy = _robot_base[0] + args.obj_dist, _robot_base[1] + args.target_dy
        # 캔은 table1(윗면 월드 0.70 = robot_base 0.73 − 베이스블록 0.03)에 안착.
        #   기존 robot_base_z+half(0.785)는 0.035 띄워 떨어지며 튀고 기울어짐 → 들쭉날쭉(0.73/0.75).
        #   책상면 바로 위(+2mm)에 놓아 드롭 최소화 → 항상 직립 0.75 안착.
        _table_top = _robot_base[2] - 0.03
        _cz = _table_top + _half + 0.002
    else:
        _cx, _cy, _cz = 0.5, 0.0, _half
    # [Phase6 DR] 물체 위치+yaw 랜덤화. nominal은 respawn 기준(드리프트 방지)으로 보존.
    _nom_x, _nom_y = _cx, _cy
    _spawn_yaw_deg = 0.0
    if args.dr:
        _cx = _nom_x + np.random.uniform(-0.03, 0.05)   # x: 검증된 도달·side파지 창 내(과도 시 grasp 불가)
        _cy = _nom_y + np.random.uniform(-0.07, 0.07)
        # 실린더(캔/병)는 yaw 대칭 → yaw 변형 무의미 + 파지창 교란. box/snack(비대칭)에만 yaw 적용.
        _spawn_yaw_deg = float(np.random.uniform(-180.0, 180.0)) if args.obj_type in ("box", "snack") else 0.0
        print(f"[DR] 위치 랜덤화 → [{_cx:.3f},{_cy:.3f}]"
              + (f" yaw={_spawn_yaw_deg:.0f}°" if _spawn_yaw_deg else " (실린더 yaw 대칭→위치만)"), flush=True)
    print(f"[물체] type={_obj_type}, 생성 위치=[{_cx:.3f}, {_cy:.3f}, {_cz:.3f}]", flush=True)

    # 목표 물체: cylinder=캔(세워서 매대 진열 목표), box=스낵 근사
    targets = []   # [Phase3 다물체] [{"obj","path","status":pending|placed|skipped,"reason"}]
    if args.mixed:
        # ── 혼합 씬: 캔2(→2층) + 페트병2(→3층) = 4개를 2×2 사각형 배치(일렬 금지, 객체별 yaw 다양화) ──
        #   객체별 타입/spec/층/파지높이를 타겟선택 시 전환. 층별 2슬롯 누적 적치.
        from omni.isaac.core.objects import cylinder as _cyl
        from omni.isaac.core.prims import RigidPrim as _RigidPrim
        _canspec, _btlspec = OBJ_SPECS["cylinder"], OBJ_SPECS["bottle"]
        _yc = _robot_base[1] + args.target_dy
        _xn, _xf = _cx + 0.05, _cx + 0.19      # 전후 두 줄(_cx=0.25 → 0.30/0.44, forward 0.55/0.69)
        _yl, _yr = _yc - 0.12, _yc + 0.12      # 좌우 두 열
        # ★[--dr-pos] 3물체 spawn을 '물체별 검증위치 주변 지터'로 무작위(매 실행) — 도달 보장.
        #   공용 큰 박스는 펫트병(키 큰 물체) 도달불가 영역 포함 → 스킵됨(B10). 물체별 reach 차이 반영:
        #   캔=관대(±6cm), 펫트병=0.44 근방 타이트(±4cm, x≥0.40 reach), 봉지=±5cm. 세 위치는 원래 떨어져 비겹침.
        _dr_pos = None
        if args.dr_pos:
            def _jit(_cx0, _cy0, _rx, _ry, _xlo, _xhi):
                return (float(np.clip(_cx0 + np.random.uniform(-_rx, _rx), _xlo, _xhi)),
                        float(_cy0 + np.random.uniform(-_ry, _ry)))
            _dr_pos = {
                "cylinder": _jit(0.32, -0.15, 0.06, 0.06, 0.28, 0.42),   # 캔: 앞쪽, 관대
                "bottle":   _jit(0.44, -0.15, 0.03, 0.06, 0.41, 0.46),   # 병: 뒤쪽, x≥0.41 reach
                "snack":    _jit(0.42,  0.07, 0.05, 0.05, 0.36, 0.46),   # 봉지: +y
            }
            print(f"[DR-pos] 3물체 spawn 지터 무작위(도달보장) 캔[{_dr_pos['cylinder'][0]:.2f},{_dr_pos['cylinder'][1]:.2f}] "
                  f"병[{_dr_pos['bottle'][0]:.2f},{_dr_pos['bottle'][1]:.2f}] 봉지[{_dr_pos['snack'][0]:.2f},{_dr_pos['snack'][1]:.2f}]", flush=True)
        def _yaw_quat(_deg):
            _r = np.radians(_deg); return np.array([np.cos(_r / 2), 0.0, 0.0, np.sin(_r / 2)])
        def _pose_quat_z(_spec, _lie, _yaw):
            # 눕힘: 90°(x축) 회전 + yaw(z) → 축이 수평, 안착높이=반경. 직립: yaw만(z), 안착=half.
            if _lie:
                _Rl = Rotation.from_euler('z', _yaw, degrees=True) * Rotation.from_euler('x', 90, degrees=True)
                _ql = _Rl.as_quat()   # xyzw
                return np.array([_ql[3], _ql[0], _ql[1], _ql[2]]), _table_top + _spec["radius"] + 0.002
            return _yaw_quat(_yaw), _table_top + _spec["height"] / 2 + 0.002
        def _apply_roll_damp(_path):   # 누운 실린더 구름 방지(robotis_lab 참고)
            from pxr import PhysxSchema as _PxS
            _rb = _PxS.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(_path))
            _rb.CreateAngularDampingAttr().Set(50.0); _rb.CreateLinearDampingAttr().Set(5.0)
        def _spawn_bottle_prim(_path, _xy, _q, _z, _lie):
            _bm = make_bottle_mesh(_btlspec["radius"], _btlspec["height"])
            _mp = UsdGeom.Mesh.Define(stage, _path)
            _mp.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in _bm.vertices])
            _mp.CreateFaceVertexCountsAttr([3] * len(_bm.faces))
            _mp.CreateFaceVertexIndicesAttr([int(i) for f in _bm.faces for i in f])
            _mp.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.45, 0.95)])
            _bp = _mp.GetPrim()
            _xf2 = UsdGeom.Xformable(_bp)
            _xf2.AddTranslateOp().Set(Gf.Vec3d(float(_xy[0]), float(_xy[1]), float(_z)))
            _xf2.AddOrientOp().Set(Gf.Quatf(float(_q[0]), float(_q[1]), float(_q[2]), float(_q[3])))
            UsdPhysics.RigidBodyAPI.Apply(_bp)
            UsdPhysics.CollisionAPI.Apply(_bp)
            UsdPhysics.MeshCollisionAPI.Apply(_bp).CreateApproximationAttr().Set("convexHull")
            UsdPhysics.MassAPI.Apply(_bp).CreateMassAttr().Set(0.60)
            if _lie:
                _apply_roll_damp(_path)
            return _RigidPrim(_path, name=_path.split("/")[-1])
        # 캔2 + 병2. (type, (x,y), 눕힘?, yaw). --dr이면 위치·자세(서/눕·눕힘yaw) 매 실행 랜더마이즈.
        if args.dr:
            # [Stage7 DR] 4코너에 캔2·병2를 무작위 배정 + 위치 지터 + 서/눕 + 눕힘yaw 랜덤(타입→층 라우팅은 유지).
            #   ★코너·지터를 도달창 안으로 제한(전후 forward≈0.52/0.64, 지터 후도 ≤0.67). 밖이면 unreachable 스킵됨.
            _xn_dr, _xf_dr = _cx + 0.02, _cx + 0.14
            _corners = [(_xn_dr, _yl), (_xn_dr, _yr), (_xf_dr, _yl), (_xf_dr, _yr)]
            _types4  = ["cylinder", "cylinder", "bottle", "bottle"]
            _perm = np.random.permutation(4)
            _layout = []
            for _k, _ti in enumerate(_perm):
                _cx0, _cy0 = _corners[_k]
                _jx = float(np.clip(_cx0 + np.random.uniform(-0.025, 0.025), _cx - 0.02, _cx + 0.16))
                _jy = float(_cy0 + np.random.uniform(-0.04, 0.04))
                _lie = bool(np.random.random() < 0.5)
                _yaw = float(np.random.uniform(-90.0, 90.0)) if _lie else 0.0
                _layout.append((_types4[int(_ti)], (_jx, _jy), _lie, _yaw))
            print("[혼합+DR] 4개 위치·자세(서/눕·눕힘yaw) 랜더마이즈", flush=True)
        else:
            # [3종 데모] 캔1(→2층) + 병1(→3층). 봉지는 아래에서 별도 spawn. --dr-pos면 무작위 위치.
            _can_xy = _dr_pos["cylinder"] if _dr_pos else (_xn, _yl)
            _btl_xy = _dr_pos["bottle"]   if _dr_pos else (_xf, _yl)
            _layout = [
                ("cylinder", _can_xy, False, 0.0),   # 캔 직립 → 2층
                ("bottle",   _btl_xy, False, 0.0),   # 병 직립 → 3층
            ]
        for _ci, (_ty, _xy, _lie, _yaw) in enumerate(_layout):
            _path = f"/World/obj_{_ty}_{_ci}"
            _spec = _canspec if _ty == "cylinder" else _btlspec
            _q, _z = _pose_quat_z(_spec, _lie, _yaw)
            if _ty == "cylinder":
                _ob = _cyl.DynamicCylinder(prim_path=_path, name=f"obj_{_ci}",
                    position=np.array([_xy[0], _xy[1], _z]), orientation=_q,
                    radius=_canspec["radius"], height=_canspec["height"],
                    color=np.array([0.85, 0.1, 0.1]), mass=0.30)
                if _lie:
                    _apply_roll_damp(_path)
                _lvl, _frac = 2, 0.18
            else:
                _ob = _spawn_bottle_prim(_path, _xy, _q, _z, _lie)
                _lvl, _frac = 3, 0.0
            # ★GPU(TGS)서 SDF 손가락-강체 접촉 안정: 솔버 pos192/vel1 + depenetration(Factory레시피, 관통/끼임 억제)
            tune_rigid_grasp_iters(stage, _path)
            targets.append({"obj": _ob, "path": _path, "status": "pending", "reason": None,
                            "type": _ty, "level": _lvl, "spec": _spec, "frac": _frac})
            print(f"[혼합] {_path}: {_ty} {('눕힘 %.0f°' % _yaw) if _lie else '직립'} → {_lvl}층 "
                  f"@[{_xy[0]:.2f},{_xy[1]:.2f}]", flush=True)
        # [Phase3] 과자봉지(cloth) — 3종의 3번째(DR 제외). 캔·병과 다른 spawn(particle cloth)·핸들러(squish→강체전환).
        _snack_cx = _snack_cy = None
        if not args.dr:
            import sys as _sys2
            _sys2.path.insert(0, "/home/devuser/shelf_grasp_dev/snack_bag")
            from snack_bag_module import spawn_snack_bag as _spawn_snack
            _scn0 = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
            _scn_path = str(_scn0.GetPath()) if _scn0 else "/physicsScene"
            _snack_cx, _snack_cy = (_dr_pos["snack"] if _dr_pos else (_xf, _yr))   # 봉지 spawn(--dr-pos면 무작위)
            # ★거치대는 환경 에셋(ENV_USD)에 포함 → 절차생성 안 함. 위치는 에셋에서 읽음(GUI 재배치 자동 추종).
            _sc = read_stand_center(stage)
            _STAND_X, _STAND_Y = (_sc[0], _sc[1]) if _sc else (0.310, 0.530)
            if args.rigid_bag:   # ★B안: 강체 봉지(GPU 불요 → 씬 전체 CPU)
                spawn_rigid_bag(stage, "/World/snack_bag", (_snack_cx, _snack_cy), _table_top)
                filter_bag_gripper_collision(stage, ROBOT_PRIM)   # 봉지↔그리퍼만 충돌 제외(콜리전 항상 ON→자연 안착)
            else:
                _spawn_snack(stage, _scn_path, (_snack_cx, _snack_cy), _table_top + 0.04, mode="cloth")
            print(f"[혼합] 거치대 위치(에셋서 읽음) = [{_STAND_X:.3f},{_STAND_Y:.3f}]", flush=True)
            targets.append({"obj": None, "path": "/World/snack_bag", "status": "pending", "reason": None,
                            "type": "snack", "level": 3, "spec": OBJ_SPECS["snack"], "frac": 0.0,
                            "spawn_xy": (_snack_cx, _snack_cy)})   # 타겟정렬(y오름차순)용 — obj=cloth라 get_world_pose 불가
            print(f"[혼합] 과자봉지 spawn @[{_snack_cx:.2f},{_snack_cy:.2f}] + 거치대(3층)", flush=True)
        # 층별 슬롯·점유. 3층은 거치대(우측)와 안 겹치게 봉지 있을 때 좌측 슬롯 1곳만.
        # ★사용자: 펫트병=(0.165,0.44) 앞쪽, 캔=펫트병과 같은 xy의 2층(z만 낮음, 앞쪽이라 진입 여유↑=눕힘 회피).
        _MIX_LSLOTS = {2: [(0.165, 0.44), (0.34, 0.44)],
                       3: ([(0.165, 0.44)] if not args.dr else [(0.165, 0.56), (0.34, 0.44)])}
        _MIX_LUSED  = {2: [False, False], 3: [False] * len(_MIX_LSLOTS[3])}
        target_cube = targets[0]["obj"]
        args.objects = len(targets)   # 다물체 분기(타겟 순회·적치) 활성화
        if args.snack_only:           # 캔·병 즉시 스킵 → 봉지만 실행(빠른 튜닝용)
            for _t in targets:
                if _t["type"] != "snack":
                    _t["status"] = "placed"; _t["reason"] = "snack_only_skip"
            print("[--snack-only] 캔·병 스킵 → 봉지 바로 실행", flush=True)
        print(f"[혼합] 타겟 {len(targets)}개 — {'캔1(2층)+병1(3층)+봉지1(3층 거치대) 3종' if not args.dr else 'DR 캔2+병2'}", flush=True)
    elif _obj_type in ("cylinder", "bottle") and args.objects > 1:
        # ── Phase3 다물체: 픽 대상 캔 N개를 y줄로 스폰(전부 동적=전부 픽 대상) ──
        from omni.isaac.core.objects import cylinder as _cyl
        s = CYLSPEC
        _tgt_colors = [np.array([0.85, 0.1, 0.1]), np.array([0.1, 0.3, 0.9]),
                       np.array([0.1, 0.7, 0.2]), np.array([0.9, 0.6, 0.1])]
        for _i in range(args.objects):
            _off_i = (_i - (args.objects - 1) / 2.0) * args.obj_gap   # 물체별 spread 오프셋
            if args.spread_axis == "x":
                _px_i, _py_i = _cx + _off_i, _robot_base[1] + args.target_dy   # 전후(거리) 방향 변화
            else:
                _px_i, _py_i = _cx, _robot_base[1] + args.target_dy + _off_i   # 좌우 방향 변화(기본)
            _op = f"/World/obj_{_i}"
            # 혼합 자세(--can-pose lying): 짝수=직립/홀수=눕힘(--all-lying이면 전부 눕힘). 누운 캔은 lying_yaw만큼 수평 회전.
            _lying_i = (args.can_pose == "lying" and (args.all_lying or _i % 2 == 1))
            if _lying_i:
                _Rl = Rotation.from_euler('z', args.lying_yaw, degrees=True) * Rotation.from_euler('x', 90, degrees=True)
                _ql = _Rl.as_quat()                                   # [x,y,z,w]
                _ori_i = np.array([_ql[3], _ql[0], _ql[1], _ql[2]])   # [w,x,y,z]
            else:
                _ori_i = np.array([1.0, 0.0, 0.0, 0.0])
            _cz_i = (_table_top + s["radius"] + 0.002) if _lying_i else _cz
            _ob = _cyl.DynamicCylinder(
                prim_path=_op, name=f"obj_{_i}",
                position=np.array([_px_i, _py_i, _cz_i]), orientation=_ori_i,
                radius=s["radius"], height=s["height"],
                color=_tgt_colors[_i % len(_tgt_colors)], mass=0.30,
            )
            if _lying_i:
                # 누운 실린더는 굴러 도달 밖(x≥0.36)으로 이탈 → 각/선 댐핑으로 제자리 안착(robotis_lab 물체 ~3.0 참고, 강화).
                from pxr import PhysxSchema as _PhysxSchema   # ★뒤쪽 import보다 먼저 써서 로컬 바인딩 필요
                _rb = _PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(_op))
                _rb.CreateAngularDampingAttr().Set(50.0)
                _rb.CreateLinearDampingAttr().Set(5.0)
            targets.append({"obj": _ob, "path": _op, "status": "pending", "reason": None})
            print(f"[다물체] {_op} @ {args.spread_axis}-spread pos=[{_px_i:.2f},{_py_i:.2f}] "
                  f"{('(눕힘 yaw=%.0f°)' % args.lying_yaw) if _lying_i else '(직립)'}", flush=True)
        target_cube = targets[0]["obj"]   # 이후 코드는 target_cube 별칭으로 현재 타겟 참조
    elif _obj_type in ("cylinder", "bottle"):
        from omni.isaac.core.objects import cylinder as _cyl
        s = CYLSPEC
        # --can-pose lying: 캔을 눕힘(축을 +z→+y로 90° 회전). 안착높이=반경. ★현 side 파지는 수직축
        #   가정이라 누운 캔은 파지 단계서 막힐 수 있음 — 견고성/한계 실측용.
        _orient = np.array([1.0, 0.0, 0.0, 0.0])
        if args.can_pose == "lying":
            _Rl = Rotation.from_euler('z', args.lying_yaw, degrees=True) * Rotation.from_euler('x', 90, degrees=True)
            _ql = _Rl.as_quat(); _orient = np.array([_ql[3], _ql[0], _ql[1], _ql[2]])  # 90° about X + lying_yaw
            _cz = (_robot_base[2] - 0.03) + s["radius"] + 0.002 if _robot_base is not None else s["radius"]
            print(f"[물체] 캔 눕힘(lying) — 안착 z={_cz:.3f}(반경기준)", flush=True)
        elif args.dr:
            _r0 = np.radians(_spawn_yaw_deg)                          # [Phase6 DR] yaw 자세(대칭이라 시각상 무영향)
            _orient = np.array([np.cos(_r0 / 2), 0.0, 0.0, np.sin(_r0 / 2)])
        if _obj_type == "bottle":
            # 컨투어 페트병(파워에이드 600ml): 회전 메시 강체 + 600g. 오목 허리/볼록 리브/넥/캡 표현.
            #   충돌=convexHull(외곽 envelope, 보수적), 점구름=동일 컨투어(GraspGen이 형상 인지).
            _bm = make_bottle_mesh(s["radius"], s["height"])
            _mp = UsdGeom.Mesh.Define(stage, "/World/target_cube")
            _mp.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in _bm.vertices])
            _mp.CreateFaceVertexCountsAttr([3] * len(_bm.faces))
            _mp.CreateFaceVertexIndicesAttr([int(i) for f in _bm.faces for i in f])
            _mp.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.45, 0.95)])   # 파워에이드 블루
            _bp = _mp.GetPrim()
            _xf = UsdGeom.Xformable(_bp)
            _xf.AddTranslateOp().Set(Gf.Vec3d(float(_cx), float(_cy), float(_cz)))
            _xf.AddOrientOp().Set(Gf.Quatf(float(_orient[0]), float(_orient[1]),
                                           float(_orient[2]), float(_orient[3])))
            UsdPhysics.RigidBodyAPI.Apply(_bp)
            UsdPhysics.CollisionAPI.Apply(_bp)
            UsdPhysics.MeshCollisionAPI.Apply(_bp).CreateApproximationAttr().Set("convexHull")
            UsdPhysics.MassAPI.Apply(_bp).CreateMassAttr().Set(0.60)   # 파워에이드 600ml ≈ 600g
            from omni.isaac.core.prims import RigidPrim as _RigidPrim
            target_cube = _RigidPrim("/World/target_cube", name="target_cube")
            print(f"[물체] 컨투어 페트병(파워에이드) spawn — r={s['radius']}m h={s['height']}m "
                  f"mass=0.60kg (오목허리/볼록리브/넥/캡)", flush=True)
        else:
            target_cube = _cyl.DynamicCylinder(
                prim_path="/World/target_cube", name="target_cube",
                position=np.array([_cx, _cy, _cz]), orientation=_orient,
                radius=s["radius"], height=s["height"],
                # 측면 타겟(--target-dy)은 파란색으로 표시(사용자: "파란색도 집어봐")
                color=(np.array([0.1, 0.3, 0.9]) if args.target_dy != 0 else np.array([0.85, 0.1, 0.1])),
                mass=0.30,   # 부분 캔 300g (그립 신뢰성↑; 만캔500g은 마진부족)
            )
    elif _obj_type == "snack":
        # 과자봉지 = particle cloth 인플레이터블(확정 모듈, snack_bag/snack_bag_module).
        #   가장자리0 2cm 베개를 공기로 부풀림 → 차분·깔끔. 강체 파이프라인 우회(snack 전용 핸들러).
        import sys as _sys
        _sys.path.insert(0, "/home/devuser/shelf_grasp_dev/snack_bag")
        from snack_bag_module import spawn_snack_bag as _spawn_snack
        _scn0 = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
        _scn_path = str(_scn0.GetPath()) if _scn0 else "/physicsScene"
        _spawn_snack(stage, _scn_path, (_cx, _cy), _table_top + 0.04, mode="cloth")
        _snack_cx, _snack_cy = _cx, _cy   # 봉지 파지 중심(핸들러 공용 — mixed는 별도 좌표로 재정의)
        # ★거치대는 환경 에셋(ENV_USD)에 포함 → 절차생성 안 함. 위치는 에셋에서 읽음(GUI 재배치 자동 추종).
        _sc = read_stand_center(stage)
        _STAND_X, _STAND_Y = (_sc[0], _sc[1]) if _sc else (0.310, 0.530)
        target_cube = None
        targets = []
        print(f"[과자봉지] cloth 봉지 spawn @[{_cx:.2f},{_cy:.2f}] + 거치대(에셋) @[{_STAND_X:.2f},{_STAND_Y:.2f}](3층) — snack 핸들러", flush=True)
    else:
        target_cube = cuboid.DynamicCuboid(
            prim_path="/World/target_cube", name="target_cube",
            position=np.array([_cx, _cy, _cz]),
            size=CUBE_SIZE, color=np.array([1.0, 0.4, 0.0]), mass=CUBE_MASS,
        )
    # 콜라캔(알루미늄)–그리퍼 고무패드 마찰. ★apply 해야 효력 발생 (stage3 교훈). ★snack도 생성해야
    #   set_gripper_friction이 손가락에 바인딩 → 그리퍼 고마찰로 FEM 봉지 그립(안 그럼 미끄러짐).
    cube_mat = PhysicsMaterial(
        prim_path="/World/Physics_Materials/cube_mat",
        static_friction=1.5, dynamic_friction=1.2, restitution=0.0,
    )
    from pxr import PhysxSchema as _PhysxSchema
    _pmat = _PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath("/World/Physics_Materials/cube_mat"))
    _pmat.CreateFrictionCombineModeAttr().Set("max")
    _pmat.CreateRestitutionCombineModeAttr().Set("min")
    if _obj_type != "snack":   # 강체 타겟에만 재질 적용(snack FEM은 자체 deformable material 사용)
        if targets:
            for _t in targets:
                try:
                    _t["obj"].apply_physics_material(cube_mat)
                except Exception:   # RigidPrim(컨투어 병)은 미지원 → USD physics 바인딩
                    from pxr import UsdShade
                    UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(_t["path"])).Bind(
                        UsdShade.Material(stage.GetPrimAtPath("/World/Physics_Materials/cube_mat")),
                        bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
        else:
            try:
                target_cube.apply_physics_material(cube_mat)
            except Exception:   # RigidPrim(메시 병)이 미지원이면 USD physics 바인딩으로 직접
                from pxr import UsdShade
                UsdShade.MaterialBindingAPI(stage.GetPrimAtPath("/World/target_cube")).Bind(
                    UsdShade.Material(stage.GetPrimAtPath("/World/Physics_Materials/cube_mat")),
                    bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
            targets = [{"obj": target_cube, "path": "/World/target_cube", "status": "pending", "reason": None}]
    # ★강체 파지 안정(GPU dynamics용): 솔버 pos192/vel1 + depenetration + contact/rest offset(관통방지).
    #   ★B안(--rigid-bag=CPU)은 제외 — CPU는 원래 클린이고, rest_offset(5mm 부풀림)이 빠듯한 2층 진입을
    #   물리 간섭시켜 캔이 기울어짐. CPU엔 GPU용 튜닝 불필요.
    if (args.obj_type == "snack" or args.mixed or args.force_gpu) and not args.rigid_bag:
        for _t in targets:
            if _t.get("type") != "snack":
                tune_rigid_grasp_iters(stage, _t["path"])
    cur_tgt_i = 0                     # 현재 타겟 인덱스
    slot_used = [False] * len(SHELF3_SLOTS)     # [Phase3] 3단 슬롯 점유맵
    place_x, place_y = SHELF3_SLOTS[0]          # 현재 사이클 적치 (x, in_y) — PLAN_CARRY에서 갱신
    cur_slot  = 0

    # ── Phase2 클러터: 타겟 양옆 정적(kinematic) 캔 = cuRobo 장애물 ──────────
    #   타겟만 ignore_substring에 남고, 클러터는 ignore에 없으므로 자동으로 장애물.
    #   ±y로 띄워 옆 접근(±y azimuth)은 막히고 정면(-x→+x) 접근만 열려 회피 검증.
    CLUTTER_Y_OFF = 0.17   # center-to-center(m). 0.12(가장자리 0.06)는 그리퍼가 못 비집음(전 후보 거부)
                           #   → 0.17(가장자리 0.11)로 정면(+x) 진입 공간 확보 (사용자 확인 2026-06-10)
    if args.clutter > 0 and _obj_type in ("cylinder", "bottle"):
        from omni.isaac.core.objects import cylinder as _cylc
        _sc = CYLSPEC
        # 타겟이 파란색일 수 있으니(--target-dy) 클러터는 녹색/주황/보라/빨강로(파랑 회피)
        _cl_colors = [np.array([0.1, 0.7, 0.2]), np.array([0.9, 0.6, 0.1]),
                      np.array([0.6, 0.1, 0.7]), np.array([0.85, 0.1, 0.1])]
        for _i in range(args.clutter):
            if args.target_dy > 0:                    # 타겟=매대쪽(+y) → 클러터는 로봇쪽(-y)에 줄세움
                _clx, _cly = _cx, _cy - CLUTTER_Y_OFF * (_i + 1)
            elif args.target_dy < 0:
                _clx, _cly = _cx, _cy + CLUTTER_Y_OFF * (_i + 1)
            else:                                     # 중앙 타겟 → 양옆 번갈아
                _sign = 1 if _i % 2 == 0 else -1
                _ring = _i // 2 + 1
                _clx, _cly = _cx, _cy + _sign * CLUTTER_Y_OFF * _ring
            _clp = f"/World/clutter_{_i}"
            _cl = _cylc.DynamicCylinder(
                prim_path=_clp, name=f"clutter_{_i}",
                position=np.array([_clx, _cly, _cz]),
                radius=_sc["radius"], height=_sc["height"],
                color=_cl_colors[_i % len(_cl_colors)], mass=0.30,
            )
            _cl.apply_physics_material(cube_mat)
            set_kinematic(stage, _clp, True)          # 정적 장애물(로봇이 쳐도 안 움직임)
            print(f"[클러터] {_clp} @ [{_clx:.3f}, {_cly:.3f}, {_cz:.3f}]", flush=True)

    # ── cuRobo 설정 ─────────────────────────────────────────────────────────
    world_cfg_table = WorldConfig.from_dict(
        load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
    )
    # 이 슬랩(5x5x0.2)은 cuRobo base 프레임 기준. 기본값이면 윗면이 로봇 마운트(월드 0.73)
    #   바로 아래(월드 0.71)에 깔려, 책상 위 캔을 잡으려는 grasp를 충돌로 거부함(거짓 데드존).
    #   → 슬랩 윗면을 실제 바닥(월드 0)에 맞춤. 실제 책상/매대 충돌은 update_world로 따로 반영.
    _floor_top = -float(_robot_base[2]) if _robot_base is not None else -0.73   # base프레임에서 월드0
    world_cfg_table.cuboid[0].pose[2] = _floor_top - world_cfg_table.cuboid[0].dims[2] / 2.0
    world_cfg = WorldConfig(cuboid=world_cfg_table.cuboid)

    tensor_args = TensorDeviceType()
    motion_gen_config = MotionGenConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args,
        collision_checker_type=CollisionCheckerType.MESH,
        num_trajopt_seeds=12, num_graph_seeds=12,
        interpolation_dt=0.03,
        collision_cache={"obb": 60, "mesh": 60},   # v2 매대(Table/Shelf 등) 충돌객체 많음
        # 책상 위 캔을 잡으려면 그리퍼가 책상 가까이 가야 함 → 과한 안전여유가 모든 파지를 막음.
        #   5mm는 추종오차(joint_2 ~4°, 실경로 수cm 이탈)를 못 흡수해 클러터/캔을 실제로 스침 →
        #   15mm로 상향(파지 높이 0.7·half에서 손목-책상 여유 59mm라 파지는 안 막힘). 기본 ~0.025.
        collision_activation_distance=0.015,
        optimize_dt=True, trajopt_dt=None, trajopt_tsteps=32, trim_steps=[1, None],
        # ★Phase2.5: plan_grasp(goalset 선택)는 내부에서 goalset↔single을 섞어 호출 → cuda graph의
        #   고정 goal shape와 충돌("changing goal type", Phase0 확인). graph 끄면 계획 ~2s로 느려지나
        #   클러터 상황적합 선택이 우선. 속도는 Phase4서 재방문.
        use_cuda_graph=False,
    )
    motion_gen = MotionGen(motion_gen_config)
    print("warming up...", flush=True)
    motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
    print("cuRobo (E0509) Ready", flush=True)

    # ── 진단용 IK 솔버 (도달 가능성 단독 검사: IK 실패 vs 충돌/궤적 실패 구분) ──
    ik_config = IKSolverConfig.load_from_robot_config(
        robot_cfg, world_cfg, tensor_args,
        rotation_threshold=0.05, position_threshold=0.005,
        num_seeds=30, self_collision_check=True, self_collision_opt=True,
        use_cuda_graph=True,
    )
    ik_solver = IKSolver(ik_config)
    print("IK 솔버 Ready", flush=True)
    # 도달 영역 진단 (캔 높이 도달성/워크스페이스 판별)
    try:
        _q_seed = tensor_args.to_device(np.array(RETRACT_CONFIG, dtype=np.float32))
        reachability_sanity(ik_solver, tensor_args, _q_seed)
    except Exception as _e:
        print(f"[reach] 진단 실패: {_e}", flush=True)

    # ── GraspGen ZMQ 클라이언트 ───────────────────────────────────────────────
    grasp_client = None
    if not args.no_graspgen:
        sys.path.insert(0, "/home/devuser/graspgen_ws/GraspGen")
        from grasp_gen.serving.zmq_client import GraspGenClient
        print(f"[ZMQ] GraspGen 서버 연결 중 (port {args.port})...", flush=True)
        grasp_client = GraspGenClient("127.0.0.1", args.port, wait_for_server=True)
        print(f"[ZMQ] 연결 완료: {grasp_client._server_metadata}", flush=True)
    else:
        print("[GraspGen] --no-graspgen: 고정 탑다운 모드(Step1)", flush=True)

    obj_type   = args.obj_type
    grasp_mode = OBJ_SPECS[obj_type]["grasp_mode"]
    # 로봇 base (작업공간 필터 기준) — 진단에서 구한 _robot_base 재사용
    robot_base = _robot_base if _robot_base is not None else np.array([0.0, 0.0, 0.0])
    # ★cuRobo는 base_link 원점 기준 → 월드 타겟에서 base 위치를 빼도록 offset 설정(pp_motion in-place).
    #   (안 하면 모든 EE 타겟이 base 위치만큼 어긋나 공중으로 감 = joint1 폭주/그리퍼 캔서 멀어짐)
    set_base_offset(robot_base)   # pp_motion._ROBOT_BASE_OFFSET in-place 갱신(import한 main도 같은 배열 참조)
    print(f"[프레임] cuRobo base offset = {_ROBOT_BASE_OFFSET.round(3)} (월드 타겟에서 차감)", flush=True)

    # 점구름 시각화용 USD Points prim
    from pxr import Gf as _Gf, Vt as _Vt
    pc_prim = UsdGeom.Points.Define(stage, "/World/debug_pc")

    # 축 부하 비용의 기준이 되는 편안한 자세(retract) 텐서
    retract_t = tensor_args.to_device(np.array(RETRACT_CONFIG, dtype=np.float32))

    plan_config = MotionGenPlanConfig(
        enable_graph=False, enable_graph_attempt=4,
        max_attempts=8, enable_finetune_trajopt=True,
        time_dilation_factor=0.85,   # ★0.92→0.85 원복: 전역 속도↑가 운반 관성으로 물체 떨굼(사용자 지적). 봉지 접근만 봉지전용 스텝으로 빠르게.
    )
    # ★Phase2.5: 파지는 plan_grasp(2단계: offset까지 풀충돌인지 → 직선 최종진입+손가락 충돌면제)가
    #   접근 제약을 내장하므로 별도 접근 메트릭 불필요(plan_grasp는 pose_cost_metric=None을 요구).
    GRIPPER_COLL_LINKS = [l for l in robot_cfg["kinematics"].get("collision_link_names", [])
                          if "gripper" in l]   # 최종 진입 때 월드충돌 면제할 링크(손가락이 캔을 감싸야 함)
    usd_help = UsdHelper()
    usd_help.load_stage(my_world.stage)
    usd_help.add_world_to_stage(world_cfg, base_frame="/World")
    # v2에 defaultGroundPlane 포함 → 중복 생성 안 함
    set_gripper_friction(stage)   # 손가락 고마찰 바인딩 — ★play() 전에(후엔 physics view 무효화)
    set_finger_sdf_collision(stage)   # Stage6-B1: 손가락 SDF collision(오목 패드 면접촉=그립 wrap 필수) — play() 전
    disable_visual_collision(stage)   # ★그리퍼 /visuals 중복 convexHull 콜리전 제거(끼임 주범) — SDF /collisions만 남김
    if (args.obj_type == "snack" or args.mixed or args.force_gpu) and not args.rigid_bag:   # 봉지 cloth=GPU 필수. ★B안(--rigid-bag)은 강체봉지라 GPU 끔 → 씬 전체 CPU(캔/병 CCD 파지)
        _scn = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
        if _scn is not None:
            _sa = PhysxSchema.PhysxSceneAPI.Apply(_scn)
            _sa.CreateEnableGPUDynamicsAttr().Set(True)
            _sa.CreateBroadphaseTypeAttr().Set("GPU")
            _sa.CreateEnableCCDAttr().Set(True)   # ★CCD: 그리퍼가 캔 뚫고 들어가는 터널링 방지(정밀 파지 정석, velog/Factory 권고)
            print("[GPU] dynamics 활성화(봉지 cloth) + CCD ON(파지 관통 방지)", flush=True)
            # ★GPU 충돌/파티클 버퍼 증설 — 봉지 단독(바디 少)은 기본값 OK지만, mixed(봉지 cloth+캔+병)는
            #   기본 버퍼 초과 → 파티클 제약 드롭 → 봉지 deflate·불안정·폭발. 실제 적용되는 여기에 넣음.
            if args.mixed or args.force_gpu:
                for _gattr, _gval in [
                    ("CreateGpuMaxRigidContactCountAttr", 2097152),
                    ("CreateGpuMaxRigidPatchCountAttr", 327680),
                    ("CreateGpuFoundLostPairsCapacityAttr", 1048576),
                    ("CreateGpuFoundLostAggregatePairsCapacityAttr", 1048576),
                    ("CreateGpuTotalAggregatePairsCapacityAttr", 4194304),
                    ("CreateGpuCollisionStackSizeAttr", 335544320),
                    ("CreateGpuMaxParticleContactsAttr", 4194304),
                    ("CreateGpuMaxNumPartitionsAttr", 8),
                ]:
                    try:
                        getattr(_sa, _gattr)().Set(_gval)
                    except Exception as _ge:
                        print(f"  [GPU버퍼] {_gattr} 실패(무시): {_ge}", flush=True)
                print("  [GPU버퍼] mixed 충돌/파티클 버퍼 증설(봉지 폭발 방지 시도)", flush=True)
        # ★GPU(TGS)서 SDF 손가락이 강체를 관통하는 문제 억제: 로봇 articulation 위치 솔버 iteration↑.
        #   SDF는 그립 wrap에 필수라 유지하되, iteration을 올려 접촉을 더 단단히 해소(관통/welded 억제).
        if args.mixed or args.force_gpu:
            try:
                _pa = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(robot_prim_path))
                _pa.CreateSolverPositionIterationCountAttr().Set(192)
                _pa.CreateSolverVelocityIterationCountAttr().Set(1)
                print("  [GPU] 로봇 articulation 솔버 iteration(pos192/vel1, Factory레시피) — 관통방지", flush=True)
            except Exception as _e:
                print(f"  [GPU] articulation 솔버 설정 실패(무시): {_e}", flush=True)
    set_scene_camera()        # PNG/뷰포트가 로봇·캔·매대를 크게 잡도록 카메라 배치 (play는 뷰포트 생성 후)

    # ── 루프 변수 ───────────────────────────────────────────────────────────
    ctrl         = None
    sim_js_names = None
    state        = GS.IDLE
    wait_cnt     = 0
    cycle        = 0
    attached     = False
    attach_off   = None
    cube_tgt_pos = None
    fail_cnt     = 0
    MAX_FAIL     = 5   # 무한 재시도 방지: 이 횟수 초과 시 진단 후 다음 사이클로
    regrasp_cnt  = 0
    MAX_REGRASP  = 3   # 물리 그립 실패 시 재파지 횟수 (마진 작아 ~67% → 재시도로 신뢰성↑)
    grasp_world  = None   # GraspGen 선택 파지 (4x4, 월드)
    pre_world    = None   # pre-grasp (4x4, 월드)
    grasp_cands  = []     # ★Phase1: plan_grasp용 side 파지후보 묶음(4x4 월드 리스트)
    grip_z_offset = 0.3 * (CYLSPEC["height"] / 2.0)   # 캔중심이 TCP보다 아래인 양(파지 후 실측 갱신)

    # E0509 EE prim 경로 (panda_hand 대신 gripper_rh_p12_rn_base)
    ee_prim_path = f"{robot_prim_path}/{ee_link}"

    # 손목 리얼센스(eye-in-hand): 내장 RealSense RGB 카메라를 두 번째 뷰포트에 바인딩(메인=persp, 보조=손목뷰)
    _wrist_cam_path = setup_wrist_camera(stage, ee_prim_path)
    if _wrist_cam_path:
        open_wrist_viewport(_wrist_cam_path)

    # ★play 전 20초 대기 — 사용자가 RealSense 뷰포트 창을 원하는 위치로 옮길 시간. 렌더만(물리 정지).
    print("  [대기] 뷰포트 배치용 20초 대기 후 play …", flush=True)
    import time as _wt
    _w0 = _wt.time()
    while _wt.time() - _w0 < 20.0:
        simulation_app.update()
    my_world.play()
    print("  [play] 시뮬 시작", flush=True)

    # 시작 시 stale stop-sentinel 제거
    if os.path.exists(STOP_FILE):
        os.remove(STOP_FILE)
    print(f"Stage 2 E0509 시작. {args.cycles}사이클. (종료: touch {STOP_FILE})", flush=True)

    # ★충돌구체 시각화: 링크 prim 자식으로 1회 부착 → USD 계층이 매 프레임 강체 추종(lag 0).
    #   (과거: 매 스텝 월드좌표 재계산 redraw라 렌더 프레임과 어긋나 뒤늦게 따라옴 → 고객영상 부적합)
    if args.viz_spheres:
        attach_cspheres_to_links(stage, robot_prim_path,
                                 f"{ROBOT_DIR}/e0509_spheres.yml", opacity=0.35)
    _viz_cb = None   # 부착형이라 동작 중 재그리기 불필요(자동 추종)

    _gtest_done = False   # --gripper-test 1회 실행 플래그 (Stage6 이식)
    _snack_done = False    # 과자봉지 squish 파지 1회 실행 플래그
    _snack_frozen = False  # 혼합: 봉지 차례 전까지 cloth 동결(폭발 방지) — soften은 snack 핸들러에서

    def _go_product_view(_cujs):
        """비전 탐지 자세(product_view)로 이동 후 3초 인식 대기 — 실기체에서 카메라가 물체를
        탐지하는 시간을 시뮬에 반영(사이클타임 현실화). 각 물체 파지 직전마다 호출."""
        import time as _pt
        _pv_goal = JointState(
            position=tensor_args.to_device(np.array(PRODUCT_VIEW_CONFIG, dtype=np.float32)).view(1, -1),
            joint_names=list(_cujs.joint_names))
        _pv_res = motion_gen.plan_single_js(_cujs.unsqueeze(0), _pv_goal, plan_config)
        if _pv_res.success.item():
            execute_plan(motion_gen.get_full_js(_pv_res.get_interpolated_plan()),
                         sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                         track_tag="product_view", arm_only=True)
            print("  [product_view] ✅ 비전 탐지 자세 도달 → 3초 인식 대기", flush=True)
            _t0 = _pt.time()
            while _pt.time() - _t0 < 3.0:   # 3초 인식 대기(벽시계)
                my_world.step(render=True)
        else:
            print("  [product_view] ❌ plan 실패 — 자세 건너뜀", flush=True)

    while simulation_app.is_running():
        # ── graceful 종료 (kill 불필요): stop-sentinel 파일 감지 ──
        if os.path.exists(STOP_FILE):
            print(f"[종료] {STOP_FILE} 감지 → graceful 종료", flush=True)
            try:
                os.remove(STOP_FILE)
            except Exception:
                pass
            simulation_app.close()
            break

        my_world.step(render=True)
        step = my_world.current_time_step_index

        if ctrl is None:
            # E0509는 ArticulationView로 직접 제어
            from omni.isaac.core.articulations import Articulation
            try:
                robot_art = Articulation(prim_path=robot_prim_path)
                robot_art.initialize()
                ctrl = robot_art.get_articulation_controller()
                sim_js_names = robot_art.dof_names
                print(f"E0509 조인트: {sim_js_names}", flush=True)
                stabilize_arm_drives(robot_art, j_names, sim_js_names)  # 출렁임 억제 게인 보정
                enable_gravity_comp(robot_art, my_world, j_names)       # [Stage5] joint_2 중력처짐 보상
            except Exception as e:
                print(f"초기화 대기: {e}", flush=True)
                continue

        if step < 10:
            try:
                arm_idx = [robot_art.get_dof_index(x) for x in j_names if x in sim_js_names]
                _q_ret = np.array(default_cfg[:len(arm_idx)], dtype=np.float32)
                robot_art.set_joint_positions(_q_ret, arm_idx)   # home으로 텔레포트
                # ★드라이브 타겟도 home으로. 안 주면 PD가 기본값(≈0)으로 끌어 처짐/joint_2 넘어감.
                #   (게인 800/80은 댐핑 충분 → 1e5 때 같은 bounce 없음)
                ctrl.apply_action(ArticulationAction(_q_ret, joint_indices=arm_idx))
                # effort/게인은 stabilize_arm_drives에서 검증값(eff 200/50)으로 설정 → 여기서 덮지 않음
            except Exception:
                pass
            continue
        if step < 20:
            continue

        # ── [혼합 이슈2] 봉지 cloth 동결 — 스폰 직후 ~20스텝 안착 후 파티클 정지.
        #   원인: mixed는 봉지가 캔·병 처리되는 긴 시간 cloth로 떠 있다 솔버 불안정 누적 → 폭발("시작하자마자 불안정").
        #   GPU 버퍼↑로는 안 잡힘(버퍼 초과 아님). 봉지 차례 전까지 정적 메시로 동결 → 폭발 원천차단.
        #   봉지 차례엔 snack 핸들러에서 soften_bag()으로 복귀시켜 파지.
        if args.mixed and not _snack_frozen and step > 20:
            try:
                _sys_sb = __import__("sys")
                if "/home/devuser/shelf_grasp_dev/snack_bag" not in _sys_sb.path:
                    _sys_sb.path.insert(0, "/home/devuser/shelf_grasp_dev/snack_bag")
                from snack_bag_module import rigidify_bag as _rig0
                _rig0(stage)
                _snack_frozen = True
                print("[혼합 이슈2] 봉지 cloth 동결(차례 전까지 폭발 방지)", flush=True)
            except Exception as _fe:
                print(f"[혼합 이슈2] 봉지 동결 실패(무시): {_fe}", flush=True)

        # ── Stage6 판별실험 (--gripper-test): 빈손 open→풀클로즈 형상·각도를 실물 사진과 대조 ──
        #   목적: 풀클로즈 부리(beak) 모양이 (1)팁 꺾임 형상(r2≈r1 평행)인지 (2)원위 추가회전(r2>r1
        #   토우인=시퀀셜 언더액추에이션)인지 판별 → RH-P12 링크 구조 모델링 방향 결정.
        if args.gripper_test and not _gtest_done:
            _gj = {n: robot_art.get_dof_index(n) for n in
                   ("gripper_rh_r1", "gripper_rh_r2", "gripper_rh_l1", "gripper_rh_l2")}

            def _glog(tag):
                _q = robot_art.get_joint_positions()
                _a = {k: float(_q[i]) for k, i in _gj.items()}
                print(f"  [GTEST:{tag}] r1={_a['gripper_rh_r1']:.3f} r2={_a['gripper_rh_r2']:.3f} "
                      f"l1={_a['gripper_rh_l1']:.3f} l2={_a['gripper_rh_l2']:.3f} rad "
                      f"(r2-r1={_a['gripper_rh_r2'] - _a['gripper_rh_r1']:+.3f})", flush=True)

            def _gcam(view):
                """그리퍼 클로즈업 3방향 카메라. 손가락(r2/l2) 월드좌표 중점을 직접 겨냥
                (EE 기준 고정 오프셋은 home 자세에서 팔 링크에 가림 — 1차 실험서 확인)."""
                _p_r2 = get_ee_world_pos(stage, f"{robot_prim_path}/gripper_rh_p12_rn_r2")
                _p_l2 = get_ee_world_pos(stage, f"{robot_prim_path}/gripper_rh_p12_rn_l2")
                _p_b  = get_ee_world_pos(stage, ee_prim_path)
                if _p_r2 is None or _p_l2 is None or _p_b is None:
                    print("  [GTEST] 손가락 prim 위치 조회 실패", flush=True); return
                _mid = [(float(_p_r2[i]) + float(_p_l2[i])) / 2 for i in range(3)]
                _d = np.array(_mid) - np.array([float(_p_b[i]) for i in range(3)])  # base→손가락(approach)
                _n = _d / (np.linalg.norm(_d) + 1e-9)
                _lat = np.cross(_n, [0, 0, 1.0]); _lat /= (np.linalg.norm(_lat) + 1e-9)
                _eyes = {"front": np.array(_mid) + _n * 0.30,
                         "side":  np.array(_mid) + _lat * 0.30,
                         "top":   np.array(_mid) + np.array([0, 0, 0.30])}
                try:
                    try:
                        from omni.isaac.core.utils.viewports import set_camera_view
                    except ImportError:
                        from isaacsim.core.utils.viewports import set_camera_view
                    set_camera_view(eye=[float(x) for x in _eyes[view]], target=_mid,
                                    camera_prim_path="/OmniverseKit_Persp")
                except Exception as _e:
                    print(f"  [GTEST] 카메라 실패(무시): {_e}", flush=True)
                for _ in range(3):
                    my_world.step(render=True)

            print("[GTEST] 빈손 그리퍼 open/풀클로즈 판별 실험 시작 (팔=home 고정)", flush=True)
            _ee = get_ee_world_pos(stage, ee_prim_path)
            _ee = [0.0, 0.0, 1.0] if _ee is None else [float(_ee[0]), float(_ee[1]), float(_ee[2])]
            set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=40)
            for _ in range(30):
                my_world.step(render=True)
            _glog("open")
            for _v in ("front", "side", "top"):
                _gcam(_v); save_shot(f"gtest_open_{_v}")
            # 풀스트로크 닫기: URDF 한계(1.101) 직전 1.10 — 실물 빈손 풀클로즈(DXL 740)에 대응
            set_gripper(ctrl, robot_art, sim_js_names, my_world, 1.10, steps=80)
            for _ in range(45):
                my_world.step(render=True)
            _glog("closed_full")
            for _v in ("front", "side", "top"):
                _gcam(_v); save_shot(f"gtest_closed_{_v}")
            print("[GTEST] 완료 — logs/shots/shot_*_gtest_*.png를 실물 사진과 대조. "
                  "종료: touch /tmp/stage7_stop", flush=True)
            _gtest_done = True
            continue
        if args.gripper_test:
            continue   # 테스트 후 장면 유지(HALT) — 파지 파이프라인 진입 안 함

        # 월드 동기화 ([Phase3] 현재 타겟만 ignore — 이웃/기적치 캔은 자동 장애물)
        if step == 50 or step % 1000 == 0:
            # ★snack도 매대·거치대를 cuRobo에 등록해야 carry plan_single이 매대 회피(미등록 시 직선경로로 매대 관통).
            #   봉지 메시·파티클시스템은 장애물에서 제외(자기 봉지와 충돌판정→정지 방지). stage7 동일 처리.
            if obj_type == "snack":
                _ig = [robot_prim_path, "/World/snack_bag", "snackParticleSystem",
                       "/World/defaultGroundPlane", "/curobo", "/World/cspheres",
                       ROBOT_BASE_BLOCK_PATH]
            else:
                _ig = [robot_prim_path, targets[cur_tgt_i]["path"],
                       "/World/defaultGroundPlane", "/curobo",
                       "/World/cspheres",   # ★시각화 구체는 장애물 아님(안 빼면 로봇이 자기 구체와 충돌→정지)
                       ROBOT_BASE_BLOCK_PATH]   # 책상 충돌 복구(팔이 책상 회피)
                if args.mixed:   # 봉지 cloth는 장애물에서 제외(get_obstacles cloth 처리 회피·책상 위라 캔/병 경로 무관). 거치대는 유지.
                    _ig += ["/World/snack_bag", "snackParticleSystem"]
            obs = usd_help.get_obstacles_from_stage(
                only_paths=["/World"],
                reference_prim_path=robot_prim_path,
                ignore_substring=_ig,
            ).get_collision_check_world()
            motion_gen.update_world(obs)
            if step == 50:   # cuRobo가 든 장애물(책상/매대) 확인 — base 프레임 z 범위
                try:
                    _objs = obs.objects if hasattr(obs, "objects") else []
                    print(f"  [충돌월드] cuRobo 장애물 {len(_objs)}개:", flush=True)
                    for _o in _objs[:12]:
                        _p = getattr(_o, "pose", None); _d = getattr(_o, "dims", None)
                        print(f"     {_o.name}: pose={None if _p is None else [round(float(x),3) for x in _p[:3]]} dims={_d}", flush=True)
                except Exception as _e:
                    print(f"  [충돌월드] 로깅 실패: {_e}", flush=True)
            if _CLEARANCE["fn"] is None:   # [간격로깅] 월드 준비 후 1회 등록
                _CLEARANCE["fn"] = make_clearance_fn(robot_art, motion_gen, tensor_args)
                _c0 = _CLEARANCE["fn"]()
                print(f"  [간격로깅] 활성화 — 홈자세 최소간격 "
                      f"{'N/A' if _c0 is None else f'{_c0*1000:+.1f}mm'}", flush=True)

        # 현재 조인트 상태
        try:
            js_pos = robot_art.get_joint_positions()
            js_vel = robot_art.get_joint_velocities()
        except Exception:
            continue
        if js_pos is None:          # 물리뷰 미생성/HALT 등 → 이번 step 건너뜀(크래시 방지)
            continue

        # arm 조인트만 추출
        arm_joint_names = motion_gen.kinematics.joint_names
        arm_pos = []
        for jn in arm_joint_names:
            if jn in sim_js_names:
                idx = sim_js_names.index(jn)
                arm_pos.append(js_pos[idx])
            else:
                arm_pos.append(0.0)
        arm_pos = np.array(arm_pos, dtype=np.float32)
        # 시작상태를 관절 한계 안으로 클램프 (측정값 미세 초과 → INVALID_START_STATE 방지)
        if arm_pos.shape[0] == ARM_JOINT_LOWER.shape[0]:
            arm_pos = np.clip(arm_pos, ARM_JOINT_LOWER + 1e-3, ARM_JOINT_UPPER - 1e-3)

        cu_js = JointState(
            position=tensor_args.to_device(arm_pos),
            velocity=tensor_args.to_device(np.zeros_like(arm_pos)),
            acceleration=tensor_args.to_device(np.zeros_like(arm_pos)),
            jerk=tensor_args.to_device(np.zeros_like(arm_pos)),
            joint_names=list(arm_joint_names),
        )

        # ── 과자봉지 squish 파지 핸들러(단독 + mixed 봉지 타겟 공용): 강체전환→carry→tilt적치 ──
        if obj_type == "snack":
            if not _snack_done and step > 80:
                _go_product_view(cu_js)   # ── product_view: 비전 탐지 자세 + 3초 인식 (봉지) ──
                _TACT.mark("snack_bag", "grasp")
                # ── [혼합 이슈2] 봉지 차례 → cloth 복귀. 캔·병 처리 동안 동결(_snack_frozen)했던
                #   파티클을 재활성 → 접근 궤적 동안 빗면/책상에 재안착 후 파지(squish 위해 cloth 필요).
                if args.mixed and _snack_frozen:
                    try:
                        from snack_bag_module import soften_bag as _soft0
                        _soft0(stage)
                        print("[혼합 이슈2] 봉지 cloth 복귀(파지 위해 soften)", flush=True)
                    except Exception as _se:
                        print(f"[혼합 이슈2] 봉지 soften 실패(무시): {_se}", flush=True)
                _bag_top = _table_top + 0.072
                # ★파지 위치는 원래대로(table+0.030, 그리퍼가 봉지를 제대로 잡는 위치). 책상 겹침은 그리퍼를 올리지 말고
                #   '봉지 부착(따라오는) 위치만' 위로 올려 해결(아래 _BAG_ATTACH_DZ). 그리퍼 올리면 눈속임 됨(사용자).
                _bag_mid = _table_top + 0.030
                # ★옆-스퀴즈 EE: 그리퍼는 EE 로컬Y축으로 닫힘(실관찰) → 폭(16cm=world X) 압축하려면 로컬Y=world X.
                #   approach(로컬Z)=아래. 닫으면 world X(폭 16) 좁아짐 → 두께 불룩→그립(실측 모델).
                def _ee_side(tx, ty, tcp_z):
                    M = np.eye(4); M[:3,0]=[0,1,0]; M[:3,1]=[1,0,0]; M[:3,2]=[0,0,-1]
                    M[:3,3] = [tx, ty, tcp_z + RHP12_TCP_DEPTH]; return M
                def _bag_worldz():   # 봉지 월드 z(들림 판정). particle cloth는 월드바운드로 측정
                    try:
                        _bd = UsdGeom.Boundable(stage.GetPrimAtPath("/World/snack_bag"))
                        _bb = _bd.ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_).ComputeAlignedBox()
                        return (_bb.GetMin()[2] + _bb.GetMax()[2]) / 2.0
                    except Exception:
                        return float("nan")
                # 현재 관절상태로 cu_js 재시드(plan_single은 매 호출 현재상태 시드 필요 — 한 iteration서 연속 동작)
                def _refresh_cujs():
                    _jp = robot_art.get_joint_positions()
                    _ap = np.array([float(_jp[sim_js_names.index(_jn)]) if _jn in sim_js_names else 0.0
                                    for _jn in arm_joint_names], dtype=np.float32)
                    _ap = np.clip(_ap, ARM_JOINT_LOWER + 1e-3, ARM_JOINT_UPPER - 1e-3)
                    return JointState(
                        position=tensor_args.to_device(_ap),
                        velocity=tensor_args.to_device(np.zeros_like(_ap)),
                        acceleration=tensor_args.to_device(np.zeros_like(_ap)),
                        jerk=tensor_args.to_device(np.zeros_like(_ap)),
                        joint_names=list(arm_joint_names))
                # 접근/리프트: cuRobo plan_single(매끄러운 시간최적 궤적, IK분기 플립·되돌아감 없음). 실패시 direct IK 폴백.
                def _plan_move(target_world, tag, arm_only=False):
                    _js = _refresh_cujs()
                    _r = motion_gen.plan_single(_js.unsqueeze(0),
                                                mat4_to_curobo_pose(target_world, tensor_args), plan_config)
                    if _r.success.item():
                        execute_plan(motion_gen.get_full_js(_r.get_interpolated_plan()),
                                     sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                                     track_tag=tag, arm_only=arm_only)
                        return True
                    print(f"  [과자봉지] {tag} plan_single 실패 → direct IK 폴백", flush=True)
                    return move_direct_ik(target_world, ik_solver, tensor_args, _js.position, arm_joint_names,
                                          robot_art, ctrl, my_world, steps=40, settle=6)  # ★폴백 빠르게(70/20→40/6, 사용자: 봉지 느림)
                _bz0 = _bag_worldz()
                def _bag_span(_axis):   # 봉지 월드 바운드 폭(0=X 그립방향, 2=Z 두께) — 그리퍼 span(11cm) 매칭용
                    try:
                        _bd = UsdGeom.Boundable(stage.GetPrimAtPath("/World/snack_bag"))
                        _bb = _bd.ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_).ComputeAlignedBox()
                        return _bb.GetMax()[_axis] - _bb.GetMin()[_axis]
                    except Exception:
                        return float("nan")
                print(f"[과자봉지] 옆-스퀴즈 파지 시작. 봉지 그립폭(X)={_bag_span(0)*1000:.0f}mm 두께(Z)={_bag_span(2)*1000:.0f}mm "
                      f"(그리퍼 span 11cm에 맞추는 게 목표). 기준 월드z={_bz0*1000:.0f}mm", flush=True)
                set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=25)   # 최대 개방(span~10.6cm)
                save_shot("snack_01_open")
                if _plan_move(_ee_side(_snack_cx, _snack_cy, _bag_top + 0.06), "above"):   # 접근(매끄럽게)
                    save_shot("snack_02_above")
                    # ★[#2b] 봉지 파지 동안 책상 콜리전만 OFF(그리퍼 손가락 책상 침투 방지).
                    #   ★봉지 콜리전은 끄지 않음 — FilteredPairs로 봉지↔그리퍼만 제외(폭발 방지) + 봉지↔거치대 충돌
                    #   살려둬 적치 시 실제 중력으로 자연 안착(런타임 재활성 불필요).
                    if args.rigid_bag:
                        toggle_table_collision(stage, False)
                    # 진입: 봉지에 일부러 접촉(plan_single은 충돌로 거부) → 직접 IK 유지
                    move_direct_ik(_ee_side(_snack_cx, _snack_cy, _bag_mid), ik_solver, tensor_args,
                                   _refresh_cujs().position, arm_joint_names, robot_art, ctrl, my_world, steps=30, settle=6)  # ★진입 빠르게(50/8→30/6, 사용자: 하강 아직 느림)
                    save_shot("snack_03_enter")
                    # ★스퀴즈 = 실측 4.3cm 침투까지 폐루프로 닫음(고정 각도 추측 금지). 봉지 반폭0.08 → 반간격0.037 → 손가락간격 74mm
                    def _finger_gap():
                        def _fx(_n):
                            _m = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Robot/{_n}")).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                            return _m.ExtractTranslation()[0]
                        return abs(_fx("gripper_rh_p12_rn_r2") - _fx("gripper_rh_p12_rn_l2"))
                    if args.rigid_bag:
                        # ★강체 봉지(kinematic 비주얼 그립): 과압착 금지(언더액추에이티드 손가락 강체에 막혀 spin) → 가볍게 닫음.
                        set_gripper(ctrl, robot_art, sim_js_names, my_world, 0.45, steps=12)
                        print(f"  [과자봉지][강체] 가벼운 닫기(0.45) — 손가락간격={_finger_gap()*1000:.0f}mm (kinematic 비주얼 그립)", flush=True)
                        for _ in range(15): my_world.step(render=True)
                    else:
                        _TARGET_GAP = 0.074   # 실측: 2cm 손가락이 양옆 4.3cm 침투(16cm폭 → 7.4cm 간격)
                        _ang = GRIP_OPEN
                        while _ang < 0.70:                          # 한계 0.60→0.70(빠른 닫힘 시 목표 74mm 도달 보장)
                            _ang += 0.10                            # ★스퀴즈 빠르게(0.03→0.10, 사용자 요청) — 폐루프 측정 유지
                            set_gripper(ctrl, robot_art, sim_js_names, my_world, _ang, steps=8)  # 스텝 12→8(빠르되 손가락 도달)
                            if _finger_gap() <= _TARGET_GAP:
                                break
                        print(f"  [과자봉지] 스퀴즈 정지 — 손가락간격={_finger_gap()*1000:.0f}mm (목표 74mm=4.3cm침투) 각도={_ang:.2f}", flush=True)
                        for _ in range(80): my_world.step(render=True)
                    save_shot("snack_04_grip")
                    # ── [Phase1] 강체전환(사용자 아이디어): 그립 직후(★리프트 전) 파티클 정지 + EE 추종 + cuRobo 프록시 attach ──
                    #   ★stage7 검증 순서. 추종을 리프트 전에 걸어야 봉지가 EE를 따라옴(리프트 후 걸면 cloth 슬립→공중부양·좌표 어긋남).
                    from snack_bag_module import rigidify_bag as _rigidify
                    from pxr import Gf as _Gf
                    _bag_c, _bag_d = _rigidify(stage)   # 파티클 정지(강체봉지는 no-op) + 봉지 월드 AABB(center,dims)
                    _Plocal = None
                    _relB = None
                    if not args.rigid_bag:
                        # [cloth] EE 추종(검증된 snack32 방식): 동결 봉지 메시를 EE 로컬프레임 고정점으로 매 스텝 갱신
                        _bagx = UsdGeom.Xformable(stage.GetPrimAtPath("/World/snack_bag"))
                        _Mbag = _bagx.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                        _W0 = [_Mbag.Transform(_Gf.Vec3d(p[0], p[1], p[2]))
                               for p in UsdGeom.Mesh(stage.GetPrimAtPath("/World/snack_bag")).GetPointsAttr().Get()]
                        _bagx.ClearXformOpOrder()   # 메시 Xform identity → 점=월드
                        UsdGeom.Mesh(stage.GetPrimAtPath("/World/snack_bag")).GetPointsAttr().Set([_Gf.Vec3f(_w) for _w in _W0])
                        _M0inv = UsdGeom.Xformable(stage.GetPrimAtPath(ee_prim_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
                        _Plocal = [_M0inv.Transform(_w) for _w in _W0]   # EE 로컬프레임 고정점(그립 시점)
                        def _snack_follow(dt):
                            _M = UsdGeom.Xformable(stage.GetPrimAtPath(ee_prim_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                            UsdGeom.Mesh(stage.GetPrimAtPath("/World/snack_bag")).GetPointsAttr().Set(
                                [_Gf.Vec3f(_M.Transform(_pl)) for _pl in _Plocal])
                        my_world.add_physics_callback("snack_follow", _snack_follow)
                    else:
                        # [B안 강체] ★고정 조인트(FixedJoint)로 봉지를 그리퍼에 물리 부착 — kinematic 추종(위치만·회전 안먹음) 대체.
                        #   실제 강체 구속이라 그리퍼 틸트/회전을 봉지가 완전히 따라감. 봉지는 dynamic(중력有)이지만
                        #   조인트가 EE에 고정 → 떨어지지 않음. 릴리즈 시 조인트만 삭제하면 즉시 중력 낙하(자연 안착).
                        from pxr import UsdPhysics as _UPb, Sdf as _Sdf
                        _bag_prim = stage.GetPrimAtPath("/World/snack_bag")
                        _BAG_JOINT = "/World/snack_bag_grip_joint"
                        _ee_w  = UsdGeom.Xformable(stage.GetPrimAtPath(ee_prim_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                        _bag_w = UsdGeom.Xformable(_bag_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                        _bag_in_ee = _bag_w * _ee_w.GetInverse()   # 봉지의 EE 로컬 포즈(현재 상대자세 보존)
                        _jt = _bag_in_ee.ExtractTranslation()
                        _jq = _bag_in_ee.ExtractRotationQuat()
                        _jre = float(_jq.GetReal()); _jim = _jq.GetImaginary()
                        _UPb.RigidBodyAPI(_bag_prim).CreateKinematicEnabledAttr(False)   # dynamic — 조인트가 잡고, 릴리즈 시 낙하
                        _fj = _UPb.FixedJoint.Define(stage, _BAG_JOINT)
                        _fj.CreateBody0Rel().SetTargets([_Sdf.Path(ee_prim_path)])
                        _fj.CreateBody1Rel().SetTargets([_Sdf.Path("/World/snack_bag")])
                        _fj.CreateLocalPos0Attr().Set(_Gf.Vec3f(float(_jt[0]), float(_jt[1]), float(_jt[2])))
                        _fj.CreateLocalRot0Attr().Set(_Gf.Quatf(_jre, float(_jim[0]), float(_jim[1]), float(_jim[2])))
                        _fj.CreateLocalPos1Attr().Set(_Gf.Vec3f(0.0, 0.0, 0.0))
                        _fj.CreateLocalRot1Attr().Set(_Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                        print("  [B안] 봉지 고정조인트 부착(그리퍼↔봉지) — 틸트 포함 강체 추종, 릴리즈 시 조인트 삭제로 낙하", flush=True)
                    # cuRobo Cuboid 프록시 attach(그립 시점 — 캔/병과 동일 carry plan_single이 봉지 부피 인지→매대 회피)
                    _bc_b = (np.asarray(_bag_c) - _ROBOT_BASE_OFFSET).astype(float)
                    _held_bag = Cuboid(name="held_bag",
                        pose=[float(_bc_b[0]), float(_bc_b[1]), float(_bc_b[2]), 1.0, 0.0, 0.0, 0.0],
                        dims=[float(_bag_d[0]) * 1.1, float(_bag_d[1]) * 1.1, float(_bag_d[2]) * 1.1])
                    try:
                        motion_gen.detach_object_from_robot()   # ★방어적: 이전 캔/병 attach 잔존 시 정리(이중 attach=크래시 방지)
                    except Exception:
                        pass
                    try:
                        _okb = motion_gen.attach_external_objects_to_robot(
                            joint_state=_refresh_cujs(), external_objects=[_held_bag],
                            surface_sphere_radius=0.005,
                            sphere_fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE)
                        print(f"  [과자봉지][attach] {'✅' if _okb else '❌'} 강체 프록시 부착 "
                              f"dims={np.round(_bag_d, 3)}×1.1", flush=True)
                    except Exception as _eb:
                        print(f"  [과자봉지][attach] 실패(무시): {_eb}", flush=True)
                    # 리프트 — ★곧장 +z 직선 상승(move_linear_ik, 자세고정). plan_single은 파지가 책상 바로 위라
                    #   '시작상태 충돌'로 실패→direct IK 폴백이 자세 플립(그리퍼 회전·책상 관통, 사용자 지적). moveL은 플립 없음.
                    _lift_from = _ee_side(_snack_cx, _snack_cy, _bag_mid)
                    _lift_to   = _ee_side(_snack_cx, _snack_cy, _bag_mid + 0.12)   # 짧은 직선상승(책상 클리어)·IK안전(긴 상승은 자세유지 도달실패)
                    move_linear_ik(_lift_from, _lift_to, ik_solver, tensor_args, _refresh_cujs().position,
                                   arm_joint_names, robot_art, ctrl, my_world, tag="봉지리프트", settle=0)
                    if args.rigid_bag:
                        toggle_table_collision(stage, True)   # ★[#2b] 리프트 후 책상 콜리전 복원
                    save_shot("snack_05_lift")
                    _TACT.mark("snack_bag", "carry")
                    # ── [봉지 거치대 적치 — 하이브리드] cuRobo plan_single = 매대 앞 도달(충돌회피·reach 담당),
                    #   moveL = 거치대 칸 안 짧은 틸트 진입+하강(좁고 의도된 접촉). 순수 cuRobo는 좁은칸+틸트 무충돌경로 없어 실패.
                    import math as _math
                    # ★side 진입 + 틸트 점진 보간: 매대 앞은 틸트 0°(IK reach 확보) → 적치 포인트에서 -30°(빗면 기댐).
                    #   move_linear_ik가 start/target 회전이 다를 때 t에 따라 자동 보간(pp_motion 개선).
                    _TILT_START  =   0.0   # 진입 시작(매대 앞) 틸트 — IK 가능 범위
                    _TILT_END    = -25.0   # 적치 포인트 틸트 — 빗면에 납작면 기댐
                    _SX  = 0.330
                    _SYf = 0.450
                    _pre_y   = SHELF3_PRE_Y
                    _entry_z = 1.285
                    def _stand_pose_tilt(_xyz, tilt_deg):
                        _b = side_grasp_from_approach(SHELF3_APPROACH, _xyz, RHP12_TCP_DEPTH)
                        _th = _math.radians(tilt_deg); _c, _s = _math.cos(_th), _math.sin(_th)
                        _b[:3, :3] = np.array([[1., 0, 0], [0, _c, -_s], [0, _s, _c]]) @ _b[:3, :3]
                        return _b
                    # (1) cuRobo plan_single: 매대 앞(틸트 -25°)까지 도달 → moveL은 y축 평행 이동만(틸트 고정)
                    _ok_carry = _plan_move(_stand_pose_tilt([_SX, _pre_y, _entry_z], _TILT_END), "carry_above", arm_only=True)
                    save_shot("snack_06_carry")
                    print(f"[과자봉지] 매대 앞 도달 {'✅' if _ok_carry else '❌'} (cuRobo, side tilt{_TILT_END}°) "
                          f"TCP=[{_SX:.2f},{_pre_y:.2f},{_entry_z:.3f}]", flush=True)
                    _TACT.mark("snack_bag", "place")
                    # (2) moveL +y 진입: 틸트 고정(-25°), 순수 y축 평행 이동만.
                    move_linear_ik(_stand_pose_tilt([_SX, _pre_y, _entry_z], _TILT_END),
                                   _stand_pose_tilt([_SX, _SYf,  _entry_z], _TILT_END),
                                   ik_solver, tensor_args, _refresh_cujs().position,
                                   arm_joint_names, robot_art, ctrl, my_world, tag="+y진입", settle=0)
                    save_shot("snack_07_placed")
                    # (4) 해제 — ★soften(물성복귀) 안 함. 사용자 관찰: soften 시 cloth가 빗면에서 흘러내림.
                    #   대신 '놓는 순간의 placed 자세 그대로' 봉지를 동결 유지. 파티클 off 상태로 snack_follow만 떼면
                    #   메시가 rest(원점)로 복귀하므로, placed 월드 점을 매 스텝 고정하는 hold 콜백으로 잡아둠.
                    if not args.rigid_bag:
                        _mesh_sb = UsdGeom.Mesh(stage.GetPrimAtPath("/World/snack_bag"))
                        # ★placed 월드 점 = EE 변환 재계산(snack_follow와 동일 공식). GetPointsAttr().Get()은 USD rest(원점)를
                        #   반환하므로(.Set은 Fabric에 씀) 그걸로 고정하면 원점복귀 → 반드시 EE@_Plocal로 재계산.
                        _M_rel = UsdGeom.Xformable(stage.GetPrimAtPath(ee_prim_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                        _Wfix = [_Gf.Vec3f(_M_rel.Transform(_pl)) for _pl in _Plocal]
                        try: my_world.remove_physics_callback("snack_follow")
                        except Exception: pass
                        def _snack_hold(dt):
                            _mesh_sb.GetPointsAttr().Set(_Wfix)
                        my_world.add_physics_callback("snack_hold", _snack_hold)
                    else:
                        # [B안 강체] 고정조인트가 봉지를 그리퍼에 잡고 있음 → 그리퍼 오픈 후 조인트 삭제로 자연 낙하(아래).
                        print("  [B안] 봉지 고정조인트 유지 — 그리퍼 오픈 후 조인트 삭제로 중력 낙하", flush=True)
                    try:
                        motion_gen.detach_object_from_robot()   # carry/insert까지 attach 유지 → 여기서 분리(home plan은 봉지 무관)
                        print("  [과자봉지][detach] 프록시 분리(적치 후)", flush=True)
                    except Exception as _ed:
                        print(f"  [과자봉지][detach] 무시: {_ed}", flush=True)
                    set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=40)   # 그리퍼 오픈(조인트가 봉지 유지)
                    if args.rigid_bag:
                        # ★실제 물리 낙하: 그리퍼 오픈 직후 고정조인트 삭제 → 봉지가 즉시 중력으로 낙하·거치대 안착.
                        #   봉지는 이미 dynamic + 콜리전 항상 ON(FilteredPairs로 그리퍼만 제외) → 거치대/매대에 정상 충돌.
                        try:
                            stage.RemovePrim("/World/snack_bag_grip_joint")
                            print("  [B안] 봉지 고정조인트 삭제 → 즉시 중력 낙하·거치대 자연 안착", flush=True)
                        except Exception as _edrop:
                            print(f"  [B안] 조인트 삭제 실패(무시): {_edrop}", flush=True)
                        for _ in range(10): my_world.step(render=True)   # 최소 대기(조인트 삭제 반영)
                    else:
                        for _ in range(40): my_world.step(render=True)
                    _TACT.mark("snack_bag", "home")
                    # (5a) -y 직선 후진 — 틸트 고정(-25°), 순수 y축 후진.
                    move_linear_ik(_stand_pose_tilt([_SX, _SYf,  _entry_z], _TILT_END),
                                   _stand_pose_tilt([_SX, _pre_y, _entry_z], _TILT_END),
                                   ik_solver, tensor_args, _refresh_cujs().position,
                                   arm_joint_names, robot_art, ctrl, my_world, tag="-y이탈", settle=0)
                    save_shot("snack_08_retract")
                    # (5b) home 복귀 — 칸 밖(앞)에서 시작 → cuRobo 충돌회피(plan_single_js)로 안전 복귀.
                    _goal_hs = JointState(position=retract_t.view(1, -1), joint_names=list(arm_joint_names))
                    _rhs = motion_gen.plan_single_js(_refresh_cujs().unsqueeze(0), _goal_hs, plan_config)
                    if _rhs.success.item():
                        execute_plan(motion_gen.get_full_js(_rhs.get_interpolated_plan()),
                                     sim_js_names, robot_art, ctrl, my_world, extra_steps=1, track_tag="home", arm_only=True)
                    else:
                        print("  [과자봉지] home plan_single_js 실패", flush=True)
                    for _ in range(50): my_world.step(render=True)
                    _TACT.mark("snack_bag", "done")
                    save_shot("snack_09_settled")
                    # [진단] 안착 후 봉지 최저점 z vs 매대판(SHELF3_FLOOR_TOP) → 관통/여유 + 봉지 중심 xy 정량 측정
                    #   (와이드 뷰 PNG로 판정 어려워 추가 — 음수=관통, 양수=여유. 보정변수 조정 근거.)
                    try:
                        _bbS = UsdGeom.Boundable(stage.GetPrimAtPath("/World/snack_bag")).ComputeWorldBound(
                            Usd.TimeCode.Default(), UsdGeom.Tokens.default_).ComputeAlignedBox()
                        _bminz = float(_bbS.GetMin()[2])
                        _bcx = (float(_bbS.GetMin()[0]) + float(_bbS.GetMax()[0])) / 2.0
                        _bcy = (float(_bbS.GetMin()[1]) + float(_bbS.GetMax()[1])) / 2.0
                        _lip_top = SHELF3_FLOOR_TOP + 0.006 + 0.029   # 받침턱 윗면 근사
                        _gapmm = (_bminz - SHELF3_FLOOR_TOP) * 1000.0
                        print(f"[과자봉지][진단] 봉지바닥 z={_bminz:.3f} (매대판 {SHELF3_FLOOR_TOP:.3f}/받침턱윗면≈{_lip_top:.3f}) → "
                              f"{'매대판 관통 '+format(-_gapmm,'.0f')+'mm ❌' if _gapmm < 0 else '매대판 여유 '+format(_gapmm,'.0f')+'mm ✅'}; "
                              f"봉지중심 xy=[{_bcx:.3f},{_bcy:.3f}] (거치대 x={_SX:.2f},받침턱 y={_SYf:.2f})", flush=True)
                    except Exception as _ed2:
                        print(f"[과자봉지][진단] 봉지 바운드 측정 실패: {_ed2}", flush=True)
                    print("[과자봉지] 매대 3층 적치 완료(top-down 파지→side 재배향→병 시퀀스).", flush=True)
                    if args.mixed:
                        targets[cur_tgt_i]["status"] = "placed"
                        obj_type = "snack_done"   # 게이트 해제 → IDLE이 다음 타겟/완료 처리
                        state = GS.IDLE
                    else:
                        _TACT.report()            # 단독 모드는 여기서 텍타임 표(HALT 유지)
                else:
                    print("[과자봉지] 접근 IK 실패", flush=True)
                _snack_done = True
            continue   # snack은 강체 상태머신 미진입(전용 핸들러)

        # target_cube=None: 봉지(전용 핸들러, obj=None) 처리 직후 obj_type=snack_done로 여기 도달 →
        #   강체 타겟 블록은 건너뛰고 아래 상태머신(IDLE)이 다음 타겟/완료 처리. (None 참조 크래시 방지)
        if target_cube is None:
            cube_pos = None
        else:
            cube_pos, _ = target_cube.get_world_pose()

        # [구체VIZ] 링크부착형(시작 시 1회)이라 매 스텝 재그리기 불필요 — USD 계층이 자동 추종.

        # [프레임검증] cuRobo FK(측정관절)를 월드로 변환 vs 실제 EE 월드 — 좌표계 일치 1회 확인.
        #   차이≈0 → base offset/회전 정확(데드존은 실제). 차이 큼 → 프레임이 범인.
        if step == 45:
            try:
                _fk = ik_solver.fk(cu_js.position.view(1, -1))
                _ee_cur = _fk.ee_position.view(-1).cpu().numpy()
                _pred = _ee_cur + np.asarray(robot_base)
                _act = get_ee_world_pos(stage, ee_prim_path)
                print(f"[프레임검증] cuRobo FK EE(base)={_ee_cur.round(3)} → +base 월드예측={_pred.round(3)}", flush=True)
                if _act is not None:
                    print(f"[프레임검증] 실제 EE 월드={np.asarray(_act).round(3)} "
                          f"차이(실제-예측)={(np.asarray(_act)-_pred).round(3)}m", flush=True)
            except Exception as _e:
                print(f"[프레임검증] 실패: {_e}", flush=True)

        # ══════════════════════════════════════════════════════════════════════
        #                    픽앤플레이스 상태머신 (실행 순서대로)
        #   [단계1] IDLE          : 다물체 파지 우선순위 → 타겟 선택 (+혼합 객체별 설정 활성화)
        #   [단계2] QUERY_GRASP   : 그랩젠 생성 (점구름→파지 후보; 누운 물체=캔축/병캡 합성)
        #   [단계3] PLAN_GRASP    : 쿠로보 파지점 선택(plan_grasp goalset) + 접근/파지(닫기)
        #   [단계4] PLAN_LIFT→MOVE_LIFT : 리프트 + 든 물체 attach
        #   [단계5] PLAN_CARRY    : 매대 앞 운반(plan_single 충돌회피)
        #   [단계6] INSERT_SHELF→LOWER_SHELF : +y 진입 → -z 하강 안착 + 그리퍼 열기
        #   [단계7] RETREAT_SHELF→GO_HOME    : -y 후퇴 → home 복귀(plan_single_js) → 다음 타겟
        #   (보조) HOLD/RELEASE/HALT : 실패·정지 처리
        #   ※ 헬퍼는 모듈로 분리: 파지기하=pp_geometry, 모션실행/간격=pp_motion.
        # ══════════════════════════════════════════════════════════════════════
        if state == GS.IDLE:
            if step > 60:
                # [Phase3 다물체] pending 타겟 순회. 실현 가능한 빈 슬롯(기하+IK+간격 사전검사)
                #   없으면 남은 타겟 일괄 no_slot 스킵 — 잡고 나서 갈 곳 없는 상황 차단.
                if args.objects > 1:
                    if not args.mixed:   # 혼합은 객체별 층(각자 단일 중앙슬롯)이라 다물체 슬롯 사전검사 생략
                        _half_s = CYLSPEC["height"] / 2.0
                        _ez = SHELF3_LIP_TOP + SHELF3_ENTRY_CLR + _half_s + GRIP_Z_OFFSET_EST
                        _pz = SHELF3_FLOOR_TOP + SHELF3_REST_CLR + _half_s + GRIP_Z_OFFSET_EST
                        _uxy = [SHELF3_SLOTS[i] for i, u in enumerate(slot_used) if u]
                        _any_ok = any(
                            slot_feasible(SHELF3_SLOTS[i], _ez, _pz, ik_solver, tensor_args,
                                          cu_js.position, motion_gen, _uxy)[0]
                            for i, u in enumerate(slot_used) if not u)
                        if not _any_ok:
                            for _t in targets:
                                if _t["status"] == "pending":
                                    _t["status"], _t["reason"] = "skipped", "no_slot"
                    _pend = [i for i, t in enumerate(targets) if t["status"] == "pending"]
                    _idle_done = not _pend
                else:
                    _idle_done = cycle >= args.cycles
                if _idle_done:
                    if args.objects > 1:
                        _pl = sum(1 for t in targets if t["status"] == "placed")
                        _sk = [(t["path"].split("/")[-1], t["reason"])
                               for t in targets if t["status"] == "skipped"]
                        print(f"\n✅ [다물체] 완료 — 적치 {_pl}/{len(targets)}"
                              f"{', 스킵 ' + str(_sk) if _sk else ''}. 장면 유지(HALT).", flush=True)
                    else:
                        print(f"\n✅ {args.cycles}사이클 종료. 장면 유지(HALT).", flush=True)
                    _TACT.report()
                    state = GS.HALT
                else:
                    if args.objects > 1:
                        # 타겟 선택(사용자 지시 2026-06-15): **매대 먼쪽(y작은) + 로봇 가까운(x작은) 먼저.**
                        #   바깥쪽(로봇 앞·매대 반대)부터 치워 후속 carry가 남은 물체 위를 안 지나게 → 충돌·모션 효율.
                        #   1순위 y오름차순(매대 먼쪽), 동률이면 x오름차순(로봇 가까운). 실측 좌표 기준.
                        def _tgt_key(_ti):
                            try:
                                _p = targets[_ti]["obj"].get_world_pose()[0]
                                return (float(_p[1]), float(_p[0]))
                            except Exception:
                                _sx = targets[_ti].get("spawn_xy")   # 봉지(obj=cloth): spawn y로 정렬
                                return (float(_sx[1]), float(_sx[0])) if _sx else (1e9, 1e9)
                        cur_tgt_i  = min(_pend, key=_tgt_key)
                        target_cube = targets[cur_tgt_i]["obj"]   # 이후 코드는 별칭으로 현재 타겟 참조
                        if args.mixed:
                            # [혼합] 이 타겟의 타입·spec·층 지오메트리·파지높이로 전역 전환(이후 grasp/place가 사용).
                            _t = targets[cur_tgt_i]
                            obj_type = _t["type"]
                            grasp_mode = OBJ_SPECS[obj_type]["grasp_mode"]
                            CYLSPEC = _t["spec"]
                            GRASP_HEIGHT_FRAC = _t["frac"]
                            if _t["level"] == 2:
                                SHELF3_FLOOR_TOP, SHELF3_LIP_TOP = 0.95, 0.96
                                # ★진입 클리어런스 5→9mm(사용자: 매대앞 정지 시 중력 처짐으로 진입이 앞턱/바닥에 박음).
                                #   천장여유 +10mm 내라 안전(처짐이 캔 전체를 낮춰 천장은 오히려 멀어짐).
                                SHELF_CEIL, SHELF3_ENTRY_CLR = 1.11, 0.009
                            else:
                                SHELF3_FLOOR_TOP, SHELF3_LIP_TOP = 1.14, 1.15
                                SHELF_CEIL, SHELF3_ENTRY_CLR = None, 0.04
                            SHELF3_SLOTS = _MIX_LSLOTS[_t["level"]]     # 층별 슬롯(2개)
                            slot_used = _MIX_LUSED[_t["level"]]         # 층별 점유(참조 — 적치 시 누적)
                            cur_slot = 0
                            print(f"  [혼합] 활성: type={obj_type} → {_t['level']}층 "
                                  f"(grasp_frac={GRASP_HEIGHT_FRAC}, ceil={SHELF_CEIL}, "
                                  f"슬롯점유={slot_used})", flush=True)
                            if obj_type == "snack":
                                # 봉지 타겟(obj=cloth): 강체 안정화/QUERY 건너뛰고 top-of-loop 전용 핸들러로(다음 iteration)
                                _TACT.mark("snack_bag", "grasp")
                                print(f"\n[다물체] 타겟 {cur_tgt_i+1}/{len(targets)} (봉지) → squish 핸들러", flush=True)
                                state = GS.IDLE
                                continue
                        tgt_retry  = 0
                        _tp = targets[cur_tgt_i]["obj"].get_world_pose()[0]
                        print(f"\n[다물체] 타겟 {cur_tgt_i+1}/{len(targets)} "
                              f"({targets[cur_tgt_i]['path']}, 바깥쪽 우선 y={_tp[1]:+.2f} x={_tp[0]:.2f}) 시작", flush=True)
                    _TACT.mark(targets[cur_tgt_i]["path"].split("/")[-1], "grasp")
                    # 캔 안정화 대기 (재배치 후 떨어지며 흔들림/기울어짐 → 멈출 때까지).
                    #   안 기다리면 grasp 계산 위치와 실제가 어긋나 그리퍼가 캔 놓침(캔-EE≫TCP).
                    for _si in range(180):
                        my_world.step(render=(_si % 6 == 0))   # [런타임] 안정화 대기는 렌더 드물게(물리는 매 스텝)
                        try:
                            if np.linalg.norm(target_cube.get_linear_velocity()) < 0.004:
                                break
                        except Exception:
                            break
                    cube_pos, _ = target_cube.get_world_pose()   # 안정 후 재측정
                    # ★넘어진 캔 감지: 직립이면 중심 z≈책상top+half. 그보다 낮으면(눕음 z≈top+r)
                    #   잡으려 들지 말고 직립 재배치(눕은 캔 파지는 전부 무의미한 실패).
                    _half_i = (CYLSPEC["height"] / 2 if _obj_type in ("cylinder", "bottle")
                               else CUBE_SIZE / 2)
                    if cube_pos[2] < _table_top + _half_i - 0.012 and not can_is_lying(target_cube):
                        print(f"  [IDLE] 캔 넘어짐 감지(z={cube_pos[2]:.3f} < 직립 "
                              f"{_table_top+_half_i:.3f}) → 직립 재배치", flush=True)
                        target_cube.set_world_pose(
                            position=np.array([cube_pos[0], cube_pos[1], _table_top + _half_i + 0.002]),
                            orientation=np.array([1.0, 0.0, 0.0, 0.0]))
                        try:
                            target_cube.set_linear_velocity(np.zeros(3))
                            target_cube.set_angular_velocity(np.zeros(3))
                        except Exception:
                            pass
                        continue   # IDLE 유지 → 다음 iteration서 settle 후 재측정
                    cycle += 1
                    cube_tgt_pos = cube_pos.copy()
                    print(f"\n[사이클 {cycle}/{args.cycles}] 큐브(안정)={cube_tgt_pos.round(3)}", flush=True)
                    # ── product_view: 비전 탐지 자세 경유 + 3초 인식 (캔/병) ──
                    _go_product_view(cu_js)
                    state    = GS.QUERY_GRASP if grasp_client is not None else GS.PLAN_PREGRASP
                    wait_cnt = 0
                    fail_cnt = 0   # ★사이클 시작에만 리셋(QUERY 재진입마다 리셋하면 무한 재선별)

        elif state == GS.QUERY_GRASP:
            # ★넘어짐 가드(사이클 중 재진입 포함): 접근/재파지 중 캔이 넘어가면 잡으려 들지 말고
            #   직립 재배치 후 사이클 재시작(눕은 캔 파지는 전부 무의미한 실패 + 무한루프 원인).
            _cz_now, _ = target_cube.get_world_pose()
            _half_q = (CYLSPEC["height"] / 2 if _obj_type in ("cylinder", "bottle") else CUBE_SIZE / 2)
            if _cz_now[2] < _table_top + _half_q - 0.012 and not can_is_lying(target_cube):
                print(f"  [QUERY] 캔 넘어짐/이탈 감지(z={_cz_now[2]:.3f}) → 직립 재배치, 사이클 재시작", flush=True)
                target_cube.set_world_pose(
                    # 현재 타겟의 실측 x,y에 재배치(다물체: _cx,_cy는 중앙 캔 자리라 틀림)
                    position=np.array([_cz_now[0], _cz_now[1], _table_top + _half_q + 0.002]),
                    orientation=np.array([1.0, 0.0, 0.0, 0.0]))
                try:
                    target_cube.set_linear_velocity(np.zeros(3))
                    target_cube.set_angular_velocity(np.zeros(3))
                except Exception:
                    pass
                cycle -= 1
                state  = GS.IDLE
                continue
            # 접근 전 그리퍼 열기 (열린 손가락 사이로 캔이 들어와야 닫을 때 감쌈)
            set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=25)
            # ★Phase2: 타겟 캔을 "이번 사이클 한정" 장애물로 등록. ignore 상태로는 접근 경로가
            #   캔을 쓸고 지나가도 플래너가 모름(클러터로 좁아지면 실제로 캔을 쳐 넘어뜨림).
            #   그리퍼 구체는 손가락 표면뿐(사이 빔)이라 캔을 감싼 최종 파지 자세는 무충돌 → 목표 성립.
            #   파지(닫기) 후 enable_obstacle(False)로 해제(공식 attach_objects_to_robot 내부와 동일 메커니즘).
            try:
                _obs_t = usd_help.get_obstacles_from_stage(
                    only_paths=["/World"], reference_prim_path=robot_prim_path,
                    ignore_substring=[robot_prim_path, "/World/defaultGroundPlane", "/curobo",
                                      "/World/cspheres", "/World/grasp_viz", "/World/debug_pc",
                                      ROBOT_BASE_BLOCK_PATH],
                ).get_collision_check_world()
                motion_gen.update_world(_obs_t)
                print("  [충돌월드] 타겟 캔 포함 동기화(접근 스윕 방지)", flush=True)
            except Exception as _e:
                print(f"  [충돌월드] 타겟 포함 동기화 실패(기존 월드 유지): {_e}", flush=True)
            # ── GraspGen 추론 → 변환 → 선택 → 시각화 ──────────────────────────
            cube_quat = target_cube.get_world_pose()[1]      # [w,x,y,z]
            _w, _x, _y, _z = cube_quat
            obj_R = Rotation.from_quat([_x, _y, _z, _w]).as_matrix()

            print("  [GraspGen] 점구름 샘플링 & 추론...", flush=True)
            pc_obj   = sample_object_pc(obj_type)
            pc_world = (obj_R @ pc_obj.T).T + cube_tgt_pos
            # 점구름 시각화(초록)
            pc_prim.CreatePointsAttr().Set(
                _Vt.Vec3fArray([_Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pc_world]))
            pc_prim.CreateWidthsAttr().Set(_Vt.FloatArray([0.003] * len(pc_world)))
            pc_prim.CreateDisplayColorAttr().Set(
                _Vt.Vec3fArray([_Gf.Vec3f(0.2, 0.8, 0.2)] * len(pc_world)))

            # [단계2: 그랩젠 생성] 점구름→파지 후보(월드). 추론·변환은 pp_phases.query_graspgen으로 분리.
            grasps_w, scores = query_graspgen(grasp_client, pc_obj, cube_tgt_pos, cube_quat, num_grasps=400)
            if len(grasps_w) == 0:
                if args.objects > 1:
                    tgt_retry += 1
                    if tgt_retry >= 2:   # [Phase3] 같은 타겟 2회 무파지 → 스킵하고 다음 타겟
                        targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "no_grasp"
                        print(f"  [다물체] {targets[cur_tgt_i]['path']} 스킵(no_grasp)", flush=True)
                        state = GS.IDLE
                    else:
                        print("  [GraspGen] 파지 없음 → 재시도", flush=True)
                        state = GS.IDLE
                else:
                    print("  [GraspGen] 파지 없음 → 재시도", flush=True)
                    state = GS.IDLE; cycle -= 1
            else:
                print(f"  [변환] robotiq→RH-P12 Z={ROBOTIQ_TO_RHP12_Z:.4f}m", flush=True)
                _obj_half_h = OBJ_SPECS[obj_type].get("height", 0.0) / 2.0   # 캔 반높이(side 게이트용)
                if can_is_lying(target_cube):
                    # 누운 캔: GraspGen/side선택 대신 캔축 기준 위에서 파지 합성(X=캔축 → carry가 직립화).
                    #   ★그리퍼는 캔축 ±180° 대칭(손가락 교체) → +축/−축 두 후보를 다 주고 plan_grasp이
                    #     손목 덜 트는 쪽 선택(불필요한 j6 180° 플립 방지). 직립 결과는 둘 다 동일.
                    _can_axis = obj_R @ np.array([0.0, 0.0, 1.0])   # 물체 로컬 +z(월드). 병은 이게 캡 방향.
                    if obj_type == "bottle":
                        # ★병은 위아래(캡) 있음 → 재배향 후 캡이 위로 가야 함. 그리퍼 X축(=재배향 후 위축=카메라축)을
                        #   캡(+_can_axis)에 맞추는 +캡 후보만 채택 → 적치 시 캡-위 + 카메라-위 동시 보장(joint_6 롤이 이 정렬).
                        grasp_cands = [g for g in (lying_grasp_from_axis(+_can_axis, cube_tgt_pos, RHP12_TCP_DEPTH),)
                                       if g is not None]
                        print(f"  [누운픽] 병 캡 방향={_can_axis.round(2)} → +캡 후보만(재배향 시 캡-위·카메라-위)", flush=True)
                    else:
                        # 캔(대칭): ±축 모두 주고 plan_grasp이 손목 최소 선택
                        grasp_cands = [g for g in (lying_grasp_from_axis(+_can_axis, cube_tgt_pos, RHP12_TCP_DEPTH),
                                                   lying_grasp_from_axis(-_can_axis, cube_tgt_pos, RHP12_TCP_DEPTH))
                                       if g is not None]
                        print(f"  [누운픽] 캔축={_can_axis.round(2)} → ±축 후보 {len(grasp_cands)}개(대칭, 손목 최소)", flush=True)
                    grasp_world = grasp_cands[0] if grasp_cands else None
                    pre_world = (pregrasp_from_grasp(grasp_world, PREGRASP_STANDOFF)
                                 if grasp_world is not None else None)
                else:
                    grasp_world, pre_world, grasp_cands = select_best_reachable_grasp(
                        grasps_w, scores, ik_solver, tensor_args, cu_js, robot_base, retract_t,
                        approach_z_max=APPROACH_Z_MAX, obj_R=obj_R, grasp_mode=grasp_mode,
                        obj_center=cube_tgt_pos, obj_half_h=_obj_half_h)
                if grasp_world is None and not can_is_lying(target_cube) and grasp_mode == "top":
                    print("  [IK 필터] 수직 파지 없음 → 사선 완화 재시도", flush=True)
                    grasp_world, pre_world, grasp_cands = select_best_reachable_grasp(
                        grasps_w, scores, ik_solver, tensor_args, cu_js, robot_base, retract_t,
                        approach_z_max=APPROACH_Z_MAX_RELAX, obj_R=obj_R, grasp_mode="top")
                # 파지 후보 시각화. ★side(캔)는 GraspGen 후보를 안 쓰고 azimuth 스윕으로 재합성하므로
                #   raw 후보(허공에 뜸)는 그리지 않고 실제 선택된 합성 파지만 표시(오해 방지).
                if grasp_mode == "side" and grasp_world is not None:
                    # ★전 후보 표시(approach 축). 실제 선택은 plan_grasp(goalset) 후 RGB축으로 갱신됨.
                    _cl = np.array(grasp_cands) if grasp_cands else np.array([grasp_world])
                    draw_grasp_candidates_usd(stage, _cl, np.ones(len(_cl)), selected_T=None)
                else:
                    _order = np.argsort(scores)[::-1]
                    draw_grasp_candidates_usd(stage, grasps_w[_order], scores[_order],
                                              selected_T=grasp_world)
                for _ in range(3):           # 시각화 렌더 반영
                    my_world.step(render=True)
                save_shot("grasps")          # 파지 후보+선택 PNG
                if grasp_world is None:
                    # 진단: 이 위치에서 E0509가 낼 수 있는 접근방향(top/side) 탐침
                    probe_reachable_approaches(cube_tgt_pos, ik_solver, tensor_args,
                                               cu_js, robot_base)
                    if args.objects > 1:   # [Phase3] feasible-first: 이 타겟 스킵, 다음으로
                        targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "unreachable"
                        print(f"  [다물체] {targets[cur_tgt_i]['path']} 스킵(unreachable)", flush=True)
                        state = GS.IDLE
                    else:
                        print("  [GraspGen] 도달 가능 파지 없음 → HALT(파지 후보는 화면 확인 가능)", flush=True)
                        state = GS.HALT
                else:
                    print(f"  [파지] 후보 {len(grasp_cands)}개 → plan_grasp(goalset)가 월드 기준 선택", flush=True)
                    # ★Phase2.5: side는 후보 "전체"를 plan_grasp에 넘겨 플래너가 상황적합 선택.
                    #   (룰베이스 가중치 선택 폐기 — 클러터/책상/캔을 아는 월드모델이 직접 고름)
                    state = GS.PLAN_GRASP if grasp_cands else GS.PLAN_PREGRASP

        elif state == GS.PLAN_PREGRASP:
            if grasp_client is not None:
                # GraspGen이 고른 pre-grasp (자세 포함)
                pre_pos  = pre_world[:3, 3]
                cpose    = mat4_to_curobo_pose(pre_world, tensor_args)
                _q = Rotation.from_matrix(pre_world[:3, :3]).as_quat()   # xyzw
                pre_quat = [float(_q[3]), float(_q[0]), float(_q[1]), float(_q[2])]
                pre_quat_dbg = "GraspGen"
            else:
                # 고정 탑다운(Step1): 큐브 위 18cm
                pre_pos  = np.array([cube_tgt_pos[0], cube_tgt_pos[1], cube_tgt_pos[2] + 0.18])
                cpose    = xyz_to_curobo_pose(pre_pos, [1,0,0,0], tensor_args)
                pre_quat = [1, 0, 0, 0]
                pre_quat_dbg = "[1,0,0,0]"
            print(f"  [1] Pre-grasp 플래닝 → {pre_pos.round(3)} ({pre_quat_dbg})", flush=True)
            result   = motion_gen.plan_single(cu_js.unsqueeze(0), cpose, plan_config)
            if result.success.item():
                cmd = motion_gen.get_full_js(result.get_interpolated_plan())
                execute_plan(cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                             track_tag="pre-grasp")
                print(f"  [2] Pre-grasp 도달", flush=True)
                state    = GS.PLAN_GRASP
                fail_cnt = 0
            else:
                fail_cnt += 1
                if fail_cnt == 1:
                    diag_pose_fail("pre", pre_pos, pre_quat, ik_solver, result, tensor_args)
                print(f"  [1] 플래닝 실패 ({fail_cnt}/{MAX_FAIL})", flush=True)
                if fail_cnt >= MAX_FAIL:
                    print("  [1] pre-grasp 도달 불가(이 위치) → 캔 재배치 후 다음 사이클", flush=True)
                    set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=20)
                    regrasp_cnt = 0
                    cycle -= 1
                    state    = GS.RELEASE
                    fail_cnt = 0

        elif state == GS.PLAN_GRASP:
            _grasp_reached = False
            _fail_status   = None
            if grasp_cands:
                # ★Phase2.5(공식 plan_grasp): 후보 묶음 → goalset이 월드(클러터·캔·책상) 기준 best 선택.
                #   2단계 내장: ①offset(standoff)까지 풀충돌인지 ②직선 최종진입(손가락 링크만 충돌면제,
                #   캔을 감싸야 하므로). 룰베이스 walk/메트릭 폐기.
                _N = len(grasp_cands)
                _pl = np.zeros((1, _N, 3), dtype=np.float32)
                _ql = np.zeros((1, _N, 4), dtype=np.float32)
                for _i, _c in enumerate(grasp_cands):
                    _pl[0, _i] = _c[:3, 3] - _ROBOT_BASE_OFFSET          # 월드→base
                    _q = Rotation.from_matrix(_c[:3, :3]).as_quat()      # xyzw
                    _ql[0, _i] = [_q[3], _q[0], _q[1], _q[2]]            # wxyz
                gposes = Pose(position=tensor_args.to_device(_pl),
                              quaternion=tensor_args.to_device(_ql))
                # 누운 캔도 top-down 접근(-tool Z). ★축방향(tool X) 슬라이드-인 시도했으나(개선10-b) plan_grasp
                #   IK 도달불가(테이블 위 누운 실린더는 그리퍼X=캔축 제약상 top-down만 가능, 2026-06-15 실측).
                #   → top-down 유지(미세 그라즈 잔존하나 2/2 성공). 깨끗한 해법은 two-phase pivot/place-and-pick.
                print(f"  [3] plan_grasp(goalset {_N}후보, 2단계 진입) ...", flush=True)
                gres = motion_gen.plan_grasp(
                    cu_js.unsqueeze(0), gposes, plan_config.clone(),
                    grasp_approach_offset=Pose.from_list([0, 0, -PREGRASP_STANDOFF, 1, 0, 0, 0]),
                    disable_collision_links=list(GRIPPER_COLL_LINKS),
                    plan_grasp_to_retract=False,   # 리프트는 닫은 뒤 plan_single로 별도 수행
                )
                if gres.success.item():
                    _gi = int(gres.goalset_index.item())
                    grasp_world = grasp_cands[_gi]
                    grasp_pos   = grasp_world[:3, 3]
                    print(f"  [3] goalset 선택 #{_gi+1}/{_N} → {grasp_pos.round(3)} "
                          f"approach={grasp_world[:3,2].round(2)}", flush=True)
                    # ★viz 갱신: 실제 로봇이 파지할 후보(goalset 선택)를 RGB 좌표축으로 표시
                    #   (이전엔 룰베이스 g_sel을 그려 실제 파지점과 어긋났음 — 사용자 지적 2026-06-11)
                    draw_grasp_candidates_usd(stage, np.array(grasp_cands),
                                              np.ones(len(grasp_cands)), selected_T=grasp_world)
                    for _ in range(2):
                        my_world.step(render=True)
                    # ★진단(2026-06-11): 접근 어느 구간이 캔을 치는지 분리 실행해 캔 이동 측정.
                    #   2단계(홈→pregrasp, 그리퍼 충돌 켜짐) vs 3단계(pregrasp→grasp 직선, 그리퍼 충돌 꺼짐).
                    _can_q0, _ = target_cube.get_world_pose()
                    cmd_a = motion_gen.get_full_js(gres.approach_result.get_interpolated_plan())
                    execute_plan(cmd_a, sim_js_names, robot_art, ctrl, my_world, extra_steps=1)
                    _can_q1, _ = target_cube.get_world_pose()
                    print(f"  [진단접근] 2단계(홈→pregrasp) 후 캔={np.round(_can_q1,3)} "
                          f"이동={np.round(_can_q1-_can_q0,3)} |Δ|={np.linalg.norm(_can_q1-_can_q0):.3f}", flush=True)
                    cmd_g = motion_gen.get_full_js(gres.grasp_result.get_interpolated_plan())
                    execute_plan(cmd_g, sim_js_names, robot_art, ctrl, my_world, extra_steps=1, track_tag="grasp")
                    _can_q2, _ = target_cube.get_world_pose()
                    print(f"  [진단접근] 3단계(pregrasp→grasp직선) 후 캔={np.round(_can_q2,3)} "
                          f"이동={np.round(_can_q2-_can_q1,3)} |Δ|={np.linalg.norm(_can_q2-_can_q1):.3f}", flush=True)
                    print(f"  [4] Grasp 위치 도달", flush=True)
                    log_arm_deg(robot_art, arm_joint_names, "grasp")
                    _grasp_reached = True
                else:
                    _fail_status = getattr(gres, "status", None)
            else:
                # top/box 경로(기존 plan_single 단일 목표)
                if grasp_client is not None:
                    grasp_pos = grasp_world[:3, 3]
                    cpose     = mat4_to_curobo_pose(grasp_world, tensor_args)
                else:
                    grasp_pos = np.array([cube_tgt_pos[0], cube_tgt_pos[1], cube_tgt_pos[2] + 0.13])
                    cpose     = xyz_to_curobo_pose(grasp_pos, [1, 0, 0, 0], tensor_args)
                print(f"  [3] Grasp 접근(plan_single, 충돌회피) → {grasp_pos.round(3)}", flush=True)
                result = motion_gen.plan_single(cu_js.unsqueeze(0), cpose, plan_config)
                if result.success.item():
                    cmd = motion_gen.get_full_js(result.get_interpolated_plan())
                    execute_plan(cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1, track_tag="grasp")
                    print(f"  [4] Grasp 위치 도달", flush=True)
                    _grasp_reached = True
                else:
                    _fail_status = getattr(result, "status", None)

            if _grasp_reached:
                _can_b, _ = target_cube.get_world_pose()
                _ee_b = get_ee_world_pos(stage, ee_prim_path)
                print(f"  [4-진단] 닫기前 캔={np.round(_can_b,3)} EE={None if _ee_b is None else np.round(_ee_b,3)} "
                      f"(캔-EE 수평거리={np.hypot(_can_b[0]-(_ee_b[0] if _ee_b is not None else 0), _can_b[1]-(_ee_b[1] if _ee_b is not None else 0)):.3f})", flush=True)
                save_shot("pre_close")   # 닫기 직전 그리퍼-캔
                # ★적응형 파지각: GRIP_CLOSE(1.05, 간격8mm)는 캔(60mm)을 뚫고 계속 닫혀 과침투.
                #   물체 지름−6mm 간격까지만 닫아 접촉 직전 정지(견고 그립, 과침투 방지). effort cap이 그립력 담당.
                _obj_d = 2.0 * CYLSPEC["radius"]               # 캔 0.06 / 병 0.07
                _grip_close = grip_angle_for_gap(max(0.02, _obj_d - 0.002))   # ★−6mm→−2mm: 원위 curl 강체침투↓(사용자 관찰 회전·박힘 완화)
                print(f"  [4] 적응형 파지각 q={_grip_close:.2f}rad (지름 {_obj_d*1000:.0f}mm) — 과침투 방지", flush=True)
                set_gripper(ctrl, robot_art, sim_js_names, my_world, _grip_close, steps=70)
                _gjp = robot_art.get_joint_positions()
                _ga = float(_gjp[robot_art.get_dof_index("gripper_rh_r1")])
                # Stage6: 원위(r2/l2) 각도 — r2>r1이면 부족구동 curl(감싸쥠) 발생 증명
                _ga2 = float(_gjp[robot_art.get_dof_index("gripper_rh_r2")])
                _gl2 = float(_gjp[robot_art.get_dof_index("gripper_rh_l2")])
                for _ in range(12):      # 닫은 뒤 잠시 쥐고 안정(대기 단축: 45→12)
                    my_world.step(render=(_ % 3 == 0))
                _can_a, _ = target_cube.get_world_pose()
                save_shot("post_close")
                print(f"  [4.5] 그리퍼 닫음 r1={_ga:.3f} r2={_ga2:.3f} l2={_gl2:.3f}rad"
                      f"{' (curl: 원위>근위 감싸쥠)' if min(_ga2,_gl2) > _ga + 0.05 else ''}. "
                      f"닫기後 캔={np.round(_can_a,3)} (이동={np.round(_can_a-_can_b,3)})", flush=True)
                for _ in range(3):
                    my_world.step(render=True)
                save_shot("grasp_reached")
                grip_z_offset = float(grasp_world[2, 3] - _can_a[2])
                if can_is_lying(target_cube):
                    # 누운 캔은 TCP가 캔 중심(축 방향)에서 잡혀 → 직립 재배향 후 TCP=캔중심 → 오프셋 0.
                    #   (파지 시점 측정값은 누운 상태 기하라 적치엔 무의미)
                    grip_z_offset = 0.0
                    print("  [누운픽] grip z-offset=0 강제(재배향 후 TCP=캔중심)", flush=True)
                print(f"  [4.6] grip z-offset={grip_z_offset:+.3f}m (캔중심이 TCP보다 이만큼 아래) "
                      f"→ 안착/진입 높이 보정에 반영", flush=True)
                # ★Phase2: 잡았으니 타겟 캔 장애물 해제 → lift/carry 시작상태가 충돌로 안 찍힘.
                #   (들고 있는 캔 부피는 attach 프록시가 담당. 다음 사이클 QUERY 동기화가 다시 등록)
                try:
                    _tkey = targets[cur_tgt_i]["path"].split("/")[-1]   # "obj_i" 또는 "target_cube"
                    _tnames = [o.name for o in motion_gen.world_model.objects
                               if _tkey in o.name]
                    for _tn in _tnames:
                        motion_gen.world_coll_checker.enable_obstacle(enable=False, name=_tn)
                    print(f"  [충돌월드] 타겟 캔 장애물 해제({len(_tnames)}개, key={_tkey})", flush=True)
                except Exception as _e:
                    print(f"  [충돌월드] 타겟 해제 실패: {_e}", flush=True)
                state    = GS.PLAN_LIFT   # [복원] 클리어 리프트(+z 0.12) 후 운반 — 든 물체가 이웃 위로 올라가 간섭 방지(MoveIt post_grasp_retreat 정석)
                wait_cnt = 0
                fail_cnt = 0
            else:
                # plan_grasp 전체 실패(goalset에 도달가능 후보 없음) 또는 top 경로 실패.
                #   직접IK 우회 안 함 → 재선별/재배치. (후보별 walk는 goalset이 내부에서 대체)
                fail_cnt += 1
                print(f"  [3] Grasp 접근 실패 ({fail_cnt}/{MAX_FAIL}) status={_fail_status} → 재선별/재배치", flush=True)
                if fail_cnt >= MAX_FAIL:
                    set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=20)
                    regrasp_cnt = 0
                    if args.objects > 1:   # [Phase3] 재배치 대신 스킵 → 다음 타겟
                        targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "unreachable"
                        print(f"  [다물체] {targets[cur_tgt_i]['path']} 스킵(unreachable, plan_grasp {MAX_FAIL}회 실패)", flush=True)
                        state = GS.IDLE
                    else:
                        print("  [3] 안전 접근 불가(이 위치) → 캔 재배치 후 다음 사이클", flush=True)
                        cycle -= 1                 # 이 사이클은 무효 처리(재배치만)
                        state    = GS.RELEASE
                    fail_cnt = 0
                else:
                    state = GS.QUERY_GRASP if grasp_client is not None else GS.PLAN_PREGRASP

        elif state == GS.PLAN_LIFT:
            # 물리 파지 리프트: kinematic attach 없이, 그리퍼 닫은 채 grasp 자세 유지하며 +z 상승.
            #   캔은 마찰로 딸려 올라와야 함(파지 성공 판정). 자세는 grasp 그대로(옆파지 유지).
            lift_world = grasp_world.copy()
            lift_world[2, 3] += 0.12    # 원복: +0.12(검증값). ★병위로 1.075 들기는 먼 DR위치서 도달불가→리프트 실패→폐기.
            #   carry-펫트병 충돌은 cuRobo 장애물 회피(병=메시 장애물, +margin)로 처리(per-object DR로 위치도 분리됨).
            # ★리프트도 plan_single(충돌회피). grasp→+z 상승 동안 손목이 책상에 안 박히게 궤적 검사.
            lift_cpose = mat4_to_curobo_pose(lift_world, tensor_args)
            res_lift   = motion_gen.plan_single(cu_js.unsqueeze(0), lift_cpose, plan_config)
            print(f"  [6] 리프트(plan_single, 회피) → z={lift_world[2,3]:.3f}m, "
                  f"{'OK' if res_lift.success.item() else '실패'}", flush=True)
            if res_lift.success.item():
                cmd = motion_gen.get_full_js(res_lift.get_interpolated_plan())
                execute_plan(cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                             track_tag="lift", arm_only=True)   # 그리퍼 닫힘 유지(캔 떨굼 방지)
                for _ in range(20):       # 정착
                    my_world.step(render=True)
                state = GS.MOVE_LIFT
            elif args.objects > 1:
                # [Phase3] 리프트 플랜 실패: 그리퍼 열고 같은 타겟 재선별 1회 → 재실패 시 스킵.
                set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=30)
                tgt_retry += 1
                if tgt_retry >= 2:
                    targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "lift_plan_fail"
                    print(f"  [다물체] {targets[cur_tgt_i]['path']} 스킵(lift_plan_fail)", flush=True)
                    _goal_h = JointState(position=retract_t.view(1, -1), joint_names=list(arm_joint_names))
                    _rh = motion_gen.plan_single_js(cu_js.unsqueeze(0), _goal_h, plan_config)
                    if _rh.success.item():
                        execute_plan(motion_gen.get_full_js(_rh.get_interpolated_plan()),
                                     sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                                     track_tag="reset_home")
                    state = GS.IDLE
                else:
                    print("  [6] 리프트 plan 실패 → 그리퍼 열고 같은 타겟 재선별", flush=True)
                    for _ in range(120):
                        my_world.step(render=True)
                        try:
                            if np.linalg.norm(target_cube.get_linear_velocity()) < 0.004:
                                break
                        except Exception:
                            break
                    cube_tgt_pos = target_cube.get_world_pose()[0].copy()
                    state = GS.QUERY_GRASP
            else:
                print("  [6] 리프트 plan_single 실패 → 잡은 채 정지(HOLD)", flush=True)
                state = GS.HOLD

        elif state == GS.MOVE_LIFT:
            # [stage8] 들어서 파지확인하는 단계 제거. 닫음을 신뢰하고 바로 attach → 통합 운반(운반이 곧 들어올림).
            save_shot("lift")
            if args.place:
                # 든 물체 부피를 robot에 attach → 통합 carry/place plan_single이 무충돌 인지(attached_object 링크).
                try:
                    _cp_w, _cq_w = target_cube.get_world_pose()          # 월드(pos, [w,x,y,z])
                    _cp_b = (np.asarray(_cp_w) - _ROBOT_BASE_OFFSET).astype(float)
                    _sc2 = CYLSPEC
                    _held = Cuboid(
                        name="held_can",
                        pose=[float(_cp_b[0]), float(_cp_b[1]), float(_cp_b[2]),
                              float(_cq_w[0]), float(_cq_w[1]), float(_cq_w[2]), float(_cq_w[3])],
                        dims=[2 * _sc2["radius"], 2 * _sc2["radius"], _sc2["height"]],
                    )
                    _ok_at = motion_gen.attach_external_objects_to_robot(
                        joint_state=cu_js,
                        external_objects=[_held],
                        surface_sphere_radius=0.005,
                        sphere_fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE,
                    )
                    attached = bool(_ok_at) if _ok_at is not None else True
                    print(f"  [attach] {'✅' if attached else '❌'} 든 물체 부피 부착(cuRobo 충돌인지)", flush=True)
                except Exception as _e:
                    print(f"  [attach] 실패(무시하고 진행): {_e}", flush=True)
                _TACT.mark(targets[cur_tgt_i]["path"].split("/")[-1], "carry")
                state = GS.PLAN_CARRY
            else:
                state = GS.HOLD     # 비-place(파지 데모)는 잡은 채 유지
            wait_cnt = 0

        elif state == GS.HOLD:
            wait_cnt += 1                # 잡은 채 잠시 유지 (그리퍼 닫힘 타겟 persist)
            if wait_cnt > 90:
                state    = GS.RELEASE
                wait_cnt = 0

        elif state == GS.RELEASE:
            set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_OPEN, steps=40)
            print("  [8] 그리퍼 열기(해제)", flush=True)
            if args.objects > 1:
                # [Phase3] 다물체: 단일 모드의 캔 텔레포트 재배치 금지(남은 캔과 겹침 위험).
                #   홈 복귀만 하고 IDLE로 — 타겟 상태는 도달 경로(스킵/HOLD)에서 이미 기록됨.
                _goal_h = JointState(position=retract_t.view(1, -1), joint_names=list(arm_joint_names))
                _rh = motion_gen.plan_single_js(cu_js.unsqueeze(0), _goal_h, plan_config)
                if _rh.success.item():
                    execute_plan(motion_gen.get_full_js(_rh.get_interpolated_plan()),
                                 sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                                 track_tag="reset_home")
                print("  [다물체] RELEASE: 재배치 없이 홈 복귀 → 다음 타겟", flush=True)
                state    = GS.IDLE
                wait_cnt = 0
                continue   # 단일 모드 재배치(텔레포트) 코드로 안 내려가게 차단
            if cycle >= args.cycles:
                print(f"\n✅ {args.cycles}사이클 완료. 종료합니다.", flush=True)
                simulation_app.close()
                break
            # ★재배치 전 로봇 홈 복귀(사용자 요청): 캔을 새로 생성하기 전에 팔을 홈으로 빼
            #   → 새 캔 생성위치와 팔이 분리되고, 다음 파지를 깨끗한 홈에서 시작(접근 일관·캔 안 침).
            _goal_h = JointState(position=retract_t.view(1, -1), joint_names=list(arm_joint_names))
            _rh = motion_gen.plan_single_js(cu_js.unsqueeze(0), _goal_h, plan_config)
            if _rh.success.item():
                _cmd = motion_gen.get_full_js(_rh.get_interpolated_plan())
                execute_plan(_cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1, track_tag="reset_home")
                print("  [재배치] 로봇 홈 복귀 → 캔 재생성", flush=True)
            else:
                print("  [재배치] 홈 복귀 plan 실패(현 자세 유지) → 캔 재생성", flush=True)
            # 도달 가능 범위로 제한 (forward ≥ ~0.47m 유지: x는 -0.03~+0.08만 → 너무 가까워 HALT 방지)
            if args.dr:
                # [Phase6 DR] nominal 기준 위치 랜덤화(검증된 파지창 내). 실린더는 yaw 대칭→직립 유지.
                new_x = _nom_x + np.random.uniform(-0.03, 0.05)
                new_y = _nom_y + np.random.uniform(-0.07, 0.07)
                _ori_r = np.array([1.0, 0.0, 0.0, 0.0])
                print(f"  [DR] 재배치 위치 → [{new_x:.2f},{new_y:.2f}]", flush=True)
            else:
                new_x = _cx + np.random.uniform(-0.03, 0.08)
                new_y = _cy + np.random.uniform(-0.10, 0.10)
                _ori_r = np.array([1.0, 0.0, 0.0, 0.0])
            # 직립 방향 리셋 + 속도 0 (이전 사이클 기울기/속도가 남으면 캔이 기울어 안착=0.73 → grasp 실패)
            target_cube.set_world_pose(position=np.array([new_x, new_y, _cz]),
                                       orientation=_ori_r)
            try:
                target_cube.set_linear_velocity(np.zeros(3))
                target_cube.set_angular_velocity(np.zeros(3))
            except Exception:
                pass
            set_kinematic(stage, "/World/target_cube", False)
            print(f"  큐브 재배치(직립) → [{new_x:.2f}, {new_y:.2f}, {_cz:.3f}]", flush=True)
            state    = GS.IDLE
            wait_cnt = 0

        elif state == GS.PLAN_CARRY:
            # 매대 3단 앞 '진입 높이'까지 plan_single(매대/책상 회피). 진입높이=앞턱+여유+half+그립오프셋
            #   → 캔 바닥이 앞턱(1.15) 위로 떠서 매대에 안 닿음. PRE_Y는 앞턱(0.37)에서 충분히 앞.
            _half_c = CYLSPEC["height"] / 2.0
            entry_z = SHELF3_LIP_TOP   + SHELF3_ENTRY_CLR + _half_c + grip_z_offset
            _pz_c   = SHELF3_FLOOR_TOP + SHELF3_REST_CLR  + _half_c + grip_z_offset
            # [Phase3] 첫 "실현 가능한" 빈 슬롯 선택(실측 grip_z_offset으로 본검사: 기하+IK+간격)
            _uxy_c = [SHELF3_SLOTS[i] for i, u in enumerate(slot_used) if u]
            cur_slot = -1
            for _si in range(len(SHELF3_SLOTS)):
                if slot_used[_si]:
                    continue
                _ok_s, _why_s = slot_feasible(SHELF3_SLOTS[_si], entry_z, _pz_c, ik_solver,
                                              tensor_args, cu_js.position, motion_gen, _uxy_c)
                if _ok_s:
                    cur_slot = _si
                    break
                print(f"  [슬롯] {_si+1} (x={SHELF3_SLOTS[_si][0]:.2f},y={SHELF3_SLOTS[_si][1]:.2f}) "
                      f"불가 — {_why_s}", flush=True)
            if cur_slot < 0:
                # 캔을 든 상태라 자동 복구 불가(되돌려 놓기 미구현) — 잡은 채 정지, 사용자 확인
                print("  [슬롯] 실현 가능한 슬롯 없음(파지 후 변동) → HOLD", flush=True)
                targets[cur_tgt_i]["status"], targets[cur_tgt_i]["reason"] = "skipped", "no_slot_runtime"
                state = GS.HOLD
                continue
            place_x, place_y = SHELF3_SLOTS[cur_slot]
            print(f"  [슬롯] {cur_slot+1}/{len(SHELF3_SLOTS)} (x={place_x:.2f}, y={place_y:.2f}) 사용", flush=True)
            # [복원] 기존 방식 — 매대 '앞'(PRE_Y) 진입높이까지 plan_single(회피) → INSERT(+y) → LOWER(-z).
            #   직행(매대 안으로 plan_single)은 천장 있는 2층에 박음 → 앞접근 후 직선 진입이 안전(검증된 시퀀스).
            pre_center  = [place_x, SHELF3_PRE_Y, entry_z]
            carry_world = side_grasp_from_approach(SHELF3_APPROACH, pre_center, RHP12_TCP_DEPTH)
            cpose = mat4_to_curobo_pose(carry_world, tensor_args)
            print(f"  [P1] 매대 앞 운반(plan_single, 회피) → TCP {np.round(pre_center,3)} "
                  f"(캔바닥≈{SHELF3_LIP_TOP+SHELF3_ENTRY_CLR:.3f}, 앞턱 위)", flush=True)
            result = motion_gen.plan_single(cu_js.unsqueeze(0), cpose, plan_config)
            if result.success.item():
                cmd = motion_gen.get_full_js(result.get_interpolated_plan())
                execute_plan(cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                             track_tag="carry", arm_only=True, viz=_viz_cb)   # 그리퍼 닫힘 유지(캔 떨굼 방지)
                save_shot("carry_preshelf")
                print("  [P2] 매대 앞 도달", flush=True)
                log_arm_deg(robot_art, arm_joint_names, "carry/매대앞")
                state = GS.INSERT_SHELF
            else:
                print("  [P1] 운반 plan_single 실패 → 직접IK 폴백", flush=True)
                if move_direct_ik(carry_world, ik_solver, tensor_args, cu_js.position,
                                  arm_joint_names, robot_art, ctrl, my_world, viz=_viz_cb):
                    save_shot("carry_preshelf"); state = GS.INSERT_SHELF
                else:
                    print("  [P1] 운반 IK도 실패 → 정지(HALT)", flush=True); state = GS.HALT

        elif state == GS.INSERT_SHELF:
            # moveL: 매대 앞(PRE) → 안(IN), +y로만 직선 이동(진입높이 유지 → 캔이 앞턱 위로 통과).
            _half_c = CYLSPEC["height"] / 2.0
            entry_z = SHELF3_LIP_TOP + SHELF3_ENTRY_CLR + _half_c + grip_z_offset
            ins_from  = side_grasp_from_approach(SHELF3_APPROACH, [place_x, SHELF3_PRE_Y, entry_z], RHP12_TCP_DEPTH)
            ins_world = side_grasp_from_approach(SHELF3_APPROACH, [place_x, place_y,      entry_z], RHP12_TCP_DEPTH)
            log_ceil_headroom("진입자세", ins_world, ik_solver, tensor_args, cu_js, robot_base,
                              _half_c, grip_z_offset)   # Stage7 2층 스파이크: 천장 간섭 예측
            print(f"  [P3] 매대 안 +y 진입(moveL) → y {SHELF3_PRE_Y}→{place_y} @TCPz {entry_z:.3f}", flush=True)
            ok = move_linear_ik(ins_from, ins_world, ik_solver, tensor_args, cu_js.position,
                                arm_joint_names, robot_art, ctrl, my_world, viz=_viz_cb, tag="+y진입",
                                settle=0)   # [Stage5] 하강과 연속(중력보상 후 추종 0.2° → 정착 불요)
            save_shot("inserted")
            state = GS.LOWER_SHELF if ok else GS.HALT

        elif state == GS.LOWER_SHELF:
            _TACT.mark(targets[cur_tgt_i]["path"].split("/")[-1], "place")
            _half_c = CYLSPEC["height"] / 2.0
            place_z = SHELF3_FLOOR_TOP + SHELF3_REST_CLR  + _half_c + grip_z_offset
            # [stage8] 진입높이 직행 후 마지막 짧은 -z 수직 하강(P4) — 톨 보틀 직립 보장(직행 안착은 기울어짐).
            entry_z = SHELF3_LIP_TOP + SHELF3_ENTRY_CLR + _half_c + grip_z_offset
            low_from  = side_grasp_from_approach(SHELF3_APPROACH, [place_x, place_y, entry_z], RHP12_TCP_DEPTH)
            low_world = side_grasp_from_approach(SHELF3_APPROACH, [place_x, place_y, place_z], RHP12_TCP_DEPTH)
            log_ceil_headroom("안착자세", low_world, ik_solver, tensor_args, cu_js, robot_base,
                              _half_c, grip_z_offset)
            print(f"  [P4] 하강 안착(moveL -z, 톨보틀 직립 보장) → @TCPz {place_z:.3f}", flush=True)
            move_linear_ik(low_from, low_world, ik_solver, tensor_args, cu_js.position,
                           arm_joint_names, robot_art, ctrl, my_world, viz=_viz_cb, tag="-z하강", settle=0)
            # ★매대 안에서는 부분 열림(GRIP_RELEASE) — 풀오픈은 근위바가 좌우로 벌어져 측벽/이웃캔 간섭 (Stage6 이식)
            set_gripper(ctrl, robot_art, sim_js_names, my_world, GRIP_RELEASE, steps=40)   # 캔 안착
            for _si in range(120):       # 캔 안정 대기
                my_world.step(render=(_si % 6 == 0))   # [런타임] 안정화 렌더 드물게
                try:
                    if np.linalg.norm(target_cube.get_linear_velocity()) < 0.004:
                        break
                except Exception:
                    break
            my_world.step(render=True)   # 캡처(placed) 직전 프레임 갱신
            _cp, _ = target_cube.get_world_pose()
            _upright = abs(_cp[2] - (SHELF3_FLOOR_TOP + _half_c)) < 0.03   # 캔중심≈바닥+half면 직립
            print(f"  [P5] 그리퍼 부분열림({GRIP_RELEASE}rad≈간격81mm, 매대간섭 회피). 캔 안착 월드={np.round(_cp,3)} "
                  f"({'직립✅' if _upright else '눕힘/기울어짐⚠'}, 직립기준 z≈{SHELF3_FLOOR_TOP+_half_c:.3f})", flush=True)
            log_arm_deg(robot_art, arm_joint_names, "place/안착")
            save_shot("placed")
            # [Phase3] 슬롯 점유 마크(직립 여부 무관 — 캔이 공간을 차지) + 타겟 상태 갱신
            slot_used[cur_slot] = True
            targets[cur_tgt_i]["status"] = "placed"
            if not _upright:
                targets[cur_tgt_i]["reason"] = "tilted"   # 적치는 됐으나 기울어짐(요약에 표시)
            if attached:   # Phase2: 캔이 매대에 놓였으니 분리 → 후퇴/home plan은 그리퍼만 인지
                try:
                    motion_gen.detach_object_from_robot()
                    attached = False
                    print("  [detach] 캔 분리(매대 안착 완료)", flush=True)
                except Exception as _e:
                    print(f"  [detach] 실패(무시): {_e}", flush=True)
            state = GS.RETREAT_SHELF   # [stage8] P6(+z 이탈) 복원 — 제거 시 home-exit가 톨 보틀을 쳐서 넘어뜨림(검증)

        elif state == GS.RETREAT_SHELF:
            # [P6] [사용자] +z 안 올라오고 바로 -y 직선 이탈(moveL, 진입 역순=기존 방식). place_z 유지.
            #   부분개방 그리퍼(간격81mm > 캔60·병70mm)라 -y로 빠지며 물체 안 끌고 슬라이드.
            #   home plan_single_js는 매대 밖(PRE_Y)서 시작 → 적치물/매대 안 침.
            _half_c = CYLSPEC["height"] / 2.0
            place_z = SHELF3_FLOOR_TOP + SHELF3_REST_CLR + _half_c + grip_z_offset
            out_from  = side_grasp_from_approach(SHELF3_APPROACH, [place_x, place_y,      place_z], RHP12_TCP_DEPTH)
            out_world = side_grasp_from_approach(SHELF3_APPROACH, [place_x, SHELF3_PRE_Y, place_z], RHP12_TCP_DEPTH)
            print(f"  [P6] -y 매대 밖 이탈(moveL, +z 생략) → y {place_y}→{SHELF3_PRE_Y} @TCPz {place_z:.3f}", flush=True)
            move_linear_ik(out_from, out_world, ik_solver, tensor_args, cu_js.position,
                           arm_joint_names, robot_art, ctrl, my_world, viz=_viz_cb, tag="-y이탈",
                           settle=0)
            state = GS.GO_HOME

        elif state == GS.GO_HOME:
            _TACT.mark(targets[cur_tgt_i]["path"].split("/")[-1], "home")
            # ★충돌회피 시연: 매대 앞 → home(retract)까지 plan_single_js(관절공간 충돌회피).
            #   직접IK 후라 cu_js는 이번 iteration 상단에서 측정값으로 갱신됨 → start 정확.
            goal_js = JointState(position=retract_t.view(1, -1),
                                 joint_names=list(arm_joint_names))
            print("  [P7] home 복귀(plan_single_js, 충돌회피)", flush=True)
            res_home = motion_gen.plan_single_js(cu_js.unsqueeze(0), goal_js, plan_config)
            if res_home.success.item():
                cmd = motion_gen.get_full_js(res_home.get_interpolated_plan())
                execute_plan(cmd, sim_js_names, robot_art, ctrl, my_world, extra_steps=1,
                             track_tag="home", viz=_viz_cb)
                log_arm_deg(robot_art, arm_joint_names, "home/복귀")
                print("  [P8] ✅✅ 매대 3단(맨 위) 배치 + home 복귀 완료.", flush=True)
            else:
                print("  [P7] home 복귀 플래닝 실패 → 현 자세 유지(HALT).", flush=True)
            save_shot("place_done")
            _TACT.mark(targets[cur_tgt_i]["path"].split("/")[-1], "done")
            # [Phase3] 다물체: 다음 pending 타겟으로 (IDLE이 잔여/슬롯 확인 후 HALT 결정)
            state = GS.IDLE if (args.objects > 1 and res_home.success.item()) else GS.HALT

        elif state == GS.HALT:
            # 진단 후 정지: 재플래닝 없이 장면만 유지 (사용자가 화면 확인)
            pass

    _TACT.report()
    simulation_app.close()


if __name__ == "__main__":
    main()
