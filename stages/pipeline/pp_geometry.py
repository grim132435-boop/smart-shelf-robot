# E0509 파지 기하 헬퍼(순수 함수) — 파지자세 합성·프레임 변환·누운픽. stage 본체와 독립, 모션팀 재사용/머지 단위.
"""1단계 모듈화: 런타임 전역(my_world·motion_gen·CYLSPEC 등) 의존 없는 '순수' 기하 함수만 분리.
의존: numpy / trimesh / scipy.Rotation + 상수 ROBOTIQ_TO_RHP12_Z. (IK·cuRobo 의존 함수는 main/별 모듈에 남김.)"""
import numpy as np
import trimesh
import trimesh.transformations as tra
from scipy.spatial.transform import Rotation

# robotiq(GraspGen 학습) → RH-P12-RN EE Z 깊이 보정(두 그리퍼 TCP 깊이 ≈ 동일 → 0).
ROBOTIQ_TO_RHP12_Z = 0.0

# 파워에이드 600ml 컨투어 프로파일 (u: 0=바닥 1=꼭대기, r=반지름 m, 최대 0.035 기준).
#   실측 사진 기반: 굽 → 하단 그립부(가로 리브 볼록 + 홈 오목) → 허리(오목) → 라벨 원통 → 어깨 → 넥 → 캡.
_BOTTLE_PROFILE = [
    (0.00, 0.0335), (0.02, 0.0340), (0.045, 0.0330), (0.07, 0.0340),  # ★평평·넓은 굽(r0.0335, 직립 안정 — 좁은굽이 기울임 유발)
    (0.13, 0.0342), (0.19, 0.0292), (0.26, 0.0340),                    # 리브(볼록)/홈(오목)
    (0.33, 0.0298), (0.40, 0.0270), (0.47, 0.0300), (0.53, 0.0340),    # 허리(오목)~볼록
    (0.59, 0.0285), (0.64, 0.0350), (0.78, 0.0350),                    # 잘록→라벨 원통
    (0.83, 0.0340), (0.88, 0.0285), (0.92, 0.0150),                    # 어깨 테이퍼
    (0.945, 0.0130), (0.965, 0.0180), (1.00, 0.0185),                  # 넥→캡
]


def make_bottle_mesh(radius, height, sections=48):
    """파워에이드 컨투어 회전체 trimesh(점구름·시각·충돌 공용). radius=최대반지름, z중심정렬([-h/2,+h/2])."""
    us = np.array([p[0] for p in _BOTTLE_PROFILE], dtype=np.float64)
    rs = np.array([p[1] for p in _BOTTLE_PROFILE], dtype=np.float64) * (radius / 0.035)
    zs = (us - 0.5) * height
    th = np.linspace(0.0, 2 * np.pi, sections, endpoint=False)
    nr = len(zs)
    verts = [[r * np.cos(t), r * np.sin(t), z] for z, r in zip(zs, rs) for t in th]
    faces = []
    for i in range(nr - 1):
        for j in range(sections):
            a, b = i * sections + j, i * sections + (j + 1) % sections
            c, d = (i + 1) * sections + j, (i + 1) * sections + (j + 1) % sections
            faces += [[a, b, d], [a, d, c]]
    bc = len(verts); verts.append([0.0, 0.0, zs[0]])      # 바닥 중심
    tc = len(verts); verts.append([0.0, 0.0, zs[-1]])     # 윗면 중심
    for j in range(sections):
        faces.append([bc, (j + 1) % sections, j])                                          # 바닥 캡
        faces.append([tc, (nr - 1) * sections + j, (nr - 1) * sections + (j + 1) % sections])  # 윗면 캡
    return trimesh.Trimesh(vertices=np.array(verts), faces=np.array(faces), process=False)


def robotiq_grasp_to_rhp12(grasp_4x4):
    """robotiq(GraspGen 학습) EE 프레임 → RH-P12-RN EE 프레임 Z 깊이 보정.
    GraspGen FAQ: new = grasp @ T([0,0,Z]). Z는 그리퍼 접촉깊이 차이(근사 0.095, 시각화로 보정)."""
    return grasp_4x4 @ tra.translation_matrix([0, 0, ROBOTIQ_TO_RHP12_Z])


def grasp_to_world(grasp_obj, obj_world_pos, obj_world_quat=None):
    """오브젝트 프레임 파지 → 월드 프레임 (물체 위치+회전 반영). quat=[w,x,y,z]."""
    T = np.eye(4)
    T[:3, 3] = obj_world_pos
    if obj_world_quat is not None:
        w, x, y, z = obj_world_quat
        T[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
    return T @ grasp_obj


def is_in_workspace(pos, base):
    """E0509 작업공간 필터 (로봇 base 기준 상대). base=[x,y,z]."""
    dx, dy, dz = pos[0] - base[0], pos[1] - base[1], pos[2] - base[2]
    r = np.sqrt(dx**2 + dy**2)
    return (0.15 < r < 0.85 and        # base로부터 수평 거리
            -0.35 < dz < 0.55 and      # base 기준 높이
            dx > 0.05)                 # 로봇 앞쪽(+x)


def pregrasp_from_grasp(grasp_world, standoff):
    pre = grasp_world.copy()
    approach = grasp_world[:3, 2]              # +Z 열 = approach
    pre[:3, 3] = grasp_world[:3, 3] - approach * standoff
    return pre


def snap_grasp_roll_90(grasp_4x4, obj_R=None):
    """닫힘축(X)을 물체 면(로컬 X/Y)에 90° 스냅 → 평행면 파지 보장 (모서리 파지 방지)."""
    R        = grasp_4x4[:3, :3]
    approach = R[:, 2]
    closing  = R[:, 0]
    grip_y   = R[:, 1]
    if obj_R is not None:
        ax, ay = obj_R[:3, 0], obj_R[:3, 1]
        candidates = [ax, -ax, ay, -ay]
    else:
        candidates = [np.array([1., 0, 0]), np.array([-1., 0, 0]),
                      np.array([0, 1., 0]), np.array([0, -1., 0])]
    dots = [np.dot(closing, c) for c in candidates]
    best = candidates[int(np.argmax(dots))]
    best_perp = best - np.dot(best, approach) * approach
    nrm = np.linalg.norm(best_perp)
    if nrm < 1e-6:
        return grasp_4x4
    best_perp /= nrm
    cos_a = np.clip(np.dot(closing, best_perp), -1, 1)
    sin_a = np.dot(np.cross(closing, best_perp), approach)
    angle = np.arctan2(sin_a, cos_a)
    if abs(angle) < 0.05:
        return grasp_4x4
    c, s = np.cos(angle), np.sin(angle)
    def rot(v):
        return c*v + s*np.cross(approach, v) + (1-c)*np.dot(approach, v)*approach
    new_R = np.column_stack([rot(closing), rot(grip_y), approach])
    out = grasp_4x4.copy()
    out[:3, :3] = new_R
    return out


def side_grasp_from_approach(a, obj_center, tcp_depth):
    """수평 approach 벡터로 RH-P12 side 파지 합성.
      - Z(approach)      = 캔 축을 향한 수평 방향 (옆면 수직 진입)
      - Y(손가락 분리축)  = 수평 접선  ← RH-P12는 base Y로 닫힘 (URDF: 손가락 ±y, x축 회전)
      - X(그리퍼 상하축) = 위(+z)로 강제 → 그리퍼 상단의 비전(RSD455)이 하늘 향함(책상 충돌 방지)
      - EE 위치 = 캔 중심 − tcp_depth·approach  → TCP(손가락 사이)가 캔 축 중앙에
    수평 성분 거의 없으면 None."""
    a = np.asarray(a, dtype=float).copy()
    a[2] = 0.0
    n = np.linalg.norm(a)
    if n < 1e-3:
        return None
    a /= n
    Y = np.array([-a[1], a[0], 0.0])
    X = np.cross(Y, a); X /= (np.linalg.norm(X) + 1e-9)
    if X[2] < 0:                       # X(그리퍼 상단/카메라축)가 하늘 향하도록
        X, Y = -X, -Y
    Y = np.cross(a, X); Y /= (np.linalg.norm(Y) + 1e-9)
    out = np.eye(4)
    out[:3, 0], out[:3, 1], out[:3, 2] = X, Y, a
    out[:3, 3] = np.asarray(obj_center, dtype=float) - tcp_depth * a
    return out


def synthesize_side_grasp_rhp12(grasp_4x4, obj_center, tcp_depth):
    """GraspGen 후보의 수평 방위만 취해 합성 (side_grasp_from_approach 래퍼)."""
    return side_grasp_from_approach(grasp_4x4[:3, 2], obj_center, tcp_depth)


# ============================ >>> LYING-PICK MERGE <<< ============================
# 누운 캔/병 파지 — 모션팀 통합 대상. lying_grasp_from_axis + can_is_lying 두 함수가 핵심.
#   사용 패턴(main): if can_is_lying(obj): _ax = obj_R @ [0,0,1];
#       캔(대칭)= ±_ax 후보, 병(위아래有)= +_ax(캡)만 → 적치 시 캡-위+카메라-위.
#   재배향은 별도 코드 없음 — 위 grasp 프레임(X=물체축) + 직립 placement pose(X=위)의 결과로 자동 직립.
def lying_grasp_from_axis(can_axis, obj_center, tcp_depth):
    """누운 캔(축 수평) 위에서 파지 합성 — Stage7 누운캔 픽.
    ★핵심: 그리퍼 X축(상하축)을 캔 축에 맞춤 → 이후 carry/place가 X=위로 강제하면 캔이 직립으로 섬.
      - X(그리퍼 상하축) = 캔 축(수평)
      - Z(approach)      = 위에서 아래(-z)로 진입 (X에 수직 보정)
      - Y(손가락 분리축)  = Z×X (캔 지름 가로질러 닫힘)
      - EE 위치 = 캔 중심 − tcp_depth·Z (TCP가 캔 중심, 그리퍼는 위로 tcp_depth 떨어짐)"""
    X = np.asarray(can_axis, dtype=float).copy()
    X[2] = 0.0                          # 수평 성분만(축이 살짝 기울어도 수평 투영)
    n = np.linalg.norm(X)
    if n < 1e-3:
        return None
    X /= n
    Z = np.array([0.0, 0.0, -1.0])      # 위에서 진입
    Z = Z - np.dot(Z, X) * X            # X에 수직화(X 수평이라 이미 수직이지만 안전)
    Z /= (np.linalg.norm(Z) + 1e-9)
    Y = np.cross(Z, X); Y /= (np.linalg.norm(Y) + 1e-9)
    out = np.eye(4)
    out[:3, 0], out[:3, 1], out[:3, 2] = X, Y, Z
    out[:3, 3] = np.asarray(obj_center, dtype=float) - tcp_depth * Z
    return out


def can_is_lying(obj):
    """캔이 누워 있는지(축이 수평) 판정 — 타겟별 누운픽 자동 감지(혼합 다물체용).
    축 z성분 |az|<0.5면 눕힘. 실패 시 False(직립 가정)."""
    try:
        _w, _x, _y, _z = obj.get_world_pose()[1]
        az = Rotation.from_quat([_x, _y, _z, _w]).as_matrix()[:, 2][2]
        return abs(float(az)) < 0.5
    except Exception:
        return False
# ============================ <<< LYING-PICK MERGE END >>> ========================
