#!/usr/bin/env python3
"""
Stage 3 파지 시각화 모듈 (perception_bridge / grasp_inference 보조)

두 가지 시각화를 제공:
  1) Isaac Sim USD 좌표축  : draw_grasp_candidates_usd() / clear_grasp_viz_usd()
     - 선택된 파지: RGB 3축 좌표계 (X=빨강 닫힘, Y=초록, Z=파랑 approach)
     - 후보 파지  : approach 축만, 점수 그라데이션 색 (파랑→빨강)
  2) Viser 웹뷰어         : ViserGraspViz
     - 그리퍼 와이어프레임 (franka_panda), 후보=점수색 / 선택=노랑 강조

좌표 규약 (GraspGen / 본 코드 4x4):
  +Z열 = approach(접근),  +X열 = closing(닫힘),  +Y열 = 나머지
"""

import sys
import numpy as np
from pathlib import Path

# ── viser 의존성 경로 (graspgen_ws) ──────────────────────────────────────────
_GRASPGEN_DIR = Path.home() / "graspgen_ws" / "GraspGen"
if str(_GRASPGEN_DIR) not in sys.path:
    sys.path.insert(0, str(_GRASPGEN_DIR))


# ── 공통: 점수 → RGB (파랑=낮음, 빨강=높음) ───────────────────────────────────
def score_to_rgb01(score: float) -> tuple:
    """0~1 점수 → (r,g,b) 0~1 범위. 파랑(0)→청록→노랑→빨강(1)."""
    s = float(np.clip(score, 0.0, 1.0))
    # 단순 blue→red 보간 (중간 초록 약간)
    r = s
    g = 1.0 - abs(0.5 - s) * 2.0
    b = 1.0 - s
    return (r, g, b)


def score_to_rgb255(score: float) -> list:
    r, g, b = score_to_rgb01(score)
    return [int(r * 255), int(g * 255), int(b * 255)]


# ══════════════════════════════════════════════════════════════════════════════
# 1) Isaac Sim USD 좌표축 시각화
# ══════════════════════════════════════════════════════════════════════════════

_VIZ_ROOT = "/World/grasp_viz"


def clear_grasp_viz_usd(stage):
    """이전 사이클의 파지 시각화 prim 모두 제거."""
    from pxr import Sdf
    root = stage.GetPrimAtPath(_VIZ_ROOT)
    if root.IsValid():
        stage.RemovePrim(Sdf.Path(_VIZ_ROOT))


def _draw_axis_line(stage, path, p0, p1, color_rgb01, width):
    """USD BasisCurves로 선분 1개(축 1개) 그리기."""
    from pxr import UsdGeom, Gf, Vt
    curve = UsdGeom.BasisCurves.Define(stage, path)
    curve.CreateTypeAttr().Set("linear")
    curve.CreateCurveVertexCountsAttr().Set(Vt.IntArray([2]))
    curve.CreatePointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(float(p0[0]), float(p0[1]), float(p0[2])),
        Gf.Vec3f(float(p1[0]), float(p1[1]), float(p1[2])),
    ]))
    curve.CreateWidthsAttr().Set(Vt.FloatArray([width, width]))
    curve.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
    curve.CreateDisplayColorAttr().Set(
        Vt.Vec3fArray([Gf.Vec3f(*color_rgb01)])
    )


def _draw_frame_axes(stage, path, T, axis_len, width):
    """4x4 T의 origin에서 RGB 3축(X빨강/Y초록/Z파랑) 좌표계 그리기."""
    origin = T[:3, 3]
    x_end = origin + T[:3, 0] * axis_len   # closing
    y_end = origin + T[:3, 1] * axis_len
    z_end = origin + T[:3, 2] * axis_len   # approach
    _draw_axis_line(stage, f"{path}/x", origin, x_end, (1.0, 0.1, 0.1), width)
    _draw_axis_line(stage, f"{path}/y", origin, y_end, (0.1, 1.0, 0.1), width)
    _draw_axis_line(stage, f"{path}/z", origin, z_end, (0.2, 0.4, 1.0), width)


def draw_grasp_candidates_usd(
    stage,
    candidates,          # (K,4,4) 월드 프레임 후보 (점수 내림차순 권장)
    scores,              # (K,) 점수
    selected_T=None,     # 4x4 선택된 파지 (강조), None 가능
    max_candidates=20,
    cand_axis_len=0.04,
    cand_width=0.0025,
    sel_axis_len=0.09,
    sel_width=0.006,
):
    """Isaac Sim 화면에 후보 파지(approach 축, 점수색) + 선택 파지(RGB 좌표계) 표시.

    매 호출 시 이전 시각화를 지우고 새로 그린다.
    """
    clear_grasp_viz_usd(stage)
    from pxr import UsdGeom
    UsdGeom.Xform.Define(stage, _VIZ_ROOT)

    # 후보: approach 축만 점수색으로
    k = min(max_candidates, len(candidates))
    for i in range(k):
        T = candidates[i]
        origin = T[:3, 3]
        approach_end = origin + T[:3, 2] * cand_axis_len
        col = score_to_rgb01(float(scores[i]))
        _draw_axis_line(stage, f"{_VIZ_ROOT}/cand_{i:02d}",
                        origin, approach_end, col, cand_width)

    # 선택: RGB 3축 좌표계 (굵고 길게)
    if selected_T is not None:
        _draw_frame_axes(stage, f"{_VIZ_ROOT}/selected",
                         selected_T, sel_axis_len, sel_width)

    print(f"  [VIZ-USD] 후보 {k}개(approach축) + "
          f"{'선택1개(RGB좌표계)' if selected_T is not None else '선택없음'} 표시",
          flush=True)


# ══════════════════════════════════════════════════════════════════════════════
# 2) Viser 웹뷰어 시각화
# ══════════════════════════════════════════════════════════════════════════════

class ViserGraspViz:
    """별도 브라우저(localhost:port)에 그리퍼 와이어프레임으로 파지 표시.

    Isaac Sim 메인 루프와 동일 프로세스에서 백그라운드 스레드로 동작.
    초기화 실패(viser 미설치 등) 시 자동으로 비활성화되어 파이프라인을 막지 않음.
    """

    def __init__(self, port: int = 8081, gripper_name: str = "franka_panda"):
        self.enabled = False
        self.vis = None
        self.gripper_name = gripper_name
        self.port = port
        try:
            from grasp_gen.utils.viser_utils import create_visualizer
            self.vis = create_visualizer(port=port)
            self.enabled = True
            print(f"  [VIZ-Viser] 웹뷰어 시작: http://localhost:{port}", flush=True)
        except Exception as e:
            print(f"  [VIZ-Viser] 비활성화 (초기화 실패: {e})", flush=True)

    def update(self, candidates, scores, selected_T=None,
               point_cloud_world=None, max_candidates=20):
        """후보 파지 + 선택 파지 + (선택)점구름 갱신.

        candidates: (K,4,4) 월드 프레임, scores: (K,), selected_T: 4x4 or None
        """
        if not self.enabled:
            return
        try:
            from grasp_gen.utils.viser_utils import (
                visualize_grasp, visualize_pointcloud,
            )
            # 이전 프레임 지우기
            self.vis.scene.reset()

            if point_cloud_world is not None:
                visualize_pointcloud(self.vis, "scene/pc", point_cloud_world,
                                     color=[100, 200, 100], point_size=0.004)

            k = min(max_candidates, len(candidates))
            for i in range(k):
                col = score_to_rgb255(float(scores[i]))
                visualize_grasp(self.vis, f"cand/{i:02d}", candidates[i],
                                color=col, gripper_name=self.gripper_name,
                                linewidth=1.5)

            if selected_T is not None:
                visualize_grasp(self.vis, "selected", selected_T,
                                color=[255, 255, 0], gripper_name=self.gripper_name,
                                linewidth=4.0)
            print(f"  [VIZ-Viser] 후보 {k}개 + "
                  f"{'선택(노랑)' if selected_T is not None else '선택없음'} 갱신",
                  flush=True)
        except Exception as e:
            print(f"  [VIZ-Viser] 갱신 실패: {e}", flush=True)
