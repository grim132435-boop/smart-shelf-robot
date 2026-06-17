# GraspGen 시각화 API 레퍼런스

> 조사일: 2026-06-02 | 출처: NVlabs/GraspGen 공식 레포 + 로컬 graspgen_ws

---

## 핵심 파일 위치 (로컬)

| 파일 | 역할 |
|------|------|
| `~/graspgen_ws/GraspGen/grasp_gen/utils/viser_utils.py` | Viser 기반 3D 시각화 (메인) |
| `~/graspgen_ws/GraspGen/grasp_gen/utils/meshcat_utils.py` | Meshcat 폴백 |
| `~/graspgen_ws/GraspGen/scripts/demo_object_pc.py` | 점구름 + 파지 시각화 예시 |
| `~/graspgen_ws/GraspGen/scripts/save_grasps_to_usd.py` | Isaac Sim USD 내보내기 |
| `~/shelf_grasp_dev/visualize_grasps.py` | 우리 프로젝트용 오프라인 시각화 |

---

## Viser API (viser_utils.py)

```python
from grasp_gen.utils.viser_utils import (
    create_visualizer,
    visualize_grasp,
    visualize_mesh,
    visualize_pointcloud,
    make_frame,
    get_color_from_score,
)

# 서버 시작 (브라우저: http://localhost:8080)
vis = create_visualizer(port=8080)

# 파지 자세 렌더링 (4x4 행렬 + 그리퍼 와이어프레임)
visualize_grasp(vis, name="grasp_0", T=grasp_4x4, color=color, gripper_name="robotiq_2f_140")

# 좌표계 프레임 (RGB 축)
make_frame(vis, name="frame_0", h=0.05, radius=0.002, T=grasp_4x4)

# 점구름
visualize_pointcloud(vis, name="pc", pc=points_Nx3, color=[100,200,100], size=0.003)

# 메시
visualize_mesh(vis, name="mesh", mesh=trimesh_obj, color=[180,180,200])

# 점수 기반 색상 (blue→red)
color = get_color_from_score(score)  # score ∈ [0,1]
```

### 색상 매핑
- `score ≈ 0` → 파란색 (낮은 신뢰도)
- `score ≈ 0.5` → 노란색
- `score ≈ 1` → 빨간색 (높은 신뢰도)

---

## USD 내보내기 (Isaac Sim 연동)

```python
from grasp_gen.scripts.save_grasps_to_usd import save_grasps_to_usd

save_grasps_to_usd(
    usd_path="grasps_debug.usd",
    grasps=grasps_Mx4x4,          # Robotiq 프레임
    confidences=scores_M,
    gripper_name="robotiq_2f_140"
)
```

Isaac Sim에서 해당 USD를 열면 모든 파지 자세를 3D로 확인 가능.

---

## 우리 visualize_grasps.py 사용법

```bash
# graspgen_venv 활성화 후
source ~/graspgen_ws/venv/bin/activate   # 또는 conda activate 환경

python ~/shelf_grasp_dev/visualize_grasps.py \
    --npz ~/shelf_grasp_dev/configs_draft/grasp_results_box_1780295156.npz

# 브라우저에서 http://localhost:8080 접속
```

- 상위 20개 파지 표시 (점수 내림차순)
- 점수 기반 컬러 그래디언트
- 그리퍼: robotiq_2f_140

---

## Stage 3 실시간 파지 시각화 방법 (미구현 → 작업 예정)

### 방법 A: Isaac Sim USD 프레임 추가 (권장)
`stage3_graspgen_curobo_gui.py`의 QUERY_GRASP 블록에 선택된 파지 + 후보군 좌표축을 USD로 그림.

```python
# 예시: 선택된 파지 좌표계 그리기
from omni.isaac.debug_draw import _debug_draw
draw = _debug_draw.acquire_debug_draw_interface()
# 또는 UsdGeom.Xform으로 프레임 생성
```

### 방법 B: 별도 Viser 창 병렬 실행
GraspGen 추론 직후 `viser_utils`로 후보 파지 전체를 웹 뷰어에 표시.

---

## 그리퍼 프레임 규약 (GraspGen)
- **Z축 (+)**: 접근 방향 (gripper가 물체 쪽으로 이동하는 방향)
- **X축**: 닫힘축 (손가락이 맞닿는 방향)
- **Y축**: X·Z에 수직
