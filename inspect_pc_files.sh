#!/usr/bin/env bash
# inspect_pc_files.sh
# 목적: point cloud가 입력되어 실행되는 파일을 "읽기 전용"으로 찾아 보여줌.
# 수정/생성/삭제 없음. 팀 레포(smart-shelf-robot)는 ROOTS에 넣지 않음 = 미탐색.

# ── 탐색 루트 (본인 경로에 맞게 수정) ─────────────────────────
ROOTS=(
  "$HOME/shelf_grasp_dev"   # 내 연습 워크스페이스
  "$HOME/graspgen_ws"       # GraspGen 클론
)

PATTERN='demo_object_pc|sample_data_dir|point_cloud|pointcloud|\.npy|\.npz|\.pcd|\.ply|np\.load|open3d|trimesh|GraspGen|infer_grasp|run_grasp'
EXCL='--exclude-dir=.git --exclude-dir=__pycache__ --exclude-dir=.venv --exclude-dir=venv --exclude-dir=graspgen_venv --exclude-dir=GraspGenModels --exclude-dir=datasets'

echo "######## 1) 디렉토리 구조 (.py만, 깊이 3) ########"
for R in "${ROOTS[@]}"; do
  [ -d "$R" ] || { echo "[없음] $R"; continue; }
  echo; echo "=== $R ==="
  find "$R" -maxdepth 3 \
    \( -name .git -o -name __pycache__ -o -name '*venv*' -o -name GraspGenModels -o -name datasets \) -prune \
    -o -type f -name '*.py' -print | sed "s|^$R|.|" | sort
done

echo; echo "######## 2) point cloud 입력 후보 파일 ########"
for R in "${ROOTS[@]}"; do
  [ -d "$R" ] || continue
  grep -rIlE $EXCL --include='*.py' "$PATTERN" "$R" 2>/dev/null || true
done | sort -u

echo; echo "######## 3) 각 후보: PC가 들어오는 지점 + CLI 인자 ########"
for R in "${ROOTS[@]}"; do
  [ -d "$R" ] || continue
  grep -rIlE $EXCL --include='*.py' "$PATTERN" "$R" 2>/dev/null || true
done | sort -u | while IFS= read -r f; do
  [ -n "$f" ] || continue
  echo; echo "=== $f ==="
  grep -nE 'np\.load|\.npy|\.npz|\.pcd|\.ply|sample_data_dir|point_cloud|pointcloud|add_argument|def main' "$f" | head -n 25 || true
done
