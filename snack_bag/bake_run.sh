#!/usr/bin/env bash
# 과자봉지 cloth 베이크 실행 런처(headless, DISPLAY 불요) — run_stage8.sh 환경 재사용
set -o pipefail
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
source ~/isaacsim/setup_conda_env.sh
TORCH_LIB=$(python -c 'import torch,os;print(os.path.dirname(torch.__file__)+"/lib")')
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$TORCH_LIB:${LD_LIBRARY_PATH:-}"
cd ~/curobo_ws
exec python ~/shelf_grasp_dev/snack_bag/bake_snack_bag.py "$@"
