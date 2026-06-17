#!/usr/bin/env bash
# Phase0 스파이크 헤드리스 러너 (cuRobo만, Isaac Sim 불필요)
set -o pipefail
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
source ~/isaacsim/setup_conda_env.sh
TORCH_LIB=$(python -c 'import torch,os;print(os.path.dirname(torch.__file__)+"/lib")')
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$TORCH_LIB:${LD_LIBRARY_PATH:-}"
exec python ~/shelf_grasp_dev/pipeline/phase0_spike_plan_grasp.py "$@"
