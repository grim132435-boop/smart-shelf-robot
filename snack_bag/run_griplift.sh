#!/usr/bin/env bash
# 과자봉지 cloth 그립 파라미터 병렬 스윕 런처 (pressure×stretch 격자, 4.3cm 스퀴즈→들림 측정)
#  - dGPU(RTX 5080) 오프로드 렌더링 + 사용자 세션 디스플레이 자동탐지
#  - 종료: touch /tmp/snackbag_stop
set -o pipefail
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
source ~/isaacsim/setup_conda_env.sh
TORCH_LIB=$(python -c 'import torch,os;print(os.path.dirname(torch.__file__)+"/lib")')
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$TORCH_LIB:${LD_LIBRARY_PATH:-}"
export __NV_PRIME_RENDER_OFFLOAD=1
export __VK_LAYER_NV_optimus=NVIDIA_only
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export XDG_RUNTIME_DIR=/run/user/1000
_GP=$(pgrep -u 1000 -n gnome-shell 2>/dev/null || pgrep -u 1000 -n gnome-session 2>/dev/null)
if [ -n "$_GP" ]; then
  _ENV=/proc/$_GP/environ
  _D=$(tr '\0' '\n' < "$_ENV" 2>/dev/null | sed -n 's/^DISPLAY=//p' | head -1)
  _X=$(tr '\0' '\n' < "$_ENV" 2>/dev/null | sed -n 's/^XAUTHORITY=//p' | head -1)
fi
export DISPLAY="${_D:-:0}"
[ -n "$_X" ] && export XAUTHORITY="$_X" || {
  _XAUTH=$(ls -t /run/user/1000/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
  [ -n "$_XAUTH" ] && export XAUTHORITY="$_XAUTH"
}
echo "[launcher] DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY (세션 자동탐지)"
cd ~/curobo_ws
exec python ~/shelf_grasp_dev/snack_bag/grip_lift_spike.py "$@"
