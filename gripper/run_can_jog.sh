#!/usr/bin/env bash
# 언더액추에이티드 파지 검증 실험 런처 (그리퍼+테스트캔, PNG 캡처)
#  - 종료: 창 닫기 또는 touch /tmp/grasp_test_stop
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
echo "[canjog런처] DISPLAY=$DISPLAY"
cd ~/curobo_ws
exec python ~/shelf_grasp_dev/gripper/gripper_can_jog.py "$@"
