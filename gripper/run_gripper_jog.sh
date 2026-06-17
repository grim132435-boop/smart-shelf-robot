#!/usr/bin/env bash
# 핑거팁 curl 진단용 그리퍼 jog 뷰어 런처 (mimic 해제 + Physics Inspector + 물리 스텝)
#  - run_sphere_editor.sh와 동일한 dGPU 오프로드 + DISPLAY/XAUTHORITY 자동탐지
#  - 종료: 창 닫기 또는 touch /tmp/gripper_jog_stop
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
echo "[jog런처] DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY"
cd ~/curobo_ws
exec python ~/shelf_grasp_dev/gripper/open_gripper_jog.py "$@"
