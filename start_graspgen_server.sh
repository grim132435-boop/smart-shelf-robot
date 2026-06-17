#!/bin/bash
# GraspGen ZMQ 서버 시작 (graspgen_venv, robotiq_2f_140 PointNet++ 백본)
source /home/devuser/graspgen_venv/bin/activate
export PYTHONUNBUFFERED=1

echo "[GraspGen Server] Starting on port 5556 ..."
echo "[GraspGen Server] Backbone: PointNet++ (Blackwell SM 12.0 compatible)"
echo "[GraspGen Server] Checkpoint: robotiq_2f_140"

python /home/devuser/graspgen_ws/GraspGen/client-server/graspgen_server.py \
    --gripper_config /home/devuser/graspgen_ws/checkpoints/graspgen_robotiq_2f_140.yml \
    --port 5556 \
    --host 127.0.0.1
