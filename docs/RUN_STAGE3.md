# RUN_STAGE3.md — Stage3 (GraspGen→cuRobo→Isaac Sim) 실행·재현 가이드

> 목적: **동일 스펙 노트북에서 처음부터 재현** 가능하도록 환경·셋업·실행을 한 곳에 정리.
> 트러블슈팅 상세는 [RUNBOOK.md](../RUNBOOK.md), 파이프라인 개념은 [docs/PIPELINE.md](PIPELINE.md) 참조.
> 최종 검증: 2026-06-02 (GraspGen→cuRobo 파지 성공, Δz≈0.17m).

---

## 1. 환경 고정값 (확정 사실)

| 항목 | 값 |
|---|---|
| GPU | RTX 5080 Laptop (Blackwell, sm_120), CUDA 12.8, Driver 13.0 |
| 빌드 ARCH | `TORCH_CUDA_ARCH_LIST=12.0+PTX` |
| OS / 사용자 | Ubuntu, devuser / `/home/devuser` |
| conda 초기화 | `~/miniconda3/etc/profile.d/conda.sh` |
| **실행 conda 환경** | **`env_isaaclab`** (python 3.11) |
| Isaac Sim | IsaacLab 번들 → `~/isaacsim` (심볼릭: `~/IsaacLab/_isaac_sim`) |
| Isaac Sim 환경설정 | `source ~/isaacsim/setup_conda_env.sh` |
| **런타임 torch** | **2.7.0+cu128 (Isaac Sim 번들)** ← cuRobo는 이 버전에 맞춰 빌드 |
| (공존) site-packages torch | 2.11.0+cu128 — **빌드/런타임에 잡히면 안 됨** |
| cuRobo | `~/IsaacLab/src/curobo` (editable 설치, nvidia_curobo 0.7.7), v1 API만 |
| GraspGen | `~/graspgen_ws/GraspGen`, venv=`/home/devuser/graspgen_venv`, ZMQ port 5556 |
| GraspGen 체크포인트 | `~/graspgen_ws/checkpoints/graspgen_robotiq_2f_140.yml` (PointNet++, robotiq_2f_140) |
| Stage3 스크립트 | `~/shelf_grasp_dev/stage3_graspgen_curobo_gui.py` (+ `grasp_viz.py`) |

> ⚠️ **핵심 원리**: cuRobo CUDA 확장은 **빌드 torch == 런타임 torch** 여야 한다.
> Isaac Sim이 번들 2.7.0에 고정이므로 cuRobo도 2.7.0으로 빌드한다.
> 자세한 사고 원인은 [RUNBOOK.md ④](../RUNBOOK.md) 참조.

---

## 2. 일회성 셋업 (최초 1회 / 환경 깨졌을 때)

### 2-1. cuRobo를 번들 torch 2.7.0으로 (재)빌드
```bash
rm -rf ~/.cache/torch_extensions/*

source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
source ~/isaacsim/setup_conda_env.sh          # ← PYTHONPATH로 번들 2.7.0 우선
python -c "import torch; print(torch.__version__)"   # ★반드시 2.7.0+cu128

export CUDA_HOME="$CONDA_PREFIX"; export PATH="$CUDA_HOME/bin:$PATH"
TORCH_LIB=$(python -c 'import torch,os;print(os.path.dirname(torch.__file__)+"/lib")')
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$TORCH_LIB:$CUDA_HOME/lib:${LD_LIBRARY_PATH:-}"
export CPATH="$CONDA_PREFIX/targets/x86_64-linux/include:${CPATH:-}"
export TORCH_CUDA_ARCH_LIST="12.0+PTX"; export MAX_JOBS=4
cd ~/IsaacLab/src/curobo && pip install -e . --no-build-isolation --no-deps -v

python -c "import curobo.curobolib.kinematics_fused_cu; print('curobo OK')"
```
- ❌ `curobo_install_fixed.sh`의 `unset PYTHONPATH`는 **쓰지 말 것** (그것이 2.11.0으로 잘못 빌드시킨 원흉).

### 2-2. GraspGen 클라이언트 의존성 (env_isaaclab)
```bash
# ※ conda activate 후에도 pip이 시스템 /usr/bin/pip 으로 잡히는 함정 → python -m pip 절대경로 사용
~/miniconda3/envs/env_isaaclab/bin/python -m pip install pyzmq msgpack msgpack_numpy --no-deps
```

---

## 3. 매 실행 절차

### 3-1. GraspGen ZMQ 서버 (graspgen_venv, port 5556)
```bash
cd ~/shelf_grasp_dev
bash start_graspgen_server.sh > logs/graspgen_server_$(date +%s).log 2>&1 &
# 준비 완료 로그: "Model loaded and ready" / "listening on tcp://127.0.0.1:5556"
```

### 3-2. Stage3 (env_isaaclab + Isaac Sim)
```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
source ~/isaacsim/setup_conda_env.sh
TORCH_LIB=$(python -c 'import torch,os;print(os.path.dirname(torch.__file__)+"/lib")')
# CXXABI_1.3.15 위해 conda libstdc++ 최우선 (시스템 libstdc++는 1.3.13까지만)
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$TORCH_LIB:${LD_LIBRARY_PATH:-}"
# ★dGPU(RTX) 렌더 강제 — Optimus 노트북에서 Isaac Sim이 Intel iGPU로 렌더되어 느려지는 것 방지
export __NV_PRIME_RENDER_OFFLOAD=1
export __VK_LAYER_NV_optimus=NVIDIA_only
export __GLX_VENDOR_LIBRARY_NAME=nvidia
cd ~/curobo_ws
python ~/shelf_grasp_dev/stage3_graspgen_curobo_gui.py --port 5556 --cycles 5
```
- Isaac Sim 창이 뜨면 (30~60초) **좌하단 ▶ Play** 클릭 → 파이프라인 시작.
- 옵션: `--cycles N` (반복 횟수), `--viser-port 8081` (viser 미설치 시 자동 비활성).

### 3-3. 정상 동작 로그 체크포인트
```
[ZMQ] 연결 완료 → warming up → Curobo is Ready → Stage 3 시작
[GraspGen] N개 파지 수신 → [IK 필터] ... 선택
[VIZ-USD] 후보 20개(approach축) + 선택1개(RGB좌표계) 표시
✅✅ GraspGen→cuRobo 파지 성공! (Δz≈0.17m)
```
화면: 큐브 주변 후보 approach 선(점수색) + 선택 파지 RGB 좌표축(🔴닫힘/🟢/🔵접근).

---

## 4. 종료 수칙
- Isaac Sim은 **정상 종료만** (창 닫기 / Ctrl+C). `kill -9` 금지 → CUDA UVM 오염 시 재로그인 필요 (RUNBOOK ①).

---

## 5. 빠른 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `undefined symbol ... c10_cuda` | 빌드≠런타임 torch | §2-1 재빌드 (번들 2.7.0) |
| `CXXABI_1.3.15 not found` | 시스템 libstdc++ 우선 | `LD_LIBRARY_PATH`에 `$CONDA_PREFIX/lib` 최우선 |
| `No module named zmq` | pyzmq 누락 | §2-2 (`python -m pip`) |
| `import torch` → 2.11.0 | setup_conda_env 미source | `source ~/isaacsim/setup_conda_env.sh` |
| franka `*_temp.usd` 경고 | 비주얼 메시 참조 | 비치명적, 무시 |
| Isaac Sim 뷰포트·조작 느림 | Optimus 노트북이 **iGPU로 렌더** (pmon에 Isaac Sim이 NVIDIA GPU에 없음, P-state P5) | dGPU 렌더 강제 3개 env (위 §3-2) |
| 로봇 모션이 느림 | `time_dilation_factor` 감속 (의도적) | 값을 1.0에 가깝게 상향 (현재 0.8/0.75) |

---

## 6. 실기체 이관 시 변경점 (Stage4 — 예정, 미구현)

현재 코드의 **시뮬레이션 전용 가정**들. 실기체로 가려면 교체 필요:

1. **점구름이 합성 PC** — `sample_cube_pc()`는 실제 센서가 아니라 trimesh로 만든 이상적 큐브 PC.
   → 실기체에선 **뎁스 카메라/비전팀의 실제 (N,3) 점구름**으로 교체 (입력 key `point_cloud`, robot base 프레임).
2. **큐브 회전 미반영** — `cube_pos, _ = get_world_pose()`로 회전을 버림. PC·파지가 큐브 자세를 무시.
   → 실물체는 임의 자세이므로 **6-DOF 자세(회전 포함) 반영** 필요.
3. **그리퍼** — 현재 Franka + Robotiq 변환. 실기체는 robotiq_2f_140 → **RH-P12-RN-A**, 로봇 **E0509**로 교체.
   (`ROBOTIQ_TO_FRANKA_Z` 오프셋도 실그리퍼 기준 재계산)
4. **충돌 월드 / 씬** — `collision_table.yml`·Franka 예제 씬 대신 **사용자 제작 매대 USD**(실제 프로젝트 매대와 동일)로 교체. 로봇도 E0509로.
   - 파지 모드: 캔/병은 **옆면 파지(side)**로 매대에 세워야 함 (윗면 파지 금지). `OBJ_SPECS[*]["grasp_mode"]` 참고.
   - 파지 선정 시 **모션 비용(관절 변화) 최소화**로 불필요한 조인트 과회전 방지.

---

## 부록: 관련 문서
- [RUNBOOK.md](../RUNBOOK.md) — 환경 트러블슈팅 (특히 ④ torch 충돌)
- [docs/PIPELINE.md](PIPELINE.md) — 파이프라인 개념
- [docs/GRASPGEN_VIS_API.md](GRASPGEN_VIS_API.md) — 시각화 API
- [docs/CUROBO_SPEED_TUNING.md](CUROBO_SPEED_TUNING.md) — 속도(time_dilation_factor) 튜닝
