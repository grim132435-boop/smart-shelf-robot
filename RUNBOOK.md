# RUNBOOK.md — E0509 Stage 2 실행 & 트러블슈팅

> 노트북 껐다 켠 뒤(재부팅·재로그인) 깨끗하게 기동하기 위한 절차.
> 해피패스는 `run_e0509_stage2.sh` 하나로 끝. 막히면 아래 "알려진 문제" 참조.
> shelf_grasp_dev/ 에 두고 CLAUDE.md와 함께 관리.

---

## 0. 환경 (확정 사실)

| 항목 | 값 |
|---|---|
| 사용자 / 홈 | devuser / /home/devuser |
| conda 초기화 | `~/miniconda3/etc/profile.d/conda.sh` |
| **conda 환경** | **`env_isaaclab`** (※ `isaacsim_env` 아님) |
| **Isaac Sim** | IsaacLab 번들 → `~/IsaacLab/_isaac_sim/setup_conda_env.sh` |
| GPU | RTX 5080 Laptop, Blackwell, CUDA 12.8 → `TORCH_CUDA_ARCH_LIST=12.0` |
| GraspGen | `~/shelf_grasp_dev` (graspgen_venv), ZMQ 서버 port 5556 |
| cuRobo | `~/curobo_ws` (env_isaaclab에 설치) |
| E0509 설정 | `~/curobo_ws/robots/e0509_gripper/` (e0509_gripper.yml, e0509_spheres.yml, meshes 33개) |
| Stage2 스크립트 | `~/curobo_ws/stage2_e0509_gui.py` |

---

## 1. 재부팅/재로그인 후 정상 기동 순서

```bash
# (1) 드라이버 정상 확인 — 출력 안 나오면 CUDA 오염, 재로그인 필요
nvidia-smi

# (2) 한 방에 실행 (GraspGen 서버 + E0509 Stage2)
bash ~/shelf_grasp_dev/run_e0509_stage2.sh

# (3) 로그 실시간 확인
tail -f ~/shelf_grasp_dev/logs/stage2_e0509_*.log
```

Isaac Sim 창은 30~60초 뒤 뜸 → Play 클릭.

---

## 2. 알려진 문제 & 해결

### ① CUDA UVM 드라이버 오염
- **증상**: `nvidia-smi` 실패 / CUDA init 에러. cuRobo·Isaac Sim 모두 안 뜸.
- **원인**: Isaac Sim을 `kill -9`(SIGKILL)로 강제 종료.
- **해결**: **로그아웃 후 재로그인** 또는 재부팅 (그 외 방법으로 복구 안 됨).
- **예방**: Isaac Sim은 창 닫기 / Ctrl+C 로 **정상 종료**. `-9` 쓰지 말 것.

### ② cuRobo 모듈 못 찾음 (ModuleNotFoundError: curobo)
- **원인**: 잘못된 conda 환경에서 실행 (`isaacsim_env` 등).
- **해결**: 반드시 **`env_isaaclab`** 활성화 후 실행. (run 스크립트에 반영됨)

### ③ cuRobo CUDA 익스텐션 빌드 실패
- **원인**: Blackwell(sm_120)용 빌드 설정 누락 / CUDA 툴킷 불일치.
- **해결**: `curobo_install_fixed.sh` 로 재설치 (빌드 시 `TORCH_CUDA_ARCH_LIST=12.0` 지정).
- **확인**: `python -c "import curobo; from curobo.geom.sdf.world import WorldCollision; print('curobo OK')"`

### ④ torch 버전 충돌 (Isaac Sim 2.7.0 vs env_isaaclab 2.11.0)  ✅ 해결 (2026-06-02)
- **증상**: `ImportError: ... undefined symbol: _ZN3c104cuda29c10_cuda_check_implementation...`
  → cuRobo CUDA 확장 로드 실패 → JIT(ninja) 재빌드 시도 → ninja 실패 → stage2/stage3 즉사.
- **근본 원인 (진단 확정)**:
  - curobo `.so`가 **site-packages torch 2.11.0**으로 빌드됨 (2026-05-27, `curobo_install_fixed.sh`의
    `unset PYTHONPATH`가 site-packages 2.11.0을 잡게 만든 것이 원흉).
  - 그러나 stage3 **런타임은 번들 torch 2.7.0**이 우선 (`setup_conda_env.sh` source 시 PYTHONPATH로 번들 우선).
  - 문제 심볼은 2.11.0 `libc10_cuda.so`에만 있고 2.7.0엔 없음 → 빌드≠런타임 → undefined symbol.
- **핵심 교훈**: **빌드 torch == 런타임 torch** 여야 함. Isaac Sim은 번들 2.7.0 고정이므로
  curobo도 **번들 2.7.0으로 빌드**한다. (`curobo_install_fixed.sh`의 `unset PYTHONPATH`는 쓰지 말 것.)

- **확정 복구 명령** (검증 완료, 순서대로):
  ```bash
  # (a) JIT 캐시 비우기
  rm -rf ~/.cache/torch_extensions/*

  # (b) 빌드 게이트: setup_conda_env 환경에서 torch가 2.7.0(번들)인지 확인 (★필수)
  source ~/miniconda3/etc/profile.d/conda.sh && conda activate env_isaaclab
  source ~/isaacsim/setup_conda_env.sh        # ← 이게 PYTHONPATH로 번들 2.7.0 우선시킴
  python -c "import torch; print(torch.__version__)"   # 반드시 2.7.0+cu128 이어야 진행

  # (c) curobo를 번들 2.7.0으로 재빌드 (--no-deps로 torch 2.11.0 끌림 차단)
  export CUDA_HOME="$CONDA_PREFIX"; export PATH="$CUDA_HOME/bin:$PATH"
  TORCH_LIB=$(python -c 'import torch,os;print(os.path.dirname(torch.__file__)+"/lib")')
  export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$TORCH_LIB:$CUDA_HOME/lib:${LD_LIBRARY_PATH:-}"
  export CPATH="$CONDA_PREFIX/targets/x86_64-linux/include:${CPATH:-}"
  export TORCH_CUDA_ARCH_LIST="12.0+PTX"; export MAX_JOBS=4
  cd ~/IsaacLab/src/curobo && pip install -e . --no-build-isolation --no-deps -v

  # (d) 검증
  python -c "import curobo.curobolib.kinematics_fused_cu; print('curobo OK')"
  ```

- **런타임 필수 환경변수** (stage3 실행 시 — `/tmp/run_stage3.sh` 참고):
  ```bash
  source ~/isaacsim/setup_conda_env.sh
  # CXXABI_1.3.15 위해 conda libstdc++ 최우선 (시스템 libstdc++는 1.3.13까지만 있어 실패)
  export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$TORCH_LIB:$LD_LIBRARY_PATH"
  ```

- **부수적으로 빠져 있던 것** (복구 과정에서 발견·재설치):
  - env_isaaclab에 GraspGen 클라이언트 의존성 누락 → 복원:
    `~/miniconda3/envs/env_isaaclab/bin/python -m pip install pyzmq msgpack msgpack_numpy --no-deps`
    (※ `conda activate` 후 `pip`이 시스템 `/usr/bin/pip`으로 잡히는 함정 있음 → **`python -m pip`** 절대경로 사용)
  - franka `franka_panda_temp*.usd` 비주얼 참조 경고 다수 발생하나 **비치명적**(모션·파지 정상, 로봇 비주얼 메시만 영향).

- **검증 결과 (2026-06-02)**: GraspGen 100파지 수신 → IK 필터 선택 → 시각화 표시 →
  `✅✅ GraspGen→cuRobo 파지 성공 (Δz=0.17m)`.

---

## 3. 예방 수칙 (재발 방지)

- Isaac Sim 종료는 항상 정상 종료 — `kill -9` 금지 (→ ① 예방).
- 실행은 항상 `run_e0509_stage2.sh` 통해서 (env·경로·ARCH 고정됨).
- torch / cuRobo 재설치 후에는 `~/.cache/torch_extensions` 비우고 import 테스트.
- env_isaaclab에 pip로 패키지 설치 시 torch가 멋대로 업그레이드되지 않는지 확인
  (`pip install ... ` 후 `python -c "import torch; print(torch.__version__)"`).
