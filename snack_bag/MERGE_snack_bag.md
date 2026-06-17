# 과자봉지 변형체 → stage7 머지 가이드

봉지 변형체를 stage7과 **독립**으로 개발하기 위한 분리 구조와, 나중에 stage7로 합치는 절차.
(다른 창은 stage7에서 실물 그리퍼 충돌구체·오프셋 작업 중 → stage7 파일은 그 창이 소유. 이쪽은 안 건드림.)

## 파일 분리 (현재)

| 역할 | 파일 | 비고 |
|---|---|---|
| 머지 단위(에셋·물리) | `multiobj_pipeline/snack_bag_module.py` | stage7이 나중에 import |
| 독립 테스트 하니스 | `multiobj_pipeline/snack_bag_spike.py` | 2cm 손가락 스퀴즈 검증 |
| 전용 러너 | `/tmp/run_snackbag.sh` | sentinel `/tmp/snackbag_stop` |
| 공유 에셋(읽기전용) | `snack_bag_pillow.usd`, `snack_bag.usd` | 두 창 모두 읽기만 |

- 로그: `logs/snackbag*.log`, 샷: `logs/shots/snackbag_*.png` (stage7의 `stage7_*`/`shot_*`와 안 겹침).
- ★ **두 Isaac 인스턴스 동시 실행 시 GPU 경합 가능** — 한 번에 한 창만 Isaac 띄우기로 조율(메모리 [[isaac_run_rules]]).

## 실행 (이쪽 창)

```bash
pgrep -af "snack_bag_spike|stage7_graspgen"   # 다른 창 인스턴스 확인
rm -f /tmp/snackbag_stop
bash /tmp/run_snackbag.sh --mode cloth        # 또는 --mode fem_beta(개발후)
# 파라미터 튜닝: --pressure 8 --stretch 20000 --bend 80 --damping 1.0
```

## stage7 머지 절차 (나중에, 그리퍼 작업 끝난 뒤)

stage7의 **기존 인라인 snack 코드 제거 → 모듈 호출로 대체**.

1. import 추가(상단):
   ```python
   from multiobj_pipeline.snack_bag_module import enable_gpu_dynamics, spawn_snack_bag
   ```
2. `my_world.play()` 직전, snack일 때 GPU dynamics:
   ```python
   if args.obj_type == "snack":
       enable_gpu_dynamics(stage)
   ```
3. 기존 snack spawn 블록(박스 FEM `add_physx_deformable_body` 등)을 다음으로 교체:
   ```python
   elif _obj_type == "snack":
       bag_path = spawn_snack_bag(stage, "/physicsScene", (_cx, _cy),
                                  _table_top + 0.045, mode="cloth")
       target_cube = None; targets = []   # 강체 상태머신 우회(기존 가드 유지)
   ```
4. **제거 대상**(stage7 현재 인라인): 박스 12삼각 메시 생성, `add_physx_deformable_body`/`add_deformable_body_material`,
   snack 핸들러의 박스 FEM 전제 코드. (squish 파지 핸들러 자체는 실물 그리퍼 파지로 교체 — 그리퍼 작업과 합류.)
5. 검증: `bash run_stage7.sh --obj-type snack` → 봉지 빵빵 + 실물 그리퍼 그립 + 3층 적치.

## 인터페이스 (모듈 API)

```python
enable_gpu_dynamics(stage, scene_path="/physicsScene") -> scene_path   # play() 전 호출
spawn_snack_bag(stage, scene_path, center_xy, rest_center_z,
                mode="cloth"|"fem_beta", prim_path="/World/snack_bag", params=None) -> bag_prim_path
```

- `center_xy=(x,y)` m, `rest_center_z` = 봉지 중심 world z(베개 7cm면 책상윗면+0.045).
- `params` dict로 pressure/stretch/bend/shear/damping/pco/sro/friction/mass 오버라이드.
- 검증 파라미터(spike9, CLOTH_PARAMS): pressure 8, stretch 20000, bend 80, shear 50, damping 1.0, pco 0.008, sro 0.004.

## 현재 상태 (2026-06-16 확정)

- [x] **cloth 모드로 확정(2026-06-16, 사용자 "깔끔·차분" 승인)** — 탄성 인플레이터블, 깔끔 puffy·무출렁. 모듈 기본값 그대로.
      mesh=가장자리0 2cm 베개(snack_bag_pillow.usd, make_pillow_cloth.py 2.0). 파라미터: pressure 12 / stretch 5000 / bend 150 / shear 50 / spring_damping 4 / pbd_damping(전역) 10 / max_velocity 2 / solver 32 / pco 0.005 / sro 0.0025 / contents False.
      ★강성 과다(stretch 1e4↑)가 꿀렁임 주원인 → 수천대로 낮춤(약간 신축 감수). spring_damping↑ 금지(폭발).
- [x] **포기 결정** — 내용물(A/C)·소성(형상유지)·FEM(fem_beta)·B(구김고정) 전부 포기. PBD cm스케일서 cloth+내용물/압력/핀치 반복 폭발 + native plasticity 없음 → 탄성 수용. (상세 메모리 snack_particle_cloth.)
- [ ] 실물 RH-P12 그리퍼(effort/전류 + 수동 적응 distal)로 파지 — 그리퍼 작업 창과 합류.
- [ ] stage7 머지(위 "머지 절차") + 3종 통합 검증.

`_spawn_fem_beta` / `apply_plastic_yield` / `add_snack_contents`는 모듈에 남겨두되 **미사용**(재시도 시 참고).
