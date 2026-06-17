# Stage4 핸드오프 (2026-06-08 작성)

## 🔴 지금 막힌 것 (최우선)
**새로 재생성한 충돌구체(96개)가 HOME 자세를 책상과 충돌시킴** → 매 사이클
`INVALID_START_STATE_WORLD_COLLISION`으로 pre-grasp/grasp 플래닝 실패 → 재배치 →
무한 루프(로봇이 GraspGen만 반복하고 안 움직임 = 사용자가 본 "안되고 있어").

- 직전까지 **물리 파지는 완벽히 작동**했음(아래 "작동하는 것"). 구체만 다시 만들면서 깨짐.
- 원인 후보: `fit_e0509_spheres.py`가 actual-mesh voxel로 더 타이트/조밀하게 만들면서
  (특히 link_4 hull 24개) 일부 구체가 home에서 책상면(월드 0.70) 아래로 내려감.
- bbox 필터·fill_holes 제거는 이미 했지만 여전히 96개 그대로 → home 충돌 지속 의심.

### 다음 작업(바로 이걸 하면 됨)
1. **어느 구체가 home에서 책상에 닿는지 확정**: stage4의 연속 cspheres 드로잉
   (`stage4_graspgen_e0509.py` line ~1132-1135, `draw_cspheres_usd(..., quiet=True)`)을
   **임시로 quiet=False**로 바꿔 1회 실행 → `[구체VIZ]` 가 home 최저구체 월드좌표 출력.
   책상footprint 안에서 z<0.70이면 그 위치(어느 링크)가 범인.
2. 범인 링크 구체를 줄이거나(NSPH↓) home 자세를 살짝 올리거나, 그 링크만 shrink.
3. **빠른 복구 대안**: 직전 작동 버전은 "convex-hull 전부 + 그리퍼 SAMPLE_SURFACE r=0.008,
   총 84개"였음. 사용자가 "구체 튀어나옴(joint4/5)"이라 더 타이트하게 바꾸려다 깨진 것.
   급하면 convex-hull 버전으로 되돌려 grasp 동작 복구 후, 튀어나옴은 따로 손보기.
   (백업 `e0509_spheres.yml.bak`은 원본 나쁜 버전이라 쓰지 말 것.)

## ✅ 작동하는 것 (절대 건드리지 말 것)
물리 파지 직전 6/6 성공. 핵심:
- **컴플라이언트 그리퍼 = 최대 돌파구**: `stabilize_arm_drives`에서 gripper **kp 2000→600,
  effort 50→25**. kp 높으면 닫을 때 캔 들이받아 관통/발사(사용자 "그리퍼가 캔 통과 후 날아감").
  600으로 부드럽게 멈춰 물림(r1~0.89).
- **TCP깊이 0.060** (`RHP12_TCP_DEPTH`): 캔을 손가락 케이지 깊숙이. 0.1034/0.078은 끝에 얕게→캐밍으로 빠짐.
- **램프 닫기**(`set_gripper` 현재각→목표 보간) + 닫은 뒤 45스텝 hold.
- **grasp 높이 obj_center+0.3·half**.
- 마찰 캔 1.5/1.2 + 그리퍼 손가락 고마찰(`set_gripper_friction`, ★play() 전에 호출 — 후엔 physics view 무효화).
- **리프트=직접IK+보간**(plan_single은 grasp가 책상 근처라 start-collision 거부).
- 캔 직립 안정: 재배치 시 orientation=[1,0,0,0]+속도0, `_cz=robot_base_z-0.03+half+0.002`,
  IDLE에서 settle 대기 후 위치 재측정.
- **재파지 retry** MAX_REGRASP=3, **pre/grasp 실패 시 HALT 대신 재배치 후 재시도**(RELEASE→IDLE, cycle-=1).
- 파지점 시각화: side는 GraspGen 후보 안 그리고 선택된 합성 파지만(허공 빨간막대 오해 제거).

## 핵심 파일/명령
- 메인: `~/shelf_grasp_dev/stage4_graspgen_e0509.py`
- 구체 피터: `~/shelf_grasp_dev/fit_e0509_spheres.py` (cuRobo `fit_spheres_to_mesh` 사용,
  env_isaaclab python, `unset PYTHONPATH`로 실행). 출력 `e0509_spheres_fitted.yml` →
  `e0509_spheres.yml`로 cp해야 적용.
- 구체 파일: `/home/devuser/curobo_ws/robots/e0509_gripper/e0509_spheres.yml`
- 실행: `bash ~/shelf_grasp_dev/run_stage4.sh --cycles N --obj-type cylinder --port 5556`
  (DISPLAY 자동탐지됨). 로그는 `~/shelf_grasp_dev/logs/`.
- GraspGen 서버: 죽으면 `nohup bash ~/shelf_grasp_dev/start_graspgen_server.sh &` (포트 5556).
- 종료: `ps -eo pid,args|grep "[s]tage4_graspgen"|awk '{print $1}'|xargs -r kill -9`
  (pkill -f가 exit1로 명령 체인 끊으니 주의).
- PNG: `~/shelf_grasp_dev/logs/shots/` (Read로 확인). SHOT_CAM_EYE/TARGET로 카메라.

## 사용자가 지적한 것 (구체 관련, 미해결)
- 구체가 링크에 **타이트하게 conform** 해야 함(cuRobo Franka 초록구체처럼). joint4/5가 튀어나옴.
- cuRobo 공식: Lula Robot Description Editor(GUI, 헤드리스 불가) 또는 fit_spheres_to_mesh.
- **딜레마: 타이트하게 하면 home 충돌로 grasp가 깨짐. 균형점 찾아야 함.**

## 다음 단계 (구체 고친 뒤)
**매대 placement** (사용자 목표): 잡은 캔을 매대 2단(바닥 z≈0.955, x중심0.25, y0.357~0.643)에 직립으로.
- side 파지라 캔 세로축=그리퍼 X(위) → 카메라업 유지하면 캔 직립 유지됨.
- placement도 approach +y(매대 안쪽)·X up으로 합성. 단 매대 높이 side 도달성이 빡빡(도달맵 확인 필요).
- GS.PLACE 상태 추가 예정이었음(MOVE_LIFT 성공 시 → carry→하강→그리퍼open→후퇴).

상세: 메모리 [[stage4-physical-grasp]] [[stage4-grasp-geometry]] [[stage4-shelf-coords]] [[stage4-drive-gains]] [[stage4-isaac-shutdown]]
