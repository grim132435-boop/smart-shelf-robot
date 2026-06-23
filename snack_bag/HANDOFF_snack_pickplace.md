# 과자봉지 픽앤플레이스 핸드오프 (이어받는 Claude용)

작성 2026-06-18. 동일 워크스테이션 다른 세션이 바로 이어가도록 정리.

## 한 줄 상태
과자봉지(cloth)를 실로봇(E0509+RH-P12)으로 **그립→변형→매대3층 운반→거치대 적치**까지 거의 완성. **남은 이슈 = 적치 시 그리퍼가 매대와 박거나 중간 투하**(reach/충돌). 마지막 미확인 런(snack54)은 사용자가 못 봐서 재실행 필요.

## 실행
```bash
cd /home/devuser/shelf_grasp_dev
bash stages/run_stage7.sh --obj-type snack --no-graspgen
# 종료: touch /tmp/stage7_stop  (또는 kill -9). 로그: logs/stage7_snackNN.log. 샷: logs/shots/snack_*
```
- 봉지는 GraspGen·cuRobo 상태머신 우회, **자체 squish 핸들러**(stage7 `elif _obj_type=="snack"` + 메인루프 `if args.obj_type=="snack"`).
- ★Isaac 한 번에 하나(GPU). 실행 전 `pgrep -af stage7 | grep python` 확인. **다른 창이 stage7 동시 편집/실행 가능** — 충돌 주의.

## ★작업 규칙 (사용자 지침)
- **시뮬 결과를 로그로 단정 금지.** 실행 후 AskUserQuestion으로 "어땠는지" 선택지로 물어볼 것(사용자가 라이브 관찰). [[sim_ask_dont_judge]]
- stage7은 다른 창 소유였으나 **사용자가 "snack 핸들러만 수정 OK" 허가**함. snack 블록만 건드리고 다른 부분 금지.
- 한국어 답변, 문장 끝 콜론 금지.

## 파일·핵심 위치
- `snack_bag/snack_bag_module.py`(내 모듈, 자유수정): `CLOTH_PARAMS`(아래), `_spawn_cloth`, `_make_pillow_mesh`(절차적 베개, hx/hy 폭조절), `add_snack_stand`(거치대 삼각프리즘).
- `stages/stage7_graspgen_e0509.py`: snack spawn `elif _obj_type=="snack"`(~1679, 거치대 생성 _STAND_X/_Y/_add_stand), 메인루프 핸들러 `if args.obj_type=="snack"`(~2068~2210, 그립→운반→적치→후퇴).

## 작동 확정된 것
- **cloth 봉지 안정 구성**(폭발 없음): pressure 11, stretch 6000, friction 2(★하한 — 0은 spawn 폭발), max_velocity 0.2, pbd_damping 18, solver 32, gravity_scale 1.0. [[snack_cloth_grip_stable]]
- **봉지 크기**: pillow_hx 0.06(폭12cm), pillow_hy 0.086(길이17.25cm) — 사용자가 그리퍼 span 11cm 맞춤. 절차적 베개(pillow_density 1.0).
- **그립**: 옆-스퀴즈, 손가락 간격 폐루프로 실측 4.3cm 침투(_TARGET_GAP=0.074, 단 봉지 폭 줄여 비례). 진입 steps40, 스퀴즈 증분 0.14·8스텝(사용자가 빠르게 요청). 그립 시 변형 보이고 스파이크 없음 확인.
- **시각 캐리-보조(핵심 기법)**: 그립 후 ① 파티클시스템 정지(CreateParticleSystemEnabledAttr False) ② 봉지 메시 Xform identity화+월드점으로 통일 ③ 봉지점을 EE로컬에 고정→매 스텝 EE월드변환 적용(강체 추종, 위치+회전) ④ 매대 도착 후 그리퍼 열고 **파티클시스템 재활성**(물성 복귀→cloth로 안착). ★정지→재활성 사이 내부상태 stale 주의(snack32 순서는 OK였음).
- **거치대**: 삼각프리즘 16.4×14×12.7cm(옆=직각삼각형). `add_snack_stand(width=0.12로 축소 — 매대 우벽 안 겹침)`. 위치 _STAND_X=0.29(SHELF3_X+0.04), _STAND_Y=0.50. 매대 3층 오른쪽-ish(왼쪽은 페트병 자리).

## ★남은 이슈 = 적치 모션 (reach vs 충돌 트레이드오프)
- **top-down(_ee_side) 적치**: 매대 안 안 뚫지만 **높은 매대에 손목 reach 초과 → plan_single 실패 + direct IK도 도달 못해 "중간 투하"**. (시도 snack47~52, 실패)
- **side_grasp_from_approach 적치**: 도달은 됨(추종오차 깨끗)지만 **그리퍼가 매대 벽/구조와 박음**(특히 오른쪽 0.32). (snack53서 "매대랑 박았어")
- 현재 코드 = **side_grasp + 거치대 앞쪽(_PY=_STAND_Y-0.06) 타겟 + 거치대 0.29(여유) + 하강 덜깊게(+0.15)**. snack54가 이 버전, **사용자 미확인 → 재실행해서 충돌/도달 확인부터.**
- 사용자 멘탈모델: "거치대 앞까지만 가면 된다"(중심 깊이까지 안 가도 됨). 봉지가 거치대 빗변에 기대게.

## 다음 단계 (추천 순서)
1. **snack54 버전 재실행 → 사용자에게 충돌/도달 물어보기**(로그로 단정 말 것).
2. 여전히 그리퍼-매대 충돌이면: carry_above/in은 cuRobo(_plan_move)라 충돌회피되나 **최종 lower(move_direct_ik)는 충돌회피 없음** → lower도 _plan_move로 바꾸거나, 거치대 더 앞/왼쪽, 하강 더 얕게.
3. 도달 안 되면: 거치대를 reach 가능 위치로(0.30이 top-down 한계, side_grasp는 더 감), 또는 봉지 적치 y를 더 앞으로.
4. 봉지 거치대 빗변에 평행 안착(사용자 원함): tilt+bag-center 역산은 추종오차 17°로 매대 통과 유발해 폐기함 — 재시도 시 추종오차 주의.
5. 완성되면 → 캔/병/봉지 3물체 도메인 랜더마이제이션.

## 막다른 벽(시도해서 안 된 것 — 반복 금지)
- FEM 소성·유체충전·작은입자: 전부 실패([[snack_fem_pillow]] [[snack_particle_cloth]]). cloth로 확정.
- 순수 물리 그립-리프트: 봉지가 그리퍼서 미끄러짐 → **시각 캐리-보조로 우회**(현재 방식).
- bag-center 역산+tilt 적치: 추종오차 17°·매대 통과 → 폐기.
