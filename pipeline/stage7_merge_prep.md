# Stage5+6 머지 → Stage7 준비 문서 (2026-06-12 작성)

Stage5(추종 정밀화, 별도 세션) 완료 후 이 문서대로 합치고 Stage7을 시작한다.

## 1. 파일 현황과 분기 관계

```
stage4_graspgen_e0509.py  ← 공통 베이스(동결). 분기 시점에 이미 포함:
  · plan_grasp(goalset) 파지선택 + jcost-only 정렬
  · joint4/5 비대칭 한계(URDF 복사본)
  · Stage6 핵심: set_finger_sdf_collision(SDF res=256) + frictionCombineMode=max + r2/l2 curl 로깅
       ↓ 분기 (둘 다 위 내용 포함)
stage5_graspgen_e0509.py  ← 추종 정밀화 세션 (run_stage5.sh, /tmp/stage5_stop)
stage6_graspgen_e0509.py  ← 그리퍼 세션 (run_stage6.sh, /tmp/stage4_stop)
```

## 2. 머지 방법 (권장: stage5를 베이스로)

**stage6의 고유 변경은 단 1기능 — `--gripper-test` 판별모드** (diff 실측 2026-06-12 마감 시점).
SDF·마찰·curl 로깅은 분기 베이스에 이미 있어 stage5도 갖고 있다. 따라서

```
cp stage5_graspgen_e0509.py stage7_graspgen_e0509.py   # stage5 작업분 전부 승계
stage6 → stage7로 이식할 3곳 (stage6_graspgen_e0509.py에서 검색):
  ① argparse: "--gripper-test" 추가 (--viz-spheres 위)
  ② while 루프 직전: `_gtest_done = False`
  ③ `if step < 20: continue` 다음: "Stage6 판별실험" 블록 전체(~60줄, _glog/_gcam 포함)
머지 후: 헤더 주석 갱신 + py_compile + --place 1회 회귀(P8·간격로그·curl 로그 모두 정상 확인)
```

⚠ 머지 검증 시 stage5의 stop 센티널은 `/tmp/stage5_stop`(분리됨) — stage6은 `/tmp/stage4_stop`.
   stage7에서 하나로 통일할 것(권장: /tmp/stage7_stop).
⚠ stage5 노트의 함정: pkill/pgrep 자기매칭 → 브래킷 트릭 `stage7[.]py` 패턴 사용.

## 3. 머지 후 회귀 기준 (한 런에서 모두 확인 가능)

- P8 클린(직립 z=1.208) + 첫 시도 파지(재시도 0)
- [4.5] r2/l2 curl 로그 표기 (Stage6)
- 추종오차 전 관절 ≤0.4° (Stage5 중력보상)
- [간격:*] 전 구간 > 0 (Stage5 ESDF)
- settle=0 연속 모션 (Stage5)

## 4. Stage7 (도메인 랜덤화) 계획 초안

목표: 실환경 동일 USD에서 조명/텍스처/포즈 랜덤마이즈에도 파이프라인이 견고함을 검증.

- 7-A 실환경 USD 정합: 실측 책상·매대 치수로 USD 확인(현 v2 USD가 실환경 기준인지 사용자 확인 필요).
- 7-B 포즈 랜덤화(우선): 캔 위치 |dy|≤0.1·obj-dist 0.45~0.58 균등 샘플 N=20 연속 사이클 → 성공률/실패사유 집계.
  (지금 파이프라인 그대로 --cycles 반복으로 가능. perception 비의존이므로 가장 먼저.)
- 7-C 조명/텍스처 랜덤화: Replicator(omni.replicator) 도메인 랜덤화 — ★주의: 현 파이프라인 perception은
  Mock(시뮬 pose 직접 주입)이라 조명/텍스처는 GraspGen 입력에 영향 없음 → 비전팀 인터페이스 합류 시점에 의미.
  먼저 사용자와 범위 합의(포즈만 vs 비전 포함) 후 진행.
- 7-D 물체 다양화: snack_bag/bottle 추가(OBJ_SPECS 확장, GraspGen top/side 모드 매핑) — 도메인 3종 목표와 연결.
- 검증 기준(초안): 랜덤 N런 성공률 ≥ X%(사용자 합의), 실패는 사유별 로깅(unreachable/grip-fail/충돌),
  전 런 간격>0·직립 z 기준 충족.

## 5. 열린 질문 (Stage7 시작 전 사용자와 결정)

1. 7-C 범위 — 비전팀 인터페이스 없이 조명/텍스처 랜덤화가 의미 있는가(Mock perception이라 영향 없음).
2. 성공률 목표치와 랜덤 런 수(N).
3. 물체 3종(snack_bag/can/bottle) 확장을 Stage7에 포함할지, 별도 단계로 뺄지.
