# 과자봉지 particle cloth — Isaac Sim GUI 라이브 튜닝 가이드

목적: 스크립트 배치(느림) 대신 **GUI Property 패널에서 값 바꿔가며 즉시** 봉지 거동을 튜닝.

## 띄우기
```
bash /tmp/run_snackbag.sh --mode cloth --idle --mesh pillow   # 봉지만(손가락 없음)
# --mesh flat       평평 두-시트(단, PBD가 flat→inflate 못 함 = 안 부풂)
# --mesh pillow_fem 두꺼운 가장자리 베개
# 종료: touch /tmp/snackbag_stop
```

## 씬 구조 (우측 Stage 패널)
| prim | 역할 | 튜닝 대상 |
|---|---|---|
| `/World/snack_bag` | 봉지 천(cloth) | **pressure, 스프링 강성** |
| `/World/snackParticleSystem` | 파티클 시스템 | offset, solver 반복 |
| `/World/Physics_Materials/snack_pbd` | PBD 재질 | friction, adhesion |
| `/physicsScene` | GPU dynamics | (건드릴 일 없음) |

## 튜닝할 값과 위치 (prim 선택 → Property 패널)

**`/World/snack_bag` 선택 시:**
- `Physx Particle Cloth` 섹션 → **Pressure** : 공기압(부피 배수). 클수록 빵빵. ★베개를 더 부풀리면 필름 늘어남 — 1에 가까울수록 늘어남↓.
- `Physx Auto Particle Cloth` 섹션 → **Spring Stretch/Bend/Shear Stiffness** : 클수록 빳빳·비신축. **Spring Damping** : 클수록 출렁임↓.

**`/World/snackParticleSystem` 선택 시:**
- **Particle Contact Offset / Solid Rest Offset** : 격자(5mm)에 맞춰 작게(0.005/0.0025). 크면 폭발.
- **Solver Position Iteration Count** : 크게(48~100) = 비신축·안정↑(단 부풀림도↑).

## ★중요 — 라이브 변경 규칙
- **Pressure** : Play 중 바꿔도 반영되는 편.
- **스프링 강성(Auto Cloth)·메시·offset** : **Play 중 변경 불가**("not supported" 경고). → **Stop(■) → 값 변경 → Play(▶)** 순서로.
- 절차: 좌측 상단 Stop → prim 선택 → Property서 값 입력 → Play → 관찰 → 반복.

## 거동별 레버 (요약)
- 너무 빵빵/늘어남 → Pressure↓ (또는 메시를 덜 부푼 걸로).
- 출렁임 → Spring Damping↑, Bend↑ (단 Bend 너무 크면 불안정).
- 고무처럼 늘어남 → Stretch Stiffness↑ + Solver Iteration↑.
- 폭발(NaN/커짐) → offset이 격자보다 큼 / Damping·Adhesion 과다 / Bend 과다 → 낮추기.

## ★엔진 한계 (튜닝으로 못 넘는 것)
- **flat→inflate 불가** : 평평 시트는 PBD가 못 부풀림(부피 솔버 시작 못 함). 부푼 베개 메시 필요.
- **소성(눌린 채 유지) 없음** : cloth는 탄성. 놓으면 복원.
- **누르면 저항(팽팽/터질듯)** : 힘제어 그리퍼라야 나옴(kinematic 손가락은 저항 무시).
- 메시 "모양"(슬랙·두께) 변경은 GUI 모델러로 한계 → **블렌더**서 수정 권장.

## 저장 → 파이프라인 연결
1. 마음에 드는 값 찾으면: 값을 적어두거나(저에게 알려주시면 모듈 기본값 반영),
2. File → Save As → `~/shelf_grasp_dev/snack_bag_tuned.usd` 로 저장 → 제가 로드해 stage7에 씁니다.
   (단, 런타임에 particleCloth를 다시 만드는 구조라, 보통은 **값만 알려주시는 게** 깔끔합니다.)
