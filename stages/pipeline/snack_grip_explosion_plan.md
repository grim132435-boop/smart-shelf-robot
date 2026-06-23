# 버그: 과자봉지 파지 close 후 로봇 팔 요동(collision 폭발) — 진단·해결 플랜

작성 2026-06-22. 대상 `stages/pipeline/stage8_main.py` snack 핸들러(강체 봉지, kinematic 비주얼 그립).

## 증상
봉지 그리퍼 close 직후 로봇 팔이 크게 요동침(이전 동일 이슈 재발).

## 진단(B22 로그)
- L780 `[책상콜리전] OFF` — #2b(책상 콜리전 OFF) 적용됨.
- L783 가벼운 닫기(0.45), L785 kinematic 추종 시작.
- **L787 `[간격:moveL 봉지리프트] 구체-장애물 최소 -299.7mm (★침투)`** — 리프트 시 30cm 침투 = 팔이 폭발로 튕겨 장애물 깊이 박힘.

## 원인(확정)
- 봉지는 **kinematic + convexHull 콜리전** 보유. 그리퍼가 close하면 손가락(SDF/convex)이 **kinematic 봉지(불변 강체 벽)** 의 convexHull 안으로 깊이 침투 → 솔버가 그 침투를 해소하려 손가락을 강하게 밀어냄 → **팔 폭발/요동**.
- 책상(#2b로 OFF)이 아니라 **봉지 자체 콜리전**이 폭발원. (kinematic이라 봉지는 안 밀리고 그리퍼만 튕김)

## 해결안
- **S1(채택)**: 봉지 파지 윈도우 동안 **봉지 자체 콜리전 OFF**. kinematic 비주얼 봉지는 물리 콜리전 불요 — 그립=kinematic 추종, 운반 충돌회피=cuRobo attach 프록시. 그리퍼가 콜리전 없는 봉지에 close → 침투/폭발 없음. (캔/병 처리 중엔 봉지 콜리전 ON 유지=cuRobo 장애물; 봉지 파지 직전 OFF.)
- S2(보조): 책상 콜리전 OFF(#2b) 유지.

## 구현
1. 헬퍼 `toggle_bag_collision(stage, on)` — /World/snack_bag CollisionEnabled 토글.
2. 봉지 파지 진입(enter) 직전 OFF(책상콜리전 OFF와 같은 지점). 이후 봉지는 kinematic 추종+프록시라 콜리전 불요 → 계속 OFF.

## 검증
- 형 라이브: close 후 팔 안 요동. 봉지 거치대 적치 정상.
- 로그: 봉지 리프트 침투 정상(거대 음수 사라짐), 적치 3/3, 크래시 0.

## 규칙
- 종료는 graceful(`touch /tmp/stage7_stop`) 우선 — kill -9 반복이 CUDA 손상 유발(2026-06-22 재부팅).
- PNG로 Claude 거동 판단 금지(형 관찰). [[sim_ask_dont_judge]]
