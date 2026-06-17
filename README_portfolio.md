# 매대 정리 Pick-and-Place — Isaac Sim 시뮬/모션/그리퍼 (개인 기여)

> 팀 프로젝트 "smart-shelf-robot"에서 내가 맡은 시뮬레이션·검증 부분을 정리한 포트폴리오용 README.
> 팀 레포: https://github.com/StealthBlack66/smart-shelf-robot

캔·병·과자봉지를 인식해 무충돌로 집어 3단 매대에 직립 적치하는 로봇(Doosan E0509 + ROBOTIS RH-P12)을,
**GraspGen → cuRobo → Isaac Sim** 파이프라인으로 구축하고 물리로 검증했다.

## 파이프라인

```
물체 점구름 (N,3)
  → GraspGen (PointNet++, ZMQ 서버)      6-DOF 파지자세 M개 + score
  → cuRobo plan_grasp / plan_single       무충돌·도달 가능 best 1개
  → Isaac Sim (PhysX)                     접근→파지→리프트→운반→매대 적치 물리 검증
```

## 내가 한 것

- **Stage1** GraspGen 추론 연결 (물체 PC → 파지자세, ZMQ 서버/클라이언트 분리로 env 충돌 회피).
- **Stage2~4** cuRobo 모션 + 실기체(E0509+RH-P12) 캔 픽앤플레이스 — 옆파지 → 3단 매대 직립 적치 → 홈복귀, 무충돌 클린런.
- **Stage5** joint_2 중력처짐 보정(중력보상 피드포워드) — 추종오차 4.55° → 0.17°.
- **그리퍼 분석** 실물 RH-P12-RN-DR vs 시뮬 RN-A 레이아웃 실측 비교(스트로크 105 ≈ 106mm) → USD 교체 불필요 입증. 적응 curl이 하드웨어 스프링 기구임을 공식 문서 분석으로 규명.
- **Stage7 변형체** 과자봉지를 PBD particle cloth로 구현(FEM·내용물 등 여러 방식의 한계 규명, 병렬 스윕으로 안정 구간 탐색).

## 레포 구조 (이 작업 디렉토리)

```
stages/     스테이지별 파이프라인 스크립트 + 러너 (stage1,3,4,5,6,7)
gripper/    그리퍼 도구 — 적응 테스트, 충돌구체 에디터, jog 진단
snack_bag/  변형체 과자봉지 — 모듈, 병렬 스윕, 메시 생성기, 가이드
assets/     USD (로봇·매대·봉지 메시)
pipeline/   파이프라인 계획·핸드오프 문서
docs/       기술문서 (moveL 핸드오프, 아키텍처 등)
refs/       참고용 외부 라이브러리 클론 (읽기 전용)
```

## 핵심 수치

| 항목 | 결과 |
|---|---|
| joint_2 중력처짐 | 4.55° → 0.17° (중력보상 FF) |
| cuRobo plan_single | 무충돌 클린런 (E0509+RH-P12 전체 픽앤플레이스) |
| 그리퍼 스트로크 | 시뮬 105mm ≈ 실물 106mm (USD 교체 불필요 입증) |
| 변형체 봉지 | 가장자리0 2cm 베개 + 공기압 → 안정 부풀림 (particle cloth) |

## 실행

```bash
bash stages/run_stage7.sh --obj-type cylinder   # 캔 픽앤플레이스
bash stages/run_stage7.sh --obj-type snack      # 변형체 과자봉지
```
환경: RTX 5080 / CUDA 12.8 / Isaac Sim 5.1 / cuRobo v0.7.x(v1 API) / GraspGen(PointNet++).

## 막혔던 것 (삽질 로그 요약)

- 변형체 봉지: 평평 두-시트는 PBD가 못 부풀림(부피 솔버 부팅 불가) → 최소 2cm 사전부풂 필요. stretch 1e4↑는 수치 불안정(스파이크). spring_damping↑은 폭발 — 병렬 스윕으로 임계 발견.
- 그리퍼 적응 curl: 어떤 공식 URDF에도 없음(하드웨어 스프링) → 무리한 재현 대신 타협안.
- v2 로봇 USD가 IsaacLab floating-base라 중력에 흔들림 → base를 월드에 고정.
