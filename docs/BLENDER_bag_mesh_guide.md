# 블렌더 과자봉지 메시 제작 가이드 (Isaac Sim 변형체용, 입문자)

목표: 꼬깔콘 봉지(16×23cm)를 **닫힌(watertight)·촘촘·균일 quad** 메시로 만들어 USD로 내보내기.
Isaac Sim에서 particle cloth + pressure(공기압)로 부풀려 중심 7cm를 맞출 것이므로 **평평하고 얇게** 만든다
(부풀리기 전 상태). 부피는 시뮬의 공기압이 만든다.

## 핵심 규칙 (이거 틀리면 시뮬서 안 부풂)
1. **닫힌 면**(watertight) — 구멍 없이 완전히 막힌 봉지. (열린 시트면 공기 안 참)
2. **촘촘·균일한 quad** — 부드럽게 구겨지려면 1cm 안팎 격자. 너무 성기면 종이처럼 꺾임.
3. **평평·얇게**(두께 1~2cm) — 부풀림은 시뮬이 함. 미리 빵빵하게 만들지 말 것.
4. **치수 정확**: 16cm × 23cm (= 0.16 × 0.23 m).
5. **Scale 적용**(Apply Scale) — 안 하면 USD에 스케일이 남아 시뮬서 크기 꼬임.

## 단계 (Blender 4.x 기준)

### 0. 설치·단위
- blender.org에서 설치 → 실행.
- 우상단 Scene Properties(원뿔+공 아이콘) → **Units: Metric, Length = Meters**.

### 1. 봉지 본체 만들기 (얇은 닫힌 박스)
- 시작 큐브 클릭 후 `X` → Delete (기본 큐브 삭제).
- `Shift+A` → Mesh → **Cube** 추가.
- 우측 `N` 키로 사이드패널 열기 → **Item 탭 → Dimensions**에 입력:
  - X = `0.16`, Y = `0.23`, Z = `0.015` (얇게 1.5cm)
- 이게 봉지의 평평한 형태(닫힌 박스).

### 2. 촘촘·균일 격자로 분할
- `Tab` → Edit Mode 진입.
- `A` → 전체 선택.
- 마우스 우클릭 → **Subdivide**.
- 좌하단에 뜨는 작은 패널 클릭 → **Number of Cuts = 20** (필요시 더). → 균일 quad 격자가 됨.
  (16×23cm에 ~1cm 격자 목표. 너무 무거우면 15, 부드러움 부족하면 25)

### 3. 정리 (닫힘·법선·중복)
- 여전히 Edit Mode, `A` 전체선택 상태에서:
  - `M` → **Merge → By Distance** (겹친 점 제거 = 누수 방지).
  - 메뉴 Mesh → Normals → **Recalculate Outside** (`Shift+N`) (법선 바깥으로).
- `Tab` → Object Mode 복귀.

### 4. 원점·스케일 적용 (중요)
- Object 메뉴 → Set Origin → **Origin to Geometry** (원점 중심으로).
- Object 메뉴 → Apply → **All Transforms** (또는 최소 **Scale**). ★스케일이 1로 베이크됨.

### 5. USD로 내보내기
- 봉지 선택된 상태로 File → Export → **Universal Scene Description (.usd/.usdc/.usda)**.
- 우측 옵션에서:
  - **Selection Only** 체크 (봉지만).
  - Format: `.usd` 또는 `.usda`(텍스트, 확인 쉬움).
- 파일명 예: `snack_bag.usd` → `~/shelf_grasp_dev/models/snack_bag.usd`에 저장.

## 체크리스트 (내보내기 전)
- [ ] Dimensions = 0.16 × 0.23 × 0.015 m
- [ ] 전체 quad, ~1cm 균일 격자 (Subdivide 20+)
- [ ] Merge By Distance 함 (누수 없음)
- [ ] Normals Recalculate Outside 함
- [ ] Apply Scale 함 (N패널 Scale이 1,1,1)
- [ ] Selection Only로 export

## 넘겨주시면
USD 경로만 알려주세요. 제가:
1. 그 메시에 particle cloth + pressure 적용,
2. pressure를 조절해 **중심 두께 7cm** 맞춤,
3. 그리퍼로 눌러/집어 변형 확인 (deformable_bag_spike.py에 메시 로드 경로만 바꿔 연결).

## 참고 (왜 평평하게?)
particle cloth pressure는 "rest 부피 × pressure"를 목표로 부풀린다. 평평한 닫힌 봉지(얇은 박스)를 넣고
pressure를 올리면 베개처럼 부푼다 → 그 부푼 두께가 7cm가 되도록 pressure 한 값만 맞추면 된다.
미리 7cm로 모델링하면 rest 부피가 커서 튜닝이 꼬인다.
