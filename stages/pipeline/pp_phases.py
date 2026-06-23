# 픽앤플레이스 단계(phase) 함수 — 실행 순서별 로직을 분리(사용자 분할 단위: 그랩젠 생성/…).
"""2단계 모듈화: FSM 단계 중 '분리 가능한' 핵심 로직을 함수로. FSM 흐름(상태전이·cu_js 재빌드)은
pick_place_main에 남기고, 단계별 알맹이만 여기로 — 동작 순서대로 읽고 재사용/통합 쉽게.
의존: numpy + pp_geometry(순수 변환). cuRobo/omni 직접 의존 없음(grasp_client만 주입)."""
import time
import numpy as np
from pp_geometry import robotiq_grasp_to_rhp12, grasp_to_world


def query_graspgen(grasp_client, pc_obj, obj_center, obj_quat, num_grasps=400):
    """[단계2: 그랩젠 생성] 물체 점구름(obj 프레임) → GraspGen 추론 → robotiq→RH-P12→월드 파지 후보.
    반환: (grasps_w[N,4,4], scores[N]). 후보 없으면 (빈[0,4,4], 빈[0]).
    ※ 점구름 샘플링·시각화·빈경우 FSM처리·후보선택은 호출측(main)에 유지(상태/전역 결합)."""
    t0 = time.time()
    grasps_obj, scores = grasp_client.infer(pc_obj, num_grasps=num_grasps)
    print(f"  [GraspGen] {len(grasps_obj)}개 파지 수신 ({time.time() - t0:.2f}s)", flush=True)
    if len(grasps_obj) == 0:
        return np.zeros((0, 4, 4), dtype=np.float32), np.zeros((0,), dtype=np.float32)
    grasps_rhp12 = np.array([robotiq_grasp_to_rhp12(g) for g in grasps_obj])
    grasps_w = np.array([grasp_to_world(g, obj_center, obj_quat) for g in grasps_rhp12])
    return grasps_w, scores
