#!/usr/bin/env python3
"""Lula Robot Description Editor Export → cuRobo 네이티브 구체 YAML 정규화.

Isaac Sim의 Lula Robot Description Editor가 내보내는 robot_description.yaml의
collision_spheres는 '리스트-of-단일키딕셔너리' 형식이다:

    collision_spheres:
      - link_1:
        - center: [x, y, z]
          radius: r

반면 cuRobo(cuda_robot_generator)는 `self.collision_spheres[link] = ...`처럼
'딕셔너리' 형식을 기대한다:

    collision_spheres:
      link_1:
      - center: [x, y, z]
        radius: r

이 스크립트는 list→dict로 1회 변환해 stage4가 그대로 로드할 수 있게 한다.
같은 링크가 여러 항목으로 흩어져 있어도 병합한다. 이미 dict 형식이면 그대로 통과.

사용:
  python normalize_lula_spheres.py <에디터export.yaml> [출력경로]
  # 출력 생략 시 기본: ~/curobo_ws/robots/e0509_gripper/e0509_spheres.yml
"""
import sys
import os
import yaml

DEFAULT_OUT = "/home/devuser/curobo_ws/robots/e0509_gripper/e0509_spheres.yml"


def normalize(raw):
    """collision_spheres 노드(list 또는 dict) → {link: [{center,radius}, ...]} dict."""
    out = {}

    def add(link, spheres):
        out.setdefault(link, [])
        for s in spheres:
            # {center:[x,y,z], radius:r} 또는 [x,y,z,r] 둘 다 허용
            if isinstance(s, dict):
                c = [float(v) for v in s["center"]]
                r = float(s["radius"])
            else:
                c = [float(s[0]), float(s[1]), float(s[2])]
                r = float(s[3])
            out[link].append({"center": c, "radius": r})

    if isinstance(raw, dict):
        for link, spheres in raw.items():
            add(link, spheres)
    elif isinstance(raw, list):
        for item in raw:           # 각 item = {link: [spheres]} 단일키 dict
            for link, spheres in item.items():
                add(link, spheres)
    else:
        raise ValueError(f"예상치 못한 collision_spheres 타입: {type(raw)}")
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    src = os.path.expanduser(sys.argv[1])
    dst = os.path.expanduser(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_OUT

    doc = yaml.safe_load(open(src))
    raw = doc["collision_spheres"] if "collision_spheres" in doc else doc
    out = normalize(raw)

    with open(dst, "w") as f:
        f.write("# cuRobo 네이티브 형식 {center,radius}. stage4는 이 파일을 변환 없이 그대로 로드.\n")
        f.write(f"# 출처: {src} (normalize_lula_spheres.py로 정규화)\n")
        yaml.safe_dump({"collision_spheres": out}, f,
                       default_flow_style=None, sort_keys=False)

    n_links = len(out)
    n_sph = sum(len(v) for v in out.values())
    print(f"정규화 완료 → {dst}")
    print(f"  링크 {n_links}개, 구체 {n_sph}개: " +
          ", ".join(f"{k}={len(v)}" for k, v in out.items()))


if __name__ == "__main__":
    main()
