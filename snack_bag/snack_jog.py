# 산업용 펜던트 스타일 XYZ jog — 키 누르는 즉시 로봇 이동
import os, sys, tty, termios, json, time

JOG_FILE  = "/tmp/snack_jog.txt"
JOG_STOP  = "/tmp/snack_jog_stop"
JOG_SAVED = "/tmp/snack_pose_saved.json"

# 초기 EE 위치 (stage7 로그에서 확인 후 맞추거나 여기서 시작)
x, y, z = 0.29, 0.22, 1.30
step = 0.005   # 기본 5mm

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(2)
            return '\x1b' + ch2
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def send(label=None):
    lbl = label or "cur"
    with open(JOG_FILE, "w") as f:
        f.write(f"{x:.4f} {y:.4f} {z:.4f} {lbl}\n")
    if label and label != "cur":   # place/pre_shelf 등 named 위치는 JSON에도 저장
        data = {}
        if os.path.exists(JOG_SAVED):
            try: data = json.load(open(JOG_SAVED))
            except: pass
        data[label] = {"xyz": [round(x,4), round(y,4), round(z,4)]}
        with open(JOG_SAVED, "w") as f: json.dump(data, f, indent=2)
        print(f"\r  [{label}] JSON 저장 완료 → {JOG_SAVED}", flush=True)

def show_saved():
    if not os.path.exists(JOG_SAVED):
        print("\r  저장된 자세 없음.")
        return
    data = json.load(open(JOG_SAVED))
    for k, v in data.items():
        jd = v.get('joints_deg', {})
        vals = ", ".join(f"{n}:{a}" for n, a in jd.items())
        print(f"\r  [{k}] xyz={v['xyz']}  →  {vals}")

print("\r=== 펜던트 jog ===")
print("\r  w/s : Y+/Y-    a/d : X+/X-    r/f : Z+/Z-")
print("\r  +/- : step 증감 (현재 5mm)")
print("\r  1   : pre_shelf 저장   2 : place 저장   p : show")
print("\r  q   : 종료")
print(f"\r시작 위치: x={x} y={y} z={z}  step={step*1000:.1f}mm")
print("\r※ stage7 로그에 [JOG] 시작. 이 뜬 뒤 조작하세요.\n")

if os.path.exists(JOG_FILE): os.remove(JOG_FILE)
if os.path.exists(JOG_STOP): os.remove(JOG_STOP)

while True:
    ch = getch()

    if ch == 'q':
        open(JOG_STOP, "w").close()
        print("\r종료.")
        break
    elif ch in ('w', '\x1b[A'):  y += step
    elif ch in ('s', '\x1b[B'):  y -= step
    elif ch in ('d', '\x1b[C'):  x += step
    elif ch in ('a', '\x1b[D'):  x -= step
    elif ch == 'r':              z += step
    elif ch == 'f':              z -= step
    elif ch == '+':              step = min(step * 2, 0.05);  print(f"\r  step={step*1000:.1f}mm   ", end="", flush=True); continue
    elif ch == '-':              step = max(step / 2, 0.001); print(f"\r  step={step*1000:.1f}mm   ", end="", flush=True); continue
    elif ch == '1':              send("pre_shelf"); print(f"\r  [저장] pre_shelf ({x:.3f},{y:.3f},{z:.3f})", flush=True); continue
    elif ch == '2':              send("place");     print(f"\r  [저장] place ({x:.3f},{y:.3f},{z:.3f})", flush=True); continue
    elif ch == 'p':              show_saved(); continue
    elif ch == '\x03':           break  # Ctrl+C
    else:                        continue

    send()
    print(f"\r  x={x:.3f}  y={y:.3f}  z={z:.3f}  step={step*1000:.1f}mm    ", end="", flush=True)
