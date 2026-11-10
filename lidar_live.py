#!/usr/bin/env python3
"""Live XY map + CSV logger for the Nano LiDAR.

Usage:  python3 lidar_live.py [PORT] [OUT.csv]
        python3 lidar_live.py /dev/ttyUSB0 corner1.csv

Logs EVERY sample (unfiltered) to CSV for the SDL notebook; filters only the plot.
Arrow keys rotate the view to line it up with the room (left/right 5 deg, up/down 1 deg);
the CSV always keeps the raw, unrotated angles.
Point math is the Unit 1 vector form:  P = s + d*u,  u = (cos th, sin th),  s = R_OFF*u
"""
import sys, csv, math, time
import numpy as np
import serial
import matplotlib.pyplot as plt

PORT    = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
OUT     = sys.argv[2] if len(sys.argv) > 2 else time.strftime("scan_%Y%m%d_%H%M%S.csv")
R_OFF   = 0.02     # lens offset from rotation axis (m) — measure yours
MIN_AMP = 100      # TF-Luna: below this, distance is unreliable
MAX_CM  = 800

BAUD    = 115200    # must match Serial.begin() in the sketch
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)                      # Nano auto-resets when the port opens
ser.reset_input_buffer()

f = open(OUT, "w", newline="")
w = csv.writer(f)
w.writerow(["scan", "step", "theta_deg", "dist_cm", "strength"])

plt.rcParams["keymap.pan"] = [k for k in plt.rcParams["keymap.pan"] if k != "p"]  # free up 'p'
for km in ("keymap.back", "keymap.forward"):        # free up the arrow keys for rotation
    plt.rcParams[km] = [k for k in plt.rcParams[km] if k not in ("left", "right")]
plt.ion()
fig, ax = plt.subplots(figsize=(7, 7))
sc = ax.scatter([], [], s=4)
ax.plot(0, 0, "r+", ms=12)
ax.set_aspect("equal"); ax.grid(True)
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

xs, ys, scan, last = [], [], -1, 0.0
persist = False                    # 'p' toggles: keep dots across revolutions
rot_deg = 0.0                      # display rotation (arrow keys), not applied to the CSV
ROT_KEYS = {"left": 5.0, "right": -5.0, "up": 1.0, "down": -1.0}

def title():
    ax.set_title(f"scan {scan}   rot {rot_deg:+.0f}°"
                 + ("   [PERSIST — p to clear]" if persist else ""))

def redraw():
    if not xs:
        return
    pts = np.column_stack((xs, ys))
    a = math.radians(rot_deg)
    c, s = math.cos(a), math.sin(a)
    pts = pts @ np.array([[c, s], [-s, c]])        # rotate CCW by rot_deg
    sc.set_offsets(pts)
    m = np.abs(pts).max() * 1.1
    ax.set_xlim(-m, m); ax.set_ylim(-m, m)

def on_key(event):
    global persist, rot_deg
    if event.key in ROT_KEYS:
        rot_deg = (rot_deg + ROT_KEYS[event.key]) % 360
        redraw(); title()
        fig.canvas.draw_idle()
    elif event.key == "p":
        persist = not persist
        if not persist:            # leaving persist mode: wipe the accumulated dots
            xs.clear(); ys.clear()
            sc.set_offsets(np.empty((0, 2)))
        title()
        fig.canvas.draw_idle()

fig.canvas.mpl_connect("key_press_event", on_key)
print(f"logging to {OUT} — arrows rotate the map, p toggles persistent dots, close the plot window to stop")

try:
    while plt.fignum_exists(fig.number):
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            plt.pause(0.001); continue
        if line.startswith("#SCAN"):
            scan = int(line.split()[1])
            if not persist:
                xs.clear(); ys.clear()
            title()
            f.flush(); continue
        if line.startswith("#"):
            continue
        try:
            step, th, d, amp = line.split(",")
            step, th, d, amp = int(step), float(th), int(d), int(amp)
        except ValueError:
            continue

        w.writerow([scan, step, th, d, amp])
        if amp < MIN_AMP or amp == 65535 or not (0 < d <= MAX_CM):
            continue

        t = math.radians(th)
        r = R_OFF + d / 100.0                       # |s + d*u| since s is parallel to u
        xs.append(r * math.cos(t)); ys.append(r * math.sin(t))

        if time.time() - last > 0.1 and xs:        # redraw at ~10 Hz
            redraw()
            plt.pause(0.001)
            last = time.time()
finally:
    f.close(); ser.close()
    print(f"saved {OUT}")