#!/usr/bin/env python3
# flow.py - optical flow pipeline + UART packet to flight controller (M3)
# Debug stream: http://<pi-ip>:8080
# Packet: [AA][55][velX:i16*100][velY:i16*100][q:u8][flags:u8][seq:u8][xor-cksum]

import time, threading
import numpy as np
import cv2
import serial
from picamera2 import Picamera2
from flask import Flask, Response

# ---- config ----
W, H = 320, 240
FPS_TARGET = 30
MAX_FEATURES = 60
MIN_FEATURES = 20

# ---- UART to FC ----
ser = serial.Serial('/dev/serial0', 115200, timeout=0)
seq = 0

# ---- camera ----
picam = Picamera2()
cfg = picam.create_video_configuration(
    main={"size": (W, H), "format": "RGB888"},
    controls={"FrameRate": FPS_TARGET})
picam.configure(cfg)
picam.start()
time.sleep(1)

# ---- debug stream ----
latest_jpeg = None
jpeg_lock = threading.Lock()
app = Flask(__name__)

@app.route('/')
def stream():
    def gen():
        while True:
            with jpeg_lock:
                frame = latest_jpeg
            if frame is not None:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + frame + b'\r\n')
            time.sleep(0.05)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

threading.Thread(target=lambda: app.run(host='0.0.0.0', port=8080,
                 debug=False, use_reloader=False), daemon=True).start()

# ---- flow state ----
lk_params = dict(winSize=(21, 21), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
feat_params = dict(maxCorners=MAX_FEATURES, qualityLevel=0.05,
                   minDistance=8, blockSize=7)

prev_gray = None
prev_pts = None
fps_t0 = time.time()
fps_n = 0
fps = 0.0

print("flow.py + UART running - debug stream at http://<pi-ip>:8080")

while True:
    frame = picam.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    flow_x, flow_y, quality = 0.0, 0.0, 0

    if prev_gray is not None and prev_pts is not None and len(prev_pts) >= MIN_FEATURES:
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray, gray, prev_pts, None, **lk_params)
        good_new = next_pts[status.flatten() == 1].reshape(-1, 2)
        good_old = prev_pts[status.flatten() == 1].reshape(-1, 2)

        if len(good_new) >= MIN_FEATURES:
            deltas = good_new - good_old
            flow_x = float(np.median(deltas[:, 0]))
            flow_y = float(np.median(deltas[:, 1]))
            quality = len(good_new)

            for p in good_new:
                cv2.circle(frame, (int(p[0]), int(p[1])), 2, (0, 255, 0), -1)
            cx, cy = W // 2, H // 2
            cv2.arrowedLine(frame, (cx, cy),
                            (int(cx + flow_x * 5), int(cy + flow_y * 5)),
                            (0, 0, 255), 2, tipLength=0.3)

            prev_pts = good_new.reshape(-1, 1, 2)
        else:
            prev_pts = None

    if prev_pts is None or len(prev_pts) < MIN_FEATURES:
        pts = cv2.goodFeaturesToTrack(gray, **feat_params)
        prev_pts = pts if pts is not None else None

    prev_gray = gray

    # ---- UART packet to FC ----
    vx = int(np.clip(flow_x * 100, -32000, 32000))
    vy = int(np.clip(flow_y * 100, -32000, 32000))
    q8 = min(quality, 255)
    flags = 1 if quality >= MIN_FEATURES else 0
    seq = (seq + 1) & 0xFF
    payload = vx.to_bytes(2, 'little', signed=True) + \
              vy.to_bytes(2, 'little', signed=True) + \
              bytes([q8, flags, seq])
    cksum = 0
    for b in payload:
        cksum ^= b
    ser.write(b'\xAA\x55' + payload + bytes([cksum]))

    # fps
    fps_n += 1
    if time.time() - fps_t0 >= 1.0:
        fps = fps_n / (time.time() - fps_t0)
        fps_t0, fps_n = time.time(), 0

    cv2.putText(frame, f"fx:{flow_x:+6.2f} fy:{flow_y:+6.2f} q:{quality:3d} fps:{fps:4.1f}",
                (5, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    print(f"fx:{flow_x:+7.2f}  fy:{flow_y:+7.2f}  q:{quality:3d}  fps:{fps:5.1f}", end='\r')

    ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    if ok:
        with jpeg_lock:
            latest_jpeg = jpg.tobytes()
