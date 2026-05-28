#!/usr/bin/env python3
"""
BEV MJPEG server — streams a bird's-eye view of the CARLA ego vehicle.
Accessible at http://<host>:8080  (green bar = connected, red = braking).
"""
import carla, numpy as np, io, threading, time, sys
from PIL import Image as PILImage
from http.server import HTTPServer, BaseHTTPRequestHandler

buf  = [None]
lock = threading.Lock()

def connect(retries=20):
    for i in range(retries):
        try:
            c = carla.Client('127.0.0.1', 2000)
            c.set_timeout(4.0)
            w = c.get_world()
            print("[bev] Connected to CARLA", flush=True)
            return c, w
        except Exception:
            print(f"[bev] Waiting for CARLA ({i+1}/{retries})...", flush=True)
            time.sleep(3)
    print("[bev] Could not connect to CARLA — exiting", flush=True)
    sys.exit(1)

client, world = connect()

def get_vehicle():
    for _ in range(30):
        vs = list(world.get_actors().filter('vehicle.*'))
        if vs:
            return vs[0]
        time.sleep(1)
    return None

vehicle = get_vehicle()
if vehicle is None:
    print("[bev] No vehicle found — exiting", flush=True)
    sys.exit(1)

bp = world.get_blueprint_library().find('sensor.camera.rgb')
bp.set_attribute('image_size_x', '640')
bp.set_attribute('image_size_y', '640')
bp.set_attribute('fov', '90')
cam = world.spawn_actor(
    bp,
    carla.Transform(carla.Location(z=25), carla.Rotation(pitch=-90)),
    attach_to=vehicle,
)

def on_frame(img):
    arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)
    rgb = arr[:, :, [2, 1, 0]].copy()
    ctrl = vehicle.get_control()
    spd  = vehicle.get_velocity()
    kmh  = 3.6 * (spd.x**2 + spd.y**2 + spd.z**2) ** 0.5
    braking = ctrl.brake > 0.05
    bar_color = (200, 40, 40) if braking else (40, 180, 40)
    label = f"  HEARTBEAT LOST — BRAKING  |  {kmh:.1f} km/h" if braking \
            else f"  OPERATOR CONNECTED — OK  |  {kmh:.1f} km/h"
    bar = np.full((32, 640, 3), bar_color, dtype=np.uint8)
    frame = np.vstack([bar, rgb])
    pil = PILImage.fromarray(frame)
    b = io.BytesIO()
    pil.save(b, format='JPEG', quality=70)
    with lock:
        buf[0] = b.getvalue()

cam.listen(on_frame)
print("[bev] Camera attached — streaming on port 8080", flush=True)

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(
                b'<html><head><title>CARLA BEV Monitor</title></head>'
                b'<body style="margin:0;background:#111;font-family:monospace">'
                b'<img src="/stream" style="display:block;width:640px;margin:auto">'
                b'<p style="color:#ccc;text-align:center;font-size:14px">'
                b'GREEN = operator connected &nbsp;|&nbsp; RED = heartbeat lost, braking'
                b'</p></body></html>'
            )
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with lock:
                        frame = buf[0]
                    if frame:
                        self.wfile.write(
                            b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                            + frame + b'\r\n'
                        )
                    time.sleep(0.08)
            except Exception:
                pass

HTTPServer(('0.0.0.0', 8080), Handler).serve_forever()
