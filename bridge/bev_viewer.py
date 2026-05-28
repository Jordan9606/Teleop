#!/usr/bin/env python3
import threading
import io
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from http.server import BaseHTTPRequestHandler, HTTPServer
import time

latest_jpeg = b""
lock = threading.Lock()
clients = []
clients_lock = threading.Lock()


def to_jpeg(arr):
    try:
        import cv2
        _, buf = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()
    except ImportError:
        pass
    import struct, zlib
    rgb = arr[:, :, ::-1]
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))
    idat = zlib.compress(raw, 1)
    def chunk(tag, data):
        c = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", c)
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", idat)
            + chunk(b"IEND", b""))


class BevNode(Node):
    def __init__(self):
        super().__init__("bev_viewer")
        self.create_subscription(Image, "/carla/ego_vehicle/bev/image", self._cb, 10)

    def _cb(self, msg):
        channels = len(msg.data) // (msg.height * msg.width)
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
            msg.height, msg.width, channels
        )
        bgr = arr[:, :, :3]
        jpeg = to_jpeg(bgr)
        with lock:
            global latest_jpeg
            latest_jpeg = jpeg
        frame = (
            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
        )
        with clients_lock:
            dead = []
            for q in clients:
                try:
                    q.append(frame)
                except Exception:
                    dead.append(q)
            for q in dead:
                clients.remove(q)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            html = (
                "<!DOCTYPE html><html><head><title>CARLA BEV</title>"
                "<style>body{background:#111;margin:0;display:flex;flex-direction:column;"
                "align-items:center;justify-content:center;height:100vh;color:#fff;"
                "font-family:sans-serif;}img{border:2px solid #444;}h3{margin-bottom:8px;}"
                "</style></head><body>"
                "<h3>CARLA Bird's Eye View - live</h3>"
                "<img src='/stream' alt='BEV stream'>"
                "</body></html>"
            ).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(html)

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = []
            with clients_lock:
                clients.append(q)
            try:
                while True:
                    if q:
                        frame = q.pop(0)
                        self.wfile.write(frame)
                        self.wfile.flush()
                    else:
                        time.sleep(0.01)
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with clients_lock:
                    if q in clients:
                        clients.remove(q)
        else:
            self.send_response(404)
            self.end_headers()


def serve():
    server = HTTPServer(("0.0.0.0", 8080), Handler)
    print("[bev] MJPEG stream at http://localhost:8080")
    server.serve_forever()


rclpy.init()
node = BevNode()
t = threading.Thread(target=serve, daemon=True)
t.start()
print("[bev] Spinning...")
rclpy.spin(node)
