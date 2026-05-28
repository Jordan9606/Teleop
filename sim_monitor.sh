#!/bin/bash
# Live numerical dashboard — TUM cmd → Vehicle state + camera pipeline.
# Run from the lab PC while simulation is running.

docker exec -i carla-ros2-bridge bash << 'EOF'
source /opt/ros/jazzy/setup.bash
source /tum_install_baked/setup.bash 2>/dev/null
python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
import threading, time

from tod_vehicle_msgs.msg import PrimaryControlCmd, PrimaryVehicleState
from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport, GearReport
from tod_status_msgs.msg import Status
from sensor_msgs.msg import Image

STATUS = {0: 'INIT', 1: 'CONNECTING', 2: 'CONNECTED', 3: 'DRIVING'}

class Monitor(Node):
    def __init__(self):
        super().__init__('sim_monitor')
        self.tum  = None
        self.vel  = None
        self.str  = None
        self.gear = None
        self.stat = None

        # Camera: bridge publishes to edgar topic (tod_vehicle side) and operator topic
        self.cam_edgar_t    = None   # /edgar/sensor/camera/basler/frontcenter/image_resized
        self.cam_operator_t = None   # /operator/network/video/frontcenter/image
        self.cam_w = self.cam_h = 0
        self.cam_edgar_count    = 0
        self.cam_operator_count = 0
        self._t0 = time.time()

        qos = 10
        self.create_subscription(PrimaryControlCmd,  '/vehicle/safety/output/primary_control_cmd',             lambda m: setattr(self, 'tum',  m), qos)
        self.create_subscription(VelocityReport,     '/vehicle/status/velocity_status',                        lambda m: setattr(self, 'vel',  m), qos)
        self.create_subscription(SteeringReport,     '/vehicle/status/steering_status',                        lambda m: setattr(self, 'str',  m), qos)
        self.create_subscription(GearReport,         '/vehicle/status/gear_status',                            lambda m: setattr(self, 'gear', m), qos)
        self.create_subscription(Status,             '/vehicle/statemachine/output/vehicle_status',            lambda m: setattr(self, 'stat', m), qos)
        self.create_subscription(Image, '/edgar/sensor/camera/basler/frontcenter/image_resized', self._on_cam_edgar,    qos)
        self.create_subscription(Image, '/operator/network/video/frontcenter/image',             self._on_cam_operator, qos)

    def _on_cam_edgar(self, m):
        self.cam_edgar_t = time.time()
        self.cam_edgar_count += 1
        self.cam_w, self.cam_h = m.width, m.height

    def _on_cam_operator(self, m):
        self.cam_operator_t = time.time()
        self.cam_operator_count += 1

    def _fps(self, count):
        elapsed = time.time() - self._t0
        return count / elapsed if elapsed > 0.5 else 0.0

    def show(self):
        def f(v): return f"{v:+.3f}" if v is not None else "  --- "

        tum_vel  = f(self.tum.velocity             if self.tum  else None)
        tum_swa  = f(self.tum.steering_wheel_angle if self.tum  else None)
        veh_spd  = f(self.vel.longitudinal_velocity if self.vel else None)
        veh_tire = f(self.str.steering_tire_angle   if self.str else None)
        gear_raw = self.gear.report if self.gear else None
        gear_str = {2: 'DRIVE', 20: 'REVERSE', 22: 'PARK', 1: 'NEUTRAL'}.get(gear_raw, '---') if gear_raw else '---'
        tod      = STATUS.get(self.stat.tod_status if self.stat else 0, '?')

        now = time.time()
        edgar_ok = self.cam_edgar_t    and (now - self.cam_edgar_t)    < 2.0
        op_ok    = self.cam_operator_t and (now - self.cam_operator_t) < 2.0
        edgar_fps = self._fps(self.cam_edgar_count)
        op_fps    = self._fps(self.cam_operator_count)
        cam_res   = f"{self.cam_w}x{self.cam_h}" if self.cam_w else "---"

        edgar_stat = f"OK  {edgar_fps:4.1f} fps  {cam_res}" if edgar_ok else "NO DATA"
        op_stat    = f"OK  {op_fps:4.1f} fps"               if op_ok    else "NO DATA"

        print("\033[H\033[2J", end="")
        print("╔══════════════════════════════════════════════════════════════════╗")
        print("║          CARLA TELEOP  ──  LIVE DATA MONITOR                    ║")
        print("╠══════════════════════════╦═════════════════════════════════════  ╣")
        print("║  TUM COMMAND (G920)      ║  VEHICLE STATE (from CARLA bridge)   ║")
        print("╠══════════════════════════╬══════════════════════════════════════╣")
        print(f"║  velocity: {tum_vel} m/s  ║  speed:  {veh_spd} m/s              ║")
        print(f"║  swa:      {tum_swa} rad  ║  tire:   {veh_tire} rad              ║")
        print(f"║  gear:           {'DRIVE' if self.tum else '---':>7}   ║  gear:  {gear_str:>12}            ║")
        print("╠══════════════════════════╩══════════════════════════════════════╣")
        print(f"║  TUM Status:  {tod:<52}║")
        print("╠══════════════════════════════════════════════════════════════════╣")
        print("║  CAMERA PIPELINE                                                 ║")
        print("║  ┌──────────────────────────────────────────────────────────┐   ║")
        print(f"║  │ 1. Bridge → edgar  /edgar/.../frontcenter/image_resized  │   ║")
        print(f"║  │    {edgar_stat:<56}│   ║")
        print(f"║  │ 2. Bridge → RTSP   /operator/network/video/frontcenter   │   ║")
        print(f"║  │    {op_stat:<56}│   ║")
        print("║  └──────────────────────────────────────────────────────────┘   ║")
        print("╚══════════════════════════════════════════════════════════════════╝")
        print("  Ctrl+C to exit")

rclpy.init()
node = Monitor()
t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
t.start()
try:
    while True:
        node.show()
        time.sleep(0.3)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PYEOF
EOF
