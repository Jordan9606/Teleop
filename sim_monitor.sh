#!/bin/bash
# Live numerical dashboard — shows TUM cmd → Autoware cmd → CARLA vehicle state + camera pipeline
# Run from any directory. Requires sim to be running.

docker exec -i carla-ros2-bridge bash << 'EOF'
source /opt/ros/jazzy/setup.bash
source /tum_install_baked/setup.bash 2>/dev/null
python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
import threading, time

from tod_vehicle_msgs.msg import PrimaryControlCmd
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport, GearReport
from tod_status_msgs.msg import Status
from sensor_msgs.msg import Image

STATUS = {0:'INIT', 1:'CONNECTING', 2:'CONNECTED', 3:'DRIVING'}

class Monitor(Node):
    def __init__(self):
        super().__init__('sim_monitor')
        self.tum  = None
        self.aw   = None
        self.vel  = None
        self.str  = None
        self.gear = None
        self.stat = None

        # Camera tracking: timestamp of last frame on each hop
        self.cam_carla_t    = None   # bridge publishes  /carla/ego_vehicle/frontcenter/image
        self.cam_operator_t = None   # operator receives /operator/network/video/frontcenter/image
        self.cam_w = self.cam_h = 0
        self.cam_carla_count    = 0
        self.cam_operator_count = 0
        self._t0 = time.time()

        qos = 10
        self.create_subscription(PrimaryControlCmd,       '/vehicle/safety/output/primary_control_cmd',  lambda m: setattr(self,'tum',m),  qos)
        self.create_subscription(AckermannControlCommand, '/external/selected/control_cmd',              lambda m: setattr(self,'aw',m),   qos)
        self.create_subscription(VelocityReport,          '/vehicle/status/velocity_status',             lambda m: setattr(self,'vel',m),  qos)
        self.create_subscription(SteeringReport,          '/vehicle/status/steering_status',             lambda m: setattr(self,'str',m),  qos)
        self.create_subscription(GearReport,              '/vehicle/status/gear_status',                 lambda m: setattr(self,'gear',m), qos)
        self.create_subscription(Status,                  '/vehicle/statemachine/output/vehicle_status', lambda m: setattr(self,'stat',m), qos)
        self.create_subscription(Image, '/carla/ego_vehicle/frontcenter/image',    self._on_cam_carla,    qos)
        self.create_subscription(Image, '/operator/network/video/frontcenter/image', self._on_cam_op,    qos)

    def _on_cam_carla(self, m):
        self.cam_carla_t = time.time()
        self.cam_carla_count += 1
        self.cam_w, self.cam_h = m.width, m.height

    def _on_cam_op(self, m):
        self.cam_operator_t = time.time()
        self.cam_operator_count += 1

    def _fps(self, count, window=3.0):
        # rough fps over last window seconds using message count since t0
        elapsed = time.time() - self._t0
        if elapsed < 0.5:
            return 0.0
        return count / elapsed

    def show(self):
        def f(v): return f"{v:+.3f}" if v is not None else "  --- "

        tum_vel  = f(self.tum.velocity                    if self.tum  else None)
        tum_swa  = f(self.tum.steering_wheel_angle        if self.tum  else None)
        aw_spd   = f(self.aw.longitudinal.speed           if self.aw   else None)
        aw_tire  = f(self.aw.lateral.steering_tire_angle  if self.aw   else None)
        veh_spd  = f(self.vel.longitudinal_velocity       if self.vel  else None)
        veh_tire = f(self.str.steering_tire_angle         if self.str  else None)
        gear_raw = self.gear.report if self.gear else None
        gear_str = {2:'DRIVE', 20:'REVERSE', 22:'PARK', 1:'NEUTRAL'}.get(gear_raw, '---') if gear_raw else '---'
        tod      = STATUS.get(self.stat.tod_status if self.stat else 0, '?')

        now = time.time()
        carla_ok = self.cam_carla_t   and (now - self.cam_carla_t)   < 2.0
        op_ok    = self.cam_operator_t and (now - self.cam_operator_t) < 2.0
        carla_fps = self._fps(self.cam_carla_count)
        op_fps    = self._fps(self.cam_operator_count)
        cam_res   = f"{self.cam_w}x{self.cam_h}" if self.cam_w else "---"

        carla_stat = f"OK  {carla_fps:4.1f} fps  {cam_res}" if carla_ok else "NO DATA"
        op_stat    = f"OK  {op_fps:4.1f} fps"               if op_ok    else "NO DATA"

        print("\033[H\033[2J", end="")
        print("╔══════════════════════════════════════════════════════════════════╗")
        print("║           CARLA TELEOP  ──  LIVE DATA MONITOR                   ║")
        print("╠══════════════════╦══════════════════════╦════════════════════════╣")
        print("║  TUM CMD         ║  AUTOWARE CMD        ║  VEHICLE STATE         ║")
        print("║  (joystick→TUM)  ║  (tod_edgarautoware) ║  (from CARLA bridge)   ║")
        print("╠══════════════════╬══════════════════════╬════════════════════════╣")
        print(f"║  vel: {tum_vel} m/s ║  long.speed:{aw_spd} m/s ║  speed:  {veh_spd} m/s  ║")
        print(f"║  swa: {tum_swa} rad ║  tire.angle:{aw_tire} rad ║  tire:   {veh_tire} rad  ║")
        print(f"║  gear:      {'DRIVE' if self.tum else '---':>7}  ║  gear:  {gear_str:>12}  ║  gear:  {gear_str:>12}  ║")
        print("╠══════════════════╩══════════════════════╩════════════════════════╣")
        print(f"║  TUM Status:  {tod:<52}║")
        print("╠══════════════════════════════════════════════════════════════════╣")
        print("║  CAMERA PIPELINE                                                 ║")
        print("║  ┌─────────────────────────────────────────────────────────┐    ║")
        print(f"║  │ 1. CARLA → bridge  /carla/ego_vehicle/frontcenter/image │    ║")
        print(f"║  │    {carla_stat:<55}│    ║")
        print(f"║  │ 2. RTSP → operator /operator/network/video/frontcenter  │    ║")
        print(f"║  │    {op_stat:<55}│    ║")
        print("║  └─────────────────────────────────────────────────────────┘    ║")
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
