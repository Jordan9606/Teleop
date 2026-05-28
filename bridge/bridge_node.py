#!/usr/bin/env python3
"""
CARLA Bridge + Mode Switch Node

Combines Jordan's CARLA bridge (cameras, state publishing, heartbeat, TUM teleop)
with Team 2's mode switch (Autoware autonomous / TUM teleop switching).

Mode: teleop (default) | autonomous

Switch modes:
  ros2 topic pub /mode_switch geometry_msgs/msg/PoseStamped "{header: {frame_id: 'autonomous'}}"
  ros2 topic pub /mode_switch geometry_msgs/msg/PoseStamped "{header: {frame_id: 'teleop'}}"

In teleop:    TUM G920 → PrimaryControlCmd → this node → CARLA
In autonomous: Autoware → /control/command/control_cmd → this node → CARLA
Heartbeat:    always active — overrides both modes if operator disconnects
"""

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped

try:
    import carla
except ImportError:
    print("[ERROR] 'carla' Python package not found.")
    sys.exit(1)

try:
    from tod_vehicle_msgs.msg import (
        PrimaryControlCmd,
        PrimaryVehicleState,
        SecondaryControlCmd,
        SecondaryVehicleState,
        SafetyDriverStatus,
    )
    from tod_status_msgs.msg import Status as TodStatus
except ImportError as e:
    print(f"[ERROR] TUM message package not found: {e}")
    sys.exit(1)

# ── Autoware message types ────────────────────────────────────────────────────
_AW_CONTROL_MSG = None   # the Control/AckermannControlCommand class
_AW_VEL_FIELD = "speed"  # field name differs between Autoware versions

try:
    from autoware_control_msgs.msg import Control as AwControl
    _AW_CONTROL_MSG = AwControl
    _AW_VEL_FIELD = "velocity"   # autoware_control_msgs uses longitudinal.velocity
    print("[bridge] Using autoware_control_msgs.Control")
except ImportError:
    try:
        from autoware_auto_control_msgs.msg import AckermannControlCommand as AwControl
        _AW_CONTROL_MSG = AwControl
        _AW_VEL_FIELD = "speed"  # autoware_auto_control_msgs uses longitudinal.speed
        print("[bridge] Using autoware_auto_control_msgs.AckermannControlCommand")
    except ImportError:
        print("[bridge][WARN] No Autoware control msgs found — autonomous mode disabled.")

_AW_VEHICLE_AVAILABLE = False
try:
    from autoware_auto_vehicle_msgs.msg import (
        VelocityReport, SteeringReport, GearReport, Engage
    )
    _AW_VEHICLE_AVAILABLE = True
except ImportError:
    print("[bridge][WARN] autoware_auto_vehicle_msgs not found — Autoware state topics disabled.")


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class CarlaBridgeNode(Node):
    """CARLA bridge with TUM teleop / Autoware autonomous mode switching."""

    _MAX_TIRE_ANGLE = 0.74   # rad — Tesla Model 3 in CARLA (~42 deg)
    _VELOCITY_KP = 1.0
    _MAX_FORWARD_SPEED = 10.0  # m/s
    _MAX_REVERSE_SPEED = 3.0   # m/s

    def __init__(self):
        super().__init__("CarlaVehicleInterface")

        self.declare_parameter("carla_host", "127.0.0.1")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("spawn_point_index", 0)
        self.declare_parameter("vehicle_filter", "vehicle.tesla.model3")
        self.declare_parameter("max_tire_angle", self._MAX_TIRE_ANGLE)
        self.declare_parameter("velocity_kp", self._VELOCITY_KP)
        self.declare_parameter("max_steering_wheel_angle", 6.28)

        host = self.get_parameter("carla_host").value
        port = self.get_parameter("carla_port").value
        self._max_tire_angle = self.get_parameter("max_tire_angle").value
        self._kp = self.get_parameter("velocity_kp").value
        self._max_swa = self.get_parameter("max_steering_wheel_angle").value
        spawn_idx = self.get_parameter("spawn_point_index").value
        veh_filter = self.get_parameter("vehicle_filter").value

        # ── State ─────────────────────────────────────────────────────────────
        self._mode = "teleop"       # "teleop" | "autonomous"
        self._current_velocity = 0.0
        self._gear_position = 0
        self._front_frame_count = 0
        self._left_frame_count = 0
        self._right_frame_count = 0
        self._bev_frame_count = 0
        self._is_reverse = False
        self._last_hb_time = time.monotonic()
        self._hb_lost = False
        self._HB_TIMEOUT = 0.3
        self._current_goal = None

        # ── Connect to CARLA ──────────────────────────────────────────────────
        self.get_logger().info(f"Connecting to CARLA at {host}:{port} ...")
        try:
            self._client = carla.Client(host, port)
            self._client.set_timeout(15.0)
            self._world = self._client.get_world()
            self.get_logger().info(f"Connected. Map: {self._world.get_map().name}")
        except Exception as e:
            self.get_logger().fatal(f"Could not connect to CARLA: {e}")
            raise

        self._vehicle = self._spawn_vehicle(veh_filter, spawn_idx)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            PrimaryControlCmd,
            "/vehicle/safety/output/primary_control_cmd",
            self._on_primary_control_cmd,
            reliable_qos,
        )
        self.create_subscription(
            SecondaryControlCmd,
            "/vehicle/safety/output/secondary_control_cmd",
            self._on_secondary_control_cmd,
            reliable_qos,
        )
        self.create_subscription(
            TodStatus,
            "/vehicle/network/data/from_operator/operator_status",
            self._on_operator_status,
            reliable_qos,
        )
        self.create_subscription(
            PoseStamped,
            "/mode_switch",
            self._on_mode_switch,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self._on_goal,
            10,
        )
        if _AW_CONTROL_MSG is not None:
            self.create_subscription(
                _AW_CONTROL_MSG,
                "/control/command/control_cmd",
                self._on_autoware_control,
                reliable_qos,
            )

        # ── Publishers — TUM ──────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(
            Odometry, "/vehicle/interface/sensing/from_sensing/odom", 10
        )
        self._primary_state_pub = self.create_publisher(
            PrimaryVehicleState,
            "/vehicle/interface/actuation/from_actuation/primary_vehicle_state", 10,
        )
        self._secondary_state_pub = self.create_publisher(
            SecondaryVehicleState,
            "/vehicle/interface/actuation/from_actuation/secondary_vehicle_state", 10,
        )
        self._safety_status_pub = self.create_publisher(
            SafetyDriverStatus,
            "/vehicle/interface/actuation/from_actuation/safety_driver_status", 10,
        )

        # ── Publishers — Autoware ─────────────────────────────────────────────
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, "/planning/goal_pose", 10
        )
        if _AW_VEHICLE_AVAILABLE:
            self._engage_pub = self.create_publisher(Engage, "/autoware/engage", 10)
            self._vel_report_pub = self.create_publisher(
                VelocityReport, "/vehicle/status/velocity_status", 10
            )
            self._steer_report_pub = self.create_publisher(
                SteeringReport, "/vehicle/status/steering_status", 10
            )
            self._gear_report_pub = self.create_publisher(
                GearReport, "/vehicle/status/gear_status", 10
            )
        self._kinematic_state_pub = self.create_publisher(
            Odometry, "/localization/kinematic_state", 10
        )

        # ── Publishers — Cameras ──────────────────────────────────────────────
        self._camera_pub = self.create_publisher(
            Image, "/operator/network/video/frontcenter/image", qos_profile_sensor_data
        )
        self._left_camera_pub = self.create_publisher(
            Image, "/operator/network/video/rearleft/image", qos_profile_sensor_data
        )
        self._right_camera_pub = self.create_publisher(
            Image, "/operator/network/video/rearright/image", qos_profile_sensor_data
        )
        self._edgar_front_pub = self.create_publisher(
            Image, "/edgar/sensor/camera/basler/frontcenter/image_resized", 10
        )
        self._edgar_left_pub = self.create_publisher(
            Image, "/edgar/sensor/camera/basler/rearleft/image_resized", 10
        )
        self._edgar_right_pub = self.create_publisher(
            Image, "/edgar/sensor/camera/basler/rearright/image_resized", 10
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo, "/carla/ego_vehicle/frontcenter/cam_info", 10
        )
        self._bev_pub = self.create_publisher(
            Image, "/carla/ego_vehicle/bev/image", 10
        )

        # Attach cameras after publishers exist
        self._camera = self._attach_rgb_camera()
        self._left_camera = self._attach_side_camera(yaw=130.0, label="leftcenter")
        self._right_camera = self._attach_side_camera(yaw=-130.0, label="rightcenter")
        self._bev_camera = self._attach_bev_camera()

        self._timer = self.create_timer(0.05, self._publish_state)  # 20 Hz

        self.get_logger().info(
            f"CARLA bridge ready. Mode: {self._mode}. "
            f"Autoware control: {'ENABLED' if _AW_CONTROL_MSG else 'DISABLED'}."
        )

    # ── Spawning ──────────────────────────────────────────────────────────────

    def _spawn_vehicle(self, vehicle_filter: str, spawn_idx: int):
        bp_lib = self._world.get_blueprint_library()
        blueprints = bp_lib.filter(vehicle_filter)
        if not blueprints:
            blueprints = bp_lib.filter("vehicle.*")
        vehicle_bp = blueprints[0]
        if vehicle_bp.has_attribute("role_name"):
            vehicle_bp.set_attribute("role_name", "ego_vehicle")
        spawn_points = self._world.get_map().get_spawn_points()
        if spawn_idx >= len(spawn_points):
            spawn_idx = 0
        vehicle = self._world.spawn_actor(vehicle_bp, spawn_points[spawn_idx])
        vehicle.set_autopilot(False)
        self.get_logger().info(f"Spawned: {vehicle.type_id} at spawn point {spawn_idx}")
        return vehicle

    # ── Camera attach ─────────────────────────────────────────────────────────

    def _attach_rgb_camera(self):
        bp_lib = self._world.get_blueprint_library()
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "960")
        cam_bp.set_attribute("image_size_y", "600")
        cam_bp.set_attribute("fov", "120")
        transform = carla.Transform(
            carla.Location(x=2.0, y=0.0, z=1.4),
            carla.Rotation(pitch=-5.0),
        )
        camera = self._world.spawn_actor(cam_bp, transform, attach_to=self._vehicle)
        camera.listen(self._on_carla_camera_image)
        return camera

    def _attach_side_camera(self, yaw: float, label: str):
        bp_lib = self._world.get_blueprint_library()
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "960")
        cam_bp.set_attribute("image_size_y", "600")
        cam_bp.set_attribute("fov", "120")
        transform = carla.Transform(
            carla.Location(x=-0.5, y=0.0, z=1.7),
            carla.Rotation(pitch=-5.0, yaw=yaw),
        )
        camera = self._world.spawn_actor(cam_bp, transform, attach_to=self._vehicle)
        if label == "leftcenter":
            camera.listen(self._on_left_camera_image)
        else:
            camera.listen(self._on_right_camera_image)
        return camera

    def _attach_bev_camera(self):
        bp_lib = self._world.get_blueprint_library()
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", "512")
        cam_bp.set_attribute("image_size_y", "512")
        cam_bp.set_attribute("fov", "90")
        transform = carla.Transform(
            carla.Location(x=0.0, y=0.0, z=12.0),
            carla.Rotation(pitch=-90.0),
        )
        camera = self._world.spawn_actor(cam_bp, transform, attach_to=self._vehicle)
        camera.listen(self._on_bev_camera_image)
        return camera

    # ── Camera callbacks ──────────────────────────────────────────────────────

    def _on_bev_camera_image(self, image):
        self._bev_frame_count += 1
        if self._bev_frame_count % 3 != 0:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ego_vehicle/bev"
        msg.height = image.height
        msg.width = image.width
        msg.encoding = "bgra8"
        msg.is_bigendian = False
        msg.step = 4 * image.width
        msg.data = bytes(image.raw_data)
        self._bev_pub.publish(msg)

    def _carla_image_to_ros(self, image, frame_id: str) -> Image:
        bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
            (image.height, image.width, 4)
        )
        rgb = bgra[:, :, [2, 1, 0]]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = image.height
        msg.width = image.width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * image.width
        msg.data = rgb.tobytes()
        return msg

    def _on_carla_camera_image(self, image):
        self._front_frame_count += 1
        if self._front_frame_count % 2 != 0:
            return
        now = self.get_clock().now().to_msg()
        img_msg = self._carla_image_to_ros(image, "ego_vehicle/frontcenter")
        self._camera_pub.publish(img_msg)
        self._edgar_front_pub.publish(img_msg)
        fx = fy = float(image.width) / (2.0 * math.tan(math.radians(60.0)))
        cx, cy = float(image.width) / 2.0, float(image.height) / 2.0
        ci = CameraInfo()
        ci.header.stamp = now
        ci.header.frame_id = "ego_vehicle/frontcenter"
        ci.width = image.width
        ci.height = image.height
        ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._camera_info_pub.publish(ci)

    def _on_left_camera_image(self, image):
        self._left_frame_count += 1
        if self._left_frame_count % 2 != 0:
            return
        img_msg = self._carla_image_to_ros(image, "ego_vehicle/leftcenter")
        self._left_camera_pub.publish(img_msg)
        self._edgar_left_pub.publish(img_msg)

    def _on_right_camera_image(self, image):
        self._right_frame_count += 1
        if self._right_frame_count % 2 != 0:
            return
        img_msg = self._carla_image_to_ros(image, "ego_vehicle/rightcenter")
        self._right_camera_pub.publish(img_msg)
        self._edgar_right_pub.publish(img_msg)

    # ── Heartbeat ─────────────────────────────────────────────────────────────

    def _on_operator_status(self, msg: TodStatus):
        if msg.tod_status == TodStatus.TOD_STATUS_TELEOPERATION:
            self._last_hb_time = time.monotonic()
            if self._hb_lost:
                self._hb_lost = False
                self.get_logger().info("Heartbeat restored.")

    def _check_heartbeat(self):
        stale = (time.monotonic() - self._last_hb_time) > self._HB_TIMEOUT
        if stale and not self._hb_lost:
            self._hb_lost = True
            self.get_logger().warn(
                f"Heartbeat lost (>{self._HB_TIMEOUT}s). Applying controlled stop."
            )
        if stale:
            ctrl = carla.VehicleControl()
            ctrl.throttle = 0.0
            ctrl.brake = 0.4
            ctrl.steer = 0.0
            self._vehicle.apply_control(ctrl)
        return stale

    # ── Mode switching ────────────────────────────────────────────────────────

    def _on_mode_switch(self, msg: PoseStamped):
        new_mode = msg.header.frame_id

        if new_mode == "autonomous" and self._mode != "autonomous":
            if _AW_CONTROL_MSG is None:
                self.get_logger().warn("Cannot switch to autonomous — Autoware msgs not available.")
                return
            self._mode = "autonomous"
            self.get_logger().info("Mode → AUTONOMOUS")
            self._start_autonomous()

        elif new_mode == "teleop" and self._mode != "teleop":
            self._mode = "teleop"
            self.get_logger().info("Mode → TELEOP")
            if _AW_VEHICLE_AVAILABLE:
                engage = Engage()
                engage.engage = False
                self._engage_pub.publish(engage)

    def _start_autonomous(self):
        t = self._vehicle.get_transform()
        x = t.location.x
        y = -t.location.y
        z = t.location.z
        yaw = -math.radians(t.rotation.yaw)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = z
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose_msg.pose.covariance = [0.0] * 36
        self._initial_pose_pub.publish(pose_msg)
        self.get_logger().info("Published initial pose to Autoware.")

        if _AW_VEHICLE_AVAILABLE:
            engage = Engage()
            engage.engage = True
            self._engage_pub.publish(engage)
            self.get_logger().info("Autoware ENGAGED.")

        if self._current_goal:
            self._goal_pub.publish(self._current_goal)

    def _on_goal(self, msg: PoseStamped):
        self._current_goal = msg
        if self._mode == "autonomous":
            self._goal_pub.publish(msg)
            self.get_logger().info("Goal forwarded to Autoware.")

    # ── Control ───────────────────────────────────────────────────────────────

    def _apply_control(self, ctrl: 'carla.VehicleControl', target_vel: float):
        target_vel = float(target_vel)
        if target_vel > 0.0:
            target_vel = min(target_vel, self._MAX_FORWARD_SPEED)
        elif target_vel < 0.0:
            target_vel = max(target_vel, -self._MAX_REVERSE_SPEED)

        ctrl.reverse = target_vel < 0.0
        abs_target = abs(target_vel)
        abs_current = abs(self._current_velocity)
        error = abs_target - abs_current

        if abs_target < 0.05:
            ctrl.throttle, ctrl.brake = 0.0, 0.3
        elif error > 0.0:
            ctrl.throttle = float(min(1.0, self._kp * error))
            ctrl.brake = 0.0
        else:
            ctrl.throttle = 0.0
            ctrl.brake = 0.0
        self._vehicle.apply_control(ctrl)

    def _on_primary_control_cmd(self, msg: PrimaryControlCmd):
        if self._mode != "teleop":
            return
        if self._hb_lost:
            return
        ctrl = carla.VehicleControl()
        ctrl.steer = float(max(-1.0, min(1.0, -msg.steering_wheel_angle / self._max_swa)))
        target_vel = -msg.velocity if self._gear_position == 1 else msg.velocity
        self._apply_control(ctrl, target_vel)

    def _on_autoware_control(self, msg):
        if self._mode != "autonomous":
            return
        if self._hb_lost:
            return
        ctrl = carla.VehicleControl()
        tire_angle = float(msg.lateral.steering_tire_angle)
        ctrl.steer = float(max(-1.0, min(1.0, -tire_angle / self._max_tire_angle)))
        target_speed = float(getattr(msg.longitudinal, _AW_VEL_FIELD))
        self._apply_control(ctrl, target_speed)

    def _on_secondary_control_cmd(self, msg: SecondaryControlCmd):
        self._gear_position = msg.gear_position

    # ── State publishing ──────────────────────────────────────────────────────

    def _publish_state(self):
        self._check_heartbeat()

        now = self.get_clock().now().to_msg()
        transform = self._vehicle.get_transform()
        velocity = self._vehicle.get_velocity()
        angular_vel = self._vehicle.get_angular_velocity()
        control = self._vehicle.get_control()

        spectator = self._world.get_spectator()
        spectator.set_transform(carla.Transform(
            transform.location + carla.Location(z=30.0),
            carla.Rotation(pitch=-90.0),
        ))

        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        signed_speed = speed * (-1.0 if control.reverse else 1.0)
        self._current_velocity = signed_speed
        self._is_reverse = control.reverse

        yaw_rad = -math.radians(transform.rotation.yaw)
        tire_angle = float(-control.steer * self._max_tire_angle)

        # ── Odometry ──────────────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = transform.location.x
        odom.pose.pose.position.y = -transform.location.y
        odom.pose.pose.position.z = transform.location.z
        odom.pose.pose.orientation = yaw_to_quaternion(yaw_rad)
        odom.twist.twist.linear.x = signed_speed
        odom.twist.twist.angular.z = -math.radians(angular_vel.z)
        self._odom_pub.publish(odom)
        self._kinematic_state_pub.publish(odom)

        # ── TUM vehicle state ──────────────────────────────────────────────────
        pv = PrimaryVehicleState()
        pv.header.stamp = now
        pv.velocity = float(signed_speed)
        pv.steering_tire_angle = tire_angle
        pv.steering_wheel_angle = tire_angle
        self._primary_state_pub.publish(pv)

        sv = SecondaryVehicleState()
        sv.header.stamp = now
        sv.gear_position = self._gear_position
        self._secondary_state_pub.publish(sv)

        safety = SafetyDriverStatus()
        safety.vehicle_emergency_stop_released = True
        safety.vehicle_long_approved = True
        safety.vehicle_lat_approved = True
        self._safety_status_pub.publish(safety)

        # ── Autoware vehicle state ─────────────────────────────────────────────
        if _AW_VEHICLE_AVAILABLE:
            vel_report = VelocityReport()
            vel_report.header.stamp = now
            vel_report.header.frame_id = "base_link"
            vel_report.longitudinal_velocity = float(signed_speed)
            vel_report.lateral_velocity = 0.0
            vel_report.heading_rate = float(-math.radians(angular_vel.z))
            self._vel_report_pub.publish(vel_report)

            steer_report = SteeringReport()
            steer_report.stamp = now
            steer_report.steering_tire_angle = tire_angle
            self._steer_report_pub.publish(steer_report)

            gear_report = GearReport()
            gear_report.stamp = now
            gear_report.report = 20 if control.reverse else 2
            self._gear_report_pub.publish(gear_report)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        for cam in ("_camera", "_left_camera", "_right_camera", "_bev_camera"):
            obj = getattr(self, cam, None)
            if obj:
                obj.stop()
                obj.destroy()
        if hasattr(self, "_vehicle") and self._vehicle:
            self._vehicle.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = CarlaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
