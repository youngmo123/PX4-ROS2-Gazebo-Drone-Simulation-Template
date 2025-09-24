#!/usr/bin/env python3
"""Pilot HUD node for PX4 offboard keyboard control with camera overlay.
Run via: ros2 launch drone_pilot pilot_hud.launch.py
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand as VehicleCommandMsg,
    VehicleGlobalPosition,
    VehicleLocalPosition,
    VehicleStatus,
)
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.image_pb2 import Image as GzImage
except ImportError:  # Gazebo transport is optional
    GzNode = None
    GzImage = None

NAV_STATE_NAMES = {
    VehicleStatus.NAVIGATION_STATE_MANUAL: "MANUAL",
    VehicleStatus.NAVIGATION_STATE_ALTCTL: "ALTCTL",
    VehicleStatus.NAVIGATION_STATE_POSCTL: "POSCTL",
    VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: "MISSION",
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: "LOITER",
    VehicleStatus.NAVIGATION_STATE_AUTO_RTL: "RTL",
    VehicleStatus.NAVIGATION_STATE_AUTO_LAND: "LAND",
    VehicleStatus.NAVIGATION_STATE_ACRO: "ACRO",
    VehicleStatus.NAVIGATION_STATE_OFFBOARD: "OFFBOARD",
}

_descend_state = getattr(VehicleStatus, "NAVIGATION_STATE_DESCEND", None)
if _descend_state is not None:
    NAV_STATE_NAMES[_descend_state] = "DESCEND"


@dataclass
class VelocityLimits:
    xy_step: float
    xy_max: float
    z_step: float
    z_max: float


class _GazeboCamera:
    def __init__(self, topic: str, logger) -> None:
        if GzNode is None or GzImage is None:  # pragma: no cover - defensive
            raise RuntimeError("Gazebo transport bindings are unavailable")
        self._logger = logger
        self._topic = topic
        self._node = GzNode()
        self._condition = threading.Condition()
        self._latest: Optional[np.ndarray] = None
        try:
            self._node.subscribe(GzImage, topic, self._cb)
        except Exception as exc:  # pragma: no cover - subscription errors
            raise RuntimeError(f"Failed to subscribe to Gazebo topic {topic}: {exc}") from exc

    def _cb(self, msg: GzImage) -> None:
        try:
            frame = self._decode(msg)
        except Exception as exc:  # pragma: no cover - decoding errors
            self._logger.warn(f"Failed to decode Gazebo image on {self._topic}: {exc}", throttle_duration_sec=2.0)
            return
        with self._condition:
            self._latest = frame
            self._condition.notify_all()

    @staticmethod
    def _decode(msg: GzImage) -> np.ndarray:
        height = msg.height
        width = msg.width
        if height == 0 or width == 0:
            raise ValueError("Received empty Gazebo image")
        data = np.frombuffer(msg.data, dtype=np.uint8)
        pf = msg.pixel_format_type

        if pf == 3:  # RGB_INT8
            frame = data.reshape((height, width, 3))
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if pf == 8:  # BGR_INT8
            return data.reshape((height, width, 3))
        if pf == 4:  # RGBA_INT8
            frame = data.reshape((height, width, 4))
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if pf == 5:  # BGRA_INT8
            frame = data.reshape((height, width, 4))
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        if pf == 1:  # L_INT8 (mono)
            frame = data.reshape((height, width))
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        raise ValueError(f"Unsupported Gazebo pixel format type: {pf}")

    def get_latest(self, timeout: float = 0.0) -> Optional[np.ndarray]:
        with self._condition:
            if self._latest is None and timeout > 0.0:
                self._condition.wait(timeout=timeout)
            if self._latest is None:
                return None
            return self._latest.copy()


class PilotHUD(Node):
    def __init__(self) -> None:
        super().__init__("pilot_hud")

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("gz_camera_topic", "/camera")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("takeoff_altitude_m", 3.0)
        self.declare_parameter("velocity_step_xy_mps", 0.3)
        self.declare_parameter("velocity_max_xy_mps", 2.0)
        self.declare_parameter("velocity_step_z_mps", 0.3)
        self.declare_parameter("velocity_max_z_mps", 1.0)

        self.camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        self.gz_camera_topic = self.get_parameter("gz_camera_topic").get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.takeoff_altitude = float(self.get_parameter("takeoff_altitude_m").value)
        self.velocity_limits = VelocityLimits(
            xy_step=float(self.get_parameter("velocity_step_xy_mps").value),
            xy_max=float(self.get_parameter("velocity_max_xy_mps").value),
            z_step=float(self.get_parameter("velocity_step_z_mps").value),
            z_max=float(self.get_parameter("velocity_max_z_mps").value),
        )

        self.bridge = CvBridge()
        self.window_name = "PX4 Pilot HUD"
        self.cv_window_available = True
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        except cv2.error as exc:
            self.get_logger().error(f"Unable to create OpenCV window: {exc}")
            self.cv_window_available = False

        self.cmd_client = self.create_client(VehicleCommandSrv, "/fmu/vehicle_command")
        self._cmd_wait_logged = False

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.history = QoSHistoryPolicy.KEEP_LAST
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Image, self.camera_topic, self._image_cb, qos_sensor)
        self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self._global_position_cb,
            qos_sensor,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_position_cb,
            qos_sensor,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._vehicle_status_cb,
            qos_sensor,
        )

        qos_pub = QoSProfile(depth=1)
        qos_pub.history = QoSHistoryPolicy.KEEP_LAST
        qos_pub.reliability = ReliabilityPolicy.BEST_EFFORT

        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_pub)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_pub)

        period = 1.0 / max(self.rate_hz, 1.0)
        self.create_timer(period, self._publish_setpoint)
        self.create_timer(1.0 / 30.0, self._update_hud)

        self.image_lock = threading.Lock()
        self.last_image: Optional[np.ndarray] = None
        self.last_image_stamp: Optional[Time] = None
        self.gz_camera = None
        if self.gz_camera_topic:
            if GzNode is None or GzImage is None:
                self.get_logger().error(
                    "Parameter 'gz_camera_topic' provided but gz transport bindings are unavailable"
                )
            else:
                try:
                    self.gz_camera = _GazeboCamera(self.gz_camera_topic, self.get_logger())
                    self.get_logger().info(
                        f"Subscribed to Gazebo camera topic {self.gz_camera_topic}"
                    )
                except Exception as exc:  # pragma: no cover - defensive
                    self.get_logger().error(f"Failed to subscribe to Gazebo camera: {exc}")
                    self.gz_camera = None

        self.vehicle_status: Optional[VehicleStatus] = None
        self.global_position: Optional[VehicleGlobalPosition] = None
        self.local_position: Optional[VehicleLocalPosition] = None

        self.body_velocity = np.zeros(2, dtype=float)  # forward (x), right (y)
        self.vertical_velocity = 0.0
        self.target_altitude = -self.takeoff_altitude
        self.yaw_reference = 0.0

        self.shutdown_requested = False
        self.get_logger().info(
            "Pilot HUD ready. Press T to arm and take off, ESC to disarm and exit."
        )

    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            except CvBridgeError as exc:
                self.get_logger().warn(f"cv_bridge error: {exc}", throttle_duration_sec=2.0)
                return
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        with self.image_lock:
            self.last_image = frame
            self.last_image_stamp = Time.from_msg(msg.header.stamp)

    def _global_position_cb(self, msg: VehicleGlobalPosition) -> None:
        self.global_position = msg

    def _local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.local_position = msg
        if not math.isnan(msg.heading):
            self.yaw_reference = msg.heading

    def _vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    # ------------------------------------------------------------------
    def _publish_setpoint(self) -> None:
        timestamp = self.get_clock().now().nanoseconds // 1000

        offboard = OffboardControlMode()
        if hasattr(offboard, "timestamp"):
            offboard.timestamp = timestamp
        if hasattr(offboard, "position"):
            offboard.position = False
        if hasattr(offboard, "velocity"):
            offboard.velocity = True
        if hasattr(offboard, "acceleration"):
            offboard.acceleration = False
        if hasattr(offboard, "attitude"):
            offboard.attitude = False
        if hasattr(offboard, "body_rate"):
            offboard.body_rate = False
        self.offboard_pub.publish(offboard)

        traj = TrajectorySetpoint()
        if hasattr(traj, "timestamp"):
            traj.timestamp = timestamp
        # Convert body-frame velocity commands to world ENU
        yaw = self.yaw_reference
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_world = self.body_velocity[0] * cos_yaw - self.body_velocity[1] * sin_yaw
        vy_world = self.body_velocity[0] * sin_yaw + self.body_velocity[1] * cos_yaw
        vz_world = self.vertical_velocity

        if hasattr(traj, "velocity"):
            traj.velocity = [vx_world, vy_world, vz_world]
        if hasattr(traj, "position"):
            traj.position = [float("nan"), float("nan"), self.target_altitude]
        if hasattr(traj, "acceleration"):
            traj.acceleration = [float("nan"), float("nan"), float("nan")]
        if hasattr(traj, "jerk"):
            traj.jerk = [float("nan"), float("nan"), float("nan")]
        if hasattr(traj, "yaw"):
            traj.yaw = yaw
        if hasattr(traj, "yawspeed"):
            traj.yawspeed = float("nan")

        self.setpoint_pub.publish(traj)

    # ------------------------------------------------------------------
    def _update_hud(self) -> None:
        if not self.cv_window_available:
            return

        frame = None
        stamp = None
        with self.image_lock:
            if self.last_image is not None:
                frame = self.last_image.copy()
                stamp = self.last_image_stamp

        if frame is None:
            if self.gz_camera is not None:
                gz_frame = self.gz_camera.get_latest()
                if gz_frame is not None:
                    frame = gz_frame
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "NO CAMERA", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
        else:
            now = self.get_clock().now()
            if stamp is not None and (now - stamp).nanoseconds / 1e9 > 1.0:
                cv2.putText(
                    frame,
                    "CAMERA DELAY",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 165, 255),
                    2,
                )

        self._draw_overlay(frame)

        try:
            cv2.imshow(self.window_name, frame)
        except cv2.error as exc:
            self.get_logger().error(f"cv2.imshow failed: {exc}")
            self.cv_window_available = False
            return

        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            self._handle_key(key)

        if self.shutdown_requested and self.cmd_client.service_is_ready():
            self.get_logger().info("Disarming before shutdown")
            self._vehicle_command(VehicleCommandMsg.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
            self.cleanup()
            if rclpy.ok():
                rclpy.shutdown()

    def _draw_overlay(self, frame: np.ndarray) -> None:
        status = self.vehicle_status
        armed = False
        nav_state = "N/A"
        if status is not None:
            armed = status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            nav_state = NAV_STATE_NAMES.get(status.nav_state, str(status.nav_state))

        lat = lon = float("nan")
        alt = float("nan")
        if self.global_position is not None:
            lat = self.global_position.lat
            lon = self.global_position.lon
            alt = self.global_position.alt
        if self.local_position is not None and not math.isnan(self.local_position.z):
            alt = -self.local_position.z

        vx = vy = vz = 0.0
        if self.local_position is not None:
            vx = self.local_position.vx
            vy = self.local_position.vy
            vz = self.local_position.vz

        overlay = [
            f"ARM: {'ARMED' if armed else 'DISARMED'}   MODE: {nav_state}",
            f"GPS: lat {lat:.6f} lon {lon:.6f}  ALT: {alt:.1f} m",
            f"VEL NED: [{vx:.1f} {vy:.1f} {vz:.1f}] m/s",
            f"CMD body: fwd {self.body_velocity[0]:+.1f}  right {self.body_velocity[1]:+.1f}  vz {self.vertical_velocity:+.1f}",
            f"TARGET ALT: {-self.target_altitude:.1f} m (up positive)",
            "Keys: W/S forward/back  A/D left/right  Q/E up/down",
            "      T arm+offboard takeoff  L land  Space zero vel  ESC disarm+exit",
        ]

        for i, text in enumerate(overlay, start=1):
            cv2.putText(
                frame,
                text,
                (12, 24 * i + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

    # ------------------------------------------------------------------
    def _handle_key(self, key: int) -> None:
        if key == 27:
            self.shutdown_requested = True
            self.get_logger().info("ESC pressed, landing then disarming.")
            self._vehicle_command(VehicleCommandMsg.VEHICLE_CMD_NAV_LAND)
            return

        if key in (ord("w"), ord("W")):
            self._nudged_velocity(dx=self.velocity_limits.xy_step)
        elif key in (ord("s"), ord("S")):
            self._nudged_velocity(dx=-self.velocity_limits.xy_step)
        elif key in (ord("a"), ord("A")):
            self._nudged_velocity(dy=-self.velocity_limits.xy_step)
        elif key in (ord("d"), ord("D")):
            self._nudged_velocity(dy=self.velocity_limits.xy_step)
        elif key in (ord("q"), ord("Q")):
            self._nudged_vertical(-self.velocity_limits.z_step)
        elif key in (ord("e"), ord("E")):
            self._nudged_vertical(self.velocity_limits.z_step)
        elif key == ord(" "):
            self.body_velocity[:] = 0.0
            self.vertical_velocity = 0.0
        elif key in (ord("t"), ord("T")):
            self._arm_and_takeoff()
        elif key in (ord("l"), ord("L")):
            self.get_logger().info("Land command requested")
            self._vehicle_command(VehicleCommandMsg.VEHICLE_CMD_NAV_LAND)

    def _nudged_velocity(self, dx: float = 0.0, dy: float = 0.0) -> None:
        self.body_velocity[0] = float(np.clip(self.body_velocity[0] + dx, -self.velocity_limits.xy_max, self.velocity_limits.xy_max))
        self.body_velocity[1] = float(np.clip(self.body_velocity[1] + dy, -self.velocity_limits.xy_max, self.velocity_limits.xy_max))

    def _nudged_vertical(self, dz: float) -> None:
        self.vertical_velocity = float(np.clip(self.vertical_velocity + dz, -self.velocity_limits.z_max, self.velocity_limits.z_max))

    def _arm_and_takeoff(self) -> None:
        if not self.cmd_client.wait_for_service(timeout_sec=0.1):
            if not self._cmd_wait_logged:
                self.get_logger().warn("Vehicle command service unavailable; cannot arm")
                self._cmd_wait_logged = True
            return
        self._cmd_wait_logged = False
        self.get_logger().info("Arming and switching to OFFBOARD")
        ok_arm = self._vehicle_command(VehicleCommandMsg.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        if not ok_arm:
            self.get_logger().error("Arm command rejected")
            return
        mode_ok = self._vehicle_command(
            VehicleCommandMsg.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        if not mode_ok:
            self.get_logger().error("Failed to set offboard mode")
            return
        self.target_altitude = -self.takeoff_altitude
        self.vertical_velocity = 0.0
        self.body_velocity[:] = 0.0
        self.get_logger().info(f"Takeoff target set to {self.takeoff_altitude:.1f} m AGL")

    def _vehicle_command(
        self,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        confirmation: int = 0,
    ) -> bool:
        if not self.cmd_client.service_is_ready():
            if not self._cmd_wait_logged:
                self.get_logger().warn("Vehicle command service not ready")
                self._cmd_wait_logged = True
            return False

        req = VehicleCommandSrv.Request()
        now_us = self.get_clock().now().nanoseconds // 1000

        def assign(field: str, value) -> None:
            if hasattr(req, field):
                setattr(req, field, value)

        assign("timestamp", now_us)
        assign("param1", float(param1))
        assign("param2", float(param2))
        assign("param3", float(param3))
        assign("param4", float(param4))
        assign("param5", float(param5))
        assign("param6", float(param6))
        assign("param7", float(param7))
        assign("command", int(command))
        assign("target_system", 1)
        assign("target_component", 1)
        assign("source_system", 1)
        assign("source_component", 1)
        assign("confirmation", int(confirmation))
        assign("from_external", True)

        future = self.cmd_client.call_async(req)
        done = rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not done:
            future.cancel()
            self.get_logger().error(f"Vehicle command {command} timed out")
            return False
        if future.result() is None:
            self.get_logger().error(f"Vehicle command {command} failed: {future.exception()}")
            return False

        ack = future.result()
        result = getattr(ack, "result", None)
        if result is None:
            self.get_logger().warn(f"Vehicle command {command} returned without result field")
            return True

        if result in (
            ack.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
            ack.VEHICLE_CMD_RESULT_DENIED,
            ack.VEHICLE_CMD_RESULT_FAILED,
            ack.VEHICLE_CMD_RESULT_UNSUPPORTED,
        ):
            reason = getattr(ack, "result_param1", 0)
            self.get_logger().error(
                f"Vehicle command {command} rejected: result={result} reason={reason}"
            )
            return False
        if result == ack.VEHICLE_CMD_RESULT_IN_PROGRESS:
            self.get_logger().info(f"Vehicle command {command} in progress")
        return True

    def cleanup(self) -> None:
        if self.cv_window_available:
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass


def main() -> None:
    rclpy.init()
    node = PilotHUD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
