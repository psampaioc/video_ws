#!/usr/bin/env python3

import json
import math
import re
from collections import Counter
from pathlib import Path

import pyproj
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


NUMBER_PATTERN = re.compile(r"[-+]?\d+(?:[.,]\d+)?")


def first_number(value) -> float | None:
    match = NUMBER_PATTERN.search(str(value))
    if match is None:
        return None
    try:
        return float(match.group(0).replace(",", "."))
    except ValueError:
        return None


def compass_to_ros_yaw(heading_deg: float) -> float:
    angle = math.radians(90.0 - heading_deg)
    return math.atan2(math.sin(angle), math.cos(angle))


def transient_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class UtmProjector:
    def __init__(self, zone: int, south: bool) -> None:
        utm_epsg = (32700 if south else 32600) + zone
        self.transformer = pyproj.Transformer.from_crs(
            "epsg:4326", 
            f"epsg:{utm_epsg}", 
            always_xy=True
        )

    def project(self, latitude: float, longitude: float) -> tuple[float, float]:
        try:
            easting, northing = self.transformer.transform(longitude, latitude)
            return float(easting), float(northing)
        except Exception as e:
            raise ValueError(f"pyproj could not project coordinate: {e}")

    def close(self) -> None:
        pass


class DroneLocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("drone_localization_node")

        self.declare_parameter("telemetry_topic", "/telemetry/data")
        self.declare_parameter("pose_topic", "/map/telemetry_pose")
        self.declare_parameter("odom_topic", "/map/telemetry_odometry")
        self.declare_parameter("path_topic", "/map/telemetry_path")
        self.declare_parameter("status_topic", "/map/telemetry_status")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame", "drone_reference")
        self.declare_parameter("utm_zone", 29)
        self.declare_parameter("utm_south", False)
        self.declare_parameter("latitude_min", 39.0)
        self.declare_parameter("latitude_max", 41.0)
        self.declare_parameter("longitude_min", -9.0)
        self.declare_parameter("longitude_max", -7.0)
        self.declare_parameter("repair_negative_longitude", True)
        self.declare_parameter("height_min", 0.0)
        self.declare_parameter("height_max", 120.0)
        self.declare_parameter("default_height_m", 25.0)
        self.declare_parameter("default_heading_deg", 90.0)
        self.declare_parameter("reported_speed_max", 30.0)
        self.declare_parameter("map_z_offset", 0.0)
        self.declare_parameter("map_margin_m", 5.0)
        self.declare_parameter("enable_map_bounds_check", False)
        self.declare_parameter("enable_jump_gate", False)
        self.declare_parameter("max_speed_mps", 8.0)
        self.declare_parameter("max_gate_dt", 10.0)
        self.declare_parameter("position_tolerance_m", 1.5)
        self.declare_parameter("path_min_distance", 0.0)
        self.declare_parameter("max_history_points", 0)
        self.declare_parameter("xy_variance", 2.25)
        self.declare_parameter("z_variance", 4.0)
        self.declare_parameter("yaw_variance", 0.12)
        self.declare_parameter("map_metadata_path", "")

        self.telemetry_topic = self.get_parameter("telemetry_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame = self.get_parameter("child_frame").value
        self.utm_zone = int(self.get_parameter("utm_zone").value)
        self.utm_south = bool(self.get_parameter("utm_south").value)
        self.latitude_min = float(self.get_parameter("latitude_min").value)
        self.latitude_max = float(self.get_parameter("latitude_max").value)
        self.longitude_min = float(self.get_parameter("longitude_min").value)
        self.longitude_max = float(self.get_parameter("longitude_max").value)
        self.repair_negative_longitude = bool(self.get_parameter("repair_negative_longitude").value)
        self.height_min = float(self.get_parameter("height_min").value)
        self.height_max = float(self.get_parameter("height_max").value)
        self.default_height_m = float(self.get_parameter("default_height_m").value)
        self.default_heading_deg = float(self.get_parameter("default_heading_deg").value)
        self.reported_speed_max = float(self.get_parameter("reported_speed_max").value)
        self.map_z_offset = float(self.get_parameter("map_z_offset").value)
        self.map_margin_m = float(self.get_parameter("map_margin_m").value)
        self.enable_map_bounds_check = bool(self.get_parameter("enable_map_bounds_check").value)
        self.enable_jump_gate = bool(self.get_parameter("enable_jump_gate").value)
        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.max_gate_dt = float(self.get_parameter("max_gate_dt").value)
        self.position_tolerance_m = float(self.get_parameter("position_tolerance_m").value)
        self.path_min_distance = float(self.get_parameter("path_min_distance").value)
        self.max_history_points = max(0, int(self.get_parameter("max_history_points").value))
        self.xy_variance = float(self.get_parameter("xy_variance").value)
        self.z_variance = float(self.get_parameter("z_variance").value)
        self.yaw_variance = float(self.get_parameter("yaw_variance").value)
        map_metadata_path = self.get_parameter("map_metadata_path").value

        self.translation = [0.0, 0.0, 0.0]
        self.map_min = [-math.inf, -math.inf, -math.inf]
        self.map_max = [math.inf, math.inf, math.inf]
        self.bounds_available = False
        if map_metadata_path:
            self._load_map_metadata(self._resolve_metadata_path(str(map_metadata_path)))

        self.projector = UtmProjector(self.utm_zone, self.utm_south)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.counts: Counter[str] = Counter()
        self.path = PathMessage()
        self.path.header.frame_id = self.frame_id
        self.last_position: tuple[float, float, float] | None = None
        self.last_path_position: tuple[float, float] | None = None
        self.last_source_time: float | None = None
        self.last_heading_deg = self.default_heading_deg

        self.pose_publisher = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.path_publisher = self.create_publisher(PathMessage, self.path_topic, transient_qos())
        self.status_publisher = self.create_publisher(String, self.status_topic, transient_qos())
        self.subscription = self.create_subscription(String, self.telemetry_topic, self.telemetry_callback, 50)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"Localization online: {self.telemetry_topic} -> pose={self.pose_topic} path={self.path_topic} frame={self.frame_id}"
        )

    def _resolve_metadata_path(self, configured_path: str) -> Path:
        path = Path(configured_path)
        if path.is_absolute():
            return path
        try:
            pkg_share = Path(get_package_share_directory("drone_mapper"))
            candidate = pkg_share / configured_path
            if candidate.exists():
                return candidate
        except Exception:
            pass
        return path

    def _load_map_metadata(self, path: Path) -> None:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            required = ("applied_translation", "bbox_min_after", "bbox_max_after")
            missing = [key for key in required if key not in payload]
            if missing:
                raise ValueError(f"missing metadata keys: {', '.join(missing)}")
            self.translation = [float(v) for v in payload["applied_translation"]]
            self.map_min = [float(v) for v in payload["bbox_min_after"]]
            self.map_max = [float(v) for v in payload["bbox_max_after"]]
            self.bounds_available = True
            self.get_logger().info(f"Loaded map metadata: {path}")
        except Exception as error:
            self.get_logger().warning(f"Could not load map metadata from {path}: {error}")

    def reject(self, reason: str) -> None:
        self.counts["received"] += 1
        self.counts[f"rejected_{reason}"] += 1

    def parse_payload(self, message: String):
        try:
            root = json.loads(message.data)
        except json.JSONDecodeError:
            self.reject("json")
            return None

        telemetry = root.get("telemetry", root)
        source_time = first_number(root.get("timestamp", 0.0))
        latitude = first_number(telemetry.get("latitude"))
        longitude = first_number(telemetry.get("longitude"))
        if latitude is None or longitude is None:
            self.reject("coordinates")
            return None
        if source_time is None:
            source_time = self.get_clock().now().nanoseconds / 1e9

        sign_repaired = False
        if (
            self.repair_negative_longitude
            and longitude > 0.0
            and self.longitude_min <= -longitude <= self.longitude_max
        ):
            longitude = -longitude
            sign_repaired = True

        if not (
            self.latitude_min <= latitude <= self.latitude_max
            and self.longitude_min <= longitude <= self.longitude_max
        ):
            self.reject("coordinates")
            return None

        height = first_number(telemetry.get("height"))
        if height is None or not (self.height_min <= height <= self.height_max):
            height = self.default_height_m
            self.counts["height_fallback"] += 1

        heading = first_number(telemetry.get("heading"))
        if heading is None or not (0.0 <= heading < 360.0):
            heading = self.last_heading_deg
            self.counts["heading_fallback"] += 1

        speed = first_number(telemetry.get("speed"))
        if speed is None or not (0.0 <= speed <= self.reported_speed_max):
            speed = math.nan

        return {
            "source_time": float(source_time),
            "latitude": float(latitude),
            "longitude": float(longitude),
            "height": float(height),
            "heading": float(heading),
            "speed": float(speed),
            "sign_repaired": sign_repaired,
        }

    def telemetry_callback(self, message: String) -> None:
        parsed = self.parse_payload(message)
        if parsed is None:
            return
        self.counts["received"] += 1

        try:
            easting, northing = self.projector.project(parsed["latitude"], parsed["longitude"])
        except Exception as error:
            self.counts["rejected_projection"] += 1
            self.get_logger().warning(str(error))
            return

        map_x = easting - self.translation[0]
        map_y = northing - self.translation[1]
        map_z = self.map_z_offset + parsed["height"] - self.translation[2]

        if self.enable_map_bounds_check and self.bounds_available:
            margin = self.map_margin_m
            if not (
                self.map_min[0] - margin <= map_x <= self.map_max[0] + margin
                and self.map_min[1] - margin <= map_y <= self.map_max[1] + margin
                and self.map_min[2] - margin <= map_z <= self.map_max[2] + margin
            ):
                self.counts["rejected_outside_map"] += 1
                return

        source_time = parsed["source_time"]
        if self.enable_jump_gate and self.last_position is not None and self.last_source_time is not None:
            dt = source_time - self.last_source_time
            distance = math.hypot(
                map_x - self.last_position[0], 
                map_y - self.last_position[1], 
                map_z - self.last_position[2]
            )
            allowed_dt = min(max(dt, 0.0), self.max_gate_dt)
            allowed_distance = self.position_tolerance_m + self.max_speed_mps * allowed_dt
            if dt <= 0.0 or distance > allowed_distance:
                self.counts["rejected_jump"] += 1
                return

        if parsed["sign_repaired"]:
            self.counts["longitude_sign_repaired"] += 1
        self.counts["accepted"] += 1
        self.last_position = (map_x, map_y, map_z)
        self.last_source_time = source_time
        self.last_heading_deg = parsed["heading"]

        yaw = compass_to_ros_yaw(self.last_heading_deg)
        stamp = self.get_clock().now().to_msg()
        pose = self.make_pose(stamp, map_x, map_y, map_z, yaw)
        self.pose_publisher.publish(pose)
        self.odom_publisher.publish(self.make_odometry(pose))
        self.publish_transform(pose)
        self.append_path_pose(pose)
        self.publish_path()

    def make_pose(self, stamp, x_value: float, y_value: float, z_value: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x_value
        pose.pose.position.y = y_value
        pose.pose.position.z = z_value
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def make_odometry(self, pose: PoseStamped) -> Odometry:
        odometry = Odometry()
        odometry.header = pose.header
        odometry.child_frame_id = self.child_frame
        odometry.pose.pose = pose.pose
        odometry.pose.covariance[0] = self.xy_variance
        odometry.pose.covariance[7] = self.xy_variance
        odometry.pose.covariance[14] = self.z_variance
        odometry.pose.covariance[35] = self.yaw_variance
        return odometry

    def publish_transform(self, pose: PoseStamped) -> None:
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def append_path_pose(self, pose: PoseStamped) -> None:
        position = (pose.pose.position.x, pose.pose.position.y)
        if self.last_path_position is not None and self.path_min_distance > 0.0:
            if math.dist(position, self.last_path_position) < self.path_min_distance:
                return
        stored = PoseStamped()
        stored.header = pose.header
        stored.pose = pose.pose
        if self.max_history_points > 0 and len(self.path.poses) >= self.max_history_points:
            self.path.poses.pop(0)
        self.path.poses.append(stored)
        self.last_path_position = position

    def publish_path(self) -> None:
        if not self.path.poses:
            return
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)

    def publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "received": self.counts["received"],
                "accepted": self.counts["accepted"],
                "rejected": self.counts["received"] - self.counts["accepted"],
                "path_poses": len(self.path.poses),
                "longitude_sign_repaired": self.counts["longitude_sign_repaired"],
                "rejected_coordinates": self.counts["rejected_coordinates"],
                "rejected_projection": self.counts["rejected_projection"],
                "rejected_outside_map": self.counts["rejected_outside_map"],
                "rejected_jump": self.counts["rejected_jump"],
                "height_fallback": self.counts["height_fallback"],
                "heading_fallback": self.counts["heading_fallback"],
            },
            sort_keys=True,
        )
        self.status_publisher.publish(message)

    def close(self) -> None:
        self.projector.close()


def main() -> int:
    rclpy.init()
    node = DroneLocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())