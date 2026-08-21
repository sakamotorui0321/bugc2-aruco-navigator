"""CQ Workshop BugC2: overhead ArUco mapping and conservative path control.

The program starts with UDP transmission disabled. Press A in Windows Terminal
or the preview window to start control and SPACE to send an emergency stop.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS", "fflags;nobuffer|flags;low_delay"
)

try:
    import cv2
    import numpy as np
except ImportError as exc:
    print("OpenCV/NumPy がありません。先に次を実行してください:")
    print("  py -m pip install -r requirements.txt")
    raise SystemExit(2) from exc

try:
    import psutil
except ImportError:
    psutil = None

from planner import astar, simplify_path

if os.name == "nt":
    import msvcrt
else:
    msvcrt = None


Point = tuple[float, float]


@dataclass
class Marker:
    marker_id: int
    corners: Any
    center: Point
    heading_deg: float
    last_seen: float


@dataclass
class MotionCommand:
    mode: str = "stop"
    forward: float = 0.0
    lateral: float = 0.0
    turn: float = 0.0
    heading_error_deg: float = 0.0
    target_heading_deg: float = 0.0
    reason: str = "startup"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def point_distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def heading_to(a: Point, b: Point) -> float:
    # Convert image y-down coordinates to mathematical y-up degrees.
    return math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0]))


def marker_pose(marker_id: int, corners: Any, now: float) -> Marker:
    center_array = np.mean(corners, axis=0)
    top_midpoint = (corners[0] + corners[1]) * 0.5
    vector = top_midpoint - center_array
    heading = math.degrees(math.atan2(-float(vector[1]), float(vector[0])))
    return Marker(
        marker_id=marker_id,
        corners=corners.astype(np.float32),
        center=(float(center_array[0]), float(center_array[1])),
        heading_deg=heading,
        last_seen=now,
    )


class MarkerTracker:
    def __init__(self, alpha: float, hold_seconds: float, static_hold_seconds: float):
        self.alpha = clamp(alpha, 0.0, 1.0)
        self.hold_seconds = max(0.0, hold_seconds)
        self.static_hold_seconds = max(self.hold_seconds, static_hold_seconds)
        self.tracks: dict[int, Marker] = {}

    def update(self, detections: dict[int, Marker], now: float) -> dict[int, Marker]:
        for marker_id, new_marker in detections.items():
            old = self.tracks.get(marker_id)
            if old is None:
                self.tracks[marker_id] = new_marker
                continue
            corners = old.corners + self.alpha * (new_marker.corners - old.corners)
            center_array = np.mean(corners, axis=0)
            angle_delta = wrap_degrees(new_marker.heading_deg - old.heading_deg)
            self.tracks[marker_id] = Marker(
                marker_id=marker_id,
                corners=corners,
                center=(float(center_array[0]), float(center_array[1])),
                heading_deg=wrap_degrees(old.heading_deg + self.alpha * angle_delta),
                last_seen=now,
            )
        kept: dict[int, Marker] = {}
        for marker_id, marker in self.tracks.items():
            hold = self.static_hold_seconds if marker_id >= 20 else self.hold_seconds
            if now - marker.last_seen <= hold:
                kept[marker_id] = marker
        self.tracks = kept
        return dict(self.tracks)


class UdpSender:
    def __init__(self, config: dict[str, Any], enabled: bool):
        configured_ip = str(config["robot_ip"]).strip()
        self.auto_discovery = configured_ip.lower() == "auto"
        self.configured_ip = configured_ip
        self.robot_port = int(config["robot_port"])
        self.robot_id = int(config["robot_id"])
        self.discovery_broadcasts = tuple(
            str(value) for value in config.get(
                "discovery_broadcasts", ["255.255.255.255"]
            )
        )
        self.probe_period = 1.0 / max(0.2, float(config.get("probe_hz", 2.0)))
        self.status_timeout = max(
            0.5, float(config.get("status_timeout_ms", 1500)) / 1000.0
        )
        self.period = 1.0 / max(1.0, float(config["send_hz"]))
        self.ttl_ms = int(config["ttl_ms"])
        self.enabled = enabled
        self.sequence = 0
        self.last_send = 0.0
        self.last_probe = 0.0
        self.last_status = 0.0
        self.status: dict[str, Any] = {}
        self.resolved_address: tuple[str, int] | None = (
            None if self.auto_discovery else (configured_ip, self.robot_port)
        )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.socket.bind(("", 0))
        self.socket.setblocking(False)

    @property
    def robot_connected(self) -> bool:
        return (
            self.last_status > 0.0
            and time.monotonic() - self.last_status <= self.status_timeout
        )

    @property
    def robot_armed(self) -> bool:
        return self.robot_connected and bool(self.status.get("armed", False))

    @property
    def destination_text(self) -> str:
        if self.resolved_address is None:
            return f"AUTO (robot ID {self.robot_id})"
        return f"{self.resolved_address[0]}:{self.resolved_address[1]}"

    @property
    def status_text(self) -> str:
        if not self.robot_connected:
            return f"WAIT ID:{self.robot_id}"
        state = str(self.status.get("state", "UNKNOWN"))
        armed = "ARMED" if self.robot_armed else "SAFE"
        return f"{armed} {self.destination_text} {state}"

    def _next_sequence(self) -> int:
        self.sequence += 1
        return self.sequence

    def _send_packet(self, packet: dict[str, Any], address: tuple[str, int]) -> None:
        payload = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        self.socket.sendto(payload, address)

    def send_probe(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_probe < self.probe_period:
            return
        packet = {
            "v": 1,
            "type": "probe",
            "seq": self._next_sequence(),
            "sent_ms": int(time.time() * 1000),
            "robot_id": self.robot_id,
        }
        if self.resolved_address is not None:
            targets = (self.resolved_address,)
        else:
            targets = tuple(
                (broadcast, self.robot_port) for broadcast in self.discovery_broadcasts
            )
        for target in targets:
            self._send_packet(packet, target)
        self.last_probe = now

    def poll_status(self) -> None:
        while True:
            try:
                payload, source = self.socket.recvfrom(1024)
            except BlockingIOError:
                break
            except OSError:
                break
            try:
                status = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if (
                status.get("v") != 1
                or status.get("type") != "status"
                or status.get("robot_id") != self.robot_id
            ):
                continue
            if not self.auto_discovery and source[0] != self.configured_ip:
                continue
            self.resolved_address = (source[0], self.robot_port)
            self.status = status
            self.last_status = time.monotonic()

    def maintain_link(self) -> None:
        self.poll_status()
        if (
            self.auto_discovery
            and self.resolved_address is not None
            and self.last_status > 0.0
            and time.monotonic() - self.last_status > self.status_timeout * 2.0
        ):
            self.resolved_address = None
            self.status = {}
        self.send_probe()
        self.poll_status()

    def set_enabled(self, enabled: bool) -> None:
        if self.enabled and not enabled:
            self.send_stop("disarmed", force=True)
        self.enabled = enabled

    def send(self, command: MotionCommand, pwm_limit: int, force: bool = False) -> None:
        if not self.enabled and not (force and command.mode == "stop"):
            return
        if self.resolved_address is None:
            return
        now = time.monotonic()
        if not force and now - self.last_send < self.period:
            return
        packet = {
            "v": 1,
            "type": "motion",
            "seq": self._next_sequence(),
            "sent_ms": int(time.time() * 1000),
            "robot_id": self.robot_id,
            "ttl_ms": self.ttl_ms,
            "mode": command.mode,
            "forward": round(command.forward, 4),
            "lateral": round(command.lateral, 4),
            "turn": round(command.turn, 4),
            "heading_error_deg": round(command.heading_error_deg, 2),
            "target_heading_deg": round(command.target_heading_deg, 2),
            "pwm_limit": int(pwm_limit),
            "reason": command.reason,
        }
        self._send_packet(packet, self.resolved_address)
        self.last_send = now

    def send_stop(self, reason: str, force: bool = False) -> None:
        self.send(MotionCommand(reason=reason), pwm_limit=0, force=force)

    def close(self) -> None:
        if self.enabled:
            for _ in range(3):
                self.send_stop("pc_shutdown", force=True)
        self.socket.close()


class CsvLogger:
    FIELDS = (
        "time",
        "self_x_px",
        "self_y_px",
        "self_x_norm",
        "self_y_norm",
        "self_heading_deg",
        "goal_x_px",
        "goal_y_px",
        "mode",
        "forward",
        "lateral",
        "turn",
        "heading_error_deg",
        "reason",
        "detected_ids",
    )

    def __init__(self, base_directory: Path, config: dict[str, Any]):
        self.file = None
        self.writer = None
        if not config.get("enabled", True):
            return
        directory = base_directory / str(config.get("directory", "logs"))
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / time.strftime("run_%Y%m%d_%H%M%S.csv")
        self.file = path.open("w", newline="", encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()

    def write(self, row: dict[str, Any]) -> None:
        if self.writer is None:
            return
        self.writer.writerow(row)

    def close(self) -> None:
        if self.file is not None:
            self.file.close()


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def create_detector(dictionary_name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco がありません。opencv-python ではなく "
            "opencv-contrib-python をインストールしてください。"
        )
    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None:
        raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 43
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.minMarkerPerimeterRate = 0.008
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def detect_markers(detector: Any, frame: Any, now: float) -> dict[int, Marker]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners_list, ids, _ = detector.detectMarkers(gray)
    result: dict[int, Marker] = {}
    if ids is None:
        return result
    for corners, marker_id in zip(corners_list, ids.flatten()):
        marker_id = int(marker_id)
        result[marker_id] = marker_pose(marker_id, corners[0], now)
    return result


def field_polygon(markers: dict[int, Marker], ids: list[int]):
    if any(marker_id not in markers for marker_id in ids):
        return None
    return np.array([markers[marker_id].center for marker_id in ids], dtype=np.int32)


def normalized_position(point: Point, polygon: Any) -> Point | None:
    if polygon is None or len(polygon) != 4:
        return None
    source = polygon.astype(np.float32)
    destination = np.array(
        [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32
    )
    transform = cv2.getPerspectiveTransform(source, destination)
    value = cv2.perspectiveTransform(
        np.array([[[point[0], point[1]]]], dtype=np.float32), transform
    )[0, 0]
    return float(value[0]), float(value[1])


def build_occupancy(
    frame_shape: tuple[int, ...], markers: dict[int, Marker], config: dict[str, Any]
):
    height, width = frame_shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    px_per_cm = float(config["pixels_per_cm"])
    robot_radius_cm = float(config["robot_width_cm"]) * 0.5
    robot_clearance = int(
        round(
            (robot_radius_cm + float(config["safety_margin_cm"]))
            * px_per_cm
        )
    )
    edge_margin = int(round(float(config["field_edge_margin_cm"]) * px_per_cm))
    corner_ids = [int(value) for value in config["field_corner_ids"]]
    polygon = field_polygon(markers, corner_ids)
    if polygon is not None:
        mask.fill(255)
        cv2.fillPoly(mask, [polygon], 0)
        cv2.polylines(mask, [polygon], True, 255, max(1, 2 * edge_margin))
    else:
        cv2.rectangle(mask, (0, 0), (width - 1, height - 1), 255, max(1, 2 * edge_margin))

    wall_min = int(config["wall_id_min"])
    wall_max = int(config["wall_id_max"])
    for even_id in range(wall_min + (wall_min % 2), wall_max + 1, 2):
        odd_id = even_id + 1
        if even_id not in markers or odd_id not in markers:
            continue
        p1 = tuple(round(value) for value in markers[even_id].center)
        p2 = tuple(round(value) for value in markers[odd_id].center)
        cv2.line(mask, p1, p2, 255, max(1, 2 * robot_clearance))

    self_id = int(config["self_id"])
    goal_id = int(config["goal_id"])
    excluded = {self_id, goal_id, *corner_ids}
    obstacle_radius = int(
        round(
            (
                float(config["marker_obstacle_radius_cm"])
                + robot_radius_cm
                + float(config["safety_margin_cm"])
            )
            * px_per_cm
        )
    )
    for marker_id, marker in markers.items():
        if marker_id in excluded or marker_id >= wall_min:
            continue
        center = tuple(round(value) for value in marker.center)
        cv2.circle(mask, center, obstacle_radius, 255, -1)
    return mask, polygon


def plan_path(mask: Any, start: Point, goal: Point, grid_px: int) -> list[Point]:
    height, width = mask.shape
    grid_width = max(1, math.ceil(width / grid_px))
    grid_height = max(1, math.ceil(height / grid_px))
    reduced = cv2.resize(mask, (grid_width, grid_height), interpolation=cv2.INTER_AREA)
    blocked = (reduced > 1).tolist()
    grid_start = (
        int(clamp(start[0] / grid_px, 0, grid_width - 1)),
        int(clamp(start[1] / grid_px, 0, grid_height - 1)),
    )
    grid_goal = (
        int(clamp(goal[0] / grid_px, 0, grid_width - 1)),
        int(clamp(goal[1] / grid_px, 0, grid_height - 1)),
    )
    route = simplify_path(blocked, astar(blocked, grid_start, grid_goal))
    return [
        (
            min(width - 1.0, x * grid_px + grid_px * 0.5),
            min(height - 1.0, y * grid_px + grid_px * 0.5),
        )
        for x, y in route
    ]


def choose_lookahead(start: Point, route: list[Point], distance_px: float) -> Point:
    if not route:
        return start
    previous = start
    travelled = 0.0
    for point in route[1:]:
        segment = point_distance(previous, point)
        if travelled + segment >= distance_px and segment > 0.0:
            ratio = (distance_px - travelled) / segment
            return (
                previous[0] + ratio * (point[0] - previous[0]),
                previous[1] + ratio * (point[1] - previous[1]),
            )
        travelled += segment
        previous = point
    return route[-1]


def make_command(
    robot: Marker, goal: Marker, route: list[Point], config: dict[str, Any]
) -> tuple[MotionCommand, Point]:
    px_per_cm = float(config["pixels_per_cm"])
    goal_distance_cm = point_distance(robot.center, goal.center) / px_per_cm
    target = choose_lookahead(
        robot.center, route, float(config["lookahead_cm"]) * px_per_cm
    )
    if goal_distance_cm <= float(config["goal_radius_cm"]):
        return MotionCommand(reason="goal_reached"), target
    if len(route) < 2:
        return MotionCommand(reason="no_safe_path"), target

    target_heading = heading_to(robot.center, target)
    robot_heading = wrap_degrees(
        robot.heading_deg + float(config.get("heading_offset_deg", 0.0))
    )
    error = wrap_degrees(target_heading - robot_heading)
    turn = clamp(float(config["heading_kp"]) * error, -1.0, 1.0)
    forward = 0.0
    reason = "turn_to_path"
    if abs(error) <= float(config["max_heading_error_to_drive_deg"]):
        slowdown = clamp(
            goal_distance_cm / max(1.0, float(config["slowdown_distance_cm"])),
            0.0,
            1.0,
        )
        forward = max(
            float(config["minimum_command"]),
            float(config["cruise_command"]) * slowdown,
        )
        reason = "follow_path"
    return (
        MotionCommand(
            mode="velocity",
            forward=forward,
            lateral=0.0,
            turn=turn,
            heading_error_deg=error,
            target_heading_deg=target_heading,
            reason=reason,
        ),
        target,
    )


def nearest_other_robot_cm(
    robot: Marker, markers: dict[int, Marker], config: dict[str, Any]
) -> tuple[float, int | None]:
    """Distance to the nearest other player or moving obstacle."""
    nearest = math.inf
    nearest_id: int | None = None
    player_min = int(config["player_id_min"])
    player_max = int(config["player_id_max"])
    moving_min = int(config["moving_obstacle_id_min"])
    moving_max = int(config["moving_obstacle_id_max"])
    for marker_id, marker in markers.items():
        if marker_id == robot.marker_id:
            continue
        is_player = player_min <= marker_id <= player_max
        is_moving = moving_min <= marker_id <= moving_max
        if not (is_player or is_moving):
            continue
        distance_cm = point_distance(robot.center, marker.center) / float(
            config["pixels_per_cm"]
        )
        if distance_cm < nearest:
            nearest = distance_cm
            nearest_id = marker_id
    return nearest, nearest_id


def draw_scene(
    frame: Any,
    markers: dict[int, Marker],
    mask: Any,
    polygon: Any,
    route: list[Point],
    target: Point | None,
    command: MotionCommand,
    sender: UdpSender,
    config: dict[str, Any],
    fps: float,
) -> Any:
    display = frame.copy()
    red = np.zeros_like(display)
    red[:, :, 2] = mask
    display = cv2.addWeighted(display, 1.0, red, 0.22, 0.0)

    robot_marker = markers.get(int(config["self_id"]))
    goal_marker = markers.get(int(config["goal_id"]))
    if robot_marker is not None and goal_marker is not None:
        direct_width_px = max(
            1,
            round(float(config["robot_width_cm"]) * float(config["pixels_per_cm"])),
        )
        direct_start = tuple(round(value) for value in robot_marker.center)
        direct_goal = tuple(round(value) for value in goal_marker.center)
        cv2.line(display, direct_start, direct_goal, (55, 55, 55), direct_width_px)
        cv2.line(display, direct_start, direct_goal, (170, 170, 170), 1)

    for marker_id, marker in markers.items():
        corners = marker.corners.reshape((-1, 1, 2)).astype(np.int32)
        center = tuple(round(value) for value in marker.center)
        color = (0, 255, 0)
        if marker_id == int(config["self_id"]):
            color = (0, 255, 255)
        elif marker_id == int(config["goal_id"]):
            color = (255, 255, 0)
        cv2.polylines(display, [corners], True, color, 2)
        cv2.circle(display, center, 4, (0, 0, 255), -1)
        cv2.putText(
            display,
            f"ID:{marker_id}",
            (center[0] - 20, center[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )
        heading = math.radians(marker.heading_deg + float(config["heading_offset_deg"]))
        arrow = (
            round(center[0] + 30 * math.cos(heading)),
            round(center[1] - 30 * math.sin(heading)),
        )
        cv2.arrowedLine(display, center, arrow, color, 2, tipLength=0.25)

    if polygon is not None:
        cv2.polylines(display, [polygon], True, (255, 0, 255), 3)
    if len(route) >= 2:
        route_points = np.array(route, dtype=np.int32).reshape((-1, 1, 2))
        corridor_px = max(
            1,
            round(float(config["robot_width_cm"]) * float(config["pixels_per_cm"])),
        )
        cv2.polylines(display, [route_points], False, (90, 45, 120), corridor_px)
        cv2.polylines(display, [route_points], False, (255, 20, 147), 3)
    if target is not None:
        cv2.circle(display, tuple(round(v) for v in target), 7, (0, 165, 255), 2)

    state = "CONTROL ARMED" if sender.enabled else "WAIT A"
    state_color = (0, 0, 255) if sender.enabled else (0, 255, 255)
    detected_ids = ",".join(str(marker_id) for marker_id in sorted(markers)) or "none"
    if len(detected_ids) > 72:
        detected_ids = f"{detected_ids[:69]}..."
    lines = [
        (f"{state}  FPS:{fps:.1f}  IDs:{len(markers)}", state_color),
        (
            f"ROBOT: {sender.status_text}",
            (0, 255, 0) if sender.robot_connected else (0, 165, 255),
        ),
        (f"ARUCO IDs: {detected_ids}", (0, 255, 255)),
        (
            f"CMD f:{command.forward:+.2f} lat:{command.lateral:+.2f} "
            f"turn:{command.turn:+.2f} err:{command.heading_error_deg:+.1f}",
            (255, 255, 255),
        ),
        (f"STATE: {command.reason}", (255, 255, 255)),
        ("Terminal/window A: START  SPACE: stop  Q: quit", (255, 255, 255)),
    ]
    for index, (text, color) in enumerate(lines):
        cv2.putText(
            display,
            text,
            (16, 28 + index * 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            color,
            2,
        )
    return display


def open_capture(source: str):
    parsed_source: Any = int(source) if source.isdigit() else source
    if isinstance(parsed_source, str) and parsed_source.startswith("http"):
        capture = cv2.VideoCapture(parsed_source, cv2.CAP_FFMPEG)
    else:
        capture = cv2.VideoCapture(parsed_source)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def read_console_key() -> str | None:
    """Read one Windows Terminal key without requiring Enter."""
    if msvcrt is None or not msvcrt.kbhit():
        return None
    key = msvcrt.getwch()
    if key in ("\x00", "\xe0"):
        if msvcrt.kbhit():
            msvcrt.getwch()
        return None
    return key


def current_windows_ssid() -> str | None:
    if os.name != "nt":
        return None
    try:
        result = subprocess.run(
            ["netsh", "wlan", "show", "interfaces"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    for line in result.stdout.splitlines():
        name, separator, value = line.partition(":")
        if separator and name.strip().upper() == "SSID":
            return value.strip()
    return None


def wait_for_expected_network(expected_ssid: str) -> bool:
    if not expected_ssid:
        return True
    current = current_windows_ssid()
    if current is None:
        print(f"Wi-Fi SSIDを取得できません。手動で{expected_ssid}を確認してください。")
        return True
    if current == expected_ssid:
        print(f"Wi-Fi OK: {current}")
        return True

    print("=" * 68)
    print(f"NETWORK WAIT: 現在のSSIDは {current}")
    print(f"WindowsのWi-Fiを {expected_ssid} へ切り替えてください。")
    print("接続を検出すると自動的にカメラへ接続します。Qで終了します。")
    print("=" * 68)
    last_reported = current
    while True:
        key = read_console_key()
        if key is not None and key.lower() == "q":
            return False
        time.sleep(1.0)
        current = current_windows_ssid()
        if current == expected_ssid:
            print(f"Wi-Fi OK: {current}")
            return True
        if current != last_reported:
            print(f"NETWORK WAIT: 現在のSSIDは {current or '未接続'}")
            last_reported = current


def handle_control_key(
    key: str | int | None,
    sender: UdpSender,
    command: MotionCommand,
) -> bool:
    """Apply a terminal/window key. Return True when the program should quit."""
    if key is None or key == 255:
        return False
    if isinstance(key, int):
        if key == 27:
            return True
        if not 0 <= key <= 0x10FFFF:
            return False
        key_text = chr(key)
    else:
        key_text = key
    if key_text.lower() == "q":
        return True
    if key_text.lower() == "a":
        if sender.enabled:
            print("Control is already ARMED. Press SPACE to stop.")
        elif not sender.robot_connected:
            print("START rejected: robot_status_not_received")
        elif not sender.robot_armed:
            print(f"START rejected: robot_not_armed ({sender.status_text})")
        elif command.mode != "velocity":
            print(f"START rejected: {command.reason}")
        else:
            sender.set_enabled(True)
            print("CONTROL STARTED: UDP transmission ARMED")
        return False
    if key_text == " ":
        sender.send_stop("keyboard_emergency_stop", force=True)
        sender.set_enabled(False)
        print("Emergency stop sent; UDP disarmed")
    return False


def parse_arguments() -> argparse.Namespace:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=base / "config.json")
    parser.add_argument("--source", help="Override URL, video path, or camera index")
    parser.add_argument("--self-id", type=int, help="Override player marker ID")
    parser.add_argument(
        "--skip-network-check",
        action="store_true",
        help="Skip the Windows Wi-Fi SSID preflight (for video/USB tests).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    config = load_config(args.config.resolve())
    if args.self_id is not None:
        config["self_id"] = args.self_id
        config["udp"]["robot_id"] = args.self_id
    if int(config["udp"]["robot_id"]) != int(config["self_id"]):
        print("設定エラー: udp.robot_id と self_id は同じ値にしてください。")
        return 2
    expected_ssid = str(config.get("network", {}).get("expected_pc_ssid", ""))
    if not args.skip_network_check and not wait_for_expected_network(expected_ssid):
        return 0
    source = str(args.source or config["stream_url"])
    detector = create_detector(str(config["aruco_dictionary"]))
    tracker = MarkerTracker(
        float(config["marker_smoothing_alpha"]),
        float(config["marker_hold_seconds"]),
        float(config["static_marker_hold_seconds"]),
    )
    udp_config = config["udp"]
    # 本番では必ず映像画面を確認してAを押した後にだけ送信を開始する。
    sender = UdpSender(udp_config, False)
    logger = CsvLogger(args.config.resolve().parent, config["logging"])
    process = psutil.Process(os.getpid()) if psutil is not None else None
    capture = open_capture(source)
    cv2.namedWindow("BugC2 ArUco Navigator", cv2.WINDOW_NORMAL)
    print(f"Stream: {source}")
    print(f"Self ID: {config['self_id']} / Goal ID: {config['goal_id']}")
    print(f"UDP robot: {sender.destination_text} (control starts OFF)")
    print("Robot discovery starts automatically. Press A in this terminal to start.")

    route: list[Point] = []
    target: Point | None = None
    command = MotionCommand(reason="waiting_for_frame")
    last_plan = 0.0
    last_log = 0.0
    last_frame_time = time.monotonic()
    fps = 0.0
    failed_reads = 0
    previous_robot_connected = False

    try:
        while True:
            sender.maintain_link()
            if sender.robot_connected != previous_robot_connected:
                if sender.robot_connected:
                    print(f"ROBOT CONNECTED: {sender.status_text}")
                else:
                    print("ROBOT DISCONNECTED: waiting for status reply")
                    if sender.enabled:
                        sender.set_enabled(False)
                        print("UDP control disarmed because robot status was lost")
                previous_robot_connected = sender.robot_connected

            if not capture.isOpened():
                command = MotionCommand(reason="stream_disconnected")
                sender.send(command, int(config["pwm_limit"]), force=True)
                if handle_control_key(read_console_key(), sender, command):
                    break
                time.sleep(0.5)
                capture.release()
                capture = open_capture(source)
                continue

            grabbed = capture.grab()
            ret, frame = capture.retrieve() if grabbed else (False, None)
            if not ret or frame is None:
                failed_reads += 1
                command = MotionCommand(reason="frame_lost")
                sender.send(command, int(config["pwm_limit"]), force=True)
                if handle_control_key(read_console_key(), sender, command):
                    break
                if failed_reads >= 20:
                    capture.release()
                time.sleep(0.01)
                continue
            failed_reads = 0
            now = time.monotonic()
            delta = max(1e-6, now - last_frame_time)
            fps = 0.9 * fps + 0.1 / delta if fps else 1.0 / delta
            last_frame_time = now

            current = detect_markers(detector, frame, now)
            markers = tracker.update(current, now)
            mask, polygon = build_occupancy(frame.shape, markers, config)
            robot = markers.get(int(config["self_id"]))
            goal = markers.get(int(config["goal_id"]))
            field_ready = polygon is not None

            if robot is None:
                command = MotionCommand(reason="self_marker_lost")
                route = []
                target = None
            elif goal is None:
                command = MotionCommand(reason="goal_marker_lost")
                route = []
                target = None
            elif not field_ready:
                command = MotionCommand(reason="field_corners_lost")
                route = []
                target = None
            else:
                if now - last_plan >= 0.12:
                    route = plan_path(
                        mask,
                        robot.center,
                        goal.center,
                        int(config["planning_grid_px"]),
                    )
                    last_plan = now
                nearest_cm, nearest_id = nearest_other_robot_cm(robot, markers, config)
                if nearest_cm <= float(config["other_robot_stop_distance_cm"]):
                    command = MotionCommand(
                        reason=f"robot_{nearest_id}_too_close_{nearest_cm:.1f}cm"
                    )
                    target = None
                else:
                    command, target = make_command(robot, goal, route, config)

            sender.send(command, int(config["pwm_limit"]))

            if now - last_log >= 0.1:
                normal = normalized_position(robot.center, polygon) if robot else None
                logger.write(
                    {
                        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                        "self_x_px": f"{robot.center[0]:.2f}" if robot else "",
                        "self_y_px": f"{robot.center[1]:.2f}" if robot else "",
                        "self_x_norm": f"{normal[0]:.5f}" if normal else "",
                        "self_y_norm": f"{normal[1]:.5f}" if normal else "",
                        "self_heading_deg": f"{robot.heading_deg:.2f}" if robot else "",
                        "goal_x_px": f"{goal.center[0]:.2f}" if goal else "",
                        "goal_y_px": f"{goal.center[1]:.2f}" if goal else "",
                        "mode": command.mode,
                        "forward": f"{command.forward:.4f}",
                        "lateral": f"{command.lateral:.4f}",
                        "turn": f"{command.turn:.4f}",
                        "heading_error_deg": f"{command.heading_error_deg:.2f}",
                        "reason": command.reason,
                        "detected_ids": " ".join(str(value) for value in sorted(markers)),
                    }
                )
                last_log = now

            display = draw_scene(
                frame,
                markers,
                mask,
                polygon,
                route,
                target,
                command,
                sender,
                config,
                fps,
            )
            if process is not None:
                memory_mb = process.memory_info().rss / (1024 * 1024)
                cv2.putText(
                    display,
                    f"RAM:{memory_mb:.1f}MB",
                    (16, display.shape[0] - 16),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )
            cv2.imshow("BugC2 ArUco Navigator", display)
            window_key = cv2.waitKey(1) & 0xFF
            console_key = read_console_key()
            if handle_control_key(console_key, sender, command):
                break
            if handle_control_key(window_key, sender, command):
                break
    except KeyboardInterrupt:
        pass
    finally:
        sender.close()
        logger.close()
        capture.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
