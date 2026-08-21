"""Send a short, low-speed camera-free UDP motion test to BugC2.

Nothing is transmitted unless --execute is supplied. Moving tests are limited
to three seconds, command magnitude 0.5, and PWM 28. STOP is sent repeatedly on
normal completion, Ctrl+C, or an exception.
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path


MOTIONS = {
    "stop": (0.0, 0.0, 0.0),
    "forward": (1.0, 0.0, 0.0),
    "backward": (-1.0, 0.0, 0.0),
    "right": (0.0, 1.0, 0.0),
    "left": (0.0, -1.0, 0.0),
    "ccw": (0.0, 0.0, 1.0),
    "cw": (0.0, 0.0, -1.0),
}


def load_default_ip() -> str:
    config_path = Path(__file__).resolve().with_name("config.json")
    return str(json.loads(config_path.read_text(encoding="utf-8"))["udp"]["robot_ip"])


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion", choices=MOTIONS)
    parser.add_argument("--robot-ip", default=load_default_ip())
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--level", type=float, default=0.25)
    parser.add_argument("--pwm-limit", type=int, default=22)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually transmit. Without this flag the command is only printed.",
    )
    args = parser.parse_args()
    if not 0.1 <= args.duration <= 3.0:
        parser.error("--duration must be between 0.1 and 3.0 seconds")
    if not 0.05 <= args.level <= 0.5:
        parser.error("--level must be between 0.05 and 0.5")
    if not 1 <= args.pwm_limit <= 28:
        parser.error("--pwm-limit must be between 1 and 28")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    return args


def packet(sequence: int, forward: float, lateral: float, turn: float, pwm: int):
    return {
        "v": 1,
        "type": "motion",
        "seq": sequence,
        "sent_ms": int(time.time() * 1000),
        "ttl_ms": 350,
        "mode": "velocity_local",
        "forward": round(forward, 4),
        "lateral": round(lateral, 4),
        "turn": round(turn, 4),
        "heading_error_deg": 0.0,
        "target_heading_deg": 0.0,
        "pwm_limit": pwm,
        "reason": "camera_free_test",
    }


def send_json(sock: socket.socket, address: tuple[str, int], value: dict) -> None:
    sock.sendto(json.dumps(value, separators=(",", ":")).encode("utf-8"), address)


def main() -> int:
    args = parse_arguments()
    direction = MOTIONS[args.motion]
    forward = direction[0] * args.level
    lateral = direction[1] * args.level
    turn = direction[2] * args.level
    preview = packet(1, forward, lateral, turn, args.pwm_limit)
    print(json.dumps(preview, ensure_ascii=False, indent=2))
    if not args.execute:
        print("DRY-RUN: add --execute only after lifting the wheels or clearing the field.")
        return 0

    address = (args.robot_ip, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    try:
        if args.motion != "stop":
            print(f"Sending {args.motion} to {args.robot_ip}:{args.port} in 3 seconds...")
            for remaining in (3, 2, 1):
                print(remaining)
                time.sleep(1.0)
        deadline = time.monotonic() + (0.2 if args.motion == "stop" else args.duration)
        while time.monotonic() < deadline:
            sequence += 1
            send_json(
                sock,
                address,
                packet(sequence, forward, lateral, turn, args.pwm_limit),
            )
            time.sleep(0.05)
    finally:
        for _ in range(5):
            sequence += 1
            stop_packet = packet(sequence, 0.0, 0.0, 0.0, 0)
            stop_packet["mode"] = "stop"
            stop_packet["reason"] = "camera_free_test_complete"
            send_json(sock, address, stop_packet)
            time.sleep(0.02)
        sock.close()
        print("STOP sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
