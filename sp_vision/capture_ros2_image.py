"""Capture one ROS 2 Foxy compressed camera frame over SSH, without robot control."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np


TOPICS = {
    "right": "/camera/right/color/image_resized/compressed",
    "top": "/camera/top/color/image_raw/compressed",
}


def capture(host: str, topic: str, timeout: float) -> bytes:
    # ROS 2 Foxy and its Python packages live on the robot development host.
    # Keep the binary image on stdout and diagnostics on stderr.
    remote = f"""source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0
python3 - <<'PY'
import sys
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

rclpy.init()
node = rclpy.create_node('sp_vision_camera_snapshot')
frame = None

def receive(message):
    global frame
    frame = bytes(message.data)

subscription = node.create_subscription(
    CompressedImage, {json.dumps(topic)}, receive, qos_profile_sensor_data
)
deadline = time.monotonic() + {timeout!r}
while frame is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)
if frame is None:
    print('Timed out waiting for ' + {json.dumps(topic)}, file=sys.stderr)
    sys.exit(2)
sys.stdout.buffer.write(frame)
sys.stdout.buffer.flush()
node.destroy_node()
rclpy.shutdown()
PY
"""
    result = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", host, "bash -s"],
        input=remote.encode(), capture_output=True, timeout=timeout + 10, check=False,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors="replace").strip() or
                           f"SSH capture failed with exit code {result.returncode}")
    return result.stdout


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=TOPICS, default="right")
    parser.add_argument("--host", default="guest@10.192.1.4")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 0 < args.timeout <= 60:
        parser.error("--timeout must be between 0 and 60 seconds")
    output = args.output or Path(__file__).resolve().parent / "data" / f"{args.camera}-camera-latest.jpg"
    try:
        payload = capture(args.host, TOPICS[args.camera], args.timeout)
        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.size == 0:
            raise ValueError("ROS 2 camera payload is not a decodable image")
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_name(output.name + ".tmp")
        temporary.write_bytes(payload)
        temporary.replace(output)
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"capture failed: {error}", file=sys.stderr)
        return 1
    print(f"{TOPICS[args.camera]} -> {output} ({image.shape[1]}x{image.shape[0]})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
