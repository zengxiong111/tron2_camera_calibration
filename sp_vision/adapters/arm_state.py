"""Optional read-only TRON2 feedback adapter using tron2_env transport."""

from __future__ import annotations

import math
import time
from copy import deepcopy

import numpy as np


def read_state(robot_config: dict, *, timeout_s: float = 5.0) -> dict:
    """Read one fresh arm/head feedback sample without a motion-capable wrapper.

    Returns the same fields the deployment ``tron2-deploy state`` command wrote:
    ``arm_q14`` (left arm then right arm, 14 measured joint angles in radians),
    ``head_q2`` (measured pitch/yaw) and ``timestamp_s``. Only feedback is
    requested; no motion command is available on the returned data.
    """
    try:
        from tron2_env.config import Tron2Config
        from tron2_env.transport.websocket import WebsocketTransport
    except ImportError as error:
        raise RuntimeError("controller state probing requires optional tron2_env") from error

    host = robot_config.get("host")
    if not isinstance(host, str) or not host.strip():
        raise ValueError("robot.host must explicitly identify the controller")
    timeout = float(timeout_s)
    feedback_hz = float(robot_config.get("feedback_hz", 200.0))
    if not math.isfinite(timeout) or timeout <= 0 or timeout > 60:
        raise ValueError("timeout_s must be finite and in (0, 60]")
    if not math.isfinite(feedback_hz) or feedback_hz <= 0:
        raise ValueError("robot.feedback_hz must be positive and finite")

    class ArmFeedbackTransport(WebsocketTransport):
        def _poll_feedback(self):
            # The upstream poll also requests claw state, which this adapter
            # neither needs nor wants. Only joint feedback is requested.
            period = 1.0 / self.config.polling_rate
            while not self.should_exit:
                started = time.monotonic()
                self._send_request("request_get_joint_state")
                time.sleep(max(0.0, period - (time.monotonic() - started)))

    transport = ArmFeedbackTransport(Tron2Config(
        robot_ip=host.strip(),
        port=int(robot_config.get("port", 5000)),
        init_joints=None,
        init_head=None,
        init_ee_z_min=None,
        polling_rate=feedback_hz,
        connection_timeout=timeout,
    ))
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with transport._state_lock:
                raw = deepcopy(transport.joint_states)
            stamp = float(raw.get("timestamp", -1)) / 1000.0
            if transport.is_connected() and stamp > 0:
                state = np.asarray(raw.get("states"), dtype=float)
                if state.shape != (18,) or not np.isfinite(state).all():
                    raise ValueError("controller state must contain 18 finite values")
                return {
                    "arm_q14": np.r_[state[:7], state[8:15]].tolist(),
                    "head_q2": state[16:18].tolist(),
                    "timestamp_s": stamp,
                    "source": "real",
                }
            time.sleep(0.001)
        raise TimeoutError("fresh controller arm feedback was not received")
    finally:
        transport.disconnect()


def read_arm_q14(robot_config: dict, *, timeout_s: float = 5.0) -> list[float]:
    """Read fresh arm feedback without constructing a motion-capable wrapper."""
    return read_state(robot_config, timeout_s=timeout_s)["arm_q14"]
