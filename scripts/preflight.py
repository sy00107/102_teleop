#!/usr/bin/env python3
"""Offline preflight: inspect files/devices only, never instantiate a motor bus."""

import argparse
import json
import os
import subprocess
from pathlib import Path

from lerobot.robots.rebot_b601_follower import (
    RebotB601Follower,
    RebotB601FollowerRobotConfig,
)
from lerobot.teleoperators.rebot_102_leader import (
    RebotArm102Leader,
    RebotArm102LeaderTeleopConfig,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args()
    errors = []
    cfg = RebotB601FollowerRobotConfig(
        port=os.environ["FOLLOWER_PORT"],
        id=os.environ["FOLLOWER_ID"],
        motor_family=os.environ["FOLLOWER_FAMILY"],
        gravity_comp=os.environ.get("GRAVITY_COMP", "0") == "1",
        gravity_urdf_path=os.environ["GRAVITY_URDF"],
        gravity_mount_roll_deg=float(os.environ["GRAVITY_MOUNT_ROLL_DEG"]),
        gravity_mount_pitch_deg=float(os.environ["GRAVITY_MOUNT_PITCH_DEG"]),
        gravity_tau_scale=float(os.environ["GRAVITY_TAU_SCALE"]),
        gravity_missing_finger_mass_kg=float(os.environ["GRAVITY_FINGER_MASS_KG"]),
        background_control=True,
        control_mode="mit",
        control_rate_hz=float(os.environ["MOTOR_CONTROL_HZ"]),
        command_timeout_s=float(os.environ["COMMAND_TIMEOUT_S"]),
        max_relative_target=float(os.environ["MAX_RELATIVE_TARGET"]),
        max_target_velocity_deg_s=float(os.environ["MAX_TARGET_VELOCITY_DEG_S"]),
        max_target_acceleration_deg_s2=float(os.environ["MAX_TARGET_ACCELERATION_DEG_S2"]),
        gripper_open_deg=float(os.environ.get("GRIPPER_OPEN_DEG", "270")),
        gripper_max_velocity_deg_s=float(os.environ.get("GRIPPER_MAX_VELOCITY_DEG_S", "300")),
        gripper_max_acceleration_deg_s2=float(os.environ.get("GRIPPER_MAX_ACCELERATION_DEG_S2", "3000")),
        comm_loss_action=os.environ.get("COMM_LOSS_ACTION", "hold"),
        comm_loss_home_delay_s=float(os.environ.get("COMM_LOSS_HOME_DELAY_S", "0.5")),
        home_velocity_deg_s=float(os.environ.get("HOME_VELOCITY_DEG_S", "20")),
        home_acceleration_deg_s2=float(os.environ.get("HOME_ACCELERATION_DEG_S2", "60")),
        require_supported_release=True,
    )
    robot = RebotB601Follower(cfg)  # model construction only; connect() is never called
    leader = RebotArm102Leader(
        RebotArm102LeaderTeleopConfig(
            port=os.environ["LEADER_PORT"], id=os.environ["LEADER_ID"]
        )
    )
    for label, device in (("follower", robot), ("leader", leader)):
        if not device.is_calibrated:
            errors.append(
                f"{label}: matching calibration file missing: {device.calibration_fpath}"
            )
        port = Path(device.config.port)
        if not port.exists():
            errors.append(f"{label}: USB device missing: {port}")
        elif not os.access(port, os.R_OK | os.W_OK):
            errors.append(f"{label}: no read/write permission: {port}")
        else:
            try:
                busy = subprocess.run(
                    ["fuser", str(port)], capture_output=True, text=True
                )
                if busy.returncode == 0:
                    errors.append(
                        f"{label}: serial port is already in use: {busy.stdout.strip()}"
                    )
            except FileNotFoundError:
                errors.append(
                    "fuser is unavailable; cannot check serial port ownership"
                )
    try:
        cameras = json.loads(os.environ.get("CAMERAS") or "{}")
        if not isinstance(cameras, dict):
            raise ValueError("CAMERAS must be a JSON object")
    except (ValueError, TypeError) as error:
        errors.append(f"Invalid camera config: {error}")
        cameras = {}
    if (
        args.record
        and not cameras
        and os.environ.get("ALLOW_STATE_ONLY_RECORDING") != "1"
    ):
        errors.append(
            "Camera config empty: configure CAMERAS, or explicitly select state-only test recording"
        )
    print(
        json.dumps(
            {
                "model": robot.control_metadata(),
                "errors": errors,
                "scope": "Offline only. Motor zero/direction, bus latency/watchdog and supported release need physical validation.",
            },
            indent=2,
        )
    )
    raise SystemExit(1 if errors else 0)


if __name__ == "__main__":
    main()
