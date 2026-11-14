#!/usr/bin/env python3
"""Calculate B601-DM gravity torques without opening a motor connection."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
DEFAULT_URDF = Path(__file__).resolve().parents[1] / "assets/ReBot_Arm_DM.urdf"


def finite_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("a finite number is required")
    return number


def gravity_in_base(roll_deg: float, pitch_deg: float) -> np.ndarray:
    """R_WB = Ry(pitch) @ Rx(roll); g_B = R_WB.T @ g_W."""
    r, p = np.deg2rad([roll_deg, pitch_deg])
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    return (ry @ rx).T @ np.array([0.0, 0.0, -9.81])


def load_arm(urdf: Path):
    # A whole mobile-robot URDF already includes the mounting transform.
    # Require an arm-only model so the tilt cannot be applied twice by accident.
    root = ET.parse(urdf).getroot()
    links = {link.attrib["name"] for link in root.findall("link")}
    children = {joint.find("child").attrib["link"] for joint in root.findall("joint")}
    if links - children != {"base_link"}:
        raise ValueError("Use the arm-only URDF rooted at base_link, without a world/mount joint.")
    model = pin.buildModelFromUrdf(str(urdf))
    indices = []
    for name in JOINT_NAMES:
        if not model.existJointName(name):
            raise ValueError(f"URDF has no joint named {name}")
        joint = model.joints[model.getJointId(name)]
        if joint.nq != 1 or joint.nv != 1:
            raise ValueError(f"{name} must be a scalar joint")
        indices.append((joint.idx_q, joint.idx_v))
    return model, indices


def calculate(model, indices, q_rad, roll_deg: float, pitch_deg: float) -> dict:
    q = pin.neutral(model)
    for angle, (idx_q, _) in zip(q_rad, indices, strict=True):
        q[idx_q] = angle
    gravity = gravity_in_base(roll_deg, pitch_deg)
    model.gravity.linear[:] = gravity
    model.gravity.angular[:] = 0.0
    tau = np.asarray(pin.computeGeneralizedGravity(model, model.createData(), q))
    arm_tau = tau[[idx_v for _, idx_v in indices]]
    if not np.all(np.isfinite(arm_tau)):
        raise ValueError("Non-finite torque returned by Pinocchio")
    outside = [
        name for name, (idx_q, _) in zip(JOINT_NAMES, indices, strict=True)
        if not model.lowerPositionLimit[idx_q] <= q[idx_q] <= model.upperPositionLimit[idx_q]
    ]
    return {
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "gravity_base_m_s2": gravity.tolist(),
        "tau_g_nm": dict(zip(JOINT_NAMES, arm_tau.tolist(), strict=True)),
        "outside_joint_limits": outside,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="Arm-only URDF; default: %(default)s")
    parser.add_argument("--q", nargs=6, type=finite_float, default=[0.0] * 6, metavar="ANGLE",
                        help="joint1..joint6, URDF joint coordinates; default: all zero")
    parser.add_argument("--unit", choices=("deg", "rad"), default="deg")
    parser.add_argument("--roll", type=finite_float, default=45.0, help="URDF mount roll in degrees")
    parser.add_argument("--pitch", type=finite_float, default=0.0, help="URDF mount pitch in degrees")
    parser.add_argument("--compare", action="store_true", help="Also calculate upright and opposite-roll cases")
    parser.add_argument("--json", action="store_true", help="Output machine-readable JSON")
    args = parser.parse_args()
    urdf = args.urdf.expanduser().resolve()
    try:
        model, indices = load_arm(urdf)
        q_rad = np.deg2rad(args.q) if args.unit == "deg" else np.asarray(args.q)
        mounts = [(args.roll, args.pitch)]
        if args.compare:
            mounts = list(dict.fromkeys([(0.0, 0.0), *mounts, (-args.roll, args.pitch)]))
        result = {
            "urdf": str(urdf),
            "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
            "pinocchio_version": pin.__version__,
            "q_rad": dict(zip(JOINT_NAMES, q_rad.tolist(), strict=True)),
            "other_joints": "Held at pin.neutral(model), including fingers",
            "torque_kind": "Raw model gravity compensation; no scaling or clipping; no motor commands",
            "cases": [calculate(model, indices, q_rad, r, p) for r, p in mounts],
        }
    except (OSError, ET.ParseError, ValueError, RuntimeError) as error:
        parser.error(str(error))
    if args.json:
        print(json.dumps(result, indent=2, allow_nan=False))
        return
    print(f"URDF: {urdf}")
    print("q [deg], joint1..joint6:", np.round(np.rad2deg(q_rad), 4).tolist())
    print("Raw model torques [N.m]; fingers at neutral; calculation only.")
    for case in result["cases"]:
        print(f"\nroll={case['roll_deg']:g} deg, pitch={case['pitch_deg']:g} deg")
        print("g_base [m/s^2]:", np.round(case["gravity_base_m_s2"], 6).tolist())
        for name, torque in case["tau_g_nm"].items():
            print(f"  {name}: {torque:+.6f}")
        if case["outside_joint_limits"]:
            print("Outside URDF joint limits:", ", ".join(case["outside_joint_limits"]))


if __name__ == "__main__":
    main()
