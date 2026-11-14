"""Hardware-free tests for the follower gravity feed-forward. Run in the yoidle env:

python -m pytest ~/yoidle/tests/test_gravity_ff.py -q
"""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import (
    RebotB601FollowerRobotConfig,
)
from lerobot.robots.rebot_b601_follower.gravity import (
    GravityCompensator,
    gravity_in_base_frame,
)
from lerobot.robots.rebot_b601_follower.rebot_b601_follower import RebotB601Follower

URDF = Path(__file__).resolve().parents[1] / "assets" / "ReBot_Arm_DM.urdf"
ARM = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
]
ZERO = dict.fromkeys(ARM, 0.0)


def comp(roll=0.0, **kw):
    return GravityCompensator(URDF, ARM, mount_roll_deg=roll, **kw)


def vec(d):
    return np.array([d[n] for n in ARM])


def test_gravity_vector_matches_urdf_mount():
    assert np.allclose(gravity_in_base_frame(0), [0, 0, -9.81])
    assert np.allclose(gravity_in_base_frame(45), [0, -6.937, -6.937], atol=1e-3)


def test_vertical_zero_pose_torque():
    # Same numbers as the ROS-side model (compute_generalized_gravity) at q=0.
    assert np.allclose(
        vec(comp(0).compute(ZERO)), [0, -1.284, -7.183, -1.98, 0, 0], atol=2e-3
    )


def test_tilt_45_zero_pose_torque():
    tau = vec(comp(45).compute(ZERO))
    assert np.allclose(tau, [-0.36, -0.908, -5.079, -1.40, -0.502, 0], atol=2e-3)
    # The opposite tilt flips only the sign of joint1 and joint5.
    tau_neg = vec(comp(-45).compute(ZERO))
    assert np.allclose(tau_neg[[0, 4]], -tau[[0, 4]], atol=1e-4)
    assert np.allclose(tau_neg[[1, 2, 3]], tau[[1, 2, 3]], atol=1e-4)


def test_scale_and_limit():
    assert np.allclose(
        vec(comp(45, tau_scale=0.5).compute(ZERO)),
        0.5 * vec(comp(45).compute(ZERO)),
        atol=1e-9,
    )
    capped = comp(0, tau_limit={"elbow_flex": 2.0}).compute(ZERO)
    assert capped["elbow_flex"] == -2.0


def test_nan_input_is_rejected():
    bad = {**ZERO, "elbow_flex": float("nan")}
    with pytest.raises(ValueError, match="Non-finite"):
        comp(45).compute(bad)


def test_config_validation():
    with pytest.raises(ValueError, match="urdf"):
        RebotB601FollowerRobotConfig(port="/dev/null", gravity_comp=True)
    with pytest.raises(ValueError, match="dm"):
        RebotB601FollowerRobotConfig(
            port="/dev/null",
            motor_family="rs",
            gravity_comp=True,
            gravity_urdf_path=str(URDF),
        )
    with pytest.raises(ValueError, match="mit"):
        RebotB601FollowerRobotConfig(
            port="/dev/null",
            control_mode="pos_vel",
            gravity_comp=True,
            gravity_urdf_path=str(URDF),
        )


class FakeMotor:
    def __init__(self, pos=0.0):
        self.state = SimpleNamespace(
            pos=pos, vel=0.0, torq=0.0, t_mos=25.0, t_rotor=25.0, status_code=0
        )
        self.sent = None

    def request_feedback(self):
        pass

    def get_state(self):
        return self.state

    def send_mit(self, pos, vel, kp, kd, tau):
        self.sent = (pos, vel, kp, kd, tau)

    def send_pos_vel(self, *a):  # pragma: no cover
        raise AssertionError("must stay in MIT mode")

    def send_force_pos(self, *a, **k):
        pass

    def disable(self):
        pass

    def close(self):
        pass


def make_robot(**cfg):
    robot = RebotB601Follower(RebotB601FollowerRobotConfig(port="/dev/null", **cfg))
    robot.bus = SimpleNamespace(poll_feedback_once=lambda: None, close=lambda: None)
    robot.motors = {name: FakeMotor() for name in robot.motor_names}
    return robot


def action_zero(robot):
    return {f"{n}.pos": 0.0 for n in robot.motor_names}


def test_default_sends_zero_tau():
    robot = make_robot()
    robot.send_action(action_zero(robot))
    assert all(robot.motors[n].sent[4] == 0.0 for n in ARM)


def test_gravity_tau_reaches_motors():
    robot = make_robot(
        gravity_comp=True,
        gravity_urdf_path=str(URDF),
        gravity_mount_roll_deg=45.0,
        max_relative_target=5.0,
    )
    robot.send_action(action_zero(robot))
    got = np.array([robot.motors[n].sent[4] for n in ARM])
    assert np.allclose(got, [-0.36, -0.908, -5.079, -1.40, -0.502, 0], atol=2e-3)
    # Gripper is not a gravity joint.
    assert (
        robot.motors["gripper"].sent is None or robot.motors["gripper"].sent[4] == 0.0
    )


def test_gravity_without_relative_limit_still_reads_feedback():
    robot = make_robot(
        gravity_comp=True, gravity_urdf_path=str(URDF), gravity_mount_roll_deg=45.0
    )
    robot.send_action(action_zero(robot))
    assert robot.motors["elbow_flex"].sent[4] == pytest.approx(-5.079, abs=2e-3)
