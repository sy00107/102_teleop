"""Real teleop regression tests; every motor/bus here is an in-memory fake."""

import __future__

import ast
import math
import os
import time
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import (
    RebotB601FollowerRobotConfig,
)
from lerobot.robots.rebot_b601_follower.gravity import GravityCompensator
from lerobot.robots.rebot_b601_follower.rebot_b601_follower import RebotB601Follower
from lerobot.teleoperators.rebot_102_leader import (
    RebotArm102Leader,
    RebotArm102LeaderTeleopConfig,
)

ARM = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
]
ROOT = Path(__file__).resolve().parents[1]
CAMERA_URDF = ROOT / "assets/B601_DM_wrist_d435i_no_plate.urdf"
SIM_MODEL = Path(os.environ.get("B601_DM_DIR", Path.home() / "b601_dm")) / "mjcf/B601_DM_wrist_d435i_no_plate.xml"
requires_sim_model = pytest.mark.skipif(
    not SIM_MODEL.is_file(), reason="Set B601_DM_DIR to the external MuJoCo model bundle"
)


class Motor:
    def __init__(self, name, calls):
        self.name, self.calls = name, calls
        self.state = SimpleNamespace(
            pos=0.0, vel=0.0, torq=0.0, t_mos=25.0, t_rotor=25.0, status_code=0
        )
        self.sent = None

    def request_feedback(self):
        pass

    def get_state(self):
        return self.state

    def send_mit(self, *values):
        self.sent = values
        self.calls.append((self.name, "mit", values))

    def send_force_pos(self, *values):
        self.calls.append((self.name, "force", values))

    def ensure_mode(self, mode):
        self.calls.append((self.name, "mode"))

    def set_can_timeout_ms(self, value):
        self.calls.append((self.name, "watchdog", value))

    def damiao_get_param_u32(self, rid, timeout_ms):
        self.calls.append((self.name, "probe"))
        return 1

    def disable(self):
        self.calls.append((self.name, "disable"))

    def close(self):
        pass


@pytest.fixture
def build():
    robots = []

    def create(**kwargs):
        settings = dict(
            port="/dev/null",
            id="test-only",
            gravity_comp=True,
            gravity_urdf_path=str(CAMERA_URDF),
            gravity_mount_roll_deg=45.0,
            gravity_missing_finger_mass_kg=0.03,
            max_relative_target=5.0,
            require_supported_release=False,
        )
        settings.update(kwargs)
        robot = RebotB601Follower(RebotB601FollowerRobotConfig(**settings))
        robot.calls = []
        robot.motors = {name: Motor(name, robot.calls) for name in robot.motor_names}

        def enable():
            robot.calls.append(("bus", "enable"))
            for motor in robot.motors.values():
                motor.state.status_code = 1

        robot.bus = SimpleNamespace(
            poll_feedback_once=lambda: None,
            close=lambda: None,
            disable_all=lambda: robot.calls.append(("bus", "disable")),
            enable_all=enable,
        )
        robots.append(robot)
        return robot

    yield create
    for robot in robots:
        if robot.bus is not None:
            robot.disconnect()


def action(robot, **values):
    return {f"{name}.pos": values.get(name, 0.0) for name in robot.motor_names}


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_bad_action_never_reaches_any_motor(build, bad):
    robot = build()
    with pytest.raises(ValueError, match="non-finite"):
        robot.send_action(action(robot, shoulder_pan=bad))
    assert not any(call[1] in ("mit", "force") for call in robot.calls)


@pytest.mark.parametrize(
    "field,value",
    [
        ("pos", float("nan")),
        ("vel", float("inf")),
        ("status_code", 8),
        ("t_mos", 120.0),
    ],
)
def test_bad_motor_state_prevents_transmission(build, field, value):
    robot = build()
    setattr(robot.motors["shoulder_pan"].state, field, value)
    with pytest.raises(RuntimeError):
        robot.send_action(action(robot))
    assert not any(call[1] == "mit" for call in robot.calls)


@requires_sim_model
def test_sim_and_real_share_exact_gravity_model():
    import sys

    sys.path.insert(0, str(ROOT / "sim"))
    from leader_sim import Simulation

    sim = Simulation(45, 0, True)
    q = np.array([0.1, -0.5, -0.8, 0.3, -0.2, 0.4])
    sim.set_action(
        {**dict(zip(ARM, np.rad2deg(q), strict=True)), "gripper": 0.0}, initialize=True
    )
    real = GravityCompensator(
        CAMERA_URDF, ARM, mount_roll_deg=45.0, missing_finger_mass_kg=0.03
    )
    np.testing.assert_allclose(
        list(real.compute(dict(zip(ARM, q, strict=True))).values()),
        sim.gravity_torque(),
        atol=1e-10,
    )


def test_seed_hold_and_watchdog_precede_enable(build):
    robot = build(background_control=True)
    robot.configure()
    enable = robot.calls.index(("bus", "enable"))
    for name in ARM:
        assert any(call[:2] == (name, "mit") for call in robot.calls[:enable])
        assert (name, "watchdog", 250) in robot.calls[:enable]


def test_startup_outside_model_range_never_enables(build):
    robot = build(background_control=True)
    robot.motors["shoulder_lift"].state.pos = math.radians(-195)
    with pytest.raises(RuntimeError, match="startup pose"):
        robot.configure()
    assert ("bus", "enable") not in robot.calls


def test_hybrid_total_torque_is_capped_with_firmware_pd_enabled(build):
    robot = build()
    robot.motors["shoulder_lift"].state.pos = -0.5
    robot.motors["shoulder_lift"].state.vel = -3.0
    robot.send_action(action(robot, shoulder_lift=-60.0))
    for name in ARM:
        state = robot.motors[name].state
        pos, vel, kp, kd, tau = robot.motors[name].sent
        assert (vel, kp, kd) == (0.0, robot._mit_kp[name], robot._mit_kd[name])
        cap = robot.config.mit_total_torque_limit[name]
        assert abs(tau) <= cap
        # Position + gravity torque of this command stays inside the cap (damping is firmware-side).
        assert abs(kp * (pos - state.pos) + tau) <= cap + 1e-9


def test_hybrid_position_command_is_pulled_toward_measurement_when_saturated(build):
    robot = build(max_relative_target=None)
    state = robot.motors["wrist_flex"].state
    robot.send_action(action(robot))
    robot._send_joint("wrist_flex", 80.0, state, 0.0)  # huge error, no gravity
    pos, _, kp, _, _ = robot.motors["wrist_flex"].sent
    assert kp * (pos - state.pos) == pytest.approx(3.0)  # wrist cap


def test_target_velocity_and_acceleration_are_time_based(build, monkeypatch):
    robot = build(max_relative_target=None)
    clock = [100.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])
    previous = 0.0
    previous_velocity = 0.0
    for _ in range(20):
        clock[0] += 0.02
        before = robot._target_at
        sent = robot.send_action(action(robot, shoulder_pan=90.0))["shoulder_pan.pos"]
        dt = 0.01 if before is None else 0.02
        velocity = (sent - previous) / dt
        assert abs(velocity) <= 30.0 + 1e-7
        assert abs(velocity - previous_velocity) / dt <= 120.0 + 1e-5
        previous, previous_velocity = sent, velocity


def test_timeout_latches_measured_hold_and_continues_refresh(build):
    robot = build(background_control=True, command_timeout_s=0.04)
    robot.configure()
    robot.send_action(action(robot, shoulder_pan=20.0))
    deadline = time.monotonic() + 0.5
    while robot._control_error is None and time.monotonic() < deadline:
        time.sleep(0.005)
    assert "timeout" in robot._control_error
    count = len(robot.calls)
    time.sleep(0.03)
    assert len(robot.calls) > count  # gravity/hold is still being refreshed
    assert robot._targets["shoulder_pan"] == 0.0
    with pytest.raises(RuntimeError, match="reconnect"):
        robot.send_action(action(robot))


def test_native_probe_failure_disables_and_latches_fault(build):
    robot = build(background_control=True)
    robot.configure()

    def lost(*args, **kwargs):
        raise RuntimeError("lost parameter response")

    for motor in robot.motors.values():
        motor.damiao_get_param_u32 = lost
    robot._last_probe = 0.0
    deadline = time.monotonic() + 2.0
    while robot._control_error is None and time.monotonic() < deadline:
        time.sleep(0.005)
    assert "lost parameter" in robot._control_error
    assert wait_for(lambda: not robot._enabled, 1.0)  # error is latched before the disable lands


def test_normal_disconnect_waits_for_supported_release(build, monkeypatch):
    robot = build(background_control=True, require_supported_release=True)
    robot.configure()

    def supported(prompt):
        assert robot._enabled
        assert robot._worker.is_alive()
        assert robot._command_at is None
        return ""

    monkeypatch.setattr("builtins.input", supported)
    robot.disconnect()
    assert robot.bus is None


def test_calibration_connection_does_not_enable_or_rezero(build, monkeypatch):
    robot = build()
    # Normal connect refuses before constructing a hardware controller.
    with pytest.raises(RuntimeError, match="calibration file"):
        robot.connect.__wrapped__(robot)


@pytest.fixture
def leader():
    result = RebotArm102Leader(
        RebotArm102LeaderTeleopConfig(port="/dev/null", id="test-only")
    )
    result.bus = SimpleNamespace(
        read_angle=lambda *a, **k: SimpleNamespace(
            raw_deg=5.0, filtered_deg=5.0, reliable=True
        ),
        ping=lambda _: True,
        close=lambda: None,
    )
    yield result
    result.disconnect()


def test_leader_read_failure_is_not_replaced_with_old_action(leader, monkeypatch):
    leader._last_raw_positions = dict.fromkeys(leader.motor_names, -30.0)

    def lost():
        raise RuntimeError("lost leader")

    monkeypatch.setattr(leader, "_read_raw_positions", lost)
    with pytest.raises(RuntimeError, match="lost leader"):
        leader.get_action()


def test_zero_filter_requires_independent_live_ping(leader):
    leader.bus.read_angle = lambda *a, **k: SimpleNamespace(
        raw_deg=0.0, filtered_deg=5.0, reliable=False
    )
    assert leader.get_action()["shoulder_pan.pos"] == 0.0
    leader.bus.ping = lambda _: False
    with pytest.raises(RuntimeError, match="fresh"):
        leader.get_action()


def test_configure_preserves_leader_origin_and_turn_count(leader, monkeypatch):
    calls = []
    leader.bus.unlock = lambda i: calls.append(i)
    monkeypatch.setattr(time, "sleep", lambda _: None)
    leader.configure()  # Fake bus deliberately has no origin/reset methods.
    assert calls == list(range(7))


def test_record_loop_saves_transmitted_target(build):
    root = ROOT / "lerobot/src/lerobot/scripts/lerobot_record.py"
    node = next(
        n
        for n in ast.parse(root.read_text()).body
        if isinstance(n, ast.FunctionDef) and n.name == "record_loop"
    )
    node.decorator_list = []
    module = ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[]))
    frames = []
    events = {"exit_early": False}
    robot = build()

    class Teleop:
        def get_action(self):
            return action(robot, shoulder_pan=30.0)

    class Dataset:
        fps = 30
        features = {}

        def add_frame(self, frame):
            frames.append(frame)
            events["exit_early"] = True

    env = {
        "time": time,
        "Teleoperator": Teleop,
        "OBS_STR": "observation",
        "ACTION": "action",
        "build_dataset_frame": lambda features, values, prefix: {prefix: dict(values)},
        "episode_ended": lambda _: False,
    }
    exec(
        compile(module, str(root), "exec", flags=__future__.annotations.compiler_flag),
        env,
    )
    timer = SimpleNamespace(
        tick=lambda: None, section=lambda _: nullcontext(), wait=lambda: None
    )
    env["record_loop"](
        robot,
        events,
        30,
        lambda x: x[0],
        lambda x: x[0],
        lambda x: x,
        dataset=Dataset(),
        teleop=Teleop(),
        control_time_s=1.0,
        timer=timer,
    )
    saved = frames[0]["action"]["shoulder_pan.pos"]
    assert saved == pytest.approx(0.012)  # 120 deg/s^2 * 0.01 s * 0.01 s
    assert (
        robot._targets["shoulder_pan"] == 0.0
    )  # phase-end finally holds measured pose
    assert saved != 30.0
    assert any(call[:2] == ("shoulder_pan", "mit") for call in robot.calls)


def test_dataset_model_identity_cannot_silently_change(build, tmp_path):
    from lerobot.robots.rebot_b601_follower.metadata import write_control_metadata

    robot = build()
    write_control_metadata(robot, tmp_path)
    write_control_metadata(robot, tmp_path)
    robot.config.gravity_tau_scale = 0.5
    with pytest.raises(ValueError, match="new dataset"):
        write_control_metadata(robot, tmp_path)


def test_passive_connection_rejects_motion(build):
    robot = build()
    robot._passive_connection = True
    with pytest.raises(RuntimeError, match="Passive"):
        robot.send_action(action(robot))
    assert not robot.calls


def test_passive_connect_never_switches_mode_enables_or_resets_zero(build, monkeypatch):
    import lerobot.robots.rebot_b601_follower.rebot_b601_follower as module

    robot = build()
    bus = robot.bus
    robot.bus = None
    bus.add_damiao_motor = lambda send_id, *args: robot.motors[
        robot.motor_names[send_id - 1]
    ]
    monkeypatch.setattr(
        module,
        "MotorBridgeController",
        SimpleNamespace(from_dm_serial=lambda **kw: bus),
    )
    robot.connect(calibrate=False)
    assert not robot.calls
    assert robot.is_connected
    with pytest.raises(RuntimeError, match="Passive"):
        robot.send_action(action(robot))


def test_partial_camera_failure_cleans_up_bus_before_enable(build, monkeypatch):
    import lerobot.robots.rebot_b601_follower.rebot_b601_follower as module

    robot = build(allow_uncalibrated=True, require_supported_release=False)
    bus = robot.bus
    robot.bus = None
    bus.add_damiao_motor = lambda send_id, *args: robot.motors[
        robot.motor_names[send_id - 1]
    ]
    monkeypatch.setattr(
        module,
        "MotorBridgeController",
        SimpleNamespace(from_dm_serial=lambda **kw: bus),
    )

    def camera_error():
        raise RuntimeError("camera unplugged")

    robot.cameras = {"wrist": SimpleNamespace(connect=camera_error, is_connected=False)}
    with pytest.raises(RuntimeError, match="camera unplugged"):
        robot.connect()
    assert robot.bus is None
    assert ("bus", "enable") not in robot.calls


def test_unreliable_nonzero_leader_angle_is_rejected(leader):
    leader.bus.read_angle = lambda *a, **k: SimpleNamespace(
        raw_deg=20.0, filtered_deg=10.0, reliable=False
    )
    with pytest.raises(RuntimeError, match="fresh"):
        leader.get_action()


@pytest.mark.parametrize(
    "pose", [[0, 0, 0, 0, 0, 0], [0, -50, -70, 0, 0, 0], [30, -100, -40, 30, 20, -20]]
)
@requires_sim_model
def test_hybrid_path_holds_nominal_mujoco(build, pose):
    import sys

    sys.path.insert(0, str(ROOT / "sim"))
    from leader_sim import Simulation

    sim = Simulation(45, 0, True)
    sim.set_action(
        {**dict(zip(ARM, pose, strict=True)), "gripper": 0.0}, initialize=True
    )
    robot = build()
    targets = dict(zip(robot.motor_names, [*pose, 0.0], strict=True))
    sim.data.qpos[sim.arm_q] += 0.003
    sim.mj.mj_forward(sim.model, sim.data)
    for _ in range(500):  # 5 s, real controller refreshes at 100 Hz, physics at 500 Hz
        for i, name in enumerate(ARM):
            robot.motors[name].state.pos = float(sim.data.qpos[sim.arm_q[i]])
            robot.motors[name].state.vel = float(sim.data.qvel[sim.arm_v[i]])
        robot._transmit(targets, {n: m.state for n, m in robot.motors.items()})
        for _ in range(5):  # motor firmware PD at the physics rate, command held between updates
            q, v = sim.data.qpos[sim.arm_q], sim.data.qvel[sim.arm_v]
            sent = [robot.motors[name].sent for name in ARM]
            sim.data.ctrl[sim.actuators] = [
                m[2] * (m[0] - q[i]) + m[3] * (m[1] - v[i]) + m[4] for i, m in enumerate(sent)
            ]
            sim.mj.mj_step(sim.model, sim.data)
    error = np.rad2deg(sim.data.qpos[sim.arm_q]) - pose
    assert np.max(np.abs(error)) < 0.01
    assert not sim.data.warning.number.sum()


def wait_for(condition, timeout=1.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.005)
    return condition()


FAST_HOME = dict(home_velocity_deg_s=400.0, home_acceleration_deg_s2=20000.0)


def test_go_home_returns_arm_to_zero_and_keeps_gripper(build):
    robot = build(background_control=True, comm_loss_action="home", **FAST_HOME)
    robot.configure()
    robot._targets.update(shoulder_pan=30.0, shoulder_lift=-60.0, elbow_flex=-40.0, gripper=-90.0)
    assert robot.on_comm_loss("test") is True
    assert wait_for(lambda: robot._home_done)
    for name in ARM:
        assert robot._targets[name] == pytest.approx(0.0, abs=0.05)
    assert robot._targets["gripper"] == -90.0
    robot.hold()  # must not cancel the return
    assert robot._homing and robot._targets["shoulder_pan"] == pytest.approx(0.0, abs=0.05)
    with pytest.raises(RuntimeError, match="reconnect"):
        robot.send_action(action(robot))


def test_home_motion_respects_velocity_limit(build):
    robot = build(
        background_control=True, comm_loss_action="home", home_velocity_deg_s=50.0,
        home_acceleration_deg_s2=400.0,
    )
    robot.configure()
    robot._targets["shoulder_pan"] = 20.0
    robot.go_home("test")
    previous, t_prev, worst = 20.0, time.monotonic(), 0.0
    while not robot._home_done and time.monotonic() - t_prev < 2.0:
        time.sleep(0.01)
        now, value = time.monotonic(), robot._targets["shoulder_pan"]
        worst = max(worst, abs(value - previous) / (now - t_prev))
        previous, t_prev = value, now
    assert robot._home_done
    assert worst <= 50.0 * 1.5  # sampling jitter margin


def test_on_comm_loss_is_noop_when_configured_to_hold(build):
    robot = build(background_control=True)
    robot.configure()
    robot._targets["shoulder_pan"] = 10.0
    assert robot.on_comm_loss("test") is False
    assert not robot._homing and robot._targets["shoulder_pan"] == 10.0


def test_silent_command_timeout_hold_then_home(build):
    robot = build(
        background_control=True, comm_loss_action="home", command_timeout_s=0.04,
        comm_loss_home_delay_s=0.1, **FAST_HOME,
    )
    robot.configure()
    robot.send_action(action(robot, shoulder_pan=3.0))
    assert wait_for(lambda: robot._control_error is not None)
    assert "timeout" in robot._control_error and not robot._homing  # held first
    assert wait_for(lambda: robot._homing, 0.5)
    assert wait_for(lambda: robot._home_done, 0.5)


def test_go_home_without_running_motor_loop_reports_failure(build):
    robot = build(background_control=True, comm_loss_action="home")
    assert robot.go_home("test") is False


def test_home_needs_background_control():
    with pytest.raises(ValueError, match="background_control"):
        RebotB601FollowerRobotConfig(
            port="/dev/null", control_mode="pos_vel", comm_loss_action="home"
        )


def test_overtemperature_returns_home_instead_of_disabling(build):
    robot = build(background_control=True, comm_loss_action="home", **FAST_HOME)
    robot.configure()
    robot._targets["shoulder_pan"] = 20.0
    robot.motors["elbow_flex"].state.t_mos = 80.0  # above 75, below hard 85
    assert wait_for(lambda: robot._homing and robot._home_done)
    assert robot._enabled and ("bus", "disable") not in robot.calls[robot.calls.index(("bus", "enable")):]
    assert robot._targets["shoulder_pan"] == pytest.approx(0.0, abs=0.05)


def test_hard_overtemperature_disables(build):
    robot = build(background_control=True, comm_loss_action="home")
    robot.configure()
    robot.motors["elbow_flex"].state.t_mos = 90.0
    assert wait_for(lambda: not robot._enabled)
    assert "hard temperature" in robot._control_error


def test_overtemperature_with_hold_policy_still_faults(build):
    robot = build(background_control=True)
    robot.configure()
    robot.motors["elbow_flex"].state.t_mos = 80.0
    assert wait_for(lambda: not robot._enabled)
    assert "temperature" in robot._control_error


def test_gripper_uses_own_speed_limit_and_ignores_relative_cap(build):
    robot = build(
        max_relative_target=5.0, gripper_max_velocity_deg_s=900.0, gripper_max_acceleration_deg_s2=20000.0
    )
    robot.send_action(action(robot))
    t0 = time.monotonic()
    sent = None
    while time.monotonic() - t0 < 0.4:
        sent = robot.send_action(action(robot, gripper=-270.0))["gripper.pos"]
        time.sleep(0.02)
    assert sent < -200.0  # a 5-degree relative cap or 300 deg/s limit would not get here
    assert abs(robot.send_action(action(robot, shoulder_pan=90.0))["shoulder_pan.pos"]) <= 5.0 + 1e-6


def test_gripper_open_deg_scales_motor_angle_only(build):
    robot = build(
        gripper_open_deg=286.4789, max_relative_target=None, gripper_max_velocity_deg_s=2000.0,
        gripper_max_acceleration_deg_s2=200000.0,
    )
    robot.send_action(action(robot))
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.5:
        sent = robot.send_action(action(robot, gripper=-270.0))["gripper.pos"]
        time.sleep(0.01)
    assert sent == pytest.approx(-270.0, abs=0.5)  # public range unchanged
    force = [c for c in robot.calls if c[0] == "gripper" and c[1] == "force"][-1][2]
    assert force[0] == pytest.approx(-5.0, abs=0.01)  # motor reaches the official -5.0 rad
    assert robot._motor_to_public_position("gripper", -286.4789) == pytest.approx(-270.0, abs=1e-3)
    assert robot._joint_limits["gripper"] == (-270.0, 0.0)


def test_command_error_returns_home_instead_of_going_limp(build):
    robot = build(background_control=True, comm_loss_action="home", **FAST_HOME)
    robot.configure()
    robot._targets["shoulder_pan"] = 15.0
    with pytest.raises(ValueError):
        robot.send_action({"shoulder_pan.pos": float("nan")})
    assert robot._homing and robot._enabled
    assert wait_for(lambda: robot._home_done)
    assert robot._targets["shoulder_pan"] == pytest.approx(0.0, abs=0.05)


def test_brief_feedback_dropout_is_tolerated_but_persistent_loss_disables(build):
    robot = build(background_control=True, comm_loss_action="home")
    robot.configure()
    motor = robot.motors["elbow_flex"]
    good = motor.state
    motor.get_state = lambda: None  # feedback missing
    time.sleep(0.03)  # ~3 cycles: below the tolerance
    assert robot._enabled and robot._control_error is None
    motor.get_state = lambda: good
    time.sleep(0.05)
    assert robot._enabled and robot._control_error is None  # recovered, no latch
    motor.get_state = lambda: None
    assert wait_for(lambda: not robot._enabled, 1.5)  # persistent loss: give up and disable


def test_telemetry_csv_is_written_while_running(build, tmp_path):
    robot = build(background_control=True, telemetry_dir=str(tmp_path))
    robot.configure()
    robot.send_action(action(robot, shoulder_pan=3.0))
    assert wait_for(lambda: any(tmp_path.glob("b601_*.csv")) and len(next(tmp_path.glob("b601_*.csv")).read_text().splitlines()) >= 3, 2.0)
    text = next(tmp_path.glob("b601_*.csv")).read_text().splitlines()
    assert text[0].startswith("t,shoulder_pan_target,shoulder_pan_pos,shoulder_pan_vel,shoulder_pan_tau")
    assert len(text[1].split(",")) == len(text[0].split(","))


@pytest.mark.parametrize("velocity", [0.0, 1.0, 2.0, 4.0, -4.0])
def test_velocity_spike_never_creates_a_command_lead(build, velocity):
    """Regression: a stale/noisy velocity sample used to force a large position lead."""
    robot = build(max_relative_target=None)
    state = robot.motors["shoulder_lift"].state
    state.pos, state.vel = -1.0, velocity
    robot._send_joint("shoulder_lift", math.degrees(-1.0), state, -2.0)  # target == position
    pos, _, kp, _, tau = robot.motors["shoulder_lift"].sent
    assert abs(pos - state.pos) < 1e-9
    assert tau == pytest.approx(-2.0)
