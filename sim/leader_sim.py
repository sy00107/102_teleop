#!/usr/bin/env python3
"""Real Arm 102 -> tilted B601-DM MuJoCo, driven through the REAL follower controller.

The LeRobot ``RebotB601Follower`` (gravity feed-forward, hybrid firmware-PD torque bounding,
setpoint rate limits, timeouts, return-to-zero on communication loss) runs unchanged. Only the
CAN bus and motors are replaced by a MuJoCo plant that behaves like the Damiao firmware:
position/velocity PD at the physics rate plus the feed-forward torque, a bus feedback delay,
and the motor CAN watchdog that disables torque when commands stop.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from contextlib import nullcontext
from types import SimpleNamespace
from pathlib import Path

import numpy as np
import pinocchio as pin
from calculate import finite_float, gravity_in_base

HERE = Path(__file__).resolve().parent
# MuJoCo model + meshes live outside the repo (large vendor assets); override with B601_DM_DIR.
MODEL_DIR = Path(os.environ.get("B601_DM_DIR", Path.home() / "b601_dm"))
ARM = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)
JOINTS = tuple(f"joint{i}" for i in range(1, 7))


class Leader:
    """Use the official reader, direction and range conventions, without zeroing."""

    def __init__(self, port: str, calibration: Path):
        from lerobot.teleoperators.rebot_102_leader.config_rebot_102_leader import (
            RebotArm102LeaderTeleopConfig,
        )
        from motorbridge_smart_servo import FashionStarServo

        self.config = RebotArm102LeaderTeleopConfig(port=port, id="leader")
        saved = json.loads(calibration.expanduser().read_text())
        for name, servo_id in self.config.joint_ids.items():
            if (
                name not in saved
                or saved[name]["id"] != servo_id
                or saved[name]["homing_offset"] != 0
            ):
                raise ValueError(
                    f"Calibration does not match the official leader mapping: {name}"
                )
        self.bus = FashionStarServo(port, baudrate=self.config.baudrate)
        try:
            for name, servo_id in self.config.joint_ids.items():
                if not self.bus.ping(servo_id):
                    raise RuntimeError(f"Leader servo not found: {name}, id={servo_id}")
            # Release leader servos for hand operation. Preserve their stored origin and counters.
            for servo_id in self.config.joint_ids.values():
                self.bus.unlock(servo_id)
            self.bus.set_loss_threshold(3)
            self.action = self.read()
        except BaseException:
            self.bus.close()
            raise
        self.lock = threading.Lock()
        self.last_time = time.monotonic()
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def read(self):
        cfg = self.config
        action = {}
        for name, servo_id in cfg.joint_ids.items():
            # Individual angle queries work at calibrated zero on this arm, where
            # sync_monitor marks near-zero samples unreliable despite fresh telemetry.
            sample = self.bus.read_angle(servo_id, multi_turn=True)
            if not math.isfinite(sample.raw_deg) or not math.isfinite(
                sample.filtered_deg
            ):
                raise RuntimeError(f"No fresh leader reading: {name}")
            angle_deg = sample.filtered_deg
            if not sample.reliable:
                # The vendor filter holds repeated near-zero values. Accept a zero
                # reading for this simulation only after an independent live ping.
                if abs(sample.raw_deg) <= 0.5 and self.bus.ping(servo_id):
                    angle_deg = sample.raw_deg
                else:
                    raise RuntimeError(f"No fresh leader reading: {name}")
            direction = cfg.joint_directions[name]
            low, high = cfg.joint_ranges[name]
            sign = 1 if direction >= 0 else -1
            center = (low + high) * sign / 2
            angle = angle_deg - round((angle_deg - center) / 360) * 360
            action[name] = float(np.clip(angle * direction, low, high))
        return action

    def _loop(self):
        while not self.stop.is_set():
            start = time.monotonic()
            try:
                action = self.read()
                with self.lock:
                    self.action, self.last_time, self.error = (
                        action,
                        time.monotonic(),
                        None,
                    )
            except Exception as error:
                with self.lock:
                    self.error = str(error)
            self.stop.wait(max(0, 0.02 - (time.monotonic() - start)))

    def latest(self):
        with self.lock:
            return self.action.copy(), time.monotonic() - self.last_time, self.error

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        if not self.thread.is_alive():
            self.bus.close()


class Simulation:
    def __init__(self, roll: float, pitch: float, gravity: bool):
        import mujoco

        self.mj = mujoco
        self.roll, self.pitch, self.gravity_on = roll, pitch, gravity
        source = MODEL_DIR / "mjcf/B601_DM_wrist_d435i_no_plate.xml"
        root = ET.parse(source).getroot()
        root.set("model", "Arm 102 -> B601-DM | mount roll +45 | Pinocchio gravity")
        for mesh in root.findall("./asset/mesh"):
            mesh.set("file", str((source.parent / mesh.attrib["file"]).resolve()))
        for body in root.findall(".//body"):
            body.set("gravcomp", "0")
        base = root.find("./worldbody/body[@name='base_link']")
        rotation = pin.rpy.rpyToMatrix(np.deg2rad([roll, pitch, 0.0]))
        quat = pin.Quaternion(rotation).coeffs()  # xyzw -> MuJoCo wxyz
        base.set("quat", " ".join(map(str, quat[[3, 0, 1, 2]])))
        base.set("pos", "0 0 0.85")
        world = root.find("worldbody")
        ET.SubElement(
            world,
            "geom",
            name="pedestal",
            type="box",
            pos="0 0 .38",
            size=".055 .055 .38",
            rgba=".20 .23 .28 1",
            contype="0",
            conaffinity="0",
        )
        ET.SubElement(
            world,
            "geom",
            name="mount",
            type="box",
            pos="0 0 .825",
            size=".09 .09 .008",
            quat=base.attrib["quat"],
            rgba=".35 .4 .48 1",
            contype="0",
            conaffinity="0",
        )
        for actuator in root.findall("./actuator/position"):
            if actuator.attrib["joint"] in JOINTS:
                actuator.tag = "motor"
                actuator.attrib.pop("kp")
                actuator.attrib.pop("kv")
                actuator.set("ctrlrange", actuator.attrib["forcerange"])
                actuator.set("ctrllimited", "true")
        self.model = mujoco.MjModel.from_xml_string(
            ET.tostring(root, encoding="unicode")
        )
        self.data = mujoco.MjData(self.model)
        self.model.opt.gravity[:] = [0, 0, -9.81]
        self.model.body_gravcomp[:] = 0
        # Use the same dynamics-model loader as real teleoperation.
        from lerobot.robots.rebot_b601_follower.gravity import load_gravity_model

        self.pin_model = load_gravity_model(
            HERE.parent / "assets/B601_DM_wrist_d435i_no_plate.urdf",
            missing_finger_mass_kg=0.03,
        )
        self.pin_model.gravity.linear[:] = gravity_in_base(roll, pitch)
        self.pin_model.gravity.angular[:] = 0
        self.pin_data = self.pin_model.createData()
        self.q = pin.neutral(self.pin_model)
        self.mapping = []
        for name in (*JOINTS, "finger_left", "finger_right"):
            mj_joint = self.model.joint(name)
            pj = self.pin_model.joints[self.pin_model.getJointId(name)]
            self.mapping.append(
                (int(mj_joint.qposadr[0]), int(mj_joint.dofadr[0]), pj.idx_q, pj.idx_v)
            )
        self.arm_q = np.array([x[0] for x in self.mapping[:6]])
        self.arm_v = np.array([x[1] for x in self.mapping[:6]])
        self.arm_pin_v = np.array([x[3] for x in self.mapping[:6]])
        self.actuators = np.array(
            [self.model.actuator(f"{j}_target").id for j in JOINTS]
        )
        self.kp = np.array([120, 120, 120, 18, 18, 18])
        self.kd = np.array([8, 8, 8, 2, 2, 2])
        self.limits = self.model.jnt_range[[self.model.joint(j).id for j in JOINTS]]
        self.target = np.zeros(6)
        self.finger_target = 0.0
        self.last_tau_g = np.zeros(6)
        self.holding = False

    def set_action(self, action, initialize=False):
        desired = np.deg2rad([action[name] for name in ARM])
        desired = np.clip(desired, self.limits[:, 0], self.limits[:, 1])
        self.target[:] = desired
        # Official leader: gripper output spans 0 (closed) to -270 degrees (open).
        self.finger_target = float(np.clip(-action["gripper"] / 270, 0, 1) * 0.05)
        self.holding = False
        if initialize:
            self.data.qpos[self.arm_q] = self.target
            self.data.qpos[self.mapping[6][0]] = self.finger_target
            self.data.qpos[self.mapping[7][0]] = -self.finger_target
            self.mj.mj_forward(self.model, self.data)

    def hold(self):
        if not self.holding:
            self.target[:] = self.data.qpos[self.arm_q]
            self.finger_target = float(self.data.qpos[self.mapping[6][0]])
            self.holding = True

    def gravity_torque(self):
        for iq, _, pq, _ in self.mapping:
            self.q[pq] = self.data.qpos[iq]
        return np.asarray(
            pin.computeGeneralizedGravity(self.pin_model, self.pin_data, self.q)
        )[self.arm_pin_v].copy()

    def step(self):
        self.last_tau_g = self.gravity_torque()
        tau = (
            self.kp * (self.target - self.data.qpos[self.arm_q])
            - self.kd * self.data.qvel[self.arm_v]
        )
        if self.gravity_on:
            tau += self.last_tau_g
        limits = self.model.actuator_ctrlrange[self.actuators]
        self.data.ctrl[self.actuators] = np.clip(tau, limits[:, 0], limits[:, 1])
        self.data.ctrl[self.model.actuator("finger_left_target").id] = (
            self.finger_target
        )
        self.mj.mj_step(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or self.data.warning.number.sum():
            raise RuntimeError(
                "MuJoCo reported invalid dynamics or a simulation warning"
            )



class SimPlant:
    """MuJoCo arm driven by firmware-like motors. Runs its own real-time physics thread."""

    def __init__(self, sim, delay_ms: float, watchdog_ms: float, vel_delay_ms: float = 0.0, vel_spike: float = 0.0):
        self.sim, self.delay, self.watchdog = sim, delay_ms / 1000, watchdog_ms / 1000
        self.vel_delay, self.vel_spike = vel_delay_ms / 1000, vel_spike
        self.rng = np.random.default_rng(0)
        self.lock = threading.RLock()
        self.viewer = None
        self.cmd = [(0.0, 0.0, 0.0, 0.0, 0.0)] * 6  # pos, vel, kp, kd, tau_ff (motor frame, rad)
        self.gripper_pos = 0.0
        self.gripper_open_deg = 270.0
        self.enabled = False
        self.watchdog_tripped = False
        self.last_cmd = time.monotonic()
        self.hist = deque(maxlen=4000)
        self.stop = threading.Event()
        self.thread = None
        self._record()

    def _record(self):
        s = self.sim
        self.hist.append(
            (time.monotonic(), s.data.qpos[s.arm_q].copy(), s.data.qvel[s.arm_v].copy())
        )

    def _at(self, delay):
        cutoff = time.monotonic() - delay
        entry = self.hist[0]
        for item in reversed(self.hist):
            if item[0] <= cutoff:
                return item
        return entry

    def state(self, index):
        with self.lock:
            position = self._at(self.delay)[1][index]
            # Velocity can arrive staler than position and occasionally spikes on real buses.
            velocity = self._at(self.delay + self.vel_delay)[2][index]
            if self.vel_spike and self.rng.random() < 0.01:
                velocity += self.vel_spike * self.rng.choice([-1.0, 1.0])
            return float(position), float(velocity)

    def start(self):
        self.last_cmd = time.monotonic()
        if self.thread is None:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def _run(self):
        s = self.sim
        dt = float(s.model.opt.timestep)
        batch = max(1, round(0.005 / dt))
        next_t = time.monotonic()
        while not self.stop.is_set():
            guard = self.viewer.lock() if self.viewer else nullcontext()
            with self.lock, guard:
                if self.enabled and time.monotonic() - self.last_cmd > self.watchdog:
                    self.enabled, self.watchdog_tripped = False, True
                for _ in range(batch):
                    ctrl = np.zeros(6)
                    if self.enabled:
                        q, v = s.data.qpos[s.arm_q], s.data.qvel[s.arm_v]
                        for i, (p, vd, kp, kd, tau) in enumerate(self.cmd):
                            ctrl[i] = kp * (p - q[i]) + kd * (vd - v[i]) + tau
                    limits = s.model.actuator_ctrlrange[s.actuators]
                    s.data.ctrl[s.actuators] = np.clip(ctrl, limits[:, 0], limits[:, 1])
                    s.data.ctrl[s.model.actuator("finger_left_target").id] = s.finger_target
                    s.mj.mj_step(s.model, s.data)
                    if not np.isfinite(s.data.qpos).all():
                        raise RuntimeError("MuJoCo produced non-finite state")
                    self._record()
            next_t += batch * dt
            self.stop.wait(max(0.0, next_t - time.monotonic()))
            if time.monotonic() - next_t > 0.1:  # fell behind; do not burst
                next_t = time.monotonic()

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)


class SimMotor:
    def __init__(self, plant: SimPlant, index: int | None):
        self.plant, self.index = plant, index

    def request_feedback(self):
        pass

    def get_state(self):
        if self.index is None:
            pos, vel = self.plant.gripper_pos, 0.0
        else:
            pos, vel = self.plant.state(self.index)
        return SimpleNamespace(
            pos=pos, vel=vel, torq=0.0, t_mos=30.0, t_rotor=30.0,
            status_code=1 if self.plant.enabled else 0,
        )

    def _set_gripper(self, pos):
        self.plant.gripper_pos = float(pos)
        # Follower gripper: 0 rad = closed, -270 deg = fully open (same as the leader).
        self.plant.sim.finger_target = float(
            np.clip(-math.degrees(pos) / self.plant.gripper_open_deg, 0, 1) * 0.05
        )

    def send_mit(self, pos, vel, kp, kd, tau):
        with self.plant.lock:
            self.plant.last_cmd = time.monotonic()
            if self.index is None:
                self._set_gripper(pos)
            else:
                self.plant.cmd[self.index] = (pos, vel, kp, kd, tau)

    def send_force_pos(self, pos, vlim=0.0, ratio=0.0):
        """DM gripper default mode (force-limited position)."""
        with self.plant.lock:
            self.plant.last_cmd = time.monotonic()
            if self.index is None:
                self._set_gripper(pos)

    def ensure_mode(self, mode):
        pass

    def set_can_timeout_ms(self, value):
        pass

    def damiao_get_param_u32(self, rid, timeout_ms=0):
        return 1

    def set_zero_position(self):
        pass

    def disable(self):
        pass

    def close(self):
        pass


class SimBus:
    def __init__(self, plant: SimPlant):
        self.plant = plant

    def poll_feedback_once(self):
        pass

    def enable_all(self):
        with self.plant.lock:
            self.plant.enabled = True
            self.plant.watchdog_tripped = False
        self.plant.start()

    def disable_all(self):
        with self.plant.lock:
            self.plant.enabled = False

    def close(self):
        self.plant.close()


def legacy_follower_class():
    """The pre-fix controller: the measured velocity shifts the torque-bound window (buggy)."""
    from lerobot.robots.rebot_b601_follower.motor_family import GRIPPER_MOTOR
    from lerobot.robots.rebot_b601_follower.rebot_b601_follower import RebotB601Follower

    class LegacyFollower(RebotB601Follower):
        def _send_joint(self, motor_name, position_deg, state=None, tau_ff=0.0):
            if motor_name == GRIPPER_MOTOR:
                return super()._send_joint(motor_name, position_deg, state, tau_ff)
            low, high = self._joint_limits[motor_name]
            position_deg = max(low, min(high, position_deg))
            position_rad = math.radians(self._public_to_motor_position(motor_name, position_deg))
            kp, kd = self._mit_kp[motor_name], self._mit_kd[motor_name]
            cap = self.config.mit_total_torque_limit[motor_name]
            tau_g = max(-cap, min(cap, tau_ff))
            lo = (-cap - tau_g + kd * state.vel) / kp
            hi = (cap - tau_g + kd * state.vel) / kp
            command = state.pos + max(lo, min(hi, position_rad - state.pos))
            self.motors[motor_name].send_mit(command, 0.0, kp, kd, tau_g)
            return position_deg

    return LegacyFollower


def make_follower(sim, plant, args):
    from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import (
        RebotB601FollowerRobotConfig,
    )
    from lerobot.robots.rebot_b601_follower.rebot_b601_follower import RebotB601Follower

    config = RebotB601FollowerRobotConfig(
        port="/dev/null",
        id="sim",
        gravity_comp=not args.no_gravity_comp,
        gravity_urdf_path=str(HERE.parent / "assets/B601_DM_wrist_d435i_no_plate.urdf"),
        gravity_mount_roll_deg=args.roll,
        gravity_mount_pitch_deg=args.pitch,
        gravity_missing_finger_mass_kg=0.03,
        max_relative_target=args.max_relative_target,
        max_target_velocity_deg_s=args.max_velocity,
        max_target_acceleration_deg_s2=args.max_acceleration,
        gripper_open_deg=args.gripper_open_deg,
        gripper_max_velocity_deg_s=args.gripper_velocity,
        gripper_max_acceleration_deg_s2=args.gripper_acceleration,
        comm_loss_action=args.comm_loss,
        comm_loss_home_delay_s=0.5,
        home_velocity_deg_s=args.home_velocity,
        require_supported_release=False,
    )
    plant.gripper_open_deg = args.gripper_open_deg
    follower_class = legacy_follower_class() if args.legacy_clamp else RebotB601Follower
    robot = follower_class(config)
    robot.bus = SimBus(plant)
    robot.motors = {
        name: SimMotor(plant, ARM.index(name) if name in ARM else None)
        for name in robot.motor_names
    }
    robot.configure()  # same startup path as the real arm: seed hold, enable, start motor loop
    return robot


def demo_action(t):
    q = [
        20 * np.sin(t),
        -65 + 20 * np.sin(t * 0.7),
        -60 + 20 * np.cos(t * 0.6),
        15 * np.sin(t),
        20 * np.sin(t * 0.8),
        20 * np.cos(t * 0.5),
    ]
    return dict(zip((*ARM, "gripper"), (*q, -135 * (1 + np.sin(t))), strict=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path.home()
        / ".cache/huggingface/lerobot/calibration/teleoperators/rebot_102_leader/leader.json",
    )
    parser.add_argument("--roll", type=finite_float, default=45.0)
    parser.add_argument("--pitch", type=finite_float, default=0.0)
    parser.add_argument("--no-gravity-comp", action="store_true")
    parser.add_argument(
        "--comm-loss",
        choices=("home", "hold"),
        default="home",
        help="Follower behaviour when the leader stream is lost (default: return to zero)",
    )
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Stage-2 speed preset: 100 deg/s, 800 deg/s^2, relative target 20 deg "
        "(explicit --max-* options still override)",
    )
    parser.add_argument(
        "--legacy-clamp",
        action="store_true",
        help="Reproduce the pre-fix controller (velocity-shifted torque window) for comparison",
    )
    parser.add_argument(
        "--vel-delay-ms", type=finite_float, default=0.0,
        help="Extra staleness of the velocity feedback only (real buses can be worse than position)",
    )
    parser.add_argument(
        "--vel-spike", type=finite_float, default=0.0,
        help="Occasional velocity feedback glitches of this size in rad/s (about 1 in 100 reads)",
    )
    parser.add_argument("--gripper-open-deg", type=finite_float, default=335.0)
    parser.add_argument("--gripper-velocity", type=finite_float, default=900.0)
    parser.add_argument("--gripper-acceleration", type=finite_float, default=20000.0)
    parser.add_argument("--home-velocity", type=finite_float, default=20.0)
    parser.add_argument("--max-velocity", type=finite_float, default=None)
    parser.add_argument("--max-acceleration", type=finite_float, default=None)
    parser.add_argument("--max-relative-target", type=finite_float, default=None)
    parser.add_argument(
        "--bus-delay-ms", type=finite_float, default=5.0, help="Simulated feedback latency"
    )
    parser.add_argument(
        "--start-at-leader",
        action="store_true",
        help="Start the plant at the leader pose instead of the set-zero pose",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Synthetic leader for checking the simulator",
    )
    parser.add_argument(
        "--loss-after",
        type=finite_float,
        default=0.0,
        help="Demo only: simulate leader loss after this many seconds",
    )
    parser.add_argument(
        "--loss-mode",
        choices=("error", "silent"),
        default="error",
        help="error: leader read fails (immediate); silent: stream just stops (timeout path)",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--duration",
        type=finite_float,
        default=0.0,
        help="Seconds; 0 means run until window closes",
    )
    parser.add_argument("--fps", type=finite_float, default=30.0)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--status", type=Path, default=HERE / "sim_status.json")
    args = parser.parse_args()
    preset = (100.0, 800.0, 20.0) if args.fast else (30.0, 120.0, 15.0)
    for name, value in zip(("max_velocity", "max_acceleration", "max_relative_target"), preset):
        if getattr(args, name) is None:
            setattr(args, name, value)
    if args.duration < 0 or (args.headless and args.duration == 0):
        parser.error("headless requires --duration > 0; duration cannot be negative")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.headless:
        os.environ.setdefault("MUJOCO_GL", "osmesa")
    sim = Simulation(args.roll, args.pitch, not args.no_gravity_comp)
    leader = robot = plant = None
    try:
        if not args.demo:
            leader = Leader(args.port, args.calibration)
        leader_action = demo_action(0) if leader is None else leader.latest()[0]
        if args.start_at_leader:
            sim.set_action(leader_action, initialize=True)
        else:
            sim.set_action({**dict.fromkeys(ARM, 0.0), "gripper": 0.0}, initialize=True)
        plant = SimPlant(
            sim, args.bus_delay_ms, watchdog_ms=250, vel_delay_ms=args.vel_delay_ms, vel_spike=args.vel_spike
        )
        robot = make_follower(sim, plant, args)
        viewer_context = nullcontext(None)
        if not args.headless:
            import mujoco.viewer

            viewer_context = mujoco.viewer.launch_passive(
                sim.model, sim.data, show_left_ui=False, show_right_ui=False
            )
        with viewer_context as viewer:
            plant.viewer = viewer
            if viewer:
                with viewer.lock():
                    viewer.opt.geomgroup[3] = 0
                    viewer.cam.lookat[:] = [0.2, 0, 1.0]
                    viewer.cam.distance = 1.65
                    viewer.cam.azimuth = 180
                    viewer.cam.elevation = -15
            print(
                f"{'DEMO' if leader is None else 'LIVE ARM 102'} | roll={args.roll:g} | "
                f"gravity FF={not args.no_gravity_comp} | comm-loss={args.comm_loss} | "
                f"bus delay={args.bus_delay_ms:g} ms | limits {args.max_velocity:g} deg/s, "
                f"{args.max_acceleration:g} deg/s2, rel {args.max_relative_target:g} | real follower controller",
                flush=True,
            )
            print(
                "Plant starts at the set-zero pose and follows the leader through the real "
                "rate limits. Turn the leader off / unplug it to test return-to-zero. "
                "Close the viewer or Ctrl+C to stop.",
                flush=True,
            )
            start = time.monotonic()
            last_status = -1.0
            lost = latched = wd_reported = False
            period = 1 / args.fps
            while viewer is None or viewer.is_running():
                frame_start = time.monotonic()
                elapsed = frame_start - start
                if args.duration and elapsed >= args.duration:
                    break
                age, error, action, silent = 0.0, None, None, False
                if leader:
                    action, age, error = leader.latest()
                elif args.loss_after and elapsed >= args.loss_after:
                    if args.loss_mode == "error":
                        error = "simulated leader read failure"
                    else:
                        silent = True  # stream just stops: the follower's own timeout acts
                else:
                    action = demo_action(elapsed)
                if (error or age > 0.25 or silent) and not lost:
                    lost = True
                    reason = error or ("command stream stopped" if silent else "leader stream stale")
                    print(f"LEADER LOST: {reason}", flush=True)
                    if not silent:
                        robot.on_comm_loss(reason)  # same call the teleop loop makes
                if not lost and action is not None:
                    leader_action = action
                    try:
                        robot.send_action({f"{n}.pos": float(v) for n, v in action.items()})
                    except RuntimeError as exc:
                        if not latched:
                            print(f"Follower stopped accepting commands: {exc}", flush=True)
                            latched = True
                if plant.watchdog_tripped and not wd_reported:
                    wd_reported = True
                    print("MOTOR CAN WATCHDOG TRIPPED: torque disabled", flush=True)
                if viewer:
                    viewer.sync()
                if elapsed - last_status >= 0.5:
                    with plant.lock:
                        actual = np.rad2deg(sim.data.qpos[sim.arm_q]).tolist()
                    targets = [robot._targets.get(n, 0.0) for n in ARM]
                    status = {
                        "running": True,
                        "source": "demo" if leader is None else args.port,
                        "elapsed_s": elapsed,
                        "roll_deg": args.roll,
                        "pitch_deg": args.pitch,
                        "gravity_ff": not args.no_gravity_comp,
                        "comm_loss_action": args.comm_loss,
                        "leader_lost": lost,
                        "homing": robot._homing,
                        "home_reached": robot._home_done,
                        "control_error": robot._control_error,
                        "motors_enabled": plant.enabled,
                        "leader_age_s": age,
                        "leader_deg": leader_action,
                        "target_deg": targets,
                        "actual_deg": actual,
                        "tau_ff_nm": [plant.cmd[i][4] for i in range(6)],
                        "max_tracking_error_deg": float(
                            np.max(np.abs(np.array(targets) - np.array(actual)))
                        ),
                    }
                    args.status.parent.mkdir(parents=True, exist_ok=True)
                    temp = args.status.with_suffix(".tmp")
                    temp.write_text(json.dumps(status, indent=2, allow_nan=False))
                    temp.replace(args.status)
                    last_status = elapsed
                time.sleep(max(0, period - (time.monotonic() - frame_start)))
            if args.snapshot:
                import mujoco
                from PIL import Image

                camera = mujoco.MjvCamera()
                camera.lookat[:] = [0.2, 0, 1.0]
                camera.distance, camera.azimuth, camera.elevation = 1.65, 180, -15
                options = mujoco.MjvOption()
                options.geomgroup[3] = 0
                with mujoco.Renderer(sim.model, width=1200, height=900) as renderer:
                    renderer.update_scene(sim.data, camera=camera, scene_option=options)
                    args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                    Image.fromarray(renderer.render()).save(args.snapshot)
                print(f"Snapshot: {args.snapshot}", flush=True)
    finally:
        if robot is not None and robot.is_connected:
            try:
                robot.disconnect()
            except Exception as exc:  # keep shutdown going
                print(f"follower disconnect: {exc}", flush=True)
        elif plant is not None:
            plant.close()
        if leader:
            leader.close()
        if args.status.exists():
            status = json.loads(args.status.read_text())
            status["running"] = False
            args.status.write_text(json.dumps(status, indent=2))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
