"""Fake-motor test for the read-only state monitor."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location(
    "motor_state_tool", Path(__file__).resolve().parents[1] / "scripts" / "10_motor_state.py"
)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


class Motor:
    def __init__(self, state):
        self.state, self.requests = state, 0

    def request_feedback(self):
        self.requests += 1

    def get_state(self):
        return self.state


def test_rows_are_read_only_and_flag_hot_motor():
    hot = SimpleNamespace(pos=0.5, vel=0.1, torq=1.2, t_mos=61.0, t_rotor=40.0, status_code=1)
    motors = {"shoulder_pan": Motor(hot), "gripper": Motor(None)}
    bus = SimpleNamespace(poll_feedback_once=lambda: None)
    rows = tool.read_rows(motors, bus, {"shoulder_pan": 1.0, "gripper": 1.0})
    assert abs(rows[0]["pos_deg"] - 28.6479) < 1e-3 and rows[1]["error"] == "no feedback"
    assert motors["shoulder_pan"].requests == 1
    assert "HOT" in tool.format_table(rows)
