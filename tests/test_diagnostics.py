"""Diagnostics failure injection: no real Controller or motor port is opened."""

import errno
import importlib.util
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest
from lerobot.robots.rebot_b601_follower.diagnostics import DiagnosticsRecorder

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "diagnostic_fakes", ROOT / "tests/test_real_teleop_safety.py"
)
fakes = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fakes)
build = fakes.build


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert predicate()


def records(folder):
    return [
        json.loads(line)
        for p in sorted(folder.glob("trace_*.jsonl"))
        for line in p.read_text().splitlines()
    ]


def test_incident_keeps_pre_and_post_with_explicit_timing(tmp_path):
    log = DiagnosticsRecorder(tmp_path, pre_seconds=1, post_seconds=0.05)
    log.emit("before", {"value": 1})
    log.trigger("injected_fault", {"error": "test"})
    wait_for(lambda: list((tmp_path / "incidents").glob("*.partial.jsonl")))
    log.emit("after", {"value": 2})
    wait_for(
        lambda: (
            list((tmp_path / "incidents").glob("incident_*.jsonl"))
            and not list((tmp_path / "incidents").glob("*.partial.jsonl"))
        )
    )
    log.close()
    path = next((tmp_path / "incidents").glob("*.jsonl"))
    data = [json.loads(line) for line in path.read_text().splitlines()]
    assert data[0]["post_window_complete"] is True
    assert [r["kind"] for r in data[1:]] == ["before", "fault", "after"]
    assert [r["seq"] for r in data[1:]] == sorted(r["seq"] for r in data[1:])
    assert all(r["mono_ns"] and r["wall_ns"] for r in data[1:])


def test_early_shutdown_marks_partial_post_window(tmp_path):
    log = DiagnosticsRecorder(tmp_path, post_seconds=5)
    log.trigger("stop_now", {})
    log.close()
    data = json.loads(
        next((tmp_path / "incidents").glob("*.jsonl")).read_text().splitlines()[0]
    )
    assert data["post_window_complete"] is False


def test_blocked_storage_has_bounded_queue_and_never_waits_in_emit(
    tmp_path, monkeypatch
):
    entered, release = threading.Event(), threading.Event()
    original = DiagnosticsRecorder._write_manifest

    def blocked(self, **kwargs):
        if not entered.is_set():
            entered.set()
            release.wait(2)
        return original(self, **kwargs)

    monkeypatch.setattr(DiagnosticsRecorder, "_write_manifest", blocked)
    log = DiagnosticsRecorder(tmp_path, queue_records=4, buffer_records=8)
    assert entered.wait(1)
    started = time.monotonic()
    for i in range(100):
        log.emit("sample", {"n": i})
    elapsed = time.monotonic() - started
    assert elapsed < 0.25
    assert log.status()["dropped_trace_records"] >= 96
    assert len(log._history) == 8 and log._queue.qsize() == 4
    # Incident pre-history bypasses the saturated trace queue.
    log.trigger("queue_full_fault", {})
    release.set()
    log.close()
    incident = [
        json.loads(line)
        for line in next((tmp_path / "incidents").glob("*.jsonl"))
        .read_text()
        .splitlines()
    ]
    assert any(r.get("kind") == "fault" for r in incident)
    assert (
        json.loads((tmp_path / "manifest.json").read_text())["dropped_trace_records"]
        > 0
    )


def test_disk_full_does_not_disable_healthy_motor_worker(build, tmp_path, monkeypatch):
    original = Path.open

    def full_disk(path, *args, **kwargs):
        if path.suffix == ".csv":
            raise OSError(errno.ENOSPC, "simulated disk full")
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", full_disk)
    robot = build(telemetry_dir=str(tmp_path))
    robot.configure()
    wait_for(lambda: robot._diagnostics.error is not None)
    start = robot.calls.index(("bus", "enable"))
    robot.send_action(fakes.action(robot))
    time.sleep(0.05)
    assert robot._enabled and robot._worker.is_alive() and robot._control_error is None
    assert ("bus", "disable") not in robot.calls[start:]


def test_final_native_parameters_feedback_and_leader_are_recorded(build, tmp_path):
    robot = build(background_control=False, telemetry_dir=str(tmp_path))
    robot.configure()
    robot.record_leader_input(
        {"wrist_flex.pos": 80.0}, {"samples": {"wrist_flex": {"raw_deg": 80.0}}}
    )
    robot.send_action(fakes.action(robot))
    robot._send_joint("wrist_flex", 80.0, robot.motors["wrist_flex"].state, 0.0)
    sent = robot.motors["wrist_flex"].sent
    robot.disconnect()
    data = records(tmp_path)
    last_command = [r for r in data if r["kind"] == "motor_commands"][-1]["data"][
        "commands"
    ][0]
    assert list(last_command["parameters"].values()) == list(sent)
    assert last_command["parameters"]["position_rad"] != pytest.approx(
        80 * 3.141592653589793 / 180
    )
    assert last_command["transport_returned"] is True
    feedback = next(r for r in data if r["kind"] == "feedback")["data"]
    assert feedback["motor_rx_timestamp_available"] is False
    assert feedback["joints"]["wrist_flex"]["rx_mono_ns"] is None
    assert any(r["kind"] == "leader_input" for r in data)
    metadata = json.loads((tmp_path / "manifest.json").read_text())
    assert metadata["transport_success_is_execution_ack"] is False
    assert metadata["metadata"]["source_sha256"]


def test_partial_send_failure_records_successful_and_failed_joints(build, tmp_path):
    robot = build(background_control=False, telemetry_dir=str(tmp_path))
    robot.configure()

    def fail(*args):
        raise RuntimeError("injected elbow send failure")

    robot.motors["elbow_flex"].send_mit = fail
    with pytest.raises(RuntimeError, match="injected elbow"):
        robot.send_action(fakes.action(robot))
    robot.disconnect()
    data = records(tmp_path)
    batch = [r for r in data if r["kind"] == "motor_commands"][-1]["data"]["commands"]
    assert [r["joint"] for r in batch] == [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
    ]
    assert [r["transport_returned"] for r in batch] == [True, True, False]
    error = next(
        r for r in data if r["kind"] == "fault" and r["data"]["code"] == "command_error"
    )
    assert "injected elbow" in error["data"]["traceback"]


def test_rotation_and_nonfinite_inputs_are_recorded_as_valid_json(tmp_path):
    log = DiagnosticsRecorder(tmp_path, rotation_bytes=500, keep_files=2)
    for i in range(30):
        log.emit("input", {"n": i, "value": float("nan")})
    log.close()
    assert 1 <= len(list(tmp_path.glob("trace_*.jsonl"))) <= 2
    assert records(tmp_path)[-1]["data"]["value"] == {"nonfinite": "nan"}


def test_manual_marker_triggers_without_motor_api(tmp_path):
    log = DiagnosticsRecorder(tmp_path, post_seconds=0.02)
    wait_for(lambda: (tmp_path / "manifest.json").exists())
    (tmp_path / "mark_incident").write_text("operator observed unexpected motion")
    wait_for(lambda: list((tmp_path / "incidents").glob("*.jsonl")))
    log.close()
    assert any(
        r["kind"] == "fault" and r["data"]["code"] == "manual_marker"
        for r in records(tmp_path)
    )


def test_terminal_relay_preserves_output_and_nonzero_exit(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/run_logged.py"),
            "--session-dir",
            str(tmp_path),
            "--",
            sys.executable,
            "-c",
            'import sys; print("stdout evidence"); print("stderr evidence", file=sys.stderr); sys.exit(7)',
        ],
        capture_output=True,
        timeout=5,
        check=False,
    )
    assert result.returncode == 7
    text = (tmp_path / "terminal.log").read_text()
    assert "stdout evidence" in text and "stderr evidence" in text
    assert json.loads((tmp_path / "launcher.json").read_text())["exit_code"] == 7


def test_relay_stdin_remains_a_tty_and_partial_prompt_is_displayed(tmp_path):
    import pty
    import select

    master, slave = pty.openpty()
    child = subprocess.Popen(
        [
            sys.executable,
            str(ROOT / "scripts/run_logged.py"),
            "--session-dir",
            str(tmp_path),
            "--",
            sys.executable,
            "-c",
            'import sys; print("TTY="+str(sys.stdin.isatty())); input("SUPPORT THEN ENTER: "); print("DONE")',
        ],
        stdin=slave,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        received = b""
        deadline = time.monotonic() + 3
        while (
            b"TTY=True\nSUPPORT THEN ENTER:" not in received
            and time.monotonic() < deadline
        ):
            ready, _, _ = select.select([child.stdout], [], [], 0.1)
            if ready:
                received += os.read(child.stdout.fileno(), 4096)
        assert b"TTY=True" in received and b"SUPPORT THEN ENTER:" in received
        os.write(master, b"\n")
        tail, _ = child.communicate(timeout=3)
        assert child.returncode == 0 and b"DONE" in tail
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
        os.close(master)
        os.close(slave)


def test_ctrl_c_reaches_child_and_shutdown_output_is_preserved(tmp_path):
    command = 'import time\nprint("READY",flush=True)\ntry: time.sleep(20)\nexcept KeyboardInterrupt: print("SUPPORTED RELEASE CLEANUP",flush=True)'
    child = subprocess.Popen(
        [
            sys.executable,
            str(ROOT / "scripts/run_logged.py"),
            "--session-dir",
            str(tmp_path),
            "--",
            sys.executable,
            "-c",
            command,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    try:
        wait_for(
            lambda: (
                (tmp_path / "terminal.log").exists()
                and b"\nREADY\n" in (tmp_path / "terminal.log").read_bytes()
            )
        )
        os.killpg(child.pid, signal.SIGINT)
        child.communicate(timeout=4)
        assert child.returncode == 0
        assert b"SUPPORTED RELEASE CLEANUP" in (tmp_path / "terminal.log").read_bytes()
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()


def test_terminal_storage_failure_does_not_stop_child(tmp_path):
    not_a_folder = tmp_path / "blocked"
    not_a_folder.write_text("file blocks the log directory")
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/run_logged.py"),
            "--session-dir",
            str(not_a_folder),
            "--",
            sys.executable,
            "-c",
            'print("CHILD CONTINUED")',
        ],
        capture_output=True,
        timeout=5,
        check=False,
    )
    assert result.returncode == 0
    assert b"CHILD CONTINUED" in result.stdout
    assert b"TERMINAL LOG STORAGE FAILED" in result.stdout


def test_terminating_relay_forwards_signal_to_child(tmp_path):
    command = 'import signal,time,sys\nsignal.signal(signal.SIGTERM,lambda *_: (print("TERM CLEANUP",flush=True),sys.exit(0)))\nprint("READY",flush=True)\ntime.sleep(20)'
    child = subprocess.Popen(
        [
            sys.executable,
            str(ROOT / "scripts/run_logged.py"),
            "--session-dir",
            str(tmp_path),
            "--",
            sys.executable,
            "-c",
            command,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    try:
        wait_for(
            lambda: (
                (tmp_path / "terminal.log").exists()
                and b"\nREADY\n" in (tmp_path / "terminal.log").read_bytes()
            )
        )
        child.send_signal(signal.SIGTERM)
        child.communicate(timeout=4)
        assert child.returncode == 0
        assert b"TERM CLEANUP" in (tmp_path / "terminal.log").read_bytes()
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
