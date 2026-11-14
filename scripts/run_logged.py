#!/usr/bin/env python3
"""Capture child stdout/stderr without file I/O or terminal backpressure in the child.

stdin stays attached to the original terminal (supported-release prompts still
work). Ctrl+C reaches the child in the same process group; the relay keeps
running to capture its shutdown messages. No motor API is imported here.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from contextlib import suppress
from pathlib import Path
from typing import BinaryIO


class AsyncOutput:
    def __init__(self, folder: Path, capacity: int = 4096):
        self.folder = folder
        self.log_queue: queue.Queue = queue.Queue(capacity)
        self.console_queue: queue.Queue = queue.Queue(capacity)
        self.stop = threading.Event()
        self.dropped_log_chunks = 0
        self.dropped_console_chunks = 0
        self.error: str | None = None
        self.commands: list[dict] = []
        self.exit_code: int | None = None
        self.log_thread = threading.Thread(
            target=self._save, daemon=True, name="terminal-file-writer"
        )
        self.console_thread = threading.Thread(
            target=self._console, daemon=True, name="terminal-console-writer"
        )
        self.log_thread.start()
        self.console_thread.start()

    def emit(self, data: bytes) -> None:
        for destination, counter in (
            (self.log_queue, "dropped_log_chunks"),
            (self.console_queue, "dropped_console_chunks"),
        ):
            try:
                destination.put_nowait(data)
            except queue.Full:
                setattr(self, counter, getattr(self, counter) + 1)

    def _drain(self, destination: queue.Queue, stream: BinaryIO) -> None:
        while not self.stop.is_set() or not destination.empty():
            try:
                data = destination.get(timeout=0.05)
            except queue.Empty:
                continue
            stream.write(data)
            stream.flush()

    def _console(self) -> None:
        try:
            self._drain(self.console_queue, sys.stdout.buffer)
        except (OSError, ValueError):
            pass

    def _save(self) -> None:
        try:
            self.folder.mkdir(parents=True, exist_ok=True)
            self.write_manifest()
            with (self.folder / "terminal.log").open("ab", buffering=0) as stream:
                self._drain(self.log_queue, stream)
            self.write_manifest()
        except Exception as error:  # noqa: BLE001 - storage failure must not stop the child
            self.error = f"{type(error).__name__}: {error}"
            try:
                self.console_queue.put_nowait(
                    f"\nTERMINAL LOG STORAGE FAILED: {self.error}; motor policy unchanged.\n".encode()
                )
            except queue.Full:
                self.dropped_console_chunks += 1

    def write_manifest(self) -> None:
        data = {
            "schema": 1,
            "commands": self.commands,
            "exit_code": self.exit_code,
            "dropped_log_chunks": self.dropped_log_chunks,
            "dropped_console_chunks": self.dropped_console_chunks,
            "writer_error": self.error,
            "stream": "merged stdout/stderr; terminal escape sequences preserved",
        }
        data["finished"] = self.stop.is_set() and self.exit_code is not None
        data["repository_commit"] = os.environ.get("TELEOP_REPO_COMMIT")
        data["control_patch_sha256"] = os.environ.get("TELEOP_PATCH_SHA256")
        temporary = self.folder / "launcher.tmp"
        temporary.write_text(json.dumps(data, indent=2), encoding="utf-8")
        temporary.replace(self.folder / "launcher.json")

    def close(self) -> None:
        self.stop.set()
        self.log_thread.join(timeout=2.0)
        self.console_thread.join(timeout=2.0)
        # File writes remain in the file thread, including final metadata.


def relay(command: list[str], output: AsyncOutput) -> int:
    output.commands.append({"argv": command, "started_wall_ns": time.time_ns()})
    output.emit(("\n$ " + " ".join(command) + "\n").encode())
    environment = dict(os.environ, PYTHONUNBUFFERED="1")
    child = subprocess.Popen(
        command,
        stdin=None,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=environment,
    )

    def forward_stop(signum: int, _frame) -> None:
        with suppress(ProcessLookupError):
            child.send_signal(signum)

    previous_handlers = {
        sig: signal.getsignal(sig) for sig in (signal.SIGTERM, signal.SIGHUP)
    }
    for sig in previous_handlers:
        signal.signal(sig, forward_stop)
    try:
        while True:
            data = os.read(child.stdout.fileno(), 8192)
            if not data:
                break
            output.emit(data)
        result = child.wait()
    finally:
        child.stdout.close()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    output.commands[-1].update(exit_code=result, ended_wall_ns=time.time_ns())
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--preflight", type=Path)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("command required after --")
    # A handler, not SIG_IGN: exec'd children retain their normal SIGINT behavior.
    signal.signal(signal.SIGINT, lambda *_: None)
    output = AsyncOutput(args.session_dir)
    output.emit(f"Diagnostics session: {args.session_dir}\n".encode())
    code = 1
    try:
        if args.preflight is not None:
            code = relay(
                [sys.executable, str(args.preflight)]
                + (["--record"] if args.record else []),
                output,
            )
            if code:
                return code
        code = relay(command, output)
        return code if code >= 0 else 128 - code
    except Exception as error:  # noqa: BLE001 - preserve launcher errors in the terminal log
        code = 1
        output.emit(f"Launcher error: {type(error).__name__}: {error}\n".encode())
        return 1
    finally:
        output.exit_code = code
        output.close()


if __name__ == "__main__":
    raise SystemExit(main())
