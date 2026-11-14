#!/usr/bin/env python3
"""Offline evidence summary. Never imports motor control or opens a device."""

import argparse
import json
from collections import Counter
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    if not args.session.is_dir():
        parser.error("session directory does not exist")
    manifest = args.session / "manifest.json"
    if manifest.exists():
        data = json.loads(manifest.read_text())
        print(
            "Recorder:",
            {
                k: data.get(k)
                for k in (
                    "finished",
                    "writer_error",
                    "dropped_trace_records",
                    "dropped_incidents",
                    "buffer_evictions",
                )
            },
        )
    else:
        print(
            "No recorder manifest; diagnostics may not have started or storage failed."
        )
    events = args.session / "events.jsonl"
    codes = Counter()
    if events.exists():
        print("\nEvents (host monotonic timestamps, not confirmed motor RX times):")
        for line in events.read_text().splitlines():
            try:
                event = json.loads(line)
            except ValueError:
                print("  Incomplete event row; process may have ended during write.")
                continue
            if event.get("kind") == "fault":
                codes[event["data"].get("code")] += 1
            if event.get("kind") in ("fault", "state", "parameter_probe_error"):
                print(
                    event.get("seq"),
                    event.get("mono_ns"),
                    event.get("kind"),
                    event.get("data"),
                )
    print("\nFault counts:", dict(codes))
    incidents = sorted((args.session / "incidents").glob("*.jsonl"))
    print("Incident windows:", len(incidents))
    for path in incidents:
        with path.open() as stream:
            header = json.loads(stream.readline())
        print(
            " ",
            path.name,
            "code=",
            header.get("code"),
            "post_complete=",
            header.get("post_window_complete"),
            "pre_s=",
            round(header.get("available_pre_s", 0), 3),
            "post_drops=",
            header.get("post_evictions", 0),
        )
    print(
        "\nterminal.log: merged terminal output; trace_*.jsonl: feedback/leader/final submitted commands."
    )
    print(
        "Transport return is not execution acknowledgement. Torque estimates are not measured torque."
    )
    print(
        "This summary identifies recorded events; it does not establish a physical root cause or safety."
    )


if __name__ == "__main__":
    main()
