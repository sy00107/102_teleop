#!/usr/bin/env python3
"""Upgrade only an exact previous patch snapshot, preserving unrelated user files."""

from __future__ import annotations

import argparse
import os
import subprocess
import tempfile
from pathlib import Path


def git(
    repo: Path, *args: str, env: dict | None = None, data: bytes | None = None
) -> bytes:
    return subprocess.run(
        ["git", "-C", str(repo), *args],
        env=env,
        input=data,
        capture_output=True,
        check=True,
    ).stdout


def build_tree(repo: Path, patch: Path, index: Path) -> str:
    env = dict(os.environ, GIT_INDEX_FILE=str(index))
    git(repo, "read-tree", "HEAD", env=env)
    git(repo, "apply", "--cached", "--binary", str(patch), env=env)
    return git(repo, "write-tree", env=env).decode().strip()


def upgrade(repo: Path, previous: Path, current: Path) -> None:
    with tempfile.TemporaryDirectory(prefix="teleop-patch-upgrade-") as directory:
        before = build_tree(repo, previous, Path(directory) / "old.index")
        after = build_tree(repo, current, Path(directory) / "new.index")
        changed = set(
            git(repo, "diff", "--name-only", "HEAD", before).decode().splitlines()
        )
        changed.update(
            git(repo, "diff", "--name-only", "HEAD", after).decode().splitlines()
        )
        for filename in sorted(changed):
            tree = git(repo, "ls-tree", before, "--", filename).decode().strip()
            target = repo / filename
            if not tree:
                if target.exists() or target.is_symlink():
                    raise RuntimeError(
                        f"Unexpected local file; refusing to overwrite: {filename}"
                    )
                continue
            metadata, _ = tree.split("\t", 1)
            mode, kind, sha = metadata.split()
            if kind != "blob" or not target.is_file() or target.is_symlink():
                raise RuntimeError(f"Unexpected file type: {filename}")
            if target.read_bytes() != git(repo, "cat-file", "blob", sha):
                raise RuntimeError(
                    f"Local edits differ from the previous release: {filename}"
                )
            if bool(target.stat().st_mode & 0o111) != (mode == "100755"):
                raise RuntimeError(f"Local executable mode changed: {filename}")
        delta = git(repo, "diff", "--binary", before, after)
        git(repo, "apply", "--check", "--binary", "-", data=delta)
        git(repo, "apply", "--binary", "-", data=delta)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--previous", type=Path, required=True)
    parser.add_argument("--current", type=Path, required=True)
    args = parser.parse_args()
    try:
        upgrade(args.repo.resolve(), args.previous.resolve(), args.current.resolve())
    except (RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(
            1, f"Patch upgrade stopped without overwriting local edits: {error}\n"
        )
    print(
        "Previous control patch upgraded; existing hardware calibration was not accessed."
    )


if __name__ == "__main__":
    main()
