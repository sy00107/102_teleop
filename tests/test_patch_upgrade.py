"""An existing install must upgrade exactly or preserve local edits intact."""

import importlib.util
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "patch_upgrade", ROOT / "scripts/upgrade_control_patch.py"
)
upgrade_tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(upgrade_tool)


def git(repo, *args):
    return subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, check=True
    ).stdout


@pytest.fixture
def installed(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    git(repo, "init", "-b", "main")
    git(repo, "config", "user.name", "Local test")
    git(repo, "config", "user.email", "test@example.invalid")
    (repo / "control.py").write_text("base\n")
    git(repo, "add", ".")
    git(repo, "commit", "-m", "base")
    old_files = {"control.py": "old controller\n", "gravity.py": "old gravity\n"}
    new_files = {
        "control.py": "new diagnostics hooks\n",
        "gravity.py": "old gravity\n",
        "diagnostics.py": "new recorder\n",
    }
    for name, body in old_files.items():
        (repo / name).write_text(body)
    git(repo, "add", "-N", "gravity.py")
    old_patch = tmp_path / "old.patch"
    old_patch.write_bytes(git(repo, "diff", "HEAD", "--binary"))
    for name, body in new_files.items():
        (repo / name).write_text(body)
    git(repo, "add", "-N", "diagnostics.py")
    new_patch = tmp_path / "new.patch"
    new_patch.write_bytes(git(repo, "diff", "HEAD", "--binary"))
    for name, body in old_files.items():
        (repo / name).write_text(body)
    (repo / "diagnostics.py").unlink()
    return repo, old_patch, new_patch, new_files


def test_exact_old_install_can_be_upgraded(installed):
    repo, old, new, expected = installed
    upgrade_tool.upgrade(repo, old, new)
    assert {name: (repo / name).read_text() for name in expected} == expected


def test_user_edits_are_not_overwritten(installed):
    repo, old, new, _ = installed
    (repo / "control.py").write_text("local user tuning\n")
    before = {p.name: p.read_bytes() for p in repo.glob("*.py")}
    with pytest.raises(RuntimeError, match="Local edits"):
        upgrade_tool.upgrade(repo, old, new)
    assert {p.name: p.read_bytes() for p in repo.glob("*.py")} == before
