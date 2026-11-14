"""08_gravity_check.py 를 가상 팔(dry-run)로 돌려서 로직(부호 선택, ON/OFF 개선)을 검증한다."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "08_gravity_check.py"


def load():
    spec = importlib.util.spec_from_file_location("gravity_check", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["gravity_check"] = mod   # @dataclass 가 모듈을 이 이름으로 찾는다
    spec.loader.exec_module(mod)
    return mod


def run(tmp_path, true_roll):
    mod = load()
    mod.main(["--dry-run", "--true-roll", str(true_roll), "--scales", "1.0", "--out", str(tmp_path)])
    return json.loads(next(tmp_path.glob("gravity_check_*.json")).read_text())


@pytest.mark.parametrize("true_roll", [45.0, -45.0])
def test_picks_the_true_mount_sign(tmp_path, true_roll):
    out = run(tmp_path, true_roll)
    roll, scale = out["recommend"]
    assert roll == true_roll and scale == 1.0


def test_hold_on_beats_off(tmp_path):
    out = run(tmp_path, 45.0)
    for row in out["hold"]:
        on = max(abs(v) for v in row["on"].values())
        off = max(abs(v) for v in row["off"].values())
        assert on < 0.5 and off > 3.0, (on, off)
