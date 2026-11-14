#!/usr/bin/env bash
# LeRobot 설치. 한 번만 실행.
#  - LeRobot은 Python >= 3.12 가 필요하다(시스템 파이썬은 3.10) → conda 환경을 새로 만든다.
#  - clone한 폴더 안의 lerobot/에 고정 버전과 로컬 패치를 설치한다.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts/_conda.sh"

LEROBOT_COMMIT="1ef599dc5a7fd6aca6ad2ded8cf710e4d0e3f1be"

if ! conda env list | awk '{print $1}' | grep -qx "$CONDA_ENV"; then
  conda create -y -n "$CONDA_ENV" python=3.12
fi
conda activate "$CONDA_ENV"

if [[ ! -d "$LEROBOT_DIR/.git" ]]; then
  git clone https://github.com/huggingface/lerobot.git "$LEROBOT_DIR"
fi
if [[ "$(git -C "$LEROBOT_DIR" rev-parse HEAD)" != "$LEROBOT_COMMIT" ]]; then
  if [[ -n "$(git -C "$LEROBOT_DIR" status --porcelain)" ]]; then
    echo "LeRobot 버전이 다르고 로컬 수정이 있습니다. 덮어쓰지 않고 중단합니다." >&2
    exit 1
  fi
  git -C "$LEROBOT_DIR" fetch --quiet origin "$LEROBOT_COMMIT"
  git -C "$LEROBOT_DIR" checkout --quiet "$LEROBOT_COMMIT"
fi

# Restore the reviewed B601/Arm102 control and dataset fixes on a fresh checkout.
CONTROL_PATCH="$YOIDLE_ROOT/patches/rebot_b601_gravity_ff.patch"
if git -C "$LEROBOT_DIR" apply --reverse --check "$CONTROL_PATCH" 2>/dev/null; then
  echo "B601 제어 수정이 이미 적용되어 있습니다."
elif git -C "$LEROBOT_DIR" apply --check "$CONTROL_PATCH" 2>/dev/null; then
  git -C "$LEROBOT_DIR" apply "$CONTROL_PATCH"
elif git -C "$LEROBOT_DIR" apply --reverse --check "$YOIDLE_ROOT/patches/previous/rebot_b601_gravity_ff.ff565db.patch" 2>/dev/null; then
  python "$YOIDLE_ROOT/scripts/upgrade_control_patch.py" \
    --repo "$LEROBOT_DIR" \
    --previous "$YOIDLE_ROOT/patches/previous/rebot_b601_gravity_ff.ff565db.patch" \
    --current "$CONTROL_PATCH"
else
  echo "기존 수정과 B601 패치가 충돌합니다. 작업 파일을 덮어쓰지 않고 중단합니다." >&2
  exit 1
fi

cd "$LEROBOT_DIR"
# rebot: 팔로워(motorbridge)+리더(motorbridge-smart-servo) / intelrealsense: D435 / core_scripts: 데이터셋(torchcodec,av)·시리얼·pynput·rerun / training: accelerate 등 (문서의 .[rebot] 만으로는 record/train이 안 돈다)
python -m pip install -c "$YOIDLE_ROOT/constraints-control.txt" -e ".[rebot,intelrealsense,core_scripts,training]"
python -m pip install -c "$YOIDLE_ROOT/constraints-control.txt" pin mujoco pytest

echo
echo "설치 완료. 확인:"
lerobot-info || true
echo
echo "Linux에서 시리얼 장치가 점유되면: sudo apt remove brltty"
echo "권한 문제 시: sudo usermod -aG dialout \$USER (재로그인 필요)"
