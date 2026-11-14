#!/usr/bin/env bash
# 리더 보정. 리더를 영점 자세(그리퍼 완전히 닫음)로 놓고 ENTER.
# 영점 자세 사진: https://files.seeedstudio.com/wiki/robotics/projects/lerobot/102_zeroposition.jpg
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
lerobot-calibrate "${TELEOP_ARGS[@]}"
