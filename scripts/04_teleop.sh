#!/usr/bin/env bash
# 리더 → 팔로워 실시간 추종(기록 없음). 처음에는 작게 움직여 관절 방향부터 확인한다.
# 종료: Ctrl+C
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
python "$YOIDLE_ROOT/scripts/preflight.py"
lerobot-teleoperate "${ROBOT_ARGS[@]}" "${TELEOP_ARGS[@]}" --fps="$FPS" "$@"
