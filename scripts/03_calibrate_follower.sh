#!/usr/bin/env bash
# 팔로워 보정. 팔을 영점 자세(그리퍼 완전히 닫음)로 놓고 ENTER. 중력으로 처질 수 있으니 팔을 받쳐 둔다.
# 영점 자세 사진: https://files.seeedstudio.com/wiki/robotics/projects/lerobot/b601dm_zeroposition.jpg
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
lerobot-calibrate \
  --robot.type=rebot_b601_follower \
  --robot.motor_family="$FOLLOWER_FAMILY" \
  --robot.port="$FOLLOWER_PORT" \
  --robot.id="$FOLLOWER_ID"
