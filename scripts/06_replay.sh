#!/usr/bin/env bash
# 기록한 에피소드의 action을 팔로워에 재생. 데이터를 많이 모으기 전에 재현성을 먼저 확인한다.
# 사용: ./scripts/06_replay.sh [에피소드 번호, 기본 0]
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
EP="${1:-0}"
run_with_diagnostics -- lerobot-replay \
  "${ROBOT_ARGS[@]}" \
  --dataset.repo_id="local/$DATASET_NAME" \
  --dataset.root="$YOIDLE_HOME/datasets/$DATASET_NAME" \
  --dataset.episode="$EP"
