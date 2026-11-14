#!/usr/bin/env bash
# 텔레오퍼레이션 + 데이터셋 기록(LeRobotDataset). Hub에 올리지 않고 ~/yoidle/datasets 에 저장한다.
# 키: → 다음 에피소드로 넘김(저장) / ← 현재 에피소드 다시 / ESC 종료  (문서 기준. 안 되면 n / r / q)
# 이어서 기록하려면 맨 뒤에 --resume=true 를 붙인다.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
if [[ -z "$CAMERAS" && "${ALLOW_STATE_ONLY_RECORDING:-0}" != "1" ]]; then
  echo "CAMERAS가 비어 있어 영상 수집을 시작하지 않습니다. config.env에 wrist 카메라를 설정하세요." >&2
  echo "관절 정보만 기록하는 점검이면 ALLOW_STATE_ONLY_RECORDING=1로 설정하세요." >&2
  exit 1
fi

DATASET_ROOT="$YOIDLE_HOME/datasets/$DATASET_NAME"
mkdir -p "$YOIDLE_HOME/datasets"

run_with_diagnostics --preflight "$YOIDLE_ROOT/scripts/preflight.py" --record -- lerobot-record \
  "${ROBOT_ARGS[@]}" "${TELEOP_ARGS[@]}" \
  --dataset.repo_id="local/$DATASET_NAME" \
  --dataset.root="$DATASET_ROOT" \
  --dataset.single_task="$TASK" \
  --dataset.fps="$FPS" \
  --dataset.num_episodes="$NUM_EPISODES" \
  --dataset.episode_time_s="$EPISODE_TIME_S" \
  --dataset.reset_time_s="$RESET_TIME_S" \
  --dataset.push_to_hub=false \
  --display_data=false \
  "$@"
