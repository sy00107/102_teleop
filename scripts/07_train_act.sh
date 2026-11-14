#!/usr/bin/env bash
# ACT 학습. 로컬(4060)·서버(5090)·Jetson 어디서든 같은 데이터셋 폴더로 실행 가능.
# 사용: ./scripts/07_train_act.sh [추가 lerobot-train 인자...]
#   예) ./scripts/07_train_act.sh --batch_size=8 --steps=50000
# 로봇과 통신하지 않으므로 _common.sh 의 포트 검사를 쓰지 않는다.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_conda.sh"
conda activate "$CONDA_ENV"

RUN="act_${DATASET_NAME}"
lerobot-train \
  --dataset.repo_id="local/$DATASET_NAME" \
  --dataset.root="$YOIDLE_HOME/datasets/$DATASET_NAME" \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --output_dir="$YOIDLE_HOME/outputs/train/$RUN" \
  --job_name="$RUN" \
  "$@"
