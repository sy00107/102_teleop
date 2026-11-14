#!/usr/bin/env bash
# 공통 설정 및 conda 로드. 환경 활성화/모터 연결은 호출자가 결정한다.
set -euo pipefail
YOIDLE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$YOIDLE_ROOT/config.env"

if ! command -v conda >/dev/null 2>&1; then
  for CONDA_CANDIDATE in "$HOME/miniconda3/bin/conda" "$HOME/anaconda3/bin/conda" "$HOME/miniforge3/bin/conda"; do
    if [[ -x "$CONDA_CANDIDATE" ]]; then
      export PATH="$(dirname "$CONDA_CANDIDATE"):$PATH"
      break
    fi
  done
fi
if ! command -v conda >/dev/null 2>&1; then
  echo "conda를 찾지 못했습니다. Miniconda/Miniforge 설치 후 conda 실행 경로를 PATH에 추가하세요." >&2
  exit 1
fi
source "$(conda info --base)/etc/profile.d/conda.sh"
