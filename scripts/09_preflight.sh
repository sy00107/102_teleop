#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_conda.sh"
conda activate "$CONDA_ENV"
exec python "$YOIDLE_ROOT/scripts/preflight.py" "$@"
