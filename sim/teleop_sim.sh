#!/usr/bin/env bash
set -euo pipefail
GRAVITY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${B601_GRAVITY_PYTHON:-}" ]]; then
  GRAVITY_PYTHON="$B601_GRAVITY_PYTHON"
else
  source "$GRAVITY_DIR/../scripts/_conda.sh"
  conda activate "$CONDA_ENV"
  GRAVITY_PYTHON="$(command -v python)"
fi
exec "$GRAVITY_PYTHON" "$GRAVITY_DIR/leader_sim.py" "$@"
