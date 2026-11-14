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
if [[ ! -x "$GRAVITY_PYTHON" ]]; then
  echo "Python not found: $GRAVITY_PYTHON; set B601_GRAVITY_PYTHON to a Python with numpy and pinocchio." >&2
  exit 1
fi
exec "$GRAVITY_PYTHON" "$GRAVITY_DIR/calculate.py" "$@"
