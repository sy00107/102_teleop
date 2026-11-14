#!/usr/bin/env bash
# 팔로워 중력 보상 점검. 팔로워를 연결하고 팔을 받친 뒤 실행.
#   ./scripts/08_gravity_check.sh                      # float + hold (기본)
#   ./scripts/08_gravity_check.sh --test float --scales 0.5,0.75,1.0
#   ./scripts/08_gravity_check.sh --dry-run            # 가상 팔(하드웨어 불필요)
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
exec python "$YOIDLE_ROOT/scripts/08_gravity_check.py" "$@"
