#!/usr/bin/env bash
# 진단 파일에 수동 사고 표식을 남긴다. 로봇을 정지하거나 움직이지 않는다.
set -euo pipefail
if [[ $# -lt 1 || ! -d "$1" ]]; then
  echo "사용: $0 <시작 시 출력된 진단 폴더> [메모]" >&2
  exit 1
fi
printf '%s\n' "${2:-manual operator marker}" > "$1/mark_incident"
echo "수동 표식 저장 요청: $1 (모터 동작은 변경하지 않음)"
