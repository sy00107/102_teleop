#!/usr/bin/env bash
# 포트 찾기. 안내에 따라 장치를 하나씩 뽑았다 꽂는다. 결과를 config.env 에 적는다.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
echo "== 시리얼 장치 =="
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "(없음)"
echo
echo "== CAN =="
ip -br link show type can 2>/dev/null || echo "(없음)"
echo
echo "리더(USB-UART)와 팔로워(DM 시리얼 브릿지)를 구분하려면 아래를 실행해 안내를 따르세요."
lerobot-find-port
echo
echo "카메라는 필요하면:  lerobot-find-cameras opencv   /   lerobot-find-cameras realsense"
