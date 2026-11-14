#!/usr/bin/env bash
# 공통: 설정 로드 + conda 환경 활성화. 각 스크립트가 source 한다.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_conda.sh"
conda activate "$CONDA_ENV"
export PYTHONPATH="$LEROBOT_DIR/src${PYTHONPATH:+:$PYTHONPATH}"
export TELEOP_SESSION_ID="$(date +%Y%m%d_%H%M%S)_$$"
export TELEOP_DIAGNOSTICS_DIR="$YOIDLE_ROOT/outputs/diagnostics/$TELEOP_SESSION_ID"
export TELEOP_REPO_COMMIT="$(git -C "$YOIDLE_ROOT" rev-parse HEAD 2>/dev/null || true)"
export TELEOP_PATCH_SHA256="$(sha256sum "$YOIDLE_ROOT/patches/rebot_b601_gravity_ff.patch" | cut -d ' ' -f 1)"

run_with_diagnostics() {
  python "$YOIDLE_ROOT/scripts/run_logged.py" --session-dir "$TELEOP_DIAGNOSTICS_DIR" "$@"
}

if ! command -v lerobot-teleoperate >/dev/null; then
  echo "lerobot 이 설치되지 않았습니다. 먼저 ./setup.sh 를 실행하세요." >&2
  exit 1
fi

# 같은 포트를 ROS 드라이버(rebotarmcontroller)가 쓰고 있으면 충돌한다.
if pgrep -af "rebotarmcontroller|run_dm_driver|rebotarm_bringup" >/dev/null 2>&1; then
  echo "경고: ROS 팔 드라이버가 실행 중일 수 있습니다. 같은 포트를 동시에 열 수 없으니 먼저 종료하세요." >&2
  pgrep -af "rebotarmcontroller|run_dm_driver|rebotarm_bringup" >&2 || true
  echo "팔 드라이버를 종료한 뒤 다시 실행하세요." >&2
  exit 1
fi

# 팔로워 공통 인자
ROBOT_ARGS=(
  --robot.type=rebot_b601_follower
  --robot.motor_family="$FOLLOWER_FAMILY"
  --robot.port="$FOLLOWER_PORT"
  --robot.id="$FOLLOWER_ID"
  --robot.max_relative_target="$MAX_RELATIVE_TARGET"
)
if [[ "${GRAVITY_COMP:-0}" == "1" ]]; then
  ROBOT_ARGS+=(
    --robot.gravity_comp=true
    --robot.gravity_urdf_path="$GRAVITY_URDF"
    --robot.gravity_mount_roll_deg="$GRAVITY_MOUNT_ROLL_DEG"
    --robot.gravity_mount_pitch_deg="$GRAVITY_MOUNT_PITCH_DEG"
    --robot.gravity_tau_scale="$GRAVITY_TAU_SCALE"
    --robot.gravity_missing_finger_mass_kg="$GRAVITY_FINGER_MASS_KG"
  )
fi
ROBOT_ARGS+=(
  --robot.control_mode=mit
  --robot.background_control=true
  --robot.control_rate_hz="$MOTOR_CONTROL_HZ"
  --robot.command_timeout_s="$COMMAND_TIMEOUT_S"
  --robot.max_target_velocity_deg_s="$MAX_TARGET_VELOCITY_DEG_S"
  --robot.max_target_acceleration_deg_s2="$MAX_TARGET_ACCELERATION_DEG_S2"
  --robot.require_supported_release=true
  --robot.gripper_max_velocity_deg_s="${GRIPPER_MAX_VELOCITY_DEG_S:-300}"
  --robot.gripper_max_acceleration_deg_s2="${GRIPPER_MAX_ACCELERATION_DEG_S2:-3000}"
  --robot.gripper_open_deg="${GRIPPER_OPEN_DEG:-270}"
  --robot.telemetry_dir="$TELEOP_DIAGNOSTICS_DIR"
  --robot.comm_loss_action="${COMM_LOSS_ACTION:-hold}"
  --robot.comm_loss_home_delay_s="${COMM_LOSS_HOME_DELAY_S:-0.5}"
  --robot.home_velocity_deg_s="${HOME_VELOCITY_DEG_S:-20}"
  --robot.home_acceleration_deg_s2="${HOME_ACCELERATION_DEG_S2:-60}"
)
if [[ -n "${CAMERAS}" ]]; then
  ROBOT_ARGS+=(--robot.cameras="$CAMERAS")
fi

# 리더 공통 인자
TELEOP_ARGS=(
  --teleop.type=rebot_102_leader
  --teleop.port="$LEADER_PORT"
  --teleop.id="$LEADER_ID"
)
