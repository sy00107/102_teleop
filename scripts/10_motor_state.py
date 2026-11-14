#!/usr/bin/env python3
"""팔로워 모터 상태 읽기 전용 모니터 (토크 인가/enable/모드 변경/영점 설정을 하지 않는다).

관절 각도·속도·토크·온도·상태코드를 표로 출력한다. 손으로 팔을 움직이며 영점(0°) 일치,
방향 부호, 온도를 확인하는 용도. 중력 모델 토크(기울기 반영)를 참고값으로 같이 보여준다.

  ./scripts/10_motor_state.sh            # 5 Hz 로 계속 출력 (Ctrl+C 종료)
  ./scripts/10_motor_state.sh --once     # 한 번만
  ./scripts/10_motor_state.sh --json     # JSON 한 줄씩 (로그 저장용)

주의: 다른 프로세스(텔레옵/ROS 드라이버)가 같은 포트를 쓰고 있으면 실행하지 말 것.
종료 시 disable 명령을 보내지 않는다 (다른 곳에서 켜 둔 팔이 떨어지지 않도록).
"""

import argparse
import json
import math
import os
import sys
import time

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
STATUS = {0: "disabled", 1: "enabled"}


def read_rows(motors, bus, joint_directions, gravity=None):
    """Request one feedback frame per motor and return a list of dict rows (public frame, deg)."""
    for motor in motors.values():
        motor.request_feedback()
    bus.poll_feedback_once()
    states = {name: motor.get_state() for name, motor in motors.items()}
    tau_g = {}
    if gravity is not None and all(states[n] is not None for n in gravity.joint_names):
        tau_g = gravity.compute({n: states[n].pos for n in gravity.joint_names})
    rows = []
    for number, (name, state) in enumerate(states.items(), start=1):  # 1 = base ... 7 = gripper
        if state is None:
            rows.append({"id": number, "joint": name, "error": "no feedback"})
            continue
        sign = joint_directions[name]
        rows.append(
            {
                "id": number,
                "joint": name,
                "pos_deg": math.degrees(state.pos) / sign,
                "vel_deg_s": math.degrees(state.vel) / sign,
                "torque_nm": state.torq,
                "t_mos_c": state.t_mos,
                "t_rotor_c": state.t_rotor,
                "status": state.status_code,
                "gravity_model_nm": tau_g.get(name),
            }
        )
    return rows


def format_table(rows, warn_c=60.0):
    head = f"{'ID':<4}{'pos[deg]':>10}{'vel[d/s]':>10}{'tau[Nm]':>9}{'MOS[C]':>8}{'rotor[C]':>9}  {'status':<9}{'g-model[Nm]':>12}"
    lines = [head, "-" * len(head)]
    for r in rows:
        if "error" in r:
            lines.append(f"{r['id']:<4}{r['error']}")
            continue
        g = "" if r["gravity_model_nm"] is None else f"{r['gravity_model_nm']:+.2f}"
        hot = "  <-- HOT" if max(r["t_mos_c"], r["t_rotor_c"]) >= warn_c else ""
        lines.append(
            f"{r['id']:<4}{r['pos_deg']:>10.2f}{r['vel_deg_s']:>10.1f}{r['torque_nm']:>9.2f}"
            f"{r['t_mos_c']:>8.0f}{r['t_rotor_c']:>9.0f}  {STATUS.get(r['status'], r['status'])!s:<9}{g:>12}{hot}"
        )
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=os.environ.get("FOLLOWER_PORT", "/dev/ttyACM0"))
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--no-gravity", action="store_true", help="중력 모델 열을 계산하지 않음")
    ap.add_argument("--warn-c", type=float, default=60.0)
    args = ap.parse_args()

    from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import RebotB601FollowerConfig
    from lerobot.robots.rebot_b601_follower.gravity import GravityCompensator
    from lerobot.robots.rebot_b601_follower.motor_family import MOTOR_PROFILES
    from motorbridge import Controller

    cfg = RebotB601FollowerConfig(port=args.port)
    profile = MOTOR_PROFILES[cfg.motor_family]
    bus = Controller.from_dm_serial(serial_port=args.port, baud=cfg.dm_serial_baud)
    motors = {
        name: bus.add_damiao_motor(send_id, recv_id, profile.motor_models[name])
        for name, (send_id, recv_id) in cfg.motor_can_ids.items()
    }
    gravity = None
    urdf = os.environ.get("GRAVITY_URDF")
    if urdf and not args.no_gravity:
        gravity = GravityCompensator(
            urdf,
            ARM,
            mount_roll_deg=float(os.environ.get("GRAVITY_MOUNT_ROLL_DEG", 0)),
            mount_pitch_deg=float(os.environ.get("GRAVITY_MOUNT_PITCH_DEG", 0)),
            missing_finger_mass_kg=float(os.environ.get("GRAVITY_FINGER_MASS_KG", 0)),
        )
    drawn = 0
    if sys.stdout.isatty() and not args.once and not args.json:
        sys.stdout.write("\033[?25l")  # hide cursor while redrawing
    try:
        while True:
            rows = read_rows(motors, bus, profile.joint_directions, gravity)
            if args.json:
                print(json.dumps({"t": time.time(), "rows": rows}, allow_nan=False), flush=True)
            else:
                frame = f"follower {args.port}  (읽기 전용, Ctrl+C 종료)\n\n" + format_table(rows, args.warn_c)
                lines = frame.split("\n")
                if args.once or not sys.stdout.isatty():
                    print(frame, flush=True)
                else:
                    # Redraw in place: move the cursor back up and overwrite only the values.
                    if drawn:
                        sys.stdout.write(f"\033[{drawn}A")
                    sys.stdout.write("".join(f"{line}\033[K\n" for line in lines))
                    sys.stdout.flush()
                    drawn = len(lines)
            if args.once:
                break
            time.sleep(1 / args.hz)
    except KeyboardInterrupt:
        pass
    finally:
        if sys.stdout.isatty():
            sys.stdout.write("\033[?25h")  # restore cursor
        for motor in motors.values():
            try:
                motor.close()  # no disable: never drop an arm enabled elsewhere
            except Exception:
                pass
        bus.close()


if __name__ == "__main__":
    main()
