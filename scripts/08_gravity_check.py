#!/usr/bin/env python
"""팔로워 중력 보상(tau_ff) 실물 점검.

팔로워를 연결한 뒤 실행해서 두 가지를 확인한다.

  float : "떠 있기" 테스트. 위치 강성을 0(kp=0)으로 두고 중력 토크만 준다.
          모델이 맞으면 팔이 어느 자세에서든 가만히 떠 있고, 틀리면 한쪽으로 흘러간다.
          roll +45 / -45 와 스케일 후보를 번갈아 걸어서 흐름(표류)이 가장 작은 설정을 고른다.
          -> 마운트 기울기 부호와 스케일을 정한다.

  hold  : 텔레옵과 같은 MIT 위치 모드(kp 정상값)에서 자세를 잡고,
          중력 보상 켠/끈 상태의 정상상태 오차(처짐)를 비교한다.
          -> 실제 사용 조건에서 몇 도 정확해지는지 확인한다.

실물 없이 로직만 확인하려면 --dry-run (가상 팔; 진짜 기울기를 --true-roll 로 정해 둔다).

안전
  * 이 스크립트는 팔을 움직이는 토크를 낸다. 처음엔 팔을 손으로 받치고, 작업 공간을 비운다.
  * 자세를 잡을 때 토크를 끈다(disable_all). 놓으면 팔이 처지므로 반드시 받친 채 ENTER.
  * 예상보다 많이 움직이면(--abort-deg) 자동으로 현재 자세 유지로 전환한다.
  * 종료/Ctrl+C 시 현재 자세를 유지한 채 "받치세요" 안내 후 ENTER 에서 토크를 끈다.
  * 중력 토크는 관절별 상한(gravity.py 의 DEFAULT_TAU_LIMIT)으로 잘린다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from lerobot.robots.rebot_b601_follower.gravity import GravityCompensator
from lerobot.robots.rebot_b601_follower.motor_family import DM_PROFILE

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]

# float 테스트용 감쇠(kd). kp=0 이라 팔은 스프링 없이 감쇠 + 중력 토크만 받는다.
# 값이 클수록 흐름이 느려서 안전하지만 작은 오차를 보기 어렵다. 큰 관절 1.5, 손목 0.3.
FLOAT_KD = {"shoulder_pan": 1.5, "shoulder_lift": 1.5, "elbow_flex": 1.5,
            "wrist_flex": 0.3, "wrist_yaw": 0.3, "wrist_roll": 0.3}

# 출력 표용 짧은 이름 (앞 8글자로 자르면 shoulder_pan/lift 가 구분이 안 된다)
SHORT = {"shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elbow",
         "wrist_flex": "wflex", "wrist_yaw": "wyaw", "wrist_roll": "wroll"}

MAX_MOVE_DEG_S = 30.0   # 자세로 되돌아갈 때 목표를 이동시키는 최대 속도(도/초). 급발진 방지.


# --------------------------------------------------------------------------------------
# Rig: 실물과 가상 팔을 같은 인터페이스로 다루는 얇은 층.
#   read()  -> {관절: 현재 위치(rad)}     send() -> MIT 명령 하나     tick() -> 한 제어 주기 대기
# --------------------------------------------------------------------------------------
class RealRig:
    """실제 팔로워. LeRobot 의 RebotB601Follower 를 감싼다(중력 보상은 여기서 끄고 직접 계산해서 보낸다)."""

    def __init__(self, port: str, rate: float):
        from lerobot.robots.rebot_b601_follower.config_rebot_b601_follower import RebotB601FollowerRobotConfig
        from lerobot.robots.rebot_b601_follower.rebot_b601_follower import RebotB601Follower

        cfg = RebotB601FollowerRobotConfig(port=port, motor_family="dm", id="follower")
        self.robot = RebotB601Follower(cfg)
        # calibrate=False: 이 스크립트는 영점을 건드리지 않는다. configure() 가 MIT 모드로 켠다.
        self.robot.connect(calibrate=False)
        self.period = 1.0 / rate
        self._next = time.monotonic()

    def now(self) -> float:
        return time.monotonic()

    def read(self) -> dict[str, float]:
        states = self.robot._read_feedback()           # request_feedback + poll
        return {n: states[n].pos for n in ARM}

    def send(self, name: str, pos: float, kp: float, kd: float, tau: float) -> None:
        self.robot.motors[name].send_mit(pos, 0.0, kp, kd, tau)

    def tick(self) -> None:
        self._next += self.period
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self._next = time.monotonic()               # 밀렸으면 따라잡지 않고 리셋

    def torque_off(self) -> None:
        self.robot.bus.disable_all()

    def torque_on(self) -> None:
        self.robot.bus.enable_all()

    def close(self) -> None:
        self.robot.disconnect()                         # disable_torque_on_disconnect=True


class SimRig:
    """가상 팔. 진짜 중력은 true_roll 로 만든 모델, 제어기는 다른 roll 을 가정해 본다(모델 오차 재현).

    관절 동역학: I*qdd = tau_motor - g_true(q) - b*qd,  tau_motor = kp*(p-q) + kd*(0-qd) + tau_ff  (MIT 식).
    g_true 는 pinocchio 가 주는 '정지 유지 토크'이므로 tau_ff == g_true 이면 가속이 0 이다.
    """

    I, B = 0.1, 0.05
    SUB = 20  # 제어 주기당 적분 스텝

    def __init__(self, urdf: str, true_roll: float, rate: float, q0: dict[str, float]):
        self.truth = GravityCompensator(urdf, ARM, mount_roll_deg=true_roll, tau_limit={n: 1e3 for n in ARM})
        self.dt = self.period = 1.0 / rate
        self.q = dict(q0)
        self.qd = dict.fromkeys(ARM, 0.0)
        self.cmd = {n: (self.q[n], 0.0, 0.0, 0.0) for n in ARM}   # pos, kp, kd, tau
        self.enabled = True
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def read(self) -> dict[str, float]:
        return dict(self.q)

    def send(self, name, pos, kp, kd, tau):
        self.cmd[name] = (pos, kp, kd, tau)

    def tick(self) -> None:
        h = self.dt / self.SUB
        for _ in range(self.SUB):
            g = self.truth.compute(self.q)
            for n in ARM:
                pos, kp, kd, tau = self.cmd[n]
                total = (kp * (pos - self.q[n]) + kd * (0.0 - self.qd[n]) + tau) if self.enabled else 0.0
                acc = (total - g[n] - self.B * self.qd[n]) / self.I
                self.qd[n] += acc * h
                self.q[n] += self.qd[n] * h
        self.t += self.dt

    def torque_off(self):
        self.enabled = False

    def torque_on(self):
        self.enabled = True
        self.qd = dict.fromkeys(ARM, 0.0)

    def close(self):
        pass


# --------------------------------------------------------------------------------------
# 공통 동작
# --------------------------------------------------------------------------------------
def deg(x: float) -> float:
    return math.degrees(x)


def send_all(rig, target: dict[str, float], kp: dict, kd: dict, tau: dict) -> None:
    for n in ARM:
        rig.send(n, target[n], kp[n], kd[n], tau[n])


def hold_gains() -> tuple[dict, dict]:
    """텔레옵과 같은 MIT 위치 제어 이득(DM 기본 프로파일)."""
    return ({n: DM_PROFILE.mit_kp[n] for n in ARM}, {n: DM_PROFILE.mit_kd[n] for n in ARM})


def move_to(rig, comp: GravityCompensator, target: dict[str, float], seconds: float) -> None:
    """현재 위치에서 target 으로 목표를 부드럽게 옮기며 유지한다(관절당 최대 MAX_MOVE_DEG_S).

    kp 가 큰 채로 먼 목표를 바로 주면 큰 토크가 나가므로, 명령 목표를 조금씩 이동시킨다.
    """
    kp, kd = hold_gains()
    q = rig.read()
    cmd = dict(q)
    step = math.radians(MAX_MOVE_DEG_S) * rig.period     # 한 주기에 목표를 옮길 최대 각도(rad)
    for _ in range(int(seconds / rig.period)):
        q = rig.read()
        for n in ARM:
            delta = target[n] - cmd[n]
            cmd[n] += max(-step, min(step, delta))
        send_all(rig, cmd, kp, kd, comp.compute(q))
        rig.tick()


def hold_here(rig, comp: GravityCompensator, seconds: float = 0.3) -> None:
    """지금 자세에서 그대로 버틴다(이상 시 안전 상태)."""
    kp, kd = hold_gains()
    q = rig.read()
    period = rig.period
    for _ in range(max(1, int(seconds / period))):
        send_all(rig, q, kp, kd, comp.compute(rig.read()))
        rig.tick()


def capture_pose(rig, comp: GravityCompensator, label: str, auto: dict[str, float] | None) -> dict[str, float]:
    """토크를 끄고 사용자가 팔을 원하는 자세로 두게 한 뒤, 그 자세를 기록하고 바로 유지한다."""
    if auto is not None:                       # dry-run: 미리 정한 자세
        rig.q.update(auto)
        rig.qd = dict.fromkeys(ARM, 0.0)
        pose = rig.read()
    else:
        rig.torque_off()
        print(f"\n[{label}] 토크 OFF. 팔을 손으로 받치고 원하는 자세로 옮긴 뒤, 그 자세를 유지한 채 ENTER.")
        input("  (받치고 있는지 확인) ENTER...")
        pose = rig.read()
    # 토크를 켜자마자 현재 자세 + 중력으로 유지(켜는 순간 처지는 것 방지)
    kp, kd = hold_gains()
    rig.torque_on()
    send_all(rig, pose, kp, kd, comp.compute(pose))
    rig.tick()
    hold_here(rig, comp, 0.5)
    print("  자세 기록: " + ", ".join(f"{n}={deg(pose[n]):+.1f}" for n in ARM))
    return pose


# --------------------------------------------------------------------------------------
# float 테스트
# --------------------------------------------------------------------------------------
@dataclass
class FloatResult:
    roll: float
    scale: float
    aborted: bool
    drift_deg: dict[str, float]            # 구간 끝 - 시작 (도)
    slope_deg_s: dict[str, float]          # 선형 근사 기울기(도/초)
    resid_tau: dict[str, float]            # 추정 잔여 토크 N.m = kd * 각속도
    score: float = field(default=0.0)      # 잔여 토크 크기 합 (작을수록 좋음)


def run_float(rig, urdf: str, roll: float, scale: float, T: float, abort_deg: float) -> FloatResult:
    comp = GravityCompensator(urdf, ARM, mount_roll_deg=roll, tau_scale=scale)
    zeros = dict.fromkeys(ARM, 0.0)
    q0 = rig.read()
    ts, qs, aborted = [], [], False
    t_start = rig.now()
    period = rig.period
    for _ in range(int(T / period)):
        q = rig.read()
        ts.append(rig.now() - t_start)
        qs.append([q[n] for n in ARM])
        if max(abs(deg(q[n] - q0[n])) for n in ARM) > abort_deg:
            aborted = True
            break
        # kp=0: 목표 위치는 의미 없음(현재 위치를 넣는다). 감쇠 + 중력 토크만.
        send_all(rig, q, zeros, FLOAT_KD, comp.compute(q))
        rig.tick()
    qs_a = np.array(qs)
    ts_a = np.array(ts)
    drift = {n: deg(qs_a[-1, i] - qs_a[0, i]) for i, n in enumerate(ARM)}
    if len(ts_a) >= 3:
        slope = {n: deg(np.polyfit(ts_a, qs_a[:, i], 1)[0]) for i, n in enumerate(ARM)}
    else:
        slope = dict.fromkeys(ARM, 0.0)
    resid = {n: FLOAT_KD[n] * math.radians(slope[n]) for n in ARM}
    res = FloatResult(roll, scale, aborted, drift, slope, resid)
    res.score = float(sum(abs(v) for v in resid.values()))
    return res


def float_test(rig, args, poses) -> list[dict]:
    rolls = [float(x) for x in args.rolls.split(",")]
    scales = [float(x) for x in args.scales.split(",")]
    all_rows = []
    print("\n=== float 테스트: kp=0, 중력 토크만. 손을 떼고 흐르는 정도를 본다 ===")
    print("후보:", ", ".join(f"roll={r:+g}/scale={s:g}" for r in rolls for s in scales))
    for pi, pose in enumerate(poses):
        print(f"\n--- 자세 {pi + 1}/{len(poses)} ---")
        for r in rolls:
            for s in scales:
                if not args.dry_run:
                    input(f"  roll={r:+g} scale={s:g}: 손을 팔 가까이 두고(떨어지면 잡을 수 있게) ENTER 로 {args.duration:.0f}초 시작...")
                comp_back = GravityCompensator(args.urdf, ARM, mount_roll_deg=r, tau_scale=s)
                move_to(rig, comp_back, pose, 1.5)                 # 자세로 복귀(부드럽게)
                res = run_float(rig, args.urdf, r, s, args.duration, args.abort_deg)
                flag = "  ※ 중단(한계 초과)" if res.aborted else ""
                print(f"  roll={r:+g} scale={s:g}  score={res.score:.3f} N.m{flag}")
                print("     drift(deg): " + " ".join(f"{SHORT[n]}={res.drift_deg[n]:+6.1f}" for n in ARM))
                print("     resid(Nm) : " + " ".join(f"{SHORT[n]}={res.resid_tau[n]:+6.2f}" for n in ARM))
                hold_here(rig, comp_back, 0.3)
                all_rows.append({"pose": pi, "roll": r, "scale": s, "aborted": res.aborted,
                                 "score": res.score, "drift_deg": res.drift_deg,
                                 "resid_tau_nm": res.resid_tau})
    return all_rows


def recommend(rows: list[dict]) -> tuple[float, float] | None:
    """(roll, scale) 후보별로 자세에 대해 점수 평균을 내고, 중단된 후보는 큰 벌점을 준다."""
    agg: dict[tuple[float, float], list[float]] = {}
    for r in rows:
        agg.setdefault((r["roll"], r["scale"]), []).append(r["score"] + (100.0 if r["aborted"] else 0.0))
    if not agg:
        return None
    mean = {k: float(np.mean(v)) for k, v in agg.items()}
    print("\n후보별 평균 score (N.m, 작을수록 좋음):")
    for k, v in sorted(mean.items(), key=lambda kv: kv[1]):
        print(f"  roll={k[0]:+g} scale={k[1]:g} : {v:.3f}")
    return min(mean, key=mean.get)


# --------------------------------------------------------------------------------------
# hold 테스트
# --------------------------------------------------------------------------------------
def hold_once(rig, comp: GravityCompensator | None, pose, comp_safe: GravityCompensator,
              settle: float, measure: float, abort_deg: float) -> dict[str, float] | None:
    """pose 를 목표로 위치 제어(kp 정상값). comp=None 이면 중력 보상 끔. 정상상태 오차(도)를 반환(중단 시 None)."""
    kp, kd = hold_gains()
    period = rig.period
    zero_tau = dict.fromkeys(ARM, 0.0)
    errs = []
    n_total = int((settle + measure) / period)
    for i in range(n_total):
        q = rig.read()
        if max(abs(deg(q[n] - pose[n])) for n in ARM) > abort_deg:
            hold_here(rig, comp_safe, 0.3)
            return None
        tau = comp.compute(q) if comp is not None else zero_tau
        send_all(rig, pose, kp, kd, tau)
        if i * period >= settle:
            errs.append([deg(q[n] - pose[n]) for n in ARM])
        rig.tick()
    return {n: float(np.mean([e[j] for e in errs])) for j, n in enumerate(ARM)}


def hold_test(rig, args, poses) -> list[dict]:
    comp = GravityCompensator(args.urdf, ARM, mount_roll_deg=args.roll, tau_scale=args.scale)
    rows = []
    print(f"\n=== hold 테스트: 위치 제어, roll={args.roll:+g} scale={args.scale:g} ===")
    print("오차 = 실제 - 목표 (도). 중력 보상 ON 이 OFF 보다 0 에 가까워야 한다.")
    for pi, pose in enumerate(poses):
        move_to(rig, comp, pose, 1.5)
        on = hold_once(rig, comp, pose, comp, 1.0, 1.0, args.abort_deg)
        move_to(rig, comp, pose, 1.0)
        off = hold_once(rig, None, pose, comp, 1.0, 1.0, args.abort_deg)   # OFF: 처짐이 생김
        move_to(rig, comp, pose, 1.0)
        print(f"\n--- 자세 {pi + 1}/{len(poses)} ---")
        for label, e in (("ON ", on), ("OFF", off)):
            if e is None:
                print(f"  {label}: 중단(한계 초과)")
            else:
                print(f"  {label}: " + " ".join(f"{SHORT[n]}={e[n]:+6.2f}" for n in ARM)
                      + f"   max|err|={max(abs(v) for v in e.values()):.2f}")
        rows.append({"pose": pi, "on": on, "off": off})
    return rows


# --------------------------------------------------------------------------------------
def build_poses_sim() -> list[dict[str, float]]:
    d = math.radians
    return [dict.fromkeys(ARM, 0.0),
            {**dict.fromkeys(ARM, 0.0), "shoulder_lift": d(-40), "elbow_flex": d(-60)},
            {**dict.fromkeys(ARM, 0.0), "shoulder_lift": d(-20), "elbow_flex": d(-30), "wrist_flex": d(30)}]


def main(argv: list[str] | None = None) -> int:
    env = os.environ.get
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--test", choices=["float", "hold", "both"], default="both")
    ap.add_argument("--port", default=env("FOLLOWER_PORT", "/dev/ttyACM0"))
    ap.add_argument("--urdf", default=env("GRAVITY_URDF", str(Path(__file__).resolve().parents[1] / "assets/ReBot_Arm_DM.urdf")))
    ap.add_argument("--roll", type=float, default=float(env("GRAVITY_MOUNT_ROLL_DEG", "45")), help="hold 테스트에 쓸 roll")
    ap.add_argument("--scale", type=float, default=float(env("GRAVITY_TAU_SCALE", "0.5")), help="hold 테스트에 쓸 스케일")
    ap.add_argument("--rolls", default="45,-45", help="float 테스트 후보 roll 들 (쉼표)")
    ap.add_argument("--scales", default="0.5", help="float 테스트 후보 스케일들 (쉼표)")
    ap.add_argument("--poses", type=int, default=3, help="점검할 자세 수")
    ap.add_argument("--duration", type=float, default=3.0, help="float 한 번의 시간(초)")
    ap.add_argument("--rate", type=float, default=50.0, help="제어 주기(Hz)")
    ap.add_argument("--abort-deg", type=float, default=25.0, help="이 이상 움직이면 중단(도)")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "outputs"))
    ap.add_argument("--dry-run", action="store_true", help="가상 팔로 로직만 확인(하드웨어 불필요)")
    ap.add_argument("--true-roll", type=float, default=45.0, help="dry-run 가상 팔의 진짜 roll")
    args = ap.parse_args(argv)
    if not args.dry_run:
        ap.error("구형 직접 토크 테스트의 실물 경로는 비활성화했습니다. 새 보호 기능을 사용하는 04_teleop.sh로 점검하세요. --dry-run은 사용 가능합니다.")

    if args.abort_deg <= 0 or args.duration <= 0 or args.rate <= 0:
        ap.error("abort-deg, duration, rate 는 양수여야 한다.")
    if not Path(args.urdf).is_file():
        ap.error(f"URDF 없음: {args.urdf}")

    sim_poses = build_poses_sim()[: args.poses] if args.dry_run else None
    if args.dry_run:
        rig = SimRig(args.urdf, args.true_roll, args.rate, sim_poses[0])
        print(f"[dry-run] 가상 팔: 진짜 roll={args.true_roll:+g}")
    else:
        print("실물 팔로워에 연결한다. 팔을 받치고 작업 공간을 비웠는지 확인하세요.")
        input("ENTER 로 연결...")
        rig = RealRig(args.port, args.rate)
    safe = GravityCompensator(args.urdf, ARM, mount_roll_deg=args.roll, tau_scale=args.scale)

    out = {"args": vars(args), "float": [], "hold": [], "recommend": None}
    try:
        poses = [capture_pose(rig, safe, f"자세 {i + 1}", sim_poses[i] if sim_poses else None) for i in range(args.poses)]
        if args.test in ("float", "both"):
            out["float"] = float_test(rig, args, poses)
            rec = recommend(out["float"])
            out["recommend"] = rec
            if rec:
                print(f"\n>>> 권장: GRAVITY_MOUNT_ROLL_DEG=\"{rec[0]:g}\"  GRAVITY_TAU_SCALE=\"{rec[1]:g}\"  (config.env)")
                if args.test == "both":
                    args.roll, args.scale = rec          # hold 는 방금 고른 설정으로 이어서
        if args.test in ("hold", "both"):
            out["hold"] = hold_test(rig, args, poses)
    except KeyboardInterrupt:
        print("\n중단됨(Ctrl+C).")
    finally:
        try:
            hold_here(rig, safe, 0.3)
        except Exception as e:  # 통신 실패 등
            print(f"안전 유지 명령 실패: {e}")
        if not args.dry_run:
            input("\n팔을 손으로 받치세요. ENTER 를 누르면 토크를 끈다...")
        rig.close()
        Path(args.out).mkdir(parents=True, exist_ok=True)
        path = Path(args.out) / f"gravity_check_{time.strftime('%Y%m%d_%H%M%S')}.json"
        path.write_text(json.dumps(out, indent=2, default=lambda o: list(o) if isinstance(o, tuple) else str(o)))
        print(f"\n결과 저장: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
