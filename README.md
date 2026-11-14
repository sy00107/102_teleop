# 102_teleop

**Seeed Star Arm 102 리더 → reBot B601-DM 팔로워, 장착 Roll +45° 중력보상 텔레옵.**
기존 `yoidle` 프로젝트의 제어 코드와 설정을 팀원 PC에 설치할 수 있도록 옮긴 저장소다.
실물 제어 로직, 관절 매핑, 중력 모델, 이득 및 속도 설정은 원본과 같다.
폴더 이름이나 사용자 홈 경로에 의존하던 설치·실행 경로와 안내를 정리했다.

## 팀원 PC에서 시작

기준 환경은 **Linux x86_64 + conda + Python 3.12**다. ROS는 필요하지 않다.
Git, Miniconda 또는 Miniforge가 필요하며, 영상 기록에는 시스템 FFmpeg도 필요하다.
Windows/macOS/Jetson에서의 설치와 실물 구동은 별도 검증하지 않았다.

```bash
git clone https://github.com/sy00107/102_teleop.git
cd 102_teleop
cp config.local.env.example config.local.env
./setup.sh
```

`setup.sh`는 conda 환경 `yoidle`을 만들고, LeRobot을 아래 커밋으로 고정해서
`lerobot/`에 받은 뒤 **이 저장소의 수정 패치**를 적용한다.
다른 프로젝트에서 `yoidle` 환경을 이미 쓴다면 설치 전 `config.local.env`에서
`export CONDA_ENV="102_teleop"`처럼 별도 이름을 지정한다.

```text
LeRobot: 1ef599dc5a7fd6aca6ad2ded8cf710e4d0e3f1be
Patch:   patches/rebot_b601_gravity_ff.patch
```

기존 LeRobot 수정과 충돌하면 덮어쓰지 않고 중단한다.
제어 SDK·Pinocchio·MuJoCo·NumPy 버전은 `constraints-control.txt`에 원본 PC 버전을 보존했다.
전체 학습/영상 의존성을 완전히 잠근 lockfile은 아니며, 새 OS에서의 설치 결과는 달라질 수 있다.

1. 팔로워를 고정하고 두 팔의 전원·USB를 연결한다. 같은 포트를 쓰는 ROS 드라이버는 종료한다.
2. `./scripts/01_find_ports.sh`로 확인한 포트를 **`config.local.env`**에 적는다.
   예시 장치명은 이 PC의 이름이므로 팀원 장치에서 다시 확인해야 한다.
3. 각 장치의 영점을 확인하고 아래 보정을 수행한다. **다른 개체의 보정 JSON을 복사하지 않는다.**
   기존 하드웨어 영점이 이미 설정돼 있다면 덮어쓰기 전에 URDF 영점과의 일치를 확인한다.

   ```bash
   ./scripts/02_calibrate_leader.sh
   ./scripts/03_calibrate_follower.sh
   ./scripts/09_preflight.sh
   ```

4. 팔을 받친 상태에서 `./scripts/10_motor_state.sh --once`로 각도·방향·온도를 확인한다.
   이 도구는 읽기 전용이며 enable·영점 변경·토크 해제 명령을 보내지 않는다.
5. 텔레옵을 시작하고 작은 관절 움직임부터 방향과 추종을 확인한다.

   ```bash
   ./scripts/04_teleop.sh
   ```

시리얼 권한이 없다면 `sudo usermod -aG dialout "$USER"` 후 다시 로그인한다.
`brltty`가 USB 시리얼을 점유한다면 점유 프로세스와 시스템 설정을 확인한다.

## 현재 제어 설정

| 항목 | 기본값 |
|---|---|
| 팔로워 | B601-DM, MIT 제어 |
| 중력보상 | ON, **Roll +45° / Pitch 0° / scale 1.0** |
| 모델 | `assets/B601_DM_wrist_d435i_no_plate.urdf` |
| 모델 부하 | D435i 72g 포함, 손가락 임시 질량 각 30g |
| 제어 | 모터 펌웨어 PD + 호스트 Pinocchio 중력 feed-forward |
| 모터 루프 요청 주기 | 100Hz |
| 리더 입력/텔레옵 | 30Hz |
| 팔 목표 속도·가속도 | 30°/s, 120°/s² |
| 현재각 대비 목표 제한 | 15° |
| 목표 갱신 타임아웃 | 0.25초 |
| 리더 통신 상실 | **0.5초 후 home(영점) 복귀**, 20°/s, 60°/s² |
| 그리퍼 | 개방 모터각 335°, 목표 속도 900°/s, 가속도 20000°/s² |

`config.env`가 공통 기본값이고 `config.local.env`가 PC별 재정의 파일이다.
장착각은 URDF rpy 기준이며 +45°일 때 베이스 중력 벡터는
`[0, -6.936718, -6.936718] m/s²`다. 실제 장착 방향·모터 영점·부호도 일치해야 한다.
팔/카메라/그리퍼 구성이나 실제 개방각이 다른 개체는 해당 설정을 먼저 확인한다.
브래킷·케이블·집은 물체의 추가 하중은 별도로 모델링하지 않았다.

호스트는 관절별 10/10/10/3/3/3 N·m 기준으로 **위치 비례항 + 중력항**을 제한하며,
펌웨어의 속도 감쇠항까지 포함한 실제 전체 출력 제한을 보장하는 것은 아니다.
100Hz도 요청값이며 실제 버스 주기·추종·발열은 해당 하드웨어에서 확인해야 한다.

**통신 상실 시 home은 충돌 검사 없는 관절공간 복귀**다. 복귀 경로를 확인하지 않았다면
`config.local.env`에 `export COMM_LOSS_ACTION="hold"`를 넣어 현재 자세 유지를 선택한다.
정상 종료는 Ctrl+C 후 안내에 따라 팔을 받치고 ENTER로 토크를 해제한다.
전원/버스 단절 시에는 자세 유지를 보장할 수 없다.

## 텔레옵 외 기능

| 파일 | 기능 |
|---|---|
| `scripts/05_record.sh` | 추종하며 관절 상태·실전송 목표·카메라 영상 기록 |
| `scripts/06_replay.sh 0` | 저장한 에피소드 0의 action 재생, 같은 팔로워 제어 설정 적용 |
| `scripts/07_train_act.sh` | 로컬 데이터셋으로 ACT 모방학습 |
| `scripts/09_preflight.sh --record` | 보정 파일·포트·모델·카메라 설정 사전 점검 |
| `scripts/10_motor_state.sh` | 모터 상태 읽기 전용 모니터 |
| `sim/calculate.sh --compare` | 모터 연결 없이 0°/+45°/-45° 중력 토크 계산 |
| `sim/teleop_sim.sh` | MuJoCo 모터를 실제 팔로워 컨트롤러에 연결하는 시뮬 |
| `tests/` | 가상 모터·중력 계산·기록 처리 회귀 검사 |

기록/학습을 하려면 `config.local.env`에서 카메라 시리얼·영상 설정을 채운다.
기본 `CAMERAS`는 비어 있고 영상 없는 본 수집은 차단된다.
관절 정보만 저장하는 시험은 `ALLOW_STATE_ONLY_RECORDING=1`로 명시한다.
데이터는 저장소 안 `datasets/`, 학습 및 텔레메트리는 `outputs/`에 생성되며 Git에서 제외된다.
Hub 업로드는 기본으로 꺼져 있다.
이 프로젝트에는 리프트·모바일 베이스 제어, 환경 충돌 검사 또는 학습 정책의 실물 배포 전용 스크립트는 없다.

## 코드 구조와 배포 방식

```text
config.env                       원본 제어 기본값, 경로는 clone 위치 기준
config.local.env.example         PC별 설정 예시
setup.sh                         고정 LeRobot 설치 + 수정 패치 적용
constraints-control.txt          제어·동역학 라이브러리 버전
patches/rebot_b601_gravity_ff.patch
assets/                          중력 계산 URDF 2종
scripts/                         장치 설정·텔레옵·수집·재생·학습·진단
sim/                             계산기 및 MuJoCo 시뮬레이션 코드
licenses/                        포함 자산·상위 프로젝트 라이선스 고지
tests/                           하드웨어 없는 회귀 검사
lerobot/                         setup.sh로 생성, Git에서 제외
```

`lerobot/` 전체 대신 **원본 커밋 + 수정 패치**를 배포한다. `setup.sh` 적용 후 핵심 파일은
원본 PC의 수정본과 바이트 단위로 같다. 주요 구현은 설치 후 다음 위치에 생긴다.

- 리더 입력·매핑: `lerobot/src/lerobot/teleoperators/rebot_102_leader/`
- 팔로워·중력보상: `lerobot/src/lerobot/robots/rebot_b601_follower/`
- 추종/수집 루프: `lerobot/src/lerobot/scripts/lerobot_teleoperate.py`, `lerobot_record.py`

텔레옵의 중력 계산에는 동봉 URDF만 있으면 된다. **시뮬레이션 시각화용 외부 MJCF·메시는
이 저장소에 포함하지 않았다.** 시뮬 실행 및 시뮬 포함 전체 테스트에는 기존 모델 묶음이
추가로 필요하다. [sim/README.md](sim/README.md)를 참고한다.

## 검증

```bash
conda activate yoidle  # config.local.env에서 바꿨다면 그 이름
python -m pip install pytest
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests -q
```

시뮬 모델이 없으면 관련 검사 4개가 skip되며, 모델을 제공하면 전체 검사를 실행한다.
배포 준비 시 원본 환경에서 **61 passed**, 외부 시뮬 모델이 없는 조건에서
**57 passed / 4 skipped**를 확인했다. +45° 중력보상 headless 데모도 5초 실행했다.
깨끗한 새 PC에서 전체 의존성을 처음 설치하는 검증과 실물 구동은 이번 배포 작업에서 수행하지 않았다.
테스트는 가상 모터/시뮬에 대한 검사다. 팀원 개체의 실제 영점·통신·제어 성능 검증은 별도다.
원본 PC의 실물 보정 파일, 데이터셋, 로그, 설치 환경 및 인증 정보는 배포하지 않는다.

## 출처

LeRobot 수정 패치의 Apache-2.0 고지와 로봇 자산의 CERN-OHL-W-2.0 / RealSense Apache-2.0
고지를 보존했다. 모델 출처와 변경 내역은 [THIRD_PARTY.md](THIRD_PARTY.md), 원문은 `licenses/`를 참고한다.
