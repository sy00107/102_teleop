# 중력 토크 계산 및 MuJoCo 시뮬레이션

## 계산기 — 추가 모델 없이 사용

```bash
./sim/calculate.sh --compare
./sim/calculate.sh --q 0 -30 -60 0 0 0 --roll 45
./sim/calculate.sh --urdf "$PWD/assets/B601_DM_wrist_d435i_no_plate.urdf" --roll 45 --json
```

기본은 `assets/ReBot_Arm_DM.urdf`(카메라 없는 기존 팔), roll +45°, pitch 0°다.
실물 텔레옵과 같은 카메라 포함 URDF는 위처럼 명시한다.
계산기는 URDF 중력항만 계산하고 스케일/토크 제한이나 실물 명령을 적용하지 않는다.
카메라 포함 URDF의 손가락 임시 질량 보완은 실물 `GravityCompensator`에서 처리하므로,
계산기의 원시 결과와 실물 보완 모델의 결과가 항상 같은 것은 아니다.
입력 관절각은 URDF 좌표계이며 모터 실측값은 영점·부호를 확인해야 한다.

## 시뮬레이션 — 외부 모델 묶음 필요

`leader_sim.py`는 실제 `RebotB601Follower` 제어 코드를 사용하고 모터/CAN만 MuJoCo로 대체한다.
원본 프로젝트의 `b601_dm` 모델 묶음이 필요하며 아래 구조가 있어야 한다.

```text
b601_dm/
  mjcf/B601_DM_wrist_d435i_no_plate.xml
  meshes/visual/...
  meshes/collision/...
  meshes/shared/...
  meshes/camera/...
```

이 저장소에는 큰 메시 묶음을 포함하지 않았다. 기존 모델 폴더 전체를 별도로 받거나
동일한 모델을 제공받은 뒤 `B601_DM_DIR`로 지정한다. 실물 텔레옵은 이 폴더가 필요 없다.

```bash
export B601_DM_DIR=/absolute/path/to/b601_dm
./sim/teleop_sim.sh --demo --headless --duration 5
./sim/teleop_sim.sh --port /dev/ttyUSB0
./sim/teleop_sim.sh --comm-loss hold
./sim/teleop_sim.sh --no-gravity-comp
./sim/teleop_sim.sh --bus-delay-ms 10
./sim/teleop_sim.sh --legacy-clamp --vel-spike 4
```

리더 입력을 쓸 때는 리더 보정 파일이 필요하다. 기본 리더 포트는 원본 PC 장치명이므로
다른 PC에서는 `--port`를 지정한다. `--calibration`으로 보정 JSON 경로를 지정할 수 있다.
기본 장착각은 +45°, 통신 상실은 home 복귀다. 실제 팔로워 모터는 연결하지 않는다.
모바일 베이스/리프트 접촉·환경 충돌·파지 검증은 포함하지 않는다.

실행기는 conda의 `CONDA_ENV`를 사용한다. 별도 Python은
`B601_GRAVITY_PYTHON=/absolute/path/to/python`으로 지정할 수 있다.
