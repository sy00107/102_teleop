# 상위 소스 및 자산 고지

## LeRobot

- Source: https://github.com/huggingface/lerobot
- Base commit: `1ef599dc5a7fd6aca6ad2ded8cf710e4d0e3f1be`
- License: Apache-2.0 (`licenses/LeRobot-Apache-2.0.txt`)
- `patches/rebot_b601_gravity_ff.patch`는 위 커밋에 대한 로컬 수정이다.
  B601 중력 feed-forward, hybrid PD 목표 제한, 통신 오류/복귀 처리,
  텔레메트리, 리더 읽기 처리, 실전송 목표 기록 및 모델 메타데이터 처리가 포함된다.
  패치 안의 상위 파일 저작권·라이선스 헤더를 보존했다.

## B601-DM / D435i URDF

- Source: https://github.com/Seeed-Projects/reBot-DevArm
- Model source revision: `52653034537b4e166d0b57a5b1c1609290aed207`
- 공식 `Rebot_Arm_description/DM` 및 `Rebot_Arm_description/Camera` 기반.
- 팔/브래킷: CERN-OHL-W-2.0 (`licenses/CERN-OHL-W-2.0.txt`)
- RealSense 자산: Apache-2.0 (`licenses/Apache-2.0.txt`)

`assets/ReBot_Arm_DM.urdf`는 기존 팔 단독 모델이다.
`assets/B601_DM_wrist_d435i_no_plate.urdf`는 로컬 `b601_dm`에서 가져온 합성 모델이며
D435i를 포함하고 하단 베이스 판/관통 나사 네 개를 제거했다.
베이스 관성은 유효 균일 밀도 추정이며 기존 base_link 프레임을 유지한다.
코드가 관성이 없는 손가락에 임시 질량 각 0.03kg을 더한다.
장착 Roll +45°는 URDF 파일에 중복 삽입하지 않고 중력 벡터 변환으로 적용한다.
외부 시각화 메시와 MJCF 파일은 이 저장소에 포함하지 않는다.

URDF SHA-256:

```text
23b5b398553f1b28ad51ceb7eefcc0926937682077cebf9d70c67f1e7d4147eb  ReBot_Arm_DM.urdf
a769154cda7f6f0df6dc4f3a60dd5b130106eaa7672f183102d07d752609766f  B601_DM_wrist_d435i_no_plate.urdf
```

원본 로컬 `yoidle` snapshot: `7b8638d`.
이 배포본에서는 실행 경로와 문서를 정리하고 재생 실행기에 공통 팔로워 설정을 전달한다.
사전 점검 출력도 실행 설정(통신 상실 동작·그리퍼 범위·속도 등)을 그대로 반영하도록 맞췄다.
두 URDF는 원본 snapshot과 동일하다. LeRobot 패치는 비동기 사고 기록·원시 리더 입력 및 최종 송신값 계측을 추가했다.
이전 배포 패치는 `patches/previous/`에 업그레이드 비교용으로 보존했다.
