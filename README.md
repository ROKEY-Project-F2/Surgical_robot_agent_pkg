# Surgical Robot Agent

Isaac Sim 환경에서 Doosan M0609 로봇을 제어하는 수술 도구 전달 프로젝트입니다.

프로젝트의 핵심 목표는 다음과 같습니다.

- 트레이 위의 수술 도구 Pick & Place
- MediaPipe 기반 실시간 손 추적
- ROS2를 통한 손 좌표 및 작업 명령 전달
- RMPFlow 기반 Pick / Place / Home 동작
- cuRobo MPC 기반 실시간 손 추종
- 상태 머신 기반 전체 작업 흐름 관리
- 기능별 모듈 분리로 재사용성·테스트성·협업성 향상

---

## System Overview

```text
Camera
  ↓
MediaPipe Hand Tracking
  ↓
ROS2 Publisher
  ├── /hand_raw
  ├── /hand_xyz
  └── /hand_mode
  ↓
Input Layer
  ↓
State Machine
  ↓
Motion Manager
  ├── RMPFlow : Pick / Place / Home
  └── cuRobo  : Real-time Hand Following
  ↓
M0609 Robot + Surface Gripper
  ↓
Isaac Sim
```

---

## Project Structure

```text
surgical_robot_agent/
├── main.py
├── application.py
├── config.py
│
├── assets/
│   ├── environments/
│   ├── models/
│   │   └── hand_landmarker.task
│   ├── robots/
│   ├── tools/
│   └── trays/
│
├── configs/
│   └── m0609_v1.yml
│
├── input/
│   ├── __init__.py
│   ├── hand_tracker.py
│   ├── hand_input.py
│   └── command_input.py
│
├── scene/
│   ├── __init__.py
│   ├── asset_loader.py
│   ├── scene_builder.py
│   └── scene_objects.py
│
├── motion/
│   ├── __init__.py
│   ├── motion_manager.py
│   ├── motion_types.py
│   ├── rmpflow/
│   │   ├── __init__.py
│   │   └── rmpflow_controller.py
│   └── curobo/
│       ├── __init__.py
│       └── curobo_follow_controller.py
│
├── robot/
│   ├── __init__.py
│   ├── robot_interface.py
│   ├── m0609_robot.py
│   └── joint_mapper.py
│
├── gripper/
│   ├── __init__.py
│   ├── gripper_interface.py
│   ├── dual_surface_gripper_adapter.py
│   └── surface_gripper_adapter.py
│
├── state_machine/
│   ├── __init__.py
│   ├── states.py
│   ├── task_context.py
│   └── tray_delivery_state_machine.py
│
└── tests/
    ├── test_state_machine.py
    ├── test_motion_manager.py
    ├── test_hand_input.py
    └── mocks/
```

---

## Module Responsibilities

### `main.py`

프로그램의 실행 진입점입니다.

- `SimulationApp` 생성
- `SurgicalRobotApplication` 생성
- 애플리케이션 실행 및 종료 처리

Isaac Sim 관련 import 순서 문제를 피하기 위해 `SimulationApp`은 다른 Isaac Sim 모듈보다 먼저 생성해야 합니다.

---

### `application.py`

전체 시스템을 조립하고 실행하는 최상위 모듈입니다.

- Isaac Sim `World` 생성
- Scene 구성
- ROS2 입력 노드 생성
- Robot / Motion / State Machine 연결
- 시뮬레이션 루프 실행
- 종료 시 ROS2 노드 및 자원 정리

이 파일에는 Pick / Place 알고리즘이나 상태 전환 로직을 직접 작성하지 않습니다.

---

### `config.py`

프로젝트의 모든 설정값과 상대경로를 관리합니다.

- Isaac Sim 시간 설정
- 로봇 Prim 및 Base 좌표
- 트레이 배치 좌표
- 수술 도구 Asset 경로
- MediaPipe 모델 경로
- ROS2 Topic 이름
- Hand Tracking 캘리브레이션 값

프로젝트 내부 Asset은 다음 방식으로 참조합니다.

```python
PROJECT_ROOT = Path(__file__).resolve().parent
ASSETS_DIR = PROJECT_ROOT / "assets"
```

이를 통해 사용자별 절대경로 의존성을 제거합니다.

---

### `input/hand_tracker.py`

카메라 영상에서 MediaPipe로 손을 추적하고 ROS2 Topic을 발행합니다.

주요 기능:

- 오른손 Hand Landmark 추출
- 손바닥 폭 기반 거리 추정
- 30cm / 100cm 2점 캘리브레이션
- 화면 중심 기준 X, Y 좌표 계산
- 핀홀 모델 기반 Z 거리 계산
- 로봇 End-Effector용 좌표 보정
- 좌표 스무딩
- 제스처 기반 `TRACKING` / `HOME` 전환

발행 Topic:

```text
/hand_raw   geometry_msgs/msg/Point
/hand_xyz   geometry_msgs/msg/Point
/hand_mode  std_msgs/msg/String
```

---

### `input/hand_input.py`

`hand_tracker.py`가 발행한 ROS2 Topic을 구독합니다.

- 최신 손 좌표 저장
- 최신 End-Effector 목표 좌표 저장
- 현재 모드 저장
- HOME 복귀 요청 관리

이 모듈은 입력만 보관하며 로봇을 직접 움직이지 않습니다.

---

### `input/command_input.py`

상위 작업 명령을 ROS2로 수신합니다.

지원 명령 예시:

```text
START
RETURN
HOME
STOP
RESET
PICK:3
TRAY:3
SELECT:3
```

Topic:

```text
/robot_command
/tray_command
```

---

### `scene/asset_loader.py`

USD Asset 하나를 Stage에 불러오는 방법을 담당합니다.

- 전체 환경 Stage 열기
- 트레이 USD Reference 추가
- 수술 도구 USD Reference 추가
- Prim 유효성 확인
- 위치 Transform 적용

도구 Asset의 기존 크기와 물리 설정을 유지하기 위해 루트 Prim에 물리 API를 중복 적용하지 않습니다.

---

### `scene/scene_builder.py`

프로젝트 전체 Scene을 구성합니다.

- 로봇 및 그리퍼 등록
- 트레이 8개 생성
- 수술 도구 8개 생성
- 손 목표 Marker 생성
- 생성 결과를 `SceneObjects`로 반환

---

### `scene/scene_objects.py`

Scene에서 생성된 실제 객체를 하나의 데이터 구조로 묶습니다.

```python
scene_objects.robot
scene_objects.gripper
scene_objects.trays
scene_objects.tools
scene_objects.hand_marker
```

---

### `motion/motion_manager.py`

상태 머신과 실제 Motion Controller 사이의 중간 계층입니다.

- Pick / Place / Home → RMPFlow
- Follow → cuRobo MPC
- 공통 Motion 상태 관리
- 활성 Controller 전환

---

### `motion/rmpflow/rmpflow_controller.py`

Pick, Place, Retreat, Home 등의 목표 Pose 이동을 담당합니다.

---

### `motion/curobo/curobo_follow_controller.py`

MediaPipe 손 좌표를 기반으로 실시간 End-Effector 추종을 담당합니다.

---

### `state_machine/tray_delivery_state_machine.py`

전체 작업 순서의 유일한 관리자입니다.

예상 흐름:

```text
IDLE
  ↓
MOVING_TO_PICK
  ↓
PICKING
  ↓
RETREATING
  ↓
FOLLOWING_HAND
  ↓
RETURNING
  ↓
PLACING
  ↓
GOING_HOME
  ↓
COMPLETED
```

상태 머신은 RMPFlow나 cuRobo의 내부 구현을 직접 알지 않고 `MotionManager`를 통해 동작합니다.

---

## Hand Tracking Calibration

`hand_tracker.py` 실행 후 다음 순서로 캘리브레이션합니다.

```text
1. 오른손을 카메라에서 약 30cm 위치에 둠
2. SPACE 입력
3. 오른손을 약 100cm 위치에 둠
4. SPACE 입력
5. 초점거리 계산
6. SPACE 입력
7. ROS2 좌표 발행 시작
```

키 조작:

```text
SPACE  캘리브레이션 단계 진행 / 발행 시작
R      캘리브레이션 초기화
ESC    종료
```

제스처:

```text
주먹 3초 유지
→ HOME 모드

HOME 모드에서 손을 반복해서 접고 펴기
→ TRACKING 모드 복귀
```

---

## Tray Layout

현재 기본 배치는 4열 × 2행, 총 8개입니다.

```text
tray 1    tray 3    tray 5    tray 7
tray 0    tray 2    tray 4    tray 6
```

기본 좌표:

```text
tray 0 = (-0.72, 0.55, 1.05)
tray 1 = (-0.72, 0.85, 1.05)
tray 2 = (-0.24, 0.55, 1.05)
tray 3 = (-0.24, 0.85, 1.05)
tray 4 = ( 0.24, 0.55, 1.05)
tray 5 = ( 0.24, 0.85, 1.05)
tray 6 = ( 0.72, 0.55, 1.05)
tray 7 = ( 0.72, 0.85, 1.05)
```

실제 최종 좌표는 Ubuntu Isaac Sim에서 기존 정상 실행 Scene과 대조해야 합니다.

---

## Environment

현재 프로젝트는 다음 환경을 기준으로 합니다.

- Ubuntu
- NVIDIA GPU
- Isaac Sim
- ROS2 Humble
- Python
- MediaPipe Tasks API
- OpenCV
- NumPy
- cuRobo
- RMPFlow
- Doosan M0609

---

## Run Hand Tracker

프로젝트 루트에서 실행합니다.

```bash
python3 -m input.hand_tracker
```

ROS2 Topic 확인:

```bash
ros2 topic echo /hand_raw
ros2 topic echo /hand_xyz
ros2 topic echo /hand_mode
```

---

## Test Command Input

트레이 선택:

```bash
ros2 topic pub --once \
/robot_command \
std_msgs/msg/String \
"{data: 'PICK:3'}"
```

복귀:

```bash
ros2 topic pub --once \
/robot_command \
std_msgs/msg/String \
"{data: 'HOME'}"
```

정지:

```bash
ros2 topic pub --once \
/robot_command \
std_msgs/msg/String \
"{data: 'STOP'}"
```

---

## Required Asset Layout

```text
assets/
├── environments/
│   └── full_scene/
│       └── full_scene.usda
│
├── models/
│   └── hand_landmarker.task
│
├── robots/
│   └── m0609/
│
├── trays/
│   └── red_tray/
│       └── model_redtray_scaled_for_180mm_pads.usda
│
└── tools/
    ├── scissors/
    ├── caliper/
    ├── clamps/
    ├── forceps/
    ├── handsaw/
    ├── knife/
    ├── ligature_needle/
    └── mallet/
```

USD 파일이 Material, Texture, Mesh 또는 다른 USD 파일을 참조한다면 원본 폴더 구조를 함께 유지해야 합니다.

---

## Development Rules

- 상태 머신은 하나만 둡니다.
- Scene 관련 로직은 `scene/`에만 둡니다.
- ROS2 입력은 `input/`에만 둡니다.
- Motion 알고리즘은 `motion/`에만 둡니다.
- Robot Articulation 접근은 `robot/`에만 둡니다.
- Gripper 구현은 `gripper/`에만 둡니다.
- `application.py`에는 알고리즘을 직접 작성하지 않습니다.
- 파일 이동과 기능 변경을 한 커밋에서 동시에 하지 않습니다.

---

## Current Status

- [x] 프로젝트 모듈 구조 정의
- [x] 상대경로 기반 Config 구조
- [x] MediaPipe Hand Tracker 구조화
- [x] ROS2 Hand Input
- [x] ROS2 Command Input
- [x] SceneObjects
- [x] AssetLoader 구조
- [x] SceneBuilder 구조
- [ ] M0609 Robot Factory 연결
- [ ] RMPFlow Pick / Place 통합
- [ ] cuRobo Follow 통합
- [ ] State Machine 통합
- [ ] `application.py` 최종 연결
- [ ] `main.py` 최종 실행
- [ ] Ubuntu Isaac Sim 통합 테스트

---

## Notes

현재 코드의 구조와 Python 문법은 분리 설계를 기준으로 작성되었지만, Isaac Sim API 버전, USD Prim 경로, Asset 내부 Reference, cuRobo 설정 파일은 Ubuntu GPU 환경에서 최종 검증이 필요합니다.
