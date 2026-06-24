# Surgical Robot Agent

Isaac Sim 기반 수술 로봇(Doosan M0609) 시뮬레이션 패키지.
**손 추적(Hand Tracking) → Pick → 추종(Tracking) → Place** 워크플로우를 ROS2로 제어한다.

---

## 요구 환경

| 항목 | 버전 |
|---|---|
| Isaac Sim | 4.x |
| ROS2 | Humble |
| cuRobo | 0.7.8+ |
| Python | Isaac Sim 내장 `python.sh` |

> 이 패키지는 **폴더 단독 배포**를 전제로 한다. 모든 경로는 패키지 루트 기준으로 자동 계산되며,
> cuRobo 설정(`m0609_v1.yml`)의 URDF 경로도 런타임에 동적으로 덮어쓴다.
> 즉 폴더를 어디에 두든 `full_scene.usda`만 배치하면 동작한다.

---

## 디렉토리 구조

```
surgical_robot_agent/
├── main.py                           # 실행 진입점
├── config.py                         # 전체 설정 (단일 소스, flat 변수)
│
├── assets/
│   ├── environments/full_scene/      # ⚠️ git 미포함 — full_scene.usda 직접 배치
│   ├── robots/doosan-robot2/         # M0609 URDF + mesh
│   ├── tools/surgical_instruments/   # 수술도구 USD
│   └── trays/red_tray/              # 트레이 USD
│
├── configs/
│   └── m0609_v1.yml                 # cuRobo 키네마틱 설정 (경로는 런타임 override)
│
├── motion/
│   ├── m0609_curobo_controller.py   # cuRobo MotionGen + MPC
│   ├── m0609_move_controller.py     # RMPFlow 단일 목표 이동
│   ├── m0609_tracking_controller.py # 손 추종 (cuRobo MPC)
│   └── rmpflow/
│       ├── m0609_rmpflow_controller.py
│       ├── m0609_pick_place_controller_surface.py
│       ├── m0609_description.yaml
│       └── m0609_rmpflow_common.yaml
│
├── gripper/
│   ├── surface_gripper_adapter.py   # Isaac Surface Gripper 래퍼
│   └── dual_surface_gripper_adapter.py  # 듀얼 흡착 그리퍼
│
├── scene/
│   └── dynamic_tray_builder.py      # 동적 큐브 트레이 생성 (DynamicCuboid)
│
├── state_machine/
│   ├── m0609_state_machine.py       # IDLE / PICK / TRACKING / PLACE
│   └── m0609_task.py                # Isaac Sim BaseTask (Scene 등록)
│
├── input/
│   └── hand_marker_visualizer.py    # 손 위치 시각화 구체
│
└── ros_bridge/
    └── m0609_ros_bridge.py          # OmniGraph 기반 ROS2 Bridge
```

---

## 동작 흐름

```
[손 추적 노드]                       [Isaac Sim]
  카메라 → mediapipe                   ROS2 Bridge (OmniGraph)
     │                                      │
     ├── /hand_raw   ───────────────▶  HandMarker 시각화
     ├── /hand_xyz   ───────────────▶  TRACKING (cuRobo MPC 추종)
     └── /hand_mode  ───────────────▶  HOME 수신 시 PLACE 전환

[운영자] /m0609/pick_command (4~7) ─▶  PICK 시작
```

상태 전이: `IDLE → (pick 명령) → PICK → TRACKING → (HOME) → PLACE → IDLE`

---

## 최초 설정

### 1. full_scene.usda 배치

`assets/environments/full_scene/` 폴더는 파일 크기(54MB+) 때문에 git에 포함되지 않는다.
아래 위치에 직접 복사한다.

```
<패키지 경로>/assets/environments/full_scene/full_scene.usda
```

### 2. 그 외 경로

URDF·RMPFlow·cuRobo 경로는 `config.py`가 패키지 루트 기준으로 자동 계산한다.
`m0609_v1.yml` 안에 절대경로가 남아 있어도 런타임에 무시되므로 수정할 필요 없다.

---

## 실행

터미널 두 개를 사용한다. **두 터미널 모두 `ROS_DOMAIN_ID=136`으로 통일.**

### 터미널 1 — 손 추적 노드

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=136 python3 /path/to/hand_tracking_node.py
```

### 터미널 2 — Isaac Sim 시뮬레이션

```bash
source /opt/ros/humble/setup.bash
cd <패키지 경로>/surgical_robot_agent
ROS_DOMAIN_ID=136 ~/.local/share/ov/pkg/isaac-sim-*/python.sh main.py
```

> Isaac Sim을 먼저 켠 뒤 손 추적 노드를 켜도 된다.
> `python.sh` 경로가 다르면 `find ~/.local/share/ov/pkg -name python.sh` 로 확인한다.

---

## ROS2 토픽

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| Subscribe | `/hand_raw` | `geometry_msgs/Point` | 손 원시 좌표 (시각화용) |
| Subscribe | `/hand_xyz` | `geometry_msgs/Point` | 보정된 EE 목표 좌표 |
| Subscribe | `/hand_mode` | `std_msgs/String` | `TRACKING` / `HOME` |
| Subscribe | `/m0609/pick_command` | `std_msgs/Int32` | 트레이 번호 (4~7) |
| Subscribe | `/m0609/move_command` | `std_msgs/String` | JSON 직접 이동 (워크플로우 모드에선 비활성) |
| Publish | `/m0609/move_result` | `std_msgs/String` | 이동 결과 응답 |

### Pick 명령 예시

```bash
ROS_DOMAIN_ID=136 ros2 topic pub --once /m0609/pick_command std_msgs/msg/Int32 "{data: 7}"
```

### 추종 종료 → Place 트리거

```bash
ROS_DOMAIN_ID=136 ros2 topic pub --once /hand_mode std_msgs/msg/String "{data: HOME}"
```

지원 트레이 번호: `4`, `5`, `6`, `7`

---

## 주요 설정

모든 파라미터는 `config.py` 한 곳에서 관리한다.

| 항목 | 변수 | 기본값 |
|---|---|---|
| 로봇 베이스 위치 | `ROBOT_BASE_POSITION` | `(0.5, 0.2, 1.0)` |
| 로봇 베이스 회전 | `ROBOT_BASE_YAW_DEG` | `90.0` |
| 트레이 생성 위치 | `TRAY_SPAWN_POSITIONS` | 4~7번 좌표 dict |
| 트레이 yaw | `TEMP_TRAY_YAW_DEGREES` | 4~7번 각도 dict |
| 큐브 트레이 크기 | `TEMP_TRAY_SIZE` | `(0.30, 0.22, 0.019)` |
| 큐브 트레이 질량 | `TEMP_TRAY_MASS` | `0.15 kg` |
| 손 추종 Z 범위 | `TRACKING_Z_MIN / MAX` | `1.10 ~ 1.55` |
| 관절 스텝 제한 | `TRACKING_MAX_JOINT_STEP` | `0.02` |
| Pick Z 보정 | `PICK_APPROACH_Z_CORRECTION` | `0.042` |
| Place link6 높이 | `PLACE_LINK6_ABOVE_TRAY` | `0.136` |
| 대기 위치 | `STAGING_POSITION` | `(0.10, 0.50, 1.35)` |

---

## 트레이 배치

```
        y=0.55   y=0.85
x=0.24   [4]      [5]
x=0.72   [6]      [7]
```
