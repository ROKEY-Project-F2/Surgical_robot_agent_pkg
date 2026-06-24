
"""
프로젝트 전체 설정.

원칙:
- 개인 컴퓨터의 절대경로를 사용하지 않는다.
- 프로젝트 루트를 기준으로 asset 경로를 계산한다.
- Isaac Sim, ROS2, MediaPipe 모듈은 여기서 import하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Final


# ============================================================
# 프로젝트 경로
# ============================================================

PROJECT_ROOT: Final[Path] = Path(__file__).resolve().parent

CONFIGS_DIR: Final[Path] = PROJECT_ROOT / "configs"

ASSETS_DIR: Final[Path] = PROJECT_ROOT / "assets"
ENVIRONMENTS_DIR: Final[Path] = ASSETS_DIR / "environments"
ROBOTS_DIR: Final[Path] = ASSETS_DIR / "robots"
TOOLS_DIR: Final[Path] = ASSETS_DIR / "tools"
TRAYS_DIR: Final[Path] = ASSETS_DIR / "trays"
MODELS_DIR: Final[Path] = ASSETS_DIR / "models"


# ============================================================
# Isaac Sim 설정
# ============================================================

@dataclass(frozen=True)
class SimulationConfig:
    headless: bool = False

    physics_dt: float = 0.01
    rendering_dt: float = 1.0 / 60.0

    stage_loading_frames: int = 80
    stabilization_frames: int = 30


# ============================================================
# M0609 로봇 설정
# ============================================================

@dataclass(frozen=True)
class RobotConfig:
    prim_path: str = "/World/m0609"
    end_effector_link_name: str = "link_6"

    joint_names: tuple[str, ...] = (
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    )

    base_position: tuple[float, float, float] = (
        0.5,
        0.2,
        1.0,
    )

    base_yaw_deg: float = 90.0

    curobo_config_path: Path = (
        CONFIGS_DIR / "m0609_v1.yml"
    )


# ============================================================
# Asset 설정
# ============================================================

@dataclass(frozen=True)
class AssetConfig:
    full_scene_usd: Path = (
        ENVIRONMENTS_DIR
        / "full_scene"
        / "full_scene.usda"
    )

    tray_usd: Path = (
        TRAYS_DIR
        / "red_tray"
        / "model_redtray_scaled_for_180mm_pads.usda"
    )

    tool_usds: tuple[Path, ...] = field(
        default_factory=lambda: (
            TOOLS_DIR
            / "surgical_instruments"
            / "Model"
            / "sm_bipolardissectingscissors_a01_01.usd",
    
            TOOLS_DIR
            / "surgical_instruments"
            / "Model"
            / "sm_caliper_a01_01.usd",
    
            TOOLS_DIR
            / "surgical_instruments"
            / "Model"
            / "sm_clamps_a01_01.usd",
    
            TOOLS_DIR
            / "surgical_instruments"
            / "Model"
            / "sm_forceps_a01_01.usd",
    
            TOOLS_DIR
            / "surgical_instruments"
            / "Model"
            / "sm_handsaws_a01_01.usd",
    
            TOOLS_DIR
            / "surgical_instruments"
            / "Model"
            / "sm_knife_a01_01.usd",
    
            TOOLS_DIR
            / "surgical_instruments"
            / "Model"
            / "sm_ligatureneedle_a01_01.usd",
    
            TOOLS_DIR
            / "surgical_instruments"
            / "Model"
            / "sm_mallet_a01_01.usd",
        )
    )


# ============================================================
# 장면 배치 설정
# ============================================================

@dataclass(frozen=True)
class SceneLayoutConfig:
    table_height: float = 1.0
    tray_z: float = 1.05

    # Quaternion 순서: w, x, y, z
    tray_orientation: tuple[float, float, float, float] = (
        0.7071,
        0.0,
        0.0,
        0.7071,
    )

    tool_drop_height: float = 0.05

    tray_x_positions: tuple[float, ...] = (
        -0.72,
        -0.24,
        0.24,
        0.72,
    )

    tray_y_positions: tuple[float, ...] = (
        0.55,
        0.85,
    )

    @property
    def tray_positions(
        self,
    ) -> tuple[tuple[float, float, float], ...]:
        return tuple(
            (
                float(x),
                float(y),
                float(self.tray_z),
            )
            for x in self.tray_x_positions
            for y in self.tray_y_positions
        )


# ============================================================
# Pick / Place 작업 설정
# ============================================================

@dataclass(frozen=True)
class TaskConfig:
    target_tray_index: int = 7

    pick_height_offset: float = 0.136
    pre_pick_height: float = 0.12
    lift_height: float = 0.10

    max_follow_joint_step: float = 0.02

    grip_wait_frames: int = 15
    release_wait_frames: int = 15


# ============================================================
# ROS2 Topic 설정
# ============================================================

@dataclass(frozen=True)
class RosConfig:
    hand_publisher_node_name: str = "hand_publisher"
    hand_input_node_name: str = "m0609_single_hand_subscriber"
    command_input_node_name: str = "surgical_robot_command_input"

    hand_raw_topic: str = "/hand_raw"
    hand_xyz_topic: str = "/hand_xyz"
    hand_mode_topic: str = "/hand_mode"

    command_topic: str = "/robot_command"
    tray_command_topic: str = "/tray_command"

    tracking_mode: str = "TRACKING"
    return_mode: str = "HOME"


# ============================================================
# MediaPipe Hand Tracking 설정
# ============================================================

@dataclass(frozen=True)
class HandTrackingConfig:
    model_path: Path = (
        MODELS_DIR / "hand_landmarker.task"
    )

    camera_index: int = 0
    camera_width: int = 960
    camera_height: int = 540

    display_scale: float = 1.0
    num_hands: int = 2

    # 실제 손바닥 폭: 9cm
    real_palm_width_m: float = 0.09

    # 캘리브레이션 기준 거리
    near_distance_cm: float = 30.0
    far_distance_cm: float = 100.0

    # ROS 발행 주기: 30Hz
    publish_interval_sec: float = 1.0 / 30.0

    # 손 좌표 후처리
    smoothing_alpha: float = 0.25

    # 기존 코드의 hand_y = raw_y - 0.3
    hand_y_offset: float = -0.30

    # 로봇 EE가 손보다 앞/위로 이동하도록 보정
    ee_y_offset: float = 0.25
    ee_z_offset: float = 0.05

    table_height: float = 1.0
    table_clearance: float = 0.05

    home_position: tuple[float, float, float] = (
        0.0,
        0.25,
        1.05,
    )

    fist_hold_sec: float = 3.0
    reactivate_sec: float = 3.0
    reactivate_min_transitions: int = 4


# ============================================================
# 전역 설정 객체
# ============================================================

SIMULATION = SimulationConfig()
ROBOT = RobotConfig()
ASSETS = AssetConfig()
SCENE = SceneLayoutConfig()
TASK = TaskConfig()
ROS = RosConfig()
HAND_TRACKING = HandTrackingConfig()


# ============================================================
# 필수 파일 검증
# ============================================================

def required_file_paths() -> tuple[Path, ...]:
    return (
        ASSETS.full_scene_usd,
        ASSETS.tray_usd,
        ROBOT.curobo_config_path,
        HAND_TRACKING.model_path,
        *ASSETS.tool_usds,
    )


def validate_required_files() -> None:
    missing_paths = [
        path
        for path in required_file_paths()
        if not path.is_file()
    ]

    if not missing_paths:
        return

    missing_text = "\n".join(
        f"  - {path}"
        for path in missing_paths
    )

    raise FileNotFoundError(
        "필수 asset 또는 설정 파일이 없습니다.\n"
        f"{missing_text}"
    )


def validate_hand_tracking_files() -> None:
    if not HAND_TRACKING.model_path.is_file():
        raise FileNotFoundError(
            "MediaPipe Hand Landmarker 모델이 없습니다.\n"
            f"path={HAND_TRACKING.model_path}"
        )


# ============================================================
# 모션 경로 (motion/ 디렉토리 기준)
# ============================================================

MOTION_DIR: Final[Path] = PROJECT_ROOT / "motion"
RMPFLOW_DIR: Final[str] = str(MOTION_DIR / "rmpflow")

M0609_URDF_PATH: Final[str] = str(
    ROBOTS_DIR / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf"
)

M0609_DESCRIPTION_PATH: Final[str] = str(
    MOTION_DIR / "rmpflow" / "m0609_description.yaml"
)

M0609_RMPFLOW_CONFIG_PATH: Final[str] = str(
    MOTION_DIR / "rmpflow" / "m0609_rmpflow_common.yaml"
)

CUROBO_ROBOT_CONFIG_PATH: Final[str] = str(ROBOT.curobo_config_path)
ROBOT_USD_PATH: Final[str] = str(ASSETS.full_scene_usd)


# ============================================================
# 로봇 / 그리퍼 flat 변수 (task 모듈 호환)
# ============================================================

ROBOT_PRIM_PATH: Final[str] = ROBOT.prim_path
ROBOT_SCENE_NAME: Final[str] = "m0609_robot"
EE_LINK_NAME: Final[str] = ROBOT.end_effector_link_name
ROBOT_BASE_POSITION: Final[tuple] = ROBOT.base_position
ROBOT_BASE_YAW_DEG: Final[float] = ROBOT.base_yaw_deg

_SURFACE_GRIPPER_BASE_PATH: Final[str] = (
    f"{ROBOT_PRIM_PATH}/onrobot_rg2ft/gripper_body/dual_suction_tool"
)

SURFACE_GRIPPER_PATHS: Final[list] = [
    f"{_SURFACE_GRIPPER_BASE_PATH}/suction_contact_left/SurfaceGripper_left",
    f"{_SURFACE_GRIPPER_BASE_PATH}/suction_contact_right/SurfaceGripper_right",
]

SURFACE_GRIPPER_WRITE_STATUS_TO_USD: Final[bool] = True

DRIVE_STIFFNESS: Final[float] = 1e8
DRIVE_DAMPING: Final[float] = 1e4
DRIVE_MAX_FORCE: Final[float] = 1e8


# ============================================================
# 트레이 / Pick-Place flat 변수 (task 모듈 호환)
# ============================================================

TABLE_HEIGHT: Final[float] = SCENE.table_height

SUPPORTED_TRAY_COMMANDS: Final[tuple] = (4, 5, 6, 7)

TRAY_SPAWN_POSITIONS: Final[dict] = {
    4: (0.24, 0.55, 1.05),
    5: (0.24, 0.85, 1.05),
    6: (0.72, 0.55, 1.05),
    7: (0.72, 0.85, 1.05),
}

TEMP_TRAY_YAW_DEGREES: Final[dict] = {
    4:  0.0,
    5: 15.0,
    6: -20.0,
    7: 35.0,
}

TEMP_TRAY_SIZE: Final[tuple] = (0.300, 0.220, 0.01861830)
TEMP_TRAY_MASS: Final[float] = 0.15
ENABLE_TEMP_DYNAMIC_TRAYS: Final[bool] = True

STAGING_POSITION: Final[tuple] = (0.10, 0.50, 1.35)

PICK_EVENTS_DT: Final[tuple] = (
    0.008, 0.005, 0.02, 0.15, 0.0025,
    0.01, 0.0025, 1.0, 0.008, 0.08,
)

PICK_DEFAULT_EE_OFFSET: Final[tuple] = (0.0, 0.0, 0.20)
PICK_APPROACH_Z_CORRECTION: Final[float] = 0.042

PLACE_LINK6_ABOVE_TRAY: Final[float] = 0.136
PLACE_HIGH_OFFSET: Final[float] = 0.10
PLACE_APPROACH_GAP: Final[float] = 0.05
PLACE_MOVE_TOLERANCE: Final[float] = 0.04


# ============================================================
# Tracking flat 변수 (task 모듈 호환)
# ============================================================

TRACKING_TOOL_ORIENTATION: Final[tuple] = (0.0, 0.0, 1.0, 0.0)
TRACKING_Z_MIN: Final[float] = 1.10
TRACKING_Z_MAX: Final[float] = 1.55
TRACKING_MAX_JOINT_STEP: Final[float] = 0.02
TRACKING_USE_MPC: Final[bool] = True

INITIAL_SETTLING_FRAMES: Final[int] = 30


def print_config_summary() -> None:
    print("=" * 70)
    print("[CONFIG] Surgical Robot Agent")
    print(f"[CONFIG] project root = {PROJECT_ROOT}")
    print(f"[CONFIG] full scene   = {ASSETS.full_scene_usd}")
    print(f"[CONFIG] tray asset   = {ASSETS.tray_usd}")
    print(f"[CONFIG] tool count   = {len(ASSETS.tool_usds)}")
    print(f"[CONFIG] tray count   = {len(SCENE.tray_positions)}")
    print(f"[CONFIG] robot prim   = {ROBOT.prim_path}")
    print(f"[CONFIG] hand model   = {HAND_TRACKING.model_path}")
    print(f"[CONFIG] physics dt   = {SIMULATION.physics_dt}")
    print("=" * 70)