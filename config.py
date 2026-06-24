"""
프로젝트 전체 설정.

원칙:
- 개인 컴퓨터의 절대경로를 사용하지 않는다.
- 프로젝트 루트를 기준으로 asset/모듈 경로를 계산한다.
- Isaac Sim, ROS2, MediaPipe 모듈은 여기서 import하지 않는다.
- 모든 설정은 flat 변수로 노출한다. (단일 소스)
"""

from __future__ import annotations

from pathlib import Path
from typing import Final


# ============================================================
# 프로젝트 경로
# ============================================================

PROJECT_ROOT: Final[Path] = Path(__file__).resolve().parent

CONFIGS_DIR: Final[Path] = PROJECT_ROOT / "configs"
MOTION_DIR: Final[Path] = PROJECT_ROOT / "motion"

ASSETS_DIR: Final[Path] = PROJECT_ROOT / "assets"
ENVIRONMENTS_DIR: Final[Path] = ASSETS_DIR / "environments"
ROBOTS_DIR: Final[Path] = ASSETS_DIR / "robots"


# ============================================================
# 시뮬레이션
# ============================================================

HEADLESS: Final[bool] = False
PHYSICS_DT: Final[float] = 0.01
RENDERING_DT: Final[float] = 1.0 / 60.0
STAGE_LOADING_FRAMES: Final[int] = 80
INITIAL_SETTLING_FRAMES: Final[int] = 30


# ============================================================
# 파일 / 모듈 경로
# ============================================================

# 전체 Scene USD (git 미포함 — 각 컴퓨터에 직접 배치)
ROBOT_USD_PATH: Final[str] = str(
    ENVIRONMENTS_DIR / "full_scene" / "full_scene.usda"
)

# cuRobo 로봇 키네마틱 설정 (urdf 경로는 런타임에 동적 override)
CUROBO_ROBOT_CONFIG_PATH: Final[str] = str(CONFIGS_DIR / "m0609_v1.yml")

RMPFLOW_DIR: Final[str] = str(MOTION_DIR / "rmpflow")

M0609_URDF_PATH: Final[str] = str(
    ROBOTS_DIR / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf"
)
M0609_ASSET_ROOT_PATH: Final[str] = str(ROBOTS_DIR / "doosan-robot2")
M0609_DESCRIPTION_PATH: Final[str] = str(
    MOTION_DIR / "rmpflow" / "m0609_description.yaml"
)
M0609_RMPFLOW_CONFIG_PATH: Final[str] = str(
    MOTION_DIR / "rmpflow" / "m0609_rmpflow_common.yaml"
)


# ============================================================
# 로봇 / 그리퍼
# ============================================================

ROBOT_PRIM_PATH: Final[str] = "/World/m0609"
ROBOT_SCENE_NAME: Final[str] = "m0609_robot"
EE_LINK_NAME: Final[str] = "link_6"

ROBOT_BASE_POSITION: Final[tuple] = (0.5, 0.2, 1.0)
ROBOT_BASE_YAW_DEG: Final[float] = 90.0

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
# 트레이 / Pick-Place
# ============================================================

TABLE_HEIGHT: Final[float] = 1.0

SUPPORTED_TRAY_COMMANDS: Final[tuple] = (4, 5, 6, 7)

TRAY_SPAWN_POSITIONS: Final[dict] = {
    4: (0.24, 0.55, 1.05),
    5: (0.24, 0.85, 1.05),
    6: (0.72, 0.55, 1.05),
    7: (0.72, 0.85, 1.05),
}

TEMP_TRAY_YAW_DEGREES: Final[dict] = {
    4: 0.0,
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
# Tracking
# ============================================================

TRACKING_TOOL_ORIENTATION: Final[tuple] = (0.0, 0.0, 1.0, 0.0)
TRACKING_Z_MIN: Final[float] = 1.10
TRACKING_Z_MAX: Final[float] = 1.55
TRACKING_MAX_JOINT_STEP: Final[float] = 0.02
TRACKING_USE_MPC: Final[bool] = True
