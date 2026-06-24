"""
Isaac Sim 전체 장면 구성 모듈.

역할:
- 전체 환경 Stage 준비
- 로봇 및 그리퍼 등록 함수 호출
- 트레이 8개 생성
- 수술도구 8개 생성
- Hand Marker 생성
- 생성 결과를 SceneObjects로 반환

Asset 하나를 실제로 불러오는 세부 처리는
AssetLoader에 위임한다.
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from isaacsim.core.api.objects import VisualSphere
from isaacsim.core.prims import SingleRigidPrim

from config import (
    ASSETS,
    SCENE,
    SIMULATION,
)
from scene.asset_loader import AssetLoader
from scene.scene_objects import SceneObjects


# world.scene을 전달받아
# 실제 robot, gripper를 반환하는 함수 타입
RobotFactory = Callable[
    [Any],
    tuple[Any, Any],
]


class SceneBuilder:
    """
    프로젝트 전체 Isaac Sim 장면을 구성한다.

    Args:
        asset_loader:
            USD reference와 Stage 처리를 담당하는 AssetLoader.
    """

    def __init__(
        self,
        asset_loader: AssetLoader,
    ) -> None:
        self.asset_loader = asset_loader

    def prepare_stage(self) -> None:
        """
        전체 환경 Stage를 연다.

        이 함수는 World 생성 전에 호출해야 한다.
        """

        self.asset_loader.open_stage(
            usd_path=ASSETS.full_scene_usd,
            wait_frames=(
                SIMULATION.stage_loading_frames
            ),
        )

    def build(
        self,
        world: Any,
        robot_factory: RobotFactory | None = None,
    ) -> SceneObjects:
        """
        World에 로봇, 트레이, 도구, 마커를 등록한다.

        Args:
            world:
                Isaac Sim World 객체.

            robot_factory:
                robot과 gripper를 생성 또는 등록하는 함수.

                함수 형태:
                    robot, gripper = robot_factory(
                        world.scene
                    )

                아직 로봇 연결을 하지 않을 경우 None 가능.

        Returns:
            SceneObjects:
                생성된 실제 장면 객체 모음.
        """

        if world is None:
            raise ValueError(
                "world는 None일 수 없습니다."
            )

        robot = None
        gripper = None

        if robot_factory is not None:
            robot, gripper = robot_factory(
                world.scene
            )

        trays = self._create_trays(
            world.scene
        )

        tools = self._create_tools()

        hand_marker = (
            self._create_hand_marker(
                world.scene
            )
        )

        scene_objects = SceneObjects(
            world=world,
            robot=robot,
            gripper=gripper,
            trays=trays,
            tools=tools,
            hand_marker=hand_marker,
        )

        print(
            "[SceneBuilder] Scene build complete: "
            f"trays={len(trays)}, "
            f"tools={len(tools)}, "
            f"robot={'yes' if robot is not None else 'no'}, "
            f"gripper={'yes' if gripper is not None else 'no'}"
        )

        return scene_objects

    def _create_trays(
        self,
        scene: Any,
    ) -> list[Any]:
        """설정된 위치에 트레이를 생성한다."""

        trays: list[Any] = []

        tray_orientation = np.asarray(
            SCENE.tray_orientation,
            dtype=np.float64,
        )

        if tray_orientation.shape != (4,):
            raise ValueError(
                "tray_orientation은 길이 4의 "
                "Quaternion이어야 합니다. "
                f"shape={tray_orientation.shape}"
            )

        for tray_index, position in enumerate(
            SCENE.tray_positions
        ):
            tray_prim_path = (
                self.asset_loader
                .load_tray_reference(
                    tray_index=tray_index,
                    tray_usd=ASSETS.tray_usd,
                    wait_frames=10,
                )
            )

            tray_position = np.asarray(
                position,
                dtype=np.float64,
            )

            tray = scene.add(
                SingleRigidPrim(
                    prim_path=tray_prim_path,
                    name=f"tray_{tray_index}",
                    position=tray_position,
                    orientation=(
                        tray_orientation.copy()
                    ),
                )
            )

            trays.append(tray)

            print(
                f"[SceneBuilder] Tray registered: "
                f"index={tray_index}, "
                f"position="
                f"{np.round(tray_position, 3)}"
            )

        return trays

    def _create_tools(
        self,
    ) -> list[str]:
        """각 트레이 위에 대응하는 수술도구를 배치한다."""

        tools: list[str] = []

        tray_positions = (
            SCENE.tray_positions
        )

        tray_count = len(
            tray_positions
        )

        tool_count = len(
            ASSETS.tool_usds
        )

        if tool_count < tray_count:
            raise ValueError(
                "트레이 개수보다 도구 Asset 개수가 적습니다.\n"
                f"tray_count={tray_count}\n"
                f"tool_count={tool_count}"
            )

        if tool_count > tray_count:
            print(
                "[SceneBuilder] Warning: "
                "도구 Asset 개수가 트레이 개수보다 많습니다. "
                "앞쪽 도구만 사용합니다."
            )

        for tool_index, tray_position in enumerate(
            tray_positions
        ):
            tool_prim_path = (
                self.asset_loader
                .load_tool_reference(
                    tool_index=tool_index,
                    tool_usd=(
                        ASSETS.tool_usds[
                            tool_index
                        ]
                    ),
                    tray_position=tray_position,
                    drop_height=(
                        SCENE.tool_drop_height
                    ),
                    wait_frames=5,
                )
            )

            # 현재 기존 코드와 동일하게
            # 실제 Isaac Wrapper 객체가 아니라
            # Stage의 Prim 경로 문자열을 저장한다.
            tools.append(
                tool_prim_path
            )

        return tools

    @staticmethod
    def _create_hand_marker(
        scene: Any,
    ) -> Any:
        """손 목표 위치 확인용 시각화 Sphere를 생성한다."""

        hand_marker = scene.add(
            VisualSphere(
                prim_path="/World/HandMarker",
                name="hand_marker",
                position=np.array(
                    [
                        0.0,
                        0.25,
                        SCENE.table_height,
                    ],
                    dtype=np.float64,
                ),
                radius=0.03,
                color=np.array(
                    [
                        0.1,
                        0.3,
                        1.0,
                    ],
                    dtype=np.float64,
                ),
            )
        )

        print(
            "[SceneBuilder] Hand marker registered: "
            "/World/HandMarker"
        )

        return hand_marker