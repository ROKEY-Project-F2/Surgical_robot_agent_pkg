"""
USD asset을 Isaac Sim Stage에 불러오는 모듈.

역할:
- 전체 환경 Stage 열기
- 트레이 USD reference 추가
- 도구 USD reference 추가
- Prim 유효성 확인
- 도구 위치 적용

중요:
- 이 파일은 asset 하나를 어떻게 불러올지만 담당한다.
- 트레이를 몇 개 배치할지 같은 장면 구성은 SceneBuilder가 담당한다.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

import numpy as np
import omni.usd

from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, Usd, UsdGeom


UpdateCallback = Callable[[], None]


class AssetLoader:
    """
    Isaac Sim USD asset 로더.

    Args:
        update_callback:
            SimulationApp의 update 함수를 전달한다.

            예:
                AssetLoader(simulation_app.update)
    """

    def __init__(
        self,
        update_callback: UpdateCallback,
    ) -> None:
        self._update_callback = update_callback

    def wait_frames(
        self,
        frame_count: int,
    ) -> None:
        """지정한 프레임 수만큼 SimulationApp을 업데이트한다."""

        if frame_count < 0:
            raise ValueError(
                "frame_count는 0 이상이어야 합니다. "
                f"frame_count={frame_count}"
            )

        for _ in range(frame_count):
            self._update_callback()

    def open_stage(
        self,
        usd_path: Path,
        wait_frames: int = 80,
    ) -> None:
        """
        전체 환경 USD Stage를 연다.

        World를 생성하기 전에 호출해야 한다.
        """

        resolved_path = self._resolve_file_path(
            usd_path,
            description="환경 USD",
        )

        print(
            f"[AssetLoader] Opening stage: "
            f"{resolved_path}"
        )

        omni.usd.get_context().open_stage(
            str(resolved_path)
        )

        self.wait_frames(wait_frames)

        stage = omni.usd.get_context().get_stage()

        if stage is None:
            raise RuntimeError(
                "USD Stage를 열지 못했습니다.\n"
                f"path={resolved_path}"
            )

        print(
            f"[AssetLoader] Stage opened: "
            f"{resolved_path}"
        )

    @staticmethod
    def get_stage() -> Usd.Stage:
        """현재 열린 USD Stage를 반환한다."""

        stage = omni.usd.get_context().get_stage()

        if stage is None:
            raise RuntimeError(
                "현재 열린 USD Stage가 없습니다."
            )

        return stage

    def add_reference(
        self,
        usd_path: Path,
        prim_path: str,
        wait_frames: int = 0,
    ) -> Usd.Prim:
        """
        USD 파일을 지정한 Prim 경로에 reference로 추가한다.
        """

        resolved_path = self._resolve_file_path(
            usd_path,
            description="Asset USD",
        )

        if not prim_path.startswith("/"):
            raise ValueError(
                "prim_path는 절대 Prim 경로여야 합니다. "
                f"prim_path={prim_path}"
            )

        add_reference_to_stage(
            usd_path=str(resolved_path),
            prim_path=prim_path,
        )

        self.wait_frames(wait_frames)

        stage = self.get_stage()
        prim = stage.GetPrimAtPath(prim_path)

        if not prim.IsValid():
            raise RuntimeError(
                "USD reference Prim 생성에 실패했습니다.\n"
                f"prim_path={prim_path}\n"
                f"usd_path={resolved_path}"
            )

        return prim

    def load_tray_reference(
        self,
        tray_index: int,
        tray_usd: Path,
        wait_frames: int = 10,
    ) -> str:
        """
        트레이 USD를 Stage에 추가하고 실제 트레이 Prim 경로를 반환한다.

        현재 트레이 USD 내부 구조:
            /World/tray_i/E_redtray_28
        """

        self._validate_index(
            tray_index,
            name="tray_index",
        )

        tray_root_path = (
            f"/World/tray_{tray_index}"
        )

        self.add_reference(
            usd_path=tray_usd,
            prim_path=tray_root_path,
            wait_frames=wait_frames,
        )

        tray_prim_path = (
            f"{tray_root_path}/E_redtray_28"
        )

        stage = self.get_stage()
        tray_prim = stage.GetPrimAtPath(
            tray_prim_path
        )

        if not tray_prim.IsValid():
            raise RuntimeError(
                "트레이 내부 Prim을 찾지 못했습니다.\n"
                f"tray_prim_path={tray_prim_path}\n"
                f"tray_usd={tray_usd}"
            )

        print(
            f"[AssetLoader] Tray loaded: "
            f"index={tray_index}, "
            f"prim={tray_prim_path}"
        )

        return tray_prim_path

    def load_tool_reference(
        self,
        tool_index: int,
        tool_usd: Path,
        tray_position: tuple[float, float, float]
        | np.ndarray,
        drop_height: float,
        wait_frames: int = 5,
    ) -> str:
        """
        도구 USD를 Stage에 추가하고 트레이 위에 배치한다.

        기존 정상 실행 코드와 동일하게:
        - /World/tool_i에 직접 reference
        - translate만 적용
        - scale을 따로 적용하지 않음
        - 루트 RigidBodyAPI를 다시 적용하지 않음
        """

        self._validate_index(
            tool_index,
            name="tool_index",
        )

        tool_root_path = (
            f"/World/tool_{tool_index}"
        )

        tool_prim = self.add_reference(
            usd_path=tool_usd,
            prim_path=tool_root_path,
            wait_frames=wait_frames,
        )

        tray_position_array = np.asarray(
            tray_position,
            dtype=np.float64,
        )

        if tray_position_array.shape != (3,):
            raise ValueError(
                "tray_position은 길이 3의 좌표여야 합니다. "
                f"shape={tray_position_array.shape}"
            )

        if not np.all(
            np.isfinite(tray_position_array)
        ):
            raise ValueError(
                "tray_position에 유효하지 않은 값이 있습니다. "
                f"position={tray_position_array}"
            )

        tool_position = (
            tray_position_array
            + np.array(
                [
                    0.0,
                    0.0,
                    float(drop_height),
                ],
                dtype=np.float64,
            )
        )

        xform = UsdGeom.Xformable(
            tool_prim
        )

        xform.ClearXformOpOrder()

        xform.AddTranslateOp().Set(
            Gf.Vec3d(
                float(tool_position[0]),
                float(tool_position[1]),
                float(tool_position[2]),
            )
        )

        # 중요:
        # 이곳에서 AddScaleOp()를 적용하지 않는다.
        # 도구 USD 내부의 기존 크기를 그대로 유지한다.
        #
        # 또한 루트 /World/tool_i에 RigidBodyAPI,
        # MassAPI 또는 CollisionAPI를 다시 적용하지 않는다.
        # 도구 USD 내부의 물리 설정과 중복될 수 있기 때문이다.

        print(
            f"[AssetLoader] Tool loaded: "
            f"index={tool_index}, "
            f"prim={tool_root_path}, "
            f"position={np.round(tool_position, 3)}"
        )

        return tool_root_path

    @staticmethod
    def _resolve_file_path(
        path: Path,
        description: str,
    ) -> Path:
        """경로를 정리하고 실제 파일 존재 여부를 확인한다."""

        resolved_path = (
            Path(path)
            .expanduser()
            .resolve()
        )

        if not resolved_path.is_file():
            raise FileNotFoundError(
                f"{description} 파일을 찾지 못했습니다.\n"
                f"path={resolved_path}"
            )

        return resolved_path

    @staticmethod
    def _validate_index(
        index: int,
        name: str,
    ) -> None:
        """음수 인덱스를 차단한다."""

        if index < 0:
            raise ValueError(
                f"{name}는 0 이상이어야 합니다. "
                f"{name}={index}"
            )