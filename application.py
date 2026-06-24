
"""
application.py

Surgical Robot Agent 전체 실행 흐름을 관리한다.

담당 역할:
- Isaac Sim World 생성
- SceneBuilder를 통한 장면 구성
- ROS2 입력 노드 생성 및 처리
- Robot / MotionManager / StateMachine 연결
- 매 프레임 StateMachine 업데이트
- 종료 시 ROS2 자원 정리

주의:
SimulationApp은 반드시 main.py에서 먼저 생성한 뒤
이 모듈을 import해야 한다.
"""

from __future__ import annotations

from typing import Any, Callable

import rclpy
from isaacsim.core.api import World

from config import (
    SCENE,
    SIMULATION,
    print_config_summary,
    validate_required_files,
)
from input.command_input import (
    CommandInput,
    create_command_input,
)
from input.hand_input import (
    HandInput,
    create_hand_input,
)
from scene.asset_loader import AssetLoader
from scene.scene_builder import SceneBuilder
from scene.scene_objects import SceneObjects


# world.scene을 받아 실제 robot과 gripper를 반환하는 함수
RobotFactory = Callable[
    [Any],
    tuple[Any, Any],
]


# 생성된 SceneObjects를 받아
# MotionManager와 StateMachine을 반환하는 함수
RuntimeFactory = Callable[
    [SceneObjects],
    tuple[Any, Any],
]


class SurgicalRobotApplication:
    """
    Surgical Robot Agent의 최상위 실행 관리자.

    application.py는 모션 알고리즘이나 상태 전환 로직을
    직접 구현하지 않고 각 모듈을 생성하고 연결한다.
    """

    def __init__(
        self,
        simulation_app: Any,
        robot_factory: RobotFactory | None = None,
        runtime_factory: RuntimeFactory | None = None,
        validate_files: bool = True,
    ) -> None:
        if simulation_app is None:
            raise ValueError(
                "simulation_app은 None일 수 없습니다."
            )

        self.simulation_app = simulation_app

        self.robot_factory = robot_factory
        self.runtime_factory = runtime_factory
        self.validate_files = validate_files

        self.world: World | None = None
        self.scene_objects: SceneObjects | None = None

        self.asset_loader: AssetLoader | None = None
        self.scene_builder: SceneBuilder | None = None

        self.hand_input: HandInput | None = None
        self.command_input: CommandInput | None = None

        self.motion_manager: Any | None = None
        self.state_machine: Any | None = None

        self._initialized = False
        self._running = False
        self._shutdown_complete = False

    # ========================================================
    # 초기화
    # ========================================================

    def initialize(self) -> None:
        """
        Stage, World, Scene, 입력 노드, Runtime을 초기화한다.
        """

        if self._initialized:
            print(
                "[Application] 이미 초기화되어 있습니다."
            )
            return

        print_config_summary()

        if self.validate_files:
            validate_required_files()

        self._initialize_ros()

        self.asset_loader = AssetLoader(
            update_callback=self.simulation_app.update,
        )

        self.scene_builder = SceneBuilder(
            asset_loader=self.asset_loader,
        )

        # 전체 환경 Stage는 World보다 먼저 연다.
        self.scene_builder.prepare_stage()

        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=SIMULATION.physics_dt,
            rendering_dt=SIMULATION.rendering_dt,
        )

        self.scene_objects = self.scene_builder.build(
            world=self.world,
            robot_factory=self.robot_factory,
        )

        self.hand_input = create_hand_input()

        self.command_input = create_command_input(
            tray_count=len(SCENE.tray_positions),
        )

        self._create_runtime()

        self.world.reset()

        self._stabilize_simulation(
            frame_count=SIMULATION.stabilization_frames,
        )

        self._initialized = True

        print(
            "[Application] 초기화 완료"
        )

    @staticmethod
    def _initialize_ros() -> None:
        """ROS2 Context를 초기화한다."""

        if not rclpy.ok():
            rclpy.init(args=None)

    def _create_runtime(self) -> None:
        """
        MotionManager와 StateMachine을 생성한다.

        runtime_factory는 동료 코드가 준비된 뒤 연결한다.
        """

        if self.runtime_factory is None:
            print(
                "[Application] runtime_factory가 없습니다. "
                "Scene과 입력 모듈만 초기화합니다."
            )
            return

        if self.scene_objects is None:
            raise RuntimeError(
                "SceneObjects가 생성되지 않았습니다."
            )

        runtime_result = self.runtime_factory(
            self.scene_objects
        )

        if (
            not isinstance(runtime_result, tuple)
            or len(runtime_result) != 2
        ):
            raise TypeError(
                "runtime_factory는 "
                "(motion_manager, state_machine) "
                "형태의 tuple을 반환해야 합니다."
            )

        (
            self.motion_manager,
            self.state_machine,
        ) = runtime_result

        print(
            "[Application] Runtime 연결 완료 | "
            f"motion_manager="
            f"{type(self.motion_manager).__name__}, "
            f"state_machine="
            f"{type(self.state_machine).__name__}"
        )

    # ========================================================
    # 외부 객체 직접 연결
    # ========================================================

    def connect_motion_manager(
        self,
        motion_manager: Any,
    ) -> None:
        """이미 생성된 MotionManager를 직접 연결한다."""

        if motion_manager is None:
            raise ValueError(
                "motion_manager는 None일 수 없습니다."
            )

        self.motion_manager = motion_manager

    def connect_state_machine(
        self,
        state_machine: Any,
    ) -> None:
        """이미 생성된 StateMachine을 직접 연결한다."""

        if state_machine is None:
            raise ValueError(
                "state_machine은 None일 수 없습니다."
            )

        self.state_machine = state_machine

    # ========================================================
    # 메인 실행 루프
    # ========================================================

    def run(self) -> None:
        """Isaac Sim 메인 반복문을 실행한다."""

        if not self._initialized:
            self.initialize()

        if self.world is None:
            raise RuntimeError(
                "World가 초기화되지 않았습니다."
            )

        self._running = True

        print(
            "[Application] 시뮬레이션 루프 시작"
        )

        try:
            while (
                self.simulation_app.is_running()
                and self._running
            ):
                self.world.step(render=True)

                self._spin_ros_once()
                self._forward_inputs()
                self._update_state_machine()

        except KeyboardInterrupt:
            print(
                "[Application] KeyboardInterrupt 수신"
            )

        except Exception as error:
            print(
                "[Application] 실행 중 오류 발생 | "
                f"{type(error).__name__}: {error}"
            )
            raise

        finally:
            self.shutdown()

    def stop(self) -> None:
        """Application 실행 루프 종료를 요청한다."""

        self._running = False

    # ========================================================
    # ROS2 처리
    # ========================================================

    def _spin_ros_once(self) -> None:
        """현재 프레임의 ROS2 메시지를 처리한다."""

        if self.hand_input is not None:
            rclpy.spin_once(
                self.hand_input,
                timeout_sec=0.0,
            )

        if self.command_input is not None:
            rclpy.spin_once(
                self.command_input,
                timeout_sec=0.0,
            )

    # ========================================================
    # 입력 → 상태 머신 전달
    # ========================================================

    def _forward_inputs(self) -> None:
        """
        HandInput과 CommandInput의 데이터를
        StateMachine으로 전달한다.
        """

        if self.state_machine is None:
            return

        self._forward_hand_input()
        self._forward_command_input()

    def _forward_hand_input(self) -> None:
        """손 추종 입력을 StateMachine에 전달한다."""

        if (
            self.hand_input is None
            or self.state_machine is None
        ):
            return

        target_position = (
            self.hand_input.get_target_position()
        )

        if target_position is not None:
            self._call_state_machine(
                "set_hand_target",
                target_position,
            )

        if self.hand_input.consume_return_request():
            self._call_state_machine(
                "request_return"
            )

    def _forward_command_input(self) -> None:
        """작업 명령을 StateMachine에 전달한다."""

        if (
            self.command_input is None
            or self.state_machine is None
        ):
            return

        selected_tray = (
            self.command_input.consume_selected_tray()
        )

        if selected_tray is not None:
            self._call_state_machine(
                "select_tray",
                selected_tray,
            )

        if self.command_input.consume_start_request():
            self._call_state_machine(
                "request_start"
            )

        if self.command_input.consume_return_request():
            self._call_state_machine(
                "request_return"
            )

        if self.command_input.consume_stop_request():
            self._call_state_machine(
                "request_stop"
            )

        if self.command_input.consume_reset_request():
            self._call_state_machine(
                "request_reset"
            )

    def _call_state_machine(
        self,
        method_name: str,
        *args: Any,
    ) -> bool:
        """
        StateMachine에 해당 메서드가 존재하면 호출한다.

        동료의 StateMachine 인터페이스가 확정되면
        메서드 이름을 이곳에서 맞춘다.
        """

        if self.state_machine is None:
            return False

        method = getattr(
            self.state_machine,
            method_name,
            None,
        )

        if not callable(method):
            print(
                "[Application] StateMachine에 "
                f"{method_name}() 메서드가 없습니다."
            )
            return False

        method(*args)

        return True

    # ========================================================
    # 상태 머신 실행
    # ========================================================

    def _update_state_machine(self) -> None:
        """상태 머신을 한 프레임 업데이트한다."""

        if self.state_machine is None:
            return

        update_method = getattr(
            self.state_machine,
            "update",
            None,
        )

        if not callable(update_method):
            raise AttributeError(
                "연결된 StateMachine에 "
                "update() 메서드가 없습니다."
            )

        update_method()

    # ========================================================
    # 시뮬레이션 보조
    # ========================================================

    def _stabilize_simulation(
        self,
        frame_count: int,
    ) -> None:
        """World reset 후 물리 장면을 안정화한다."""

        if self.world is None:
            raise RuntimeError(
                "World가 생성되지 않았습니다."
            )

        if frame_count < 0:
            raise ValueError(
                "frame_count는 0 이상이어야 합니다."
            )

        for _ in range(frame_count):
            self.world.step(render=True)

    # ========================================================
    # 종료 처리
    # ========================================================

    def shutdown(self) -> None:
        """ROS2 노드와 실행 상태를 정리한다."""

        if self._shutdown_complete:
            return

        self._running = False

        self._destroy_node(
            self.hand_input,
            "HandInput",
        )

        self._destroy_node(
            self.command_input,
            "CommandInput",
        )

        if rclpy.ok():
            try:
                rclpy.shutdown()

            except Exception as error:
                print(
                    "[Application] rclpy 종료 오류 | "
                    f"{error}"
                )

        self._shutdown_complete = True

        print(
            "[Application] 종료 처리 완료"
        )

    @staticmethod
    def _destroy_node(
        node: Any | None,
        node_name: str,
    ) -> None:
        """ROS2 Node를 안전하게 제거한다."""

        if node is None:
            return

        destroy_method = getattr(
            node,
            "destroy_node",
            None,
        )

        if not callable(destroy_method):
            return

        try:
            destroy_method()

        except Exception as error:
            print(
                f"[Application] {node_name} 종료 오류 | "
                f"{error}"
            )

