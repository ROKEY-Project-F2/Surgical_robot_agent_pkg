
"""
ROS2 작업 명령 입력 모듈.

수신 명령 예시:
- PICK:3
- TRAY:3
- START
- RETURN
- HOME
- STOP
- RESET

이 모듈은 명령을 해석하고 최신 요청 상태만 저장한다.
실제 로봇 제어와 상태 전환은 수행하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
from threading import Lock

import rclpy

from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Int32, String

from config import ROS


@dataclass(frozen=True)
class CommandSnapshot:
    """현재 저장된 작업 명령 상태."""

    selected_tray: int | None
    start_requested: bool
    return_requested: bool
    stop_requested: bool
    reset_requested: bool
    last_command: str | None


class CommandInput(Node):
    """
    로봇 작업 명령을 ROS2 topic으로 수신한다.

    지원 topic:
    - ROS.command_topic
      문자열 명령 수신

    - ROS.tray_command_topic
      Int32 형태의 트레이 번호 수신
    """

    def __init__(
        self,
        tray_count: int = 8,
    ) -> None:
        super().__init__("surgical_robot_command_input")

        if tray_count <= 0:
            raise ValueError(
                "tray_count는 1 이상이어야 합니다. "
                f"tray_count={tray_count}"
            )

        self._tray_count = tray_count
        self._lock = Lock()

        self._selected_tray: int | None = None

        self._start_requested = False
        self._return_requested = False
        self._stop_requested = False
        self._reset_requested = False

        self._last_command: str | None = None

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._command_subscription = (
            self.create_subscription(
                String,
                ROS.command_topic,
                self._command_callback,
                reliable_qos,
            )
        )

        self._tray_subscription = (
            self.create_subscription(
                Int32,
                ROS.tray_command_topic,
                self._tray_callback,
                reliable_qos,
            )
        )

        self.get_logger().info(
            "CommandInput initialized | "
            f"command={ROS.command_topic}, "
            f"tray={ROS.tray_command_topic}, "
            f"tray_count={tray_count}"
        )

    def _command_callback(
        self,
        msg: String,
    ) -> None:
        """문자열 작업 명령을 해석한다."""

        command = msg.data.strip().upper()

        if not command:
            self.get_logger().warning(
                "빈 명령을 무시했습니다."
            )
            return

        try:
            self._handle_command(command)

        except ValueError as error:
            self.get_logger().warning(
                f"명령 처리 실패: {error}"
            )

    def _tray_callback(
        self,
        msg: Int32,
    ) -> None:
        """Int32 형태로 전달된 트레이 번호를 저장한다."""

        tray_index = int(msg.data)

        if not self._is_valid_tray_index(
            tray_index
        ):
            self.get_logger().warning(
                "유효하지 않은 트레이 번호를 "
                f"무시했습니다: {tray_index}"
            )
            return

        with self._lock:
            self._selected_tray = tray_index
            self._last_command = (
                f"TRAY:{tray_index}"
            )

        self.get_logger().info(
            f"Tray selected: {tray_index}"
        )

    def _handle_command(
        self,
        command: str,
    ) -> None:
        """
        문자열 명령을 실제 요청 상태로 변환한다.
        """

        if command in {
            "START",
            "RUN",
        }:
            with self._lock:
                self._start_requested = True
                self._last_command = command

            self.get_logger().info(
                "Start requested."
            )
            return

        if command in {
            "RETURN",
            "HOME",
            "BACK",
        }:
            with self._lock:
                self._return_requested = True
                self._last_command = command

            self.get_logger().info(
                "Return requested."
            )
            return

        if command in {
            "STOP",
            "HALT",
            "EMERGENCY_STOP",
        }:
            with self._lock:
                self._stop_requested = True
                self._last_command = command

            self.get_logger().warning(
                "Stop requested."
            )
            return

        if command in {
            "RESET",
            "CLEAR",
        }:
            with self._lock:
                self._reset_requested = True
                self._last_command = command

            self.get_logger().info(
                "Reset requested."
            )
            return

        tray_index = (
            self._parse_tray_command(
                command
            )
        )

        if tray_index is not None:
            with self._lock:
                self._selected_tray = tray_index
                self._last_command = command

            self.get_logger().info(
                f"Tray selected: {tray_index}"
            )
            return

        raise ValueError(
            f"지원하지 않는 명령입니다: {command}"
        )

    def snapshot(self) -> CommandSnapshot:
        """
        현재 명령 상태를 한 번에 복사해 반환한다.
        """

        with self._lock:
            return CommandSnapshot(
                selected_tray=(
                    self._selected_tray
                ),
                start_requested=(
                    self._start_requested
                ),
                return_requested=(
                    self._return_requested
                ),
                stop_requested=(
                    self._stop_requested
                ),
                reset_requested=(
                    self._reset_requested
                ),
                last_command=(
                    self._last_command
                ),
            )

    def get_selected_tray(
        self,
    ) -> int | None:
        """현재 선택된 트레이 번호를 반환한다."""

        with self._lock:
            return self._selected_tray

    def consume_selected_tray(
        self,
    ) -> int | None:
        """
        선택된 트레이 번호를 한 번 반환하고 초기화한다.

        같은 트레이 선택 요청이 여러 번 처리되는 것을
        방지할 때 사용한다.
        """

        with self._lock:
            selected_tray = (
                self._selected_tray
            )

            self._selected_tray = None

            return selected_tray

    def consume_start_request(
        self,
    ) -> bool:
        """새 시작 요청이 있으면 한 번만 True를 반환한다."""

        with self._lock:
            requested = (
                self._start_requested
            )

            self._start_requested = False

            return requested

    def consume_return_request(
        self,
    ) -> bool:
        """새 복귀 요청이 있으면 한 번만 True를 반환한다."""

        with self._lock:
            requested = (
                self._return_requested
            )

            self._return_requested = False

            return requested

    def consume_stop_request(
        self,
    ) -> bool:
        """새 정지 요청이 있으면 한 번만 True를 반환한다."""

        with self._lock:
            requested = (
                self._stop_requested
            )

            self._stop_requested = False

            return requested

    def consume_reset_request(
        self,
    ) -> bool:
        """새 초기화 요청이 있으면 한 번만 True를 반환한다."""

        with self._lock:
            requested = (
                self._reset_requested
            )

            self._reset_requested = False

            return requested

    def clear_requests(self) -> None:
        """
        선택된 트레이와 모든 요청 플래그를 초기화한다.
        """

        with self._lock:
            self._selected_tray = None

            self._start_requested = False
            self._return_requested = False
            self._stop_requested = False
            self._reset_requested = False

            self._last_command = None

    def _parse_tray_command(
        self,
        command: str,
    ) -> int | None:
        """
        PICK:3, TRAY:3, SELECT:3 형식에서
        트레이 번호를 추출한다.
        """

        prefixes = (
            "PICK:",
            "TRAY:",
            "SELECT:",
        )

        for prefix in prefixes:
            if not command.startswith(prefix):
                continue

            value = command[
                len(prefix):
            ].strip()

            if not value:
                raise ValueError(
                    "트레이 번호가 없습니다. "
                    f"command={command}"
                )

            try:
                tray_index = int(value)

            except ValueError as error:
                raise ValueError(
                    "트레이 번호는 정수여야 합니다. "
                    f"value={value}"
                ) from error

            if not self._is_valid_tray_index(
                tray_index
            ):
                raise ValueError(
                    "트레이 번호가 범위를 "
                    "벗어났습니다. "
                    f"tray_index={tray_index}, "
                    f"valid=0~{self._tray_count - 1}"
                )

            return tray_index

        return None

    def _is_valid_tray_index(
        self,
        tray_index: int,
    ) -> bool:
        """트레이 번호가 유효한 범위인지 확인한다."""

        return (
            0
            <= tray_index
            < self._tray_count
        )


def create_command_input(
    tray_count: int = 8,
) -> CommandInput:
    """
    ROS2가 초기화되지 않았다면 초기화하고
    CommandInput을 생성한다.
    """

    if not rclpy.ok():
        rclpy.init(args=None)

    return CommandInput(
        tray_count=tray_count
    )
