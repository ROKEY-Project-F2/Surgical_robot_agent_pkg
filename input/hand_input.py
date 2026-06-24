"""
ROS2 Hand Tracking 입력 모듈.

hand_tracker.py가 발행하는 다음 토픽을 구독한다.

- /hand_raw  : 실제 손의 월드 좌표
- /hand_xyz  : 로봇 End-Effector가 추종할 목표 좌표
- /hand_mode : TRACKING 또는 HOME

이 모듈은 입력만 저장한다.
로봇 제어와 상태 전환은 수행하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
from threading import Lock

import numpy as np
import rclpy

from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from config import ROS


@dataclass(frozen=True)
class HandInputSnapshot:
    """
    특정 시점의 Hand Tracking 입력값.

    배열은 snapshot() 호출 시 복사되므로,
    외부에서 수정해도 HandInput 내부 값은 변하지 않는다.
    """

    raw_position: np.ndarray | None
    target_position: np.ndarray | None
    mode: str
    raw_received: bool
    target_received: bool


class HandInput(Node):
    """
    Hand Tracking ROS2 토픽을 구독하고 최신 값을 보관한다.

    application.py는 이 객체에서 최신 값을 읽어
    상태 머신이나 Motion Manager에 전달한다.
    """

    def __init__(self) -> None:
        super().__init__(ROS.hand_node_name)

        self._lock = Lock()

        self._raw_position: np.ndarray | None = None
        self._target_position: np.ndarray | None = None

        self._mode = ROS.tracking_mode

        self._raw_received = False
        self._target_received = False
        self._return_requested = False

        stream_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._raw_subscription = self.create_subscription(
            Point,
            ROS.hand_raw_topic,
            self._raw_callback,
            stream_qos,
        )

        self._target_subscription = self.create_subscription(
            Point,
            ROS.hand_xyz_topic,
            self._target_callback,
            stream_qos,
        )

        self._mode_subscription = self.create_subscription(
            String,
            ROS.hand_mode_topic,
            self._mode_callback,
            reliable_qos,
        )

        self.get_logger().info(
            "HandInput initialized | "
            f"raw={ROS.hand_raw_topic}, "
            f"target={ROS.hand_xyz_topic}, "
            f"mode={ROS.hand_mode_topic}"
        )

    def _raw_callback(self, msg: Point) -> None:
        """
        손 자체의 위치를 저장한다.

        기존 hand_tracker.py에서는 파란색 손 마커에 사용할
        좌표가 /hand_raw로 발행된다.
        """

        position = self._point_to_array(msg)

        if position is None:
            self.get_logger().warning(
                "유효하지 않은 /hand_raw 좌표를 무시했습니다."
            )
            return

        with self._lock:
            self._raw_position = position
            self._raw_received = True

    def _target_callback(self, msg: Point) -> None:
        """
        로봇 End-Effector가 따라갈 목표 위치를 저장한다.

        기존 hand_tracker.py에서 offset과 smoothing이 적용된
        로봇용 좌표가 /hand_xyz로 발행된다.
        """

        position = self._point_to_array(msg)

        if position is None:
            self.get_logger().warning(
                "유효하지 않은 /hand_xyz 좌표를 무시했습니다."
            )
            return

        with self._lock:
            self._target_position = position
            self._target_received = True

    def _mode_callback(self, msg: String) -> None:
        """
        TRACKING 또는 HOME 모드를 저장한다.
        """

        mode = msg.data.strip().upper()

        if not mode:
            self.get_logger().warning(
                "빈 /hand_mode 메시지를 무시했습니다."
            )
            return

        with self._lock:
            previous_mode = self._mode
            self._mode = mode

            # HOME이 처음 들어오는 순간에만 복귀 요청을 생성한다.
            if (
                mode == ROS.return_mode
                and previous_mode != ROS.return_mode
            ):
                self._return_requested = True

        if mode != previous_mode:
            self.get_logger().info(
                f"Hand mode changed: "
                f"{previous_mode} -> {mode}"
            )

    def snapshot(self) -> HandInputSnapshot:
        """
        현재 저장된 모든 입력값을 한 번에 복사해 반환한다.
        """

        with self._lock:
            return HandInputSnapshot(
                raw_position=self._copy_array(
                    self._raw_position
                ),
                target_position=self._copy_array(
                    self._target_position
                ),
                mode=self._mode,
                raw_received=self._raw_received,
                target_received=self._target_received,
            )

    def get_raw_position(
        self,
    ) -> np.ndarray | None:
        """현재 손 위치의 복사본을 반환한다."""

        with self._lock:
            return self._copy_array(
                self._raw_position
            )

    def get_target_position(
        self,
    ) -> np.ndarray | None:
        """현재 로봇 추종 목표 좌표의 복사본을 반환한다."""

        with self._lock:
            return self._copy_array(
                self._target_position
            )

    def get_mode(self) -> str:
        """현재 Hand Tracking 모드를 반환한다."""

        with self._lock:
            return self._mode

    def has_raw_position(self) -> bool:
        """유효한 손 좌표를 한 번 이상 받았는지 반환한다."""

        with self._lock:
            return self._raw_received

    def has_target_position(self) -> bool:
        """유효한 로봇 목표 좌표를 받았는지 반환한다."""

        with self._lock:
            return self._target_received

    def is_tracking(self) -> bool:
        """현재 모드가 TRACKING인지 반환한다."""

        with self._lock:
            return self._mode == ROS.tracking_mode

    def is_home_mode(self) -> bool:
        """현재 모드가 HOME인지 반환한다."""

        with self._lock:
            return self._mode == ROS.return_mode

    def consume_return_request(self) -> bool:
        """
        새 HOME 요청이 있으면 한 번만 True를 반환한다.

        상태 머신이 요청을 한 번 처리한 뒤 같은 HOME 메시지를
        반복해서 처리하지 않도록 요청 플래그를 초기화한다.
        """

        with self._lock:
            requested = self._return_requested
            self._return_requested = False
            return requested

    def clear_raw_position(self) -> None:
        """저장된 손 위치를 초기화한다."""

        with self._lock:
            self._raw_position = None
            self._raw_received = False

    def clear_target_position(self) -> None:
        """저장된 로봇 목표 좌표를 초기화한다."""

        with self._lock:
            self._target_position = None
            self._target_received = False

    def clear_all(self) -> None:
        """저장된 좌표와 요청 상태를 모두 초기화한다."""

        with self._lock:
            self._raw_position = None
            self._target_position = None

            self._raw_received = False
            self._target_received = False
            self._return_requested = False

    @staticmethod
    def _point_to_array(
        msg: Point,
    ) -> np.ndarray | None:
        """
        geometry_msgs/Point를 NumPy 배열로 변환한다.

        NaN 또는 무한대가 포함된 경우 None을 반환한다.
        """

        position = np.array(
            [
                float(msg.x),
                float(msg.y),
                float(msg.z),
            ],
            dtype=np.float64,
        )

        if not np.all(np.isfinite(position)):
            return None

        return position

    @staticmethod
    def _copy_array(
        value: np.ndarray | None,
    ) -> np.ndarray | None:
        """배열이 있으면 복사본을 반환한다."""

        if value is None:
            return None

        return value.copy()


def create_hand_input() -> HandInput:
    """
    ROS2가 초기화되지 않았다면 초기화하고 HandInput을 생성한다.

    application.py 또는 main.py에서 사용할 수 있다.
    """

    if not rclpy.ok():
        rclpy.init(args=None)

    return HandInput()