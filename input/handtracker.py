
"""
MediaPipe 기반 Hand Tracker ROS2 Publisher.

발행 Topic:
- /hand_raw
    실제 손의 변환된 위치

- /hand_xyz
    로봇 End-Effector가 따라갈 목표 위치

- /hand_mode
    TRACKING 또는 HOME

실행:
    프로젝트 루트에서

    python3 -m input.hand_tracker
"""

from __future__ import annotations

import math
import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy

from geometry_msgs.msg import Point
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from config import (
    HAND_TRACKING,
    ROS,
    validate_hand_tracking_files,
)


# ============================================================
# 손 연결선 및 화면 스타일
# ============================================================

HAND_CONNECTIONS = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),

    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),

    (5, 9),
    (9, 10),
    (10, 11),
    (11, 12),

    (9, 13),
    (13, 14),
    (14, 15),
    (15, 16),

    (13, 17),
    (0, 17),
    (17, 18),
    (18, 19),
    (19, 20),
]

HAND_STYLE = {
    "Right": {
        "bone": (60, 220, 180),
        "joint": (0, 255, 200),
        "label_color": (0, 255, 255),
    },
    "Left": {
        "bone": (255, 140, 30),
        "joint": (255, 200, 60),
        "label_color": (255, 150, 0),
    },
}

PALM_LINE_COLOR = (255, 180, 40)


# ============================================================
# 좌표 후처리
# ============================================================

class CoordProcessor:
    def __init__(
        self,
        alpha: float,
    ) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError(
                "alpha는 0보다 크고 1 이하여야 합니다."
            )

        self.alpha = alpha
        self.pos: np.ndarray | None = None

        self.z_floor = (
            HAND_TRACKING.table_height
            - HAND_TRACKING.table_clearance
        )

    def update(
        self,
        raw_x: float,
        raw_y: float,
        raw_z: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        핀홀 계산 좌표를 손 위치와 EE 목표 위치로 변환한다.
        """

        hand_x = raw_x

        hand_y = (
            raw_y
            + HAND_TRACKING.hand_y_offset
        )

        hand_z = (
            raw_z
            + HAND_TRACKING.table_height
        )

        hand_pos = np.array(
            [
                hand_x,
                hand_y,
                hand_z,
            ],
            dtype=np.float64,
        )

        target = np.array(
            [
                hand_x,
                hand_y
                + HAND_TRACKING.ee_y_offset,
                max(
                    hand_z
                    + HAND_TRACKING.ee_z_offset,
                    self.z_floor,
                ),
            ],
            dtype=np.float64,
        )

        if self.pos is None:
            self.pos = target.copy()
        else:
            self.pos = (
                self.alpha * target
                + (1.0 - self.alpha) * self.pos
            )

        return self.pos.copy(), hand_pos

    def reset(self) -> None:
        self.pos = None


# ============================================================
# 제스처 감지
# ============================================================

def is_fist(hand_landmarks) -> bool:
    tips = (8, 12, 16, 20)
    pips = (6, 10, 14, 18)

    curled_count = sum(
        1
        for tip, pip in zip(tips, pips)
        if hand_landmarks[tip].y
        > hand_landmarks[pip].y
    )

    return curled_count >= 3


class OpenCloseTracker:
    def __init__(self) -> None:
        self.last_state: str | None = None
        self.transitions: list[float] = []

    def update(self, fist: bool) -> bool:
        now = time.time()

        current_state = (
            "closed"
            if fist
            else "open"
        )

        if (
            self.last_state is not None
            and current_state != self.last_state
        ):
            self.transitions.append(now)

        self.last_state = current_state

        self.transitions = [
            transition_time
            for transition_time in self.transitions
            if (
                now - transition_time
                < HAND_TRACKING.reactivate_sec
            )
        ]

        return (
            len(self.transitions)
            >= HAND_TRACKING.reactivate_min_transitions
        )

    def reset(self) -> None:
        self.last_state = None
        self.transitions.clear()

    def count(self) -> int:
        return len(self.transitions)


# ============================================================
# 그리기 함수
# ============================================================

def draw_center_banner(
    frame: np.ndarray,
    text: str,
    color: tuple[int, int, int],
    width: int,
    height: int,
) -> None:
    cv2.rectangle(
        frame,
        (0, height // 2 - 50),
        (width, height // 2 + 50),
        (0, 0, 0),
        -1,
    )

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 1.2
    thickness = 3

    (text_width, text_height), _ = (
        cv2.getTextSize(
            text,
            font,
            scale,
            thickness,
        )
    )

    cv2.putText(
        frame,
        text,
        (
            (width - text_width) // 2,
            (height + text_height) // 2,
        ),
        font,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def draw_skeleton(
    frame: np.ndarray,
    hand_landmarks,
    width: int,
    height: int,
    style: dict,
) -> None:
    points = [
        (
            int(landmark.x * width),
            int(landmark.y * height),
        )
        for landmark in hand_landmarks
    ]

    for start_index, end_index in HAND_CONNECTIONS:
        cv2.line(
            frame,
            points[start_index],
            points[end_index],
            (0, 0, 0),
            4,
        )

        cv2.line(
            frame,
            points[start_index],
            points[end_index],
            style["bone"],
            2,
            cv2.LINE_AA,
        )

    for index, point in enumerate(points):
        if index == 9:
            cv2.circle(
                frame,
                point,
                8,
                (0, 0, 0),
                -1,
            )

            cv2.circle(
                frame,
                point,
                6,
                (0, 0, 255),
                -1,
                cv2.LINE_AA,
            )

        elif index == 0:
            cv2.circle(
                frame,
                point,
                6,
                (0, 0, 0),
                -1,
            )

            cv2.circle(
                frame,
                point,
                4,
                style["joint"],
                -1,
                cv2.LINE_AA,
            )

        else:
            cv2.circle(
                frame,
                point,
                5,
                (0, 0, 0),
                -1,
            )

            cv2.circle(
                frame,
                point,
                3,
                style["joint"],
                -1,
                cv2.LINE_AA,
            )


def draw_palm_width_line(
    frame: np.ndarray,
    hand_landmarks,
    width: int,
    height: int,
) -> float:
    x1 = int(hand_landmarks[5].x * width)
    y1 = int(hand_landmarks[5].y * height)

    x2 = int(hand_landmarks[17].x * width)
    y2 = int(hand_landmarks[17].y * height)

    cv2.line(
        frame,
        (x1, y1),
        (x2, y2),
        (0, 0, 0),
        7,
    )

    cv2.line(
        frame,
        (x1, y1),
        (x2, y2),
        PALM_LINE_COLOR,
        3,
        cv2.LINE_AA,
    )

    for endpoint in (
        (x1, y1),
        (x2, y2),
    ):
        cv2.circle(
            frame,
            endpoint,
            7,
            (0, 0, 0),
            -1,
        )

        cv2.circle(
            frame,
            endpoint,
            5,
            (255, 255, 255),
            -1,
            cv2.LINE_AA,
        )

    return math.hypot(
        x2 - x1,
        y2 - y1,
    )


def draw_mode_overlay(
    frame: np.ndarray,
    current_mode: str,
    fist: bool,
    transition_count: int,
    fist_elapsed: float,
    width: int,
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX

    if current_mode == ROS.tracking_mode:
        banner_color = (0, 200, 80)
    else:
        banner_color = (30, 100, 255)

    cv2.rectangle(
        frame,
        (0, 0),
        (width, 40),
        (10, 10, 10),
        -1,
    )

    cv2.putText(
        frame,
        f"MODE: {current_mode}",
        (12, 30),
        font,
        1.0,
        banner_color,
        2,
        cv2.LINE_AA,
    )

    fist_label = (
        "FIST"
        if fist
        else "OPEN"
    )

    fist_color = (
        (0, 60, 255)
        if fist
        else (160, 160, 160)
    )

    cv2.putText(
        frame,
        fist_label,
        (width - 130, 30),
        font,
        0.9,
        fist_color,
        2,
        cv2.LINE_AA,
    )

    bar_y = 48
    bar_height = 12
    bar_width = width - 40

    if (
        current_mode == ROS.tracking_mode
        and fist
        and fist_elapsed > 0.0
    ):
        progress = min(
            fist_elapsed
            / HAND_TRACKING.fist_hold_sec,
            1.0,
        )

        cv2.rectangle(
            frame,
            (20, bar_y),
            (
                20 + bar_width,
                bar_y + bar_height,
            ),
            (50, 50, 50),
            -1,
        )

        cv2.rectangle(
            frame,
            (20, bar_y),
            (
                20 + int(bar_width * progress),
                bar_y + bar_height,
            ),
            (0, 60, 255),
            -1,
        )

        cv2.putText(
            frame,
            (
                "Hold fist to go HOME "
                f"{fist_elapsed:.1f}/"
                f"{HAND_TRACKING.fist_hold_sec:.0f}s"
            ),
            (22, bar_y - 3),
            font,
            0.48,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

    elif current_mode == ROS.return_mode:
        progress = min(
            transition_count
            / HAND_TRACKING.reactivate_min_transitions,
            1.0,
        )

        cv2.rectangle(
            frame,
            (20, bar_y),
            (
                20 + bar_width,
                bar_y + bar_height,
            ),
            (50, 50, 50),
            -1,
        )

        cv2.rectangle(
            frame,
            (20, bar_y),
            (
                20 + int(bar_width * progress),
                bar_y + bar_height,
            ),
            (0, 210, 255),
            -1,
        )

        cv2.putText(
            frame,
            (
                "Open/Close to resume "
                f"{transition_count}/"
                f"{HAND_TRACKING.reactivate_min_transitions}"
            ),
            (22, bar_y - 3),
            font,
            0.48,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )


# ============================================================
# Hand Tracker 실행 클래스
# ============================================================

class HandTrackerPublisher:
    def __init__(self) -> None:
        validate_hand_tracking_files()

        self.node = Node(
            ROS.hand_publisher_node_name
        )

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

        self.raw_publisher = (
            self.node.create_publisher(
                Point,
                ROS.hand_raw_topic,
                stream_qos,
            )
        )

        self.target_publisher = (
            self.node.create_publisher(
                Point,
                ROS.hand_xyz_topic,
                stream_qos,
            )
        )

        self.mode_publisher = (
            self.node.create_publisher(
                String,
                ROS.hand_mode_topic,
                reliable_qos,
            )
        )

        options = vision.HandLandmarkerOptions(
            base_options=python.BaseOptions(
                model_asset_path=str(
                    HAND_TRACKING.model_path
                )
            ),
            num_hands=HAND_TRACKING.num_hands,
        )

        self.detector = (
            vision.HandLandmarker
            .create_from_options(options)
        )

        self.camera = cv2.VideoCapture(
            HAND_TRACKING.camera_index
        )

        if not self.camera.isOpened():
            raise RuntimeError(
                "카메라를 열지 못했습니다. "
                f"camera_index="
                f"{HAND_TRACKING.camera_index}"
            )

        self.camera.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            HAND_TRACKING.camera_width,
        )

        self.camera.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            HAND_TRACKING.camera_height,
        )

        self.processor = CoordProcessor(
            alpha=HAND_TRACKING.smoothing_alpha
        )

        self.open_close_tracker = (
            OpenCloseTracker()
        )

        self.mode = ROS.tracking_mode

        self.near_px: float | None = None
        self.far_px: float | None = None
        self.focal_length: float | None = None

        self.calibration_step = 0
        self.recording = False

        self.fist_hold_start: float | None = None
        self.home_published = False
        self.last_publish_time = 0.0

    def compute_focal(self) -> None:
        if (
            self.near_px is None
            or self.far_px is None
        ):
            return

        focal_near = (
            self.near_px
            * (
                HAND_TRACKING.near_distance_cm
                / 100.0
            )
            / HAND_TRACKING.real_palm_width_m
        )

        focal_far = (
            self.far_px
            * (
                HAND_TRACKING.far_distance_cm
                / 100.0
            )
            / HAND_TRACKING.real_palm_width_m
        )

        self.focal_length = (
            focal_near + focal_far
        ) / 2.0

        print(
            "[CALIB] focal_length = "
            f"{self.focal_length:.1f}"
        )

    def compute_coordinates(
        self,
        palm_width_px: float,
        dx_px: float,
        dy_px: float,
    ) -> tuple[
        float | None,
        float | None,
        float | None,
    ]:
        if (
            self.focal_length is None
            or palm_width_px <= 0.0
        ):
            return None, None, None

        distance_m = (
            self.focal_length
            * HAND_TRACKING.real_palm_width_m
            / palm_width_px
        )

        meter_per_pixel = (
            HAND_TRACKING.real_palm_width_m
            / palm_width_px
        )

        x_m = dx_px * meter_per_pixel
        y_m = -dy_px * meter_per_pixel

        return x_m, y_m, distance_m

    def publish_mode(
        self,
        mode: str,
    ) -> None:
        self.mode_publisher.publish(
            String(data=mode)
        )

    def handle_gesture(
        self,
        right_fist: bool,
        right_hand_detected: bool,
    ) -> None:
        if (
            not self.recording
            or not right_hand_detected
        ):
            return

        if self.mode == ROS.tracking_mode:
            if right_fist:
                if self.fist_hold_start is None:
                    self.fist_hold_start = time.time()

                elif (
                    time.time()
                    - self.fist_hold_start
                    >= HAND_TRACKING.fist_hold_sec
                ):
                    self.mode = ROS.return_mode

                    self.fist_hold_start = None
                    self.home_published = False

                    self.open_close_tracker.reset()

                    self.publish_mode(
                        ROS.return_mode
                    )

                    print(
                        "[MODE] TRACKING -> HOME "
                        "(주먹 유지)"
                    )

            else:
                self.fist_hold_start = None

        elif self.mode == ROS.return_mode:
            if self.open_close_tracker.update(
                right_fist
            ):
                self.mode = ROS.tracking_mode

                self.fist_hold_start = None
                self.home_published = False

                self.open_close_tracker.reset()

                self.publish_mode(
                    ROS.tracking_mode
                )

                print(
                    "[MODE] HOME -> TRACKING "
                    "(손 접기/펴기)"
                )

    def publish_tracking_position(
        self,
        raw_coordinates: tuple[
            float | None,
            float | None,
            float | None,
        ],
        frame: np.ndarray,
    ) -> None:
        raw_x, raw_y, raw_z = raw_coordinates

        if (
            raw_x is None
            or raw_y is None
            or raw_z is None
        ):
            return

        ee_position, hand_position = (
            self.processor.update(
                raw_x,
                raw_y,
                raw_z,
            )
        )

        self.raw_publisher.publish(
            Point(
                x=float(hand_position[0]),
                y=float(hand_position[1]),
                z=float(hand_position[2]),
            )
        )

        now = time.time()

        if (
            now - self.last_publish_time
            >= HAND_TRACKING.publish_interval_sec
        ):
            self.target_publisher.publish(
                Point(
                    x=float(ee_position[0]),
                    y=float(ee_position[1]),
                    z=float(ee_position[2]),
                )
            )

            self.last_publish_time = now

            print(
                f"[HAND] {hand_position.round(2)} "
                f"[EE] {ee_position.round(2)}"
            )

        font = cv2.FONT_HERSHEY_SIMPLEX

        cv2.putText(
            frame,
            "[RIGHT] Pinhole raw",
            (20, 80),
            font,
            0.8,
            (0, 165, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"X: {raw_x:+.3f} m",
            (20, 113),
            font,
            0.75,
            (0, 165, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"Y: {raw_y:+.3f} m",
            (20, 146),
            font,
            0.75,
            (0, 165, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"Z: {raw_z:.3f} m",
            (20, 179),
            font,
            0.75,
            (0, 165, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            "[EE -> PUBLISH]",
            (20, 220),
            font,
            0.8,
            (100, 255, 100),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"X: {ee_position[0]:+.3f} m",
            (20, 253),
            font,
            0.75,
            (100, 255, 100),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"Y: {ee_position[1]:+.3f} m",
            (20, 286),
            font,
            0.75,
            (100, 255, 100),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"Z: {ee_position[2]:.3f} m",
            (20, 319),
            font,
            0.75,
            (100, 255, 100),
            2,
            cv2.LINE_AA,
        )

    def publish_home_position(
        self,
        frame: np.ndarray,
    ) -> None:
        home_x, home_y, home_z = (
            HAND_TRACKING.home_position
        )

        if not self.home_published:
            self.target_publisher.publish(
                Point(
                    x=float(home_x),
                    y=float(home_y),
                    z=float(home_z),
                )
            )

            self.home_published = True

            print(
                "[HOME] EE -> "
                f"{HAND_TRACKING.home_position}"
            )

        font = cv2.FONT_HERSHEY_SIMPLEX

        cv2.putText(
            frame,
            "[HOME] Return position",
            (20, 80),
            font,
            0.8,
            (30, 120, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"X: {home_x:+.3f} m",
            (20, 113),
            font,
            0.75,
            (30, 120, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"Y: {home_y:+.3f} m",
            (20, 146),
            font,
            0.75,
            (30, 120, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            frame,
            f"Z: {home_z:.3f} m",
            (20, 179),
            font,
            0.75,
            (30, 120, 255),
            2,
            cv2.LINE_AA,
        )

    def reset_calibration(self) -> None:
        self.near_px = None
        self.far_px = None
        self.focal_length = None

        self.calibration_step = 0
        self.recording = False

        self.processor.reset()
        self.open_close_tracker.reset()

        self.mode = ROS.tracking_mode
        self.fist_hold_start = None
        self.home_published = False

        print("[CALIB] 재설정")

    def handle_space_key(
        self,
        current_palm_width_px: float,
    ) -> None:
        if (
            self.calibration_step == 0
            and current_palm_width_px > 0.0
        ):
            self.near_px = current_palm_width_px

            print(
                "[CALIB] "
                f"{HAND_TRACKING.near_distance_cm:.0f}cm "
                f"= {self.near_px:.1f}px"
            )

            self.calibration_step = 1
            return

        if (
            self.calibration_step == 1
            and current_palm_width_px > 0.0
        ):
            self.far_px = current_palm_width_px

            print(
                "[CALIB] "
                f"{HAND_TRACKING.far_distance_cm:.0f}cm "
                f"= {self.far_px:.1f}px"
            )

            self.compute_focal()

            self.calibration_step = 2
            return

        if (
            self.calibration_step == 2
            and not self.recording
        ):
            self.recording = True
            self.mode = ROS.tracking_mode

            self.publish_mode(
                ROS.tracking_mode
            )

            print(
                "[REC] 발행 시작 | "
                f"mode={ROS.tracking_mode}"
            )

    def run(self) -> None:
        print("=" * 60)
        print("M0609 MediaPipe Hand Tracker")
        print("SPACE: calibration / start")
        print("R: reset")
        print("ESC: exit")
        print(
            "Topics: "
            f"{ROS.hand_raw_topic}, "
            f"{ROS.hand_xyz_topic}, "
            f"{ROS.hand_mode_topic}"
        )
        print("=" * 60)

        while rclpy.ok():
            success, frame = self.camera.read()

            if not success:
                print("[ERROR] Camera frame read failed.")
                break

            frame = cv2.resize(
                frame,
                (
                    HAND_TRACKING.camera_width,
                    HAND_TRACKING.camera_height,
                ),
            )

            frame = cv2.flip(
                frame,
                1,
            )

            height, width, _ = frame.shape

            center_x = width // 2
            center_y = height // 2

            rgb_frame = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2RGB,
            )

            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=rgb_frame,
            )

            result = self.detector.detect(
                mp_image
            )

            cv2.circle(
                frame,
                (center_x, center_y),
                6,
                (0, 0, 0),
                -1,
            )

            cv2.circle(
                frame,
                (center_x, center_y),
                4,
                (255, 255, 255),
                -1,
                cv2.LINE_AA,
            )

            current_palm_width_px = 0.0

            right_coordinates = (
                None,
                None,
                None,
            )

            right_hand_detected = False
            right_fist = False

            font = cv2.FONT_HERSHEY_SIMPLEX

            if result.hand_landmarks:
                for hand_index, hand_landmarks in enumerate(
                    result.hand_landmarks
                ):
                    raw_label = (
                        result.handedness[
                            hand_index
                        ][0].category_name
                    )

                    # 화면을 좌우 반전했기 때문에 보정
                    real_label = (
                        "Right"
                        if raw_label == "Left"
                        else "Left"
                    )

                    style = HAND_STYLE[
                        real_label
                    ]

                    draw_skeleton(
                        frame,
                        hand_landmarks,
                        width,
                        height,
                        style,
                    )

                    palm_width_px = (
                        draw_palm_width_line(
                            frame,
                            hand_landmarks,
                            width,
                            height,
                        )
                    )

                    center_landmark = (
                        hand_landmarks[9]
                    )

                    pixel_x = int(
                        center_landmark.x * width
                    )

                    pixel_y = int(
                        center_landmark.y * height
                    )

                    dx_px = pixel_x - center_x
                    dy_px = pixel_y - center_y

                    cv2.line(
                        frame,
                        (center_x, center_y),
                        (pixel_x, pixel_y),
                        (200, 200, 200),
                        1,
                    )

                    cv2.putText(
                        frame,
                        real_label,
                        (
                            pixel_x + 15,
                            pixel_y,
                        ),
                        font,
                        0.7,
                        style["label_color"],
                        2,
                        cv2.LINE_AA,
                    )

                    if real_label == "Right":
                        current_palm_width_px = (
                            palm_width_px
                        )

                        right_coordinates = (
                            self.compute_coordinates(
                                palm_width_px,
                                dx_px,
                                dy_px,
                            )
                        )

                        right_hand_detected = True

                        right_fist = is_fist(
                            hand_landmarks
                        )

            self.handle_gesture(
                right_fist=right_fist,
                right_hand_detected=(
                    right_hand_detected
                ),
            )

            if self.recording:
                if self.mode == ROS.tracking_mode:
                    if (
                        right_coordinates[0]
                        is not None
                    ):
                        self.publish_tracking_position(
                            raw_coordinates=(
                                right_coordinates
                            ),
                            frame=frame,
                        )
                    else:
                        cv2.putText(
                            frame,
                            "RIGHT hand not detected",
                            (20, 110),
                            font,
                            0.7,
                            (0, 0, 255),
                            2,
                            cv2.LINE_AA,
                        )

                elif self.mode == ROS.return_mode:
                    self.publish_home_position(
                        frame
                    )

                fist_elapsed = (
                    time.time()
                    - self.fist_hold_start
                    if self.fist_hold_start
                    is not None
                    else 0.0
                )

                draw_mode_overlay(
                    frame=frame,
                    current_mode=self.mode,
                    fist=right_fist,
                    transition_count=(
                        self.open_close_tracker
                        .count()
                    ),
                    fist_elapsed=fist_elapsed,
                    width=width,
                )

            if self.calibration_step == 0:
                draw_center_banner(
                    frame,
                    (
                        "RIGHT hand at "
                        f"{HAND_TRACKING.near_distance_cm:.0f}cm, "
                        "press SPACE"
                    ),
                    (0, 255, 255),
                    width,
                    height,
                )

            elif self.calibration_step == 1:
                draw_center_banner(
                    frame,
                    (
                        "RIGHT hand at "
                        f"{HAND_TRACKING.far_distance_cm:.0f}cm, "
                        "press SPACE"
                    ),
                    (0, 255, 255),
                    width,
                    height,
                )

            elif (
                self.calibration_step == 2
                and not self.recording
            ):
                draw_center_banner(
                    frame,
                    "SPACE -> Start Publishing",
                    (0, 255, 0),
                    width,
                    height,
                )

            display_frame = cv2.resize(
                frame,
                None,
                fx=HAND_TRACKING.display_scale,
                fy=HAND_TRACKING.display_scale,
            )

            cv2.imshow(
                "Hand Tracker",
                display_frame,
            )

            rclpy.spin_once(
                self.node,
                timeout_sec=0.0,
            )

            key = cv2.waitKey(1) & 0xFF

            if key == 32:
                self.handle_space_key(
                    current_palm_width_px
                )

            elif key in (
                ord("r"),
                ord("R"),
            ):
                self.reset_calibration()

            elif key == 27:
                break

    def shutdown(self) -> None:
        try:
            self.camera.release()
        finally:
            cv2.destroyAllWindows()

        try:
            self.detector.close()
        except Exception:
            pass

        try:
            self.node.destroy_node()
        except Exception:
            pass


# ============================================================
# 실행 진입점
# ============================================================

def main() -> None:
    if not rclpy.ok():
        rclpy.init(args=None)

    tracker: HandTrackerPublisher | None = None

    try:
        tracker = HandTrackerPublisher()
        tracker.run()

    except KeyboardInterrupt:
        print("[HandTracker] KeyboardInterrupt")

    finally:
        if tracker is not None:
            tracker.shutdown()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()