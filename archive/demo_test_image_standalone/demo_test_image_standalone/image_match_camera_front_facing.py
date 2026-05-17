from dataclasses import dataclass
import math
import time
from typing import Optional

import cv2
import numpy as np

from image_match_camera import (
    DEFAULT_MATCH_CONF_PERCENT,
    GREEN_CONFIRM_FRAMES,
    MIN_INLIERS,
    ORB_FEATURES,
    ORB_RATIO_TEST,
    RATIO_TEST,
    RED_CONFIRM_FRAMES,
    ROI_REACQUIRE_AFTER_MISSES,
    SIFT_FEATURES,
    build_feature_backend,
    load_target_image,
    match_with_fallback,
    open_camera,
    polygon_to_roi,
    preferred_camera_candidates,
    put_text,
    select_backends,
    update_stable_state,
)


CONTROL_WINDOW = "Front Facing Controls"
DEFAULT_FRONT_SCORE_PERCENT = 72

# These tolerances define how close the matched quadrilateral must be to a
# real rectangle. A front-facing target should have near-90-degree corners,
# mostly parallel opposite edges, and similar opposite edge lengths.
MAX_ANGLE_ERROR_DEGREES = 18.0
MAX_PARALLEL_ERROR_DEGREES = 16.0
MIN_OPPOSITE_EDGE_RATIO = 0.68
MAX_ASPECT_LOG_ERROR = math.log(1.45)
SIDE_BIAS_DEADBAND = 0.06


@dataclass
class FrontFacingResult:
    score: float
    max_angle_error: float
    max_parallel_error: float
    top_bottom_ratio: float
    left_right_ratio: float
    aspect_error_percent: float
    side_bias: str
    side_bias_percent: float
    is_front_facing: bool = False


def clamp01(value):
    return max(0.0, min(float(value), 1.0))


def vector_angle_degrees(a, b):
    norm_a = float(np.linalg.norm(a))
    norm_b = float(np.linalg.norm(b))
    if norm_a < 1e-6 or norm_b < 1e-6:
        return 180.0

    cosine = float(np.dot(a, b) / (norm_a * norm_b))
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def parallel_error_degrees(a, b):
    angle = vector_angle_degrees(a, b)
    return min(angle, abs(180.0 - angle))


def ratio_score(first, second, minimum_ratio):
    larger = max(float(first), float(second), 1e-6)
    smaller = min(float(first), float(second))
    ratio = smaller / larger
    return ratio, clamp01((ratio - minimum_ratio) / (1.0 - minimum_ratio))


def estimate_side_bias(left_len, right_len):
    larger = max(float(left_len), float(right_len), 1e-6)
    diff_ratio = abs(float(left_len) - float(right_len)) / larger
    if diff_ratio < SIDE_BIAS_DEADBAND:
        return "UNCLEAR", diff_ratio * 100.0

    # The shorter vertical edge is usually the farther side of the tilted book.
    if left_len < right_len:
        return "LEFT", diff_ratio * 100.0
    return "RIGHT", diff_ratio * 100.0


def error_score(error, tolerance):
    return clamp01(1.0 - float(error) / float(tolerance))


def evaluate_front_facing(polygon, target_shape, front_score_threshold):
    points = polygon.reshape(-1, 2).astype(np.float32)
    if len(points) != 4:
        return None

    top_left, top_right, bottom_right, bottom_left = points

    top = top_right - top_left
    bottom = bottom_right - bottom_left
    left = bottom_left - top_left
    right = bottom_right - top_right

    top_len = float(np.linalg.norm(top))
    bottom_len = float(np.linalg.norm(bottom))
    left_len = float(np.linalg.norm(left))
    right_len = float(np.linalg.norm(right))
    if min(top_len, bottom_len, left_len, right_len) < 1e-6:
        return None

    corner_angles = [
        vector_angle_degrees(top_right - top_left, bottom_left - top_left),
        vector_angle_degrees(top_left - top_right, bottom_right - top_right),
        vector_angle_degrees(top_right - bottom_right, bottom_left - bottom_right),
        vector_angle_degrees(top_left - bottom_left, bottom_right - bottom_left),
    ]
    max_angle_error = max(abs(angle - 90.0) for angle in corner_angles)

    top_bottom_parallel_error = parallel_error_degrees(top, bottom)
    left_right_parallel_error = parallel_error_degrees(left, right)
    max_parallel_error = max(top_bottom_parallel_error, left_right_parallel_error)

    top_bottom_ratio, top_bottom_score = ratio_score(
        top_len, bottom_len, MIN_OPPOSITE_EDGE_RATIO
    )
    left_right_ratio, left_right_score = ratio_score(
        left_len, right_len, MIN_OPPOSITE_EDGE_RATIO
    )
    side_bias, side_bias_percent = estimate_side_bias(left_len, right_len)
    opposite_edge_score = (top_bottom_score + left_right_score) / 2.0

    detected_aspect = ((top_len + bottom_len) / 2.0) / ((left_len + right_len) / 2.0)
    target_h, target_w = target_shape[:2]
    target_aspect = float(target_w) / max(float(target_h), 1.0)
    aspect_log_error = abs(math.log(max(detected_aspect, 1e-6) / target_aspect))
    aspect_error_percent = abs(detected_aspect / target_aspect - 1.0) * 100.0
    aspect_score = error_score(aspect_log_error, MAX_ASPECT_LOG_ERROR)

    angle_score = error_score(max_angle_error, MAX_ANGLE_ERROR_DEGREES)
    parallel_score = error_score(max_parallel_error, MAX_PARALLEL_ERROR_DEGREES)

    score = (
        0.35 * angle_score
        + 0.25 * parallel_score
        + 0.25 * opposite_edge_score
        + 0.15 * aspect_score
    )

    return FrontFacingResult(
        score=score,
        max_angle_error=max_angle_error,
        max_parallel_error=max_parallel_error,
        top_bottom_ratio=top_bottom_ratio,
        left_right_ratio=left_right_ratio,
        aspect_error_percent=aspect_error_percent,
        side_bias=side_bias,
        side_bias_percent=side_bias_percent,
        is_front_facing=score >= front_score_threshold,
    )


def create_front_control_panel():
    cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROL_WINDOW, 420, 140)
    cv2.createTrackbar(
        "MatchConf%", CONTROL_WINDOW, DEFAULT_MATCH_CONF_PERCENT, 100, lambda value: None
    )
    cv2.createTrackbar(
        "FrontScore%",
        CONTROL_WINDOW,
        DEFAULT_FRONT_SCORE_PERCENT,
        100,
        lambda value: None,
    )


def read_front_control_panel():
    match_conf_percent = cv2.getTrackbarPos("MatchConf%", CONTROL_WINDOW)
    front_score_percent = cv2.getTrackbarPos("FrontScore%", CONTROL_WINDOW)
    match_conf_threshold = match_conf_percent / 100.0
    front_score_threshold = max(front_score_percent / 100.0, 0.01)
    return match_conf_threshold, front_score_threshold


def draw_polygon_overlay(frame, polygon, color):
    if polygon is None:
        return frame

    overlay = frame.copy()
    cv2.fillPoly(overlay, [polygon], color)
    frame = cv2.addWeighted(overlay, 0.16, frame, 0.84, 0)
    cv2.polylines(frame, [polygon], True, color, 3, cv2.LINE_AA)

    for point in polygon.reshape(-1, 2):
        cv2.circle(frame, (int(point[0]), int(point[1])), 5, color, -1, cv2.LINE_AA)

    return frame


def draw_status_border(frame, color):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w - 1, h - 1), color, 10)


def main():
    cv2.setUseOptimized(True)

    target_image = load_target_image()
    orb_backend = build_feature_backend(
        "ORB",
        cv2.ORB_create(nfeatures=ORB_FEATURES),
        cv2.NORM_HAMMING,
        ORB_RATIO_TEST,
        target_image,
    )
    sift_backend = build_feature_backend(
        "SIFT",
        cv2.SIFT_create(nfeatures=SIFT_FEATURES),
        cv2.NORM_L2,
        RATIO_TEST,
        target_image,
    )
    create_front_control_panel()

    camera_candidates = preferred_camera_candidates()
    cap = open_camera(camera_candidates)
    if cap is None:
        raise SystemExit(
            "Could not open camera. Check camera permission and whether another app is using it."
        )

    previous_time = time.time()
    window_name = "Image Match Front Facing - press q or Esc to quit"
    print("Camera opened. Put the target image in view. Green means front-facing.")
    print(f"Front-facing check uses matched polygon geometry. MIN_INLIERS={MIN_INLIERS}")

    stable_front_facing = False
    green_count = 0
    red_count = 0
    roi_rect = None
    roi_miss_streak = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read camera frame.")
            break

        match_conf_threshold, front_score_threshold = read_front_control_panel()

        using_roi = roi_rect is not None and roi_miss_streak < ROI_REACQUIRE_AFTER_MISSES
        active_backends = select_backends(orb_backend, sift_backend, using_roi)
        match_result = match_with_fallback(
            frame, active_backends, roi_rect if using_roi else None, match_conf_threshold
        )

        if match_result.accepted:
            roi_rect = polygon_to_roi(match_result.polygon, frame.shape)
            roi_miss_streak = 0
        elif using_roi:
            roi_miss_streak += 1
            if roi_miss_streak >= ROI_REACQUIRE_AFTER_MISSES:
                full_backends = select_backends(orb_backend, sift_backend, False)
                match_result = match_with_fallback(
                    frame, full_backends, None, match_conf_threshold
                )
                if match_result.accepted:
                    roi_rect = polygon_to_roi(match_result.polygon, frame.shape)
                    roi_miss_streak = 0
                else:
                    roi_rect = None
                    roi_miss_streak = 0
        elif match_result.search_scope == "FULL" and not match_result.accepted:
            roi_rect = None

        polygon = match_result.polygon
        front_result: Optional[FrontFacingResult] = None
        if polygon is not None:
            front_result = evaluate_front_facing(
                polygon, target_image.shape, front_score_threshold
            )

        candidate_front_facing = (
            match_result.accepted
            and front_result is not None
            and front_result.is_front_facing
        )
        stable_front_facing, green_count, red_count = update_stable_state(
            stable_front_facing, candidate_front_facing, green_count, red_count
        )

        color = (0, 255, 0) if stable_front_facing else (0, 0, 255)
        if polygon is not None:
            frame = draw_polygon_overlay(frame, polygon, color)
        draw_status_border(frame, color)

        if polygon is None:
            status = (
                f"{match_result.backend_name} {match_result.search_scope} | NOT FOUND "
                f"| matches={match_result.good_count} inliers={match_result.inlier_count}"
            )
            text_color = (0, 0, 255)
        elif match_result.match_confidence < match_conf_threshold:
            status = (
                f"{match_result.backend_name} {match_result.search_scope} | LOW MATCH "
                f"| conf={match_result.match_confidence*100:.1f}% < {match_conf_threshold*100:.1f}% "
                f"| matches={match_result.good_count} inliers={match_result.inlier_count}"
            )
            text_color = (0, 0, 255)
        elif stable_front_facing:
            status = (
                f"{match_result.backend_name} {match_result.search_scope} | FRONT FACING "
                f"| conf={match_result.match_confidence*100:.1f}%"
            )
            text_color = (0, 255, 0)
        elif front_result is not None and front_result.side_bias != "UNCLEAR":
            status = (
                f"{match_result.backend_name} {match_result.search_scope} | TILTED {front_result.side_bias} "
                f"| conf={match_result.match_confidence*100:.1f}%"
            )
            text_color = (0, 0, 255)
        else:
            status = (
                f"{match_result.backend_name} {match_result.search_scope} | TILTED "
                f"| conf={match_result.match_confidence*100:.1f}%"
            )
            text_color = (0, 0, 255)

        now = time.time()
        fps = 1.0 / max(now - previous_time, 1e-6)
        previous_time = now

        put_text(frame, status, (20, 36), text_color)
        put_text(
            frame,
            f"Match threshold: {match_conf_threshold*100:.0f}% | Front score threshold: {front_score_threshold*100:.0f}%",
            (20, 72),
            (0, 255, 255),
        )

        if front_result is not None:
            put_text(
                frame,
                f"Front score: {front_result.score*100:.1f}% | angle err: {front_result.max_angle_error:.1f} deg | parallel err: {front_result.max_parallel_error:.1f} deg",
                (20, 108),
                (255, 255, 255),
            )
            put_text(
                frame,
                f"Tilt side: {front_result.side_bias} | side diff={front_result.side_bias_percent:.1f}% | edge ratios T/B={front_result.top_bottom_ratio:.2f} L/R={front_result.left_right_ratio:.2f}",
                (20, 144),
                (255, 255, 255),
            )
        else:
            put_text(frame, "Front score: N/A", (20, 108), (255, 255, 255))

        put_text(
            frame,
            f"Candidate: {'YES' if candidate_front_facing else 'NO'} | green={green_count}/{GREEN_CONFIRM_FRAMES} red={red_count}/{RED_CONFIRM_FRAMES}",
            (20, 180),
            (255, 255, 255),
        )
        put_text(frame, f"FPS: {fps:.1f}", (20, 216), (255, 255, 255))

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
