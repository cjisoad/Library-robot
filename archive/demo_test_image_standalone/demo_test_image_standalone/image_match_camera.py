from dataclasses import dataclass
from pathlib import Path
import platform
import time
from typing import Optional

import cv2
import numpy as np


BASE_DIR = Path(__file__).resolve().parent
TARGET_IMAGE = BASE_DIR / "test1.jpg"
CONTROL_WINDOW = "Controls"
PREFERRED_CAMERA_NAMES = ("USB", "UVC")

MIN_GOOD_MATCHES = 18
RATIO_TEST = 0.72
MIN_INLIERS = 12
DEFAULT_MATCH_CONF_PERCENT = 65
DEFAULT_CENTER_TOL_PERCENT = 3
FULL_FRAME_MAX_SIDE = 960
ROI_FRAME_MAX_SIDE = 960
TARGET_MAX_SIDE = 1000
CENTER_SMOOTHING_ALPHA = 0.25
GREEN_CONFIRM_FRAMES = 2
RED_CONFIRM_FRAMES = 5
MIN_CENTER_TOL_PX = 45
ORB_FEATURES = 1600
SIFT_FEATURES = 2500
ORB_RATIO_TEST = 0.82
ROI_EXPAND_RATIO = 0.45
ROI_MIN_PAD = 60
ROI_REACQUIRE_AFTER_MISSES = 3
USE_SIFT_IN_FULL_SEARCH = True
USE_SIFT_IN_ROI = True


@dataclass
class FeatureBackend:
    name: str
    detector: object
    matcher: object
    ratio_test: float
    min_good_matches: int
    min_inliers: int
    target_keypoints: list
    target_descriptors: np.ndarray
    target_corners: np.ndarray


@dataclass
class MatchResult:
    polygon: Optional[np.ndarray]
    good_count: int
    inlier_count: int
    match_confidence: float
    backend_name: str
    search_scope: str
    accepted: bool = False


@dataclass
class CameraCandidate:
    source: object
    label: str
    backend: Optional[int] = None


def resize_keep_aspect(image, max_side):
    h, w = image.shape[:2]
    scale = min(max_side / max(h, w), 1.0)
    if scale >= 1.0:
        return image, 1.0
    resized = cv2.resize(image, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    return resized, scale


def put_text(image, text, origin, color):
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)


def create_control_panel():
    cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROL_WINDOW, 420, 140)
    cv2.createTrackbar("MatchConf%", CONTROL_WINDOW, DEFAULT_MATCH_CONF_PERCENT, 100, lambda value: None)
    cv2.createTrackbar("CenterTol%", CONTROL_WINDOW, DEFAULT_CENTER_TOL_PERCENT, 30, lambda value: None)


def read_control_panel():
    match_conf_percent = cv2.getTrackbarPos("MatchConf%", CONTROL_WINDOW)
    center_tol_percent = cv2.getTrackbarPos("CenterTol%", CONTROL_WINDOW)
    match_conf_threshold = match_conf_percent / 100.0
    center_tol_ratio = max(center_tol_percent / 100.0, 0.005)
    return match_conf_threshold, center_tol_ratio


def load_target_image():
    target = cv2.imread(str(TARGET_IMAGE))
    if target is None:
        raise SystemExit(f"无法读取目标图片: {TARGET_IMAGE}")
    return resize_keep_aspect(target, TARGET_MAX_SIDE)[0]


def build_feature_backend(name, detector, norm_type, ratio_test, target_image):
    gray = cv2.cvtColor(target_image, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = detector.detectAndCompute(gray, None)

    if descriptors is None or len(keypoints) < MIN_GOOD_MATCHES:
        raise SystemExit(f"{name} 目标图片特征点太少，无法做图像匹配。")

    h, w = target_image.shape[:2]
    corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    print(f"目标图片 {name} 特征点: {len(keypoints)}")

    return FeatureBackend(
        name=name,
        detector=detector,
        matcher=cv2.BFMatcher(norm_type),
        ratio_test=ratio_test,
        min_good_matches=MIN_GOOD_MATCHES,
        min_inliers=MIN_INLIERS,
        target_keypoints=keypoints,
        target_descriptors=descriptors,
        target_corners=corners,
    )


def list_avfoundation_video_devices():
    try:
        from AVFoundation import AVCaptureDevice, AVMediaTypeVideo
    except Exception:
        return []

    devices = []
    for device in AVCaptureDevice.devicesWithMediaType_(AVMediaTypeVideo):
        try:
            name = str(device.localizedName())
        except Exception:
            name = ""
        try:
            unique_id = str(device.uniqueID())
        except Exception:
            unique_id = ""
        devices.append((name, unique_id))
    return devices


def list_linux_video_devices():
    devices = []
    sys_class = Path("/sys/class/video4linux")

    for device_path in sorted(Path("/dev").glob("video*")):
        if not device_path.name[5:].isdigit():
            continue

        label = device_path.name
        sys_name_path = sys_class / device_path.name / "name"
        try:
            if sys_name_path.exists():
                sys_label = sys_name_path.read_text(encoding="utf-8", errors="ignore").strip()
                if sys_label:
                    label = sys_label
        except Exception:
            pass

        devices.append((str(device_path), label))

    by_id_dir = Path("/dev/v4l/by-id")
    if by_id_dir.exists():
        for link in sorted(by_id_dir.iterdir()):
            if not link.is_symlink():
                continue
            try:
                target = link.resolve()
            except Exception:
                continue
            if not target.name.startswith("video"):
                continue

            label = link.name
            sys_name_path = sys_class / target.name / "name"
            try:
                if sys_name_path.exists():
                    sys_label = sys_name_path.read_text(encoding="utf-8", errors="ignore").strip()
                    if sys_label:
                        label = f"{label} -> {sys_label}"
            except Exception:
                pass

            devices.append((str(link), label))

    return devices


def preferred_camera_candidates():
    system_name = platform.system().lower()

    if system_name == "linux":
        video_devices = list_linux_video_devices()
        if video_devices:
            print("检测到 Linux 视频设备:")
            for index, (source, label) in enumerate(video_devices):
                print(f"  [{index}] {label} ({source})")

            preferred = []
            fallback = []
            for source, label in video_devices:
                candidate = CameraCandidate(source, label, cv2.CAP_V4L2)
                device_label = f"{label} {source}".upper()
                if any(keyword.upper() in device_label for keyword in PREFERRED_CAMERA_NAMES):
                    preferred.append(candidate)
                else:
                    fallback.append(candidate)
            return preferred + fallback

        print("未发现 /dev/video* 设备，回退尝试 /dev/video0。")
        return [CameraCandidate("/dev/video0", "video0", cv2.CAP_V4L2)]

    video_devices = list_avfoundation_video_devices()
    if video_devices:
        print("检测到视频设备:")
        candidates = []
        for index, (name, unique_id) in enumerate(video_devices):
            print(f"  [{index}] {name} ({unique_id})")
            candidates.append(CameraCandidate(index, name))

        for index, (name, unique_id) in enumerate(video_devices):
            device_label = f"{name} {unique_id}".upper()
            if any(keyword.upper() in device_label for keyword in PREFERRED_CAMERA_NAMES):
                print(f"优先使用外接摄像头: [{index}] {name}")
                candidates.insert(0, candidates.pop(index))
                break
        return candidates

    print("未自动匹配到摄像头，回退使用摄像头 0。")
    return [CameraCandidate(0, "camera 0")]


def configure_camera(cap, width=1280, height=720, fps=30):
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    except Exception:
        pass
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
    except Exception:
        pass


def verify_capture_stream(cap, attempts=6, pause_seconds=0.05):
    for _ in range(attempts):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size:
            return True
        time.sleep(pause_seconds)
    return False


def camera_backend_name(cap):
    try:
        return cap.getBackendName()
    except Exception:
        return "unknown"


def open_camera(candidates):
    for candidate in candidates:
        print(f"尝试摄像头: {candidate.label} ({candidate.source})")
        cap = (
            cv2.VideoCapture(candidate.source, candidate.backend)
            if candidate.backend is not None
            else cv2.VideoCapture(candidate.source)
        )

        if not cap.isOpened():
            cap.release()
            print(f"  打不开，跳过。")
            continue

        configure_camera(cap, width=1280, height=720, fps=30)
        if verify_capture_stream(cap):
            print(f"已打开摄像头: {candidate.label} [{camera_backend_name(cap)}]")
            return cap

        cap.release()
        print(f"  能打开但读不到有效画面，跳过。")

    return None


def clamp_rect(x0, y0, x1, y1, frame_width, frame_height):
    x0 = max(0, min(int(x0), frame_width - 1))
    y0 = max(0, min(int(y0), frame_height - 1))
    x1 = max(x0 + 1, min(int(x1), frame_width))
    y1 = max(y0 + 1, min(int(y1), frame_height))
    return x0, y0, x1, y1


def polygon_to_roi(polygon, frame_shape):
    h, w = frame_shape[:2]
    points = polygon.reshape(-1, 2).astype(np.float32)
    xs = points[:, 0]
    ys = points[:, 1]
    left_x = float(xs.min())
    right_x = float(xs.max())
    top_y = float(ys.min())
    bottom_y = float(ys.max())
    box_w = max(right_x - left_x, 1.0)
    box_h = max(bottom_y - top_y, 1.0)
    pad_x = max(int(box_w * ROI_EXPAND_RATIO), ROI_MIN_PAD)
    pad_y = max(int(box_h * ROI_EXPAND_RATIO), ROI_MIN_PAD)
    return clamp_rect(left_x - pad_x, top_y - pad_y, right_x + pad_x, bottom_y + pad_y, w, h)


def match_target(frame, backend, search_rect=None):
    if search_rect is None:
        search_frame = frame
        offset_x = 0
        offset_y = 0
        search_scope = "FULL"
    else:
        x0, y0, x1, y1 = search_rect
        search_frame = frame[y0:y1, x0:x1]
        offset_x = x0
        offset_y = y0
        search_scope = "ROI"

    max_side = ROI_FRAME_MAX_SIDE if search_rect is not None else FULL_FRAME_MAX_SIDE
    frame_small, frame_scale = resize_keep_aspect(search_frame, max_side)
    gray = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)
    frame_keypoints, frame_descriptors = backend.detector.detectAndCompute(gray, None)

    if frame_descriptors is None or len(frame_keypoints) < backend.min_good_matches:
        return MatchResult(None, 0, 0, 0.0, backend.name, search_scope)

    raw_matches = backend.matcher.knnMatch(backend.target_descriptors, frame_descriptors, k=2)
    good_matches = []
    for pair in raw_matches:
        if len(pair) < 2:
            continue
        first, second = pair
        if first.distance < backend.ratio_test * second.distance:
            good_matches.append(first)

    if len(good_matches) < backend.min_good_matches:
        return MatchResult(None, len(good_matches), 0, 0.0, backend.name, search_scope)

    src_pts = np.float32([backend.target_keypoints[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([frame_keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    matrix, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

    if matrix is None or mask is None:
        return MatchResult(None, len(good_matches), 0, 0.0, backend.name, search_scope)

    inliers = int(mask.ravel().sum())
    match_confidence = inliers / max(len(good_matches), 1)
    if inliers < backend.min_inliers:
        return MatchResult(None, len(good_matches), inliers, match_confidence, backend.name, search_scope)

    projected = cv2.perspectiveTransform(backend.target_corners, matrix)
    projected = projected / frame_scale
    if offset_x or offset_y:
        projected[:, 0, 0] += offset_x
        projected[:, 0, 1] += offset_y

    return MatchResult(projected.astype(np.int32), len(good_matches), inliers, match_confidence, backend.name, search_scope)


def match_with_fallback(frame, backends, search_rect, match_conf_threshold):
    best_result = None
    for backend in backends:
        result = match_target(frame, backend, search_rect)
        result.accepted = result.polygon is not None and result.match_confidence >= match_conf_threshold
        if result.accepted:
            return result
        if best_result is None:
            best_result = result
        elif result.polygon is not None and result.match_confidence > best_result.match_confidence:
            best_result = result
        elif best_result.polygon is None and result.inlier_count > best_result.inlier_count:
            best_result = result

    if best_result is None:
        search_scope = "ROI" if search_rect is not None else "FULL"
        return MatchResult(None, 0, 0, 0.0, "NONE", search_scope, False)

    return best_result


def select_backends(orb_backend, sift_backend, use_roi):
    if use_roi and USE_SIFT_IN_ROI:
        return (orb_backend, sift_backend)
    if not use_roi and USE_SIFT_IN_FULL_SEARCH:
        return (orb_backend, sift_backend)
    return (orb_backend,)


def evaluate_center_alignment(frame_shape, polygon, center_tol_ratio, smoothed_center_x=None):
    h, w = frame_shape[:2]
    frame_center_x = w / 2.0
    points = polygon.reshape(-1, 2).astype(np.float32)
    xs = points[:, 0]
    left_x = float(xs.min())
    right_x = float(xs.max())
    detected_center_x = (left_x + right_x) / 2.0
    book_center_x = smoothed_center_x if smoothed_center_x is not None else detected_center_x
    offset_px = abs(book_center_x - frame_center_x)
    tolerance_px = max(w * center_tol_ratio, MIN_CENTER_TOL_PX)
    centered = offset_px <= tolerance_px
    center_score = max(0.0, 1.0 - offset_px / tolerance_px)
    return centered, center_score, offset_px, tolerance_px, frame_center_x, book_center_x, detected_center_x, left_x, right_x


def update_stable_state(current_state, candidate_state, green_count, red_count):
    if candidate_state:
        green_count += 1
        red_count = 0
        if green_count >= GREEN_CONFIRM_FRAMES:
            current_state = True
    else:
        red_count += 1
        green_count = 0
        if red_count >= RED_CONFIRM_FRAMES:
            current_state = False

    return current_state, green_count, red_count


def draw_center_guides(frame, centered, center_tol_ratio):
    h, w = frame.shape[:2]
    center_x = w // 2
    tolerance_px = max(int(w * center_tol_ratio), MIN_CENTER_TOL_PX)
    left_x = max(center_x - tolerance_px, 0)
    right_x = min(center_x + tolerance_px, w - 1)
    status_color = (0, 255, 0) if centered else (0, 0, 255)

    overlay = frame.copy()
    cv2.rectangle(overlay, (left_x, 0), (right_x, h), status_color, -1)
    frame = cv2.addWeighted(overlay, 0.08, frame, 0.92, 0)

    cv2.line(frame, (left_x, 0), (left_x, h), (220, 220, 220), 1)
    cv2.line(frame, (right_x, 0), (right_x, h), (220, 220, 220), 1)
    cv2.rectangle(frame, (center_x - 4, 0), (center_x + 4, h), status_color, -1)
    return frame


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
    create_control_panel()

    camera_candidates = preferred_camera_candidates()
    cap = open_camera(camera_candidates)
    if cap is None:
        raise SystemExit("无法打开摄像头，请确认摄像头权限已开启，且外接摄像头未被其他软件占用。")

    previous_time = time.time()
    window_name = "Image Match Camera - press q or Esc to quit"
    print("已打开摄像头。把 test1.jpg 对应的书/图片放进画面，按 q 或 Esc 退出。")
    stable_centered = False
    green_count = 0
    red_count = 0
    smoothed_book_center_x = None
    roi_rect = None
    roi_miss_streak = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("读取摄像头画面失败。")
            break

        match_conf_threshold, center_tol_ratio = read_control_panel()

        using_roi = roi_rect is not None and roi_miss_streak < ROI_REACQUIRE_AFTER_MISSES
        active_backends = select_backends(orb_backend, sift_backend, using_roi)
        match_result = match_with_fallback(frame, active_backends, roi_rect if using_roi else None, match_conf_threshold)

        if match_result.accepted:
            roi_miss_streak = 0
        elif using_roi:
            roi_miss_streak += 1
            if roi_miss_streak >= ROI_REACQUIRE_AFTER_MISSES:
                full_backends = select_backends(orb_backend, sift_backend, False)
                match_result = match_with_fallback(frame, full_backends, None, match_conf_threshold)
                if match_result.accepted:
                    roi_miss_streak = 0
                else:
                    roi_rect = None
                    roi_miss_streak = 0
        elif match_result.search_scope == "FULL" and not match_result.accepted:
            roi_rect = None

        polygon = match_result.polygon
        good_count = match_result.good_count
        inlier_count = match_result.inlier_count
        match_confidence = match_result.match_confidence

        if match_result.accepted:
            roi_rect = polygon_to_roi(polygon, frame.shape)
            roi_miss_streak = 0
        elif match_result.search_scope == "FULL":
            roi_rect = None

        centered = False
        center_score = 0.0
        offset_px = 0.0
        tolerance_px = 0.0
        frame_center_x = frame.shape[1] / 2.0
        book_center_x = 0.0
        detected_center_x = 0.0

        if polygon is not None:
            raw_points = polygon.reshape(-1, 2).astype(np.float32)
            detected_center_x = float((raw_points[:, 0].min() + raw_points[:, 0].max()) / 2.0)
            if smoothed_book_center_x is None:
                smoothed_book_center_x = detected_center_x
            else:
                smoothed_book_center_x = (
                    CENTER_SMOOTHING_ALPHA * detected_center_x
                    + (1.0 - CENTER_SMOOTHING_ALPHA) * smoothed_book_center_x
                )

            centered, center_score, offset_px, tolerance_px, frame_center_x, book_center_x, detected_center_x, left_x, right_x = evaluate_center_alignment(
                frame.shape, polygon, center_tol_ratio, smoothed_book_center_x
            )
            cv2.rectangle(frame, (int(left_x), 0), (int(right_x), frame.shape[0] - 1), (180, 180, 180), 1)
        else:
            smoothed_book_center_x = None

        candidate_centered = polygon is not None and centered and match_confidence >= match_conf_threshold
        stable_centered, green_count, red_count = update_stable_state(
            stable_centered, candidate_centered, green_count, red_count
        )

        if polygon is not None:
            poly_color = (0, 255, 0) if stable_centered else (0, 0, 255)
            cv2.polylines(frame, [polygon], True, poly_color, 3, cv2.LINE_AA)

        frame = draw_center_guides(frame, stable_centered, center_tol_ratio)

        if polygon is None:
            status = f"{match_result.backend_name} {match_result.search_scope} | NOT FOUND | matches={good_count} inliers={inlier_count}"
            color = (0, 0, 255)
        elif match_confidence < match_conf_threshold:
            status = (
                f"{match_result.backend_name} {match_result.search_scope} | LOW MATCH | conf={match_confidence*100:.1f}% < {match_conf_threshold*100:.1f}% "
                f"| matches={good_count} inliers={inlier_count}"
            )
            color = (0, 0, 255)
        elif stable_centered:
            status = (
                f"{match_result.backend_name} {match_result.search_scope} | CENTERED | conf={match_confidence*100:.1f}% | offset={offset_px:.0f}px / tol={tolerance_px:.0f}px"
            )
            color = (0, 255, 0)
        else:
            status = (
                f"{match_result.backend_name} {match_result.search_scope} | OFF CENTER | conf={match_confidence*100:.1f}% | offset={offset_px:.0f}px / tol={tolerance_px:.0f}px"
            )
            color = (0, 0, 255)

        now = time.time()
        fps = 1.0 / max(now - previous_time, 1e-6)
        previous_time = now

        put_text(frame, status, (20, 36), color)
        put_text(frame, f"Match threshold: {match_conf_threshold*100:.0f}%", (20, 72), (0, 255, 255))
        put_text(frame, f"Center tol: {center_tol_ratio*100:.0f}%", (20, 108), (255, 255, 0))
        put_text(
            frame,
            f"Candidate: {'YES' if candidate_centered else 'NO'} | green={green_count}/{GREEN_CONFIRM_FRAMES} red={red_count}/{RED_CONFIRM_FRAMES}",
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
