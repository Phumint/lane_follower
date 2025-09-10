import cv2
import numpy as np
from .lane_detection import canny, region_of_interest, detect_lines, average_slope_intercept, make_coordinates

def detect_lane_params(frame):
    """
    Returns: offset_norm [-1,1], heading_rad, confidence
    """
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = canny(blur)
    roi = region_of_interest(edges)

    lines = detect_lines(roi)
    if lines is None:
        return 0.0, 0.0, 0.0

    averaged = average_slope_intercept(frame, lines)
    if averaged is None or len(averaged) < 2:
        return 0.0, 0.0, 0.2  # low confidence

    left, right = averaged
    xL_bottom, _, xL_mid, yL_mid = make_coordinates(frame, left)
    xR_bottom, _, xR_mid, yR_mid = make_coordinates(frame, right)

    lane_center_bottom = (xL_bottom + xR_bottom) / 2
    lane_center_mid = (xL_mid + xR_mid) / 2
    pixel_offset = lane_center_bottom - width / 2
    offset_norm = pixel_offset / (width / 2)

    dx = lane_center_mid - lane_center_bottom
    dy = abs(yL_mid - height)  # vertical distance
    heading_rad = np.arctan2(dx, dy) if dy > 0 else 0.0

    return float(offset_norm), float(heading_rad), 1.0
