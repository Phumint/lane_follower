import cv2
import numpy as np
from .lane_detection import (
    canny, region_of_interest, detect_lines,
    average_slope_intercept, make_coordinates,
    perspective_transform, sliding_window_lane
)

# ---------------- Adapter for Hough ---------------- #
def detect_lane_params(frame):
    """Hough-based lane detection (original)."""
    canny_img = canny(frame)
    roi = region_of_interest(canny_img)
    lines = detect_lines(roi)
    left_line, right_line = average_slope_intercept(frame, lines)

    line_image = np.zeros_like(frame)
    for line_params in (left_line, right_line):
        coords = make_coordinates(frame, line_params)
        if coords is not None:
            x1,y1,x2,y2 = coords
            cv2.line(line_image, (x1,y1), (x2,y2), (0,255,0), 5)

    combo = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    # crude offset/heading calculation
    if left_line is not None and right_line is not None:
        lane_center = (make_coordinates(frame,left_line)[0] +
                       make_coordinates(frame,right_line)[0]) / 2
        offset = (lane_center - frame.shape[1]/2) / (frame.shape[1]/2)
    else:
        offset = 0.0
    heading = 0.0
    confidence = 1.0 if lines is not None else 0.0
    return float(offset), float(heading), confidence, combo

# ---------------- Adapter for Sliding Window ---------------- #
def detect_lane_params_sliding(frame):
    height, width = frame.shape[:2]

    # --- Define ROI polygon (same trapezoid used in perspective transform) ---
    roi_points = np.array([[
        (0, height-50),
        (width, height-50),
        (width, int(height*0.5)),
        (0, int(height*0.5))
    ]], dtype=np.int32)

    # Draw ROI for visualization
    roi_frame = frame.copy()
    cv2.polylines(roi_frame, roi_points, isClosed=True, color=(255,0,0), thickness=2)

    # --- Perspective transform using same ROI points ---
    pts1 = np.float32([roi_points[0][3], roi_points[0][0], roi_points[0][2], roi_points[0][1]])  # tl, bl, tr, br
    pts2 = np.float32([[0,0],[0,height],[width,0],[width,height]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    inv_matrix = cv2.getPerspectiveTransform(pts2, pts1)
    warped = cv2.warpPerspective(frame, matrix, (width,height))

    # --- Mask for blue lanes ---
    hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
    # lower_blue = np.array([100, 100, 100])
    # upper_blue = np.array([140, 255, 255])

    lower_blue = np.array([86, 40, 0])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # --- Sliding window search ---
    lx, rx = sliding_window_lane(mask)
    if len(lx) == 0 or len(rx) == 0:
        return 0.0, 0.0, 0.0, roi_frame  # show ROI if nothing detected

    # --- Lane polygon ---
    min_length = min(len(lx), len(rx))
    top_left = (lx[0], height)
    bottom_left = (lx[min_length-1], 0)
    top_right = (rx[0], height)
    bottom_right = (rx[min_length-1], 0)

    quad_points = np.array([[top_left, bottom_left, bottom_right, top_right]], dtype=np.int32).reshape((-1,1,2))

    # overlay = warped.copy()
    # cv2.fillPoly(overlay, [quad_points], (0,255,0))
    # alpha = 0.3
    # warped_lane = cv2.addWeighted(overlay, alpha, warped, 1-alpha, 0)

    # # --- Map back to original view ---
    # unwarped_lane = cv2.warpPerspective(warped_lane, inv_matrix, (width,height))
    # result = cv2.addWeighted(frame, 1, unwarped_lane, 0.7, 0)

    # --- Create a black mask for the lane polygon ---
    lane_mask = np.zeros_like(warped)

    # Draw filled green polygon on the mask
    cv2.fillPoly(lane_mask, [quad_points], (0, 255, 0))

    # Warp mask back to original perspective
    unwarped_lane = cv2.warpPerspective(lane_mask, inv_matrix, (width, height))

    # Combine with original frame (only polygon area is green)
    alpha = 0.3
    result = cv2.addWeighted(frame, 1, unwarped_lane, alpha, 0)

    # Draw ROI lines on result for clarity
    cv2.polylines(result, roi_points, isClosed=True, color=(0,0,255), thickness=1)

    # --- Offset calculation ---
    lane_center = (np.mean(lx) + np.mean(rx)) / 2
    pixel_offset = lane_center - width/2
    offset_norm = pixel_offset / (width/2)
    heading_rad = 0.0

    return float(offset_norm), float(heading_rad), 1.0, result

