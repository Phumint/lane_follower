import cv2
import numpy as np

# ---------------- Hough Transform pipeline ---------------- #
def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    return cv2.Canny(blur, 50, 150)

def region_of_interest(image):
    height, width = image.shape[:2]
    mask = np.zeros_like(image)
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, int(height*0.6)),
        (0, int(height*0.6))
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(image, mask)

def detect_lines(image):
    return cv2.HoughLinesP(image, 2, np.pi/180, 100,
                           np.array([]), minLineLength=40, maxLineGap=5)

def average_slope_intercept(image, lines):
    left, right = [], []
    if lines is None:
        return None
    for line in lines:
        x1,y1,x2,y2 = line.reshape(4)
        if x2-x1 == 0:
            continue
        slope = (y2-y1)/(x2-x1)
        intercept = y1 - slope*x1
        if slope < 0:
            left.append((slope, intercept))
        else:
            right.append((slope, intercept))
    left_line = np.mean(left, axis=0) if left else None
    right_line = np.mean(right, axis=0) if right else None
    return left_line, right_line

def make_coordinates(image, line_params):
    if line_params is None:
        return None
    slope, intercept = line_params
    y1 = image.shape[0]
    y2 = int(y1*0.6)
    if slope == 0:
        return None
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1,y1,x2,y2])

# ---------------- Sliding Window pipeline ---------------- #
def perspective_transform(frame):
    """Warp perspective to bird's eye view."""
    height, width = frame.shape[:2]

    # example trapezoid ROI for perspective transform
    tl, bl, tr, br = (
        (0, height),
        (width, height),
        (width, int(height*0.6)),
        (0, int(height*0.6))
    )

    pts1 = np.float32([tl, bl, tr, br])
    pts2 = np.float32([[0,0],[0,height],[width,0],[width,height]])

    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    inv_matrix = cv2.getPerspectiveTransform(pts2, pts1)
    warped = cv2.warpPerspective(frame, matrix, (width,height))

    return warped, matrix, inv_matrix


def sliding_window_lane(mask):
    """Apply sliding window lane detection on a binary mask."""
    histogram = np.sum(mask[mask.shape[0]//2:,:], axis=0)
    midpoint = histogram.shape[0]//2
    left_base = np.argmax(histogram[:midpoint])
    right_base = np.argmax(histogram[midpoint:]) + midpoint

    n_windows = 12
    window_height = mask.shape[0]//n_windows
    nonzero = mask.nonzero()
    nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])

    margin, minpix = 50, 50
    lx, rx = [], []
    l_current, r_current = left_base, right_base

    for window in range(n_windows):
        win_y_low = mask.shape[0] - (window+1)*window_height
        win_y_high = mask.shape[0] - window*window_height
        win_xleft_low, win_xleft_high = l_current - margin, l_current + margin
        win_xright_low, win_xright_high = r_current - margin, r_current + margin

        good_left = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                     (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                      (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        if len(good_left) > minpix:
            l_current = int(np.mean(nonzerox[good_left]))
        if len(good_right) > minpix:
            r_current = int(np.mean(nonzerox[good_right]))

        lx.extend(nonzerox[good_left])
        rx.extend(nonzerox[good_right])

    return np.array(lx), np.array(rx)
