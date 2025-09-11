import cv2
import numpy as np

# ----------------------------
# Core image processing functions
# ----------------------------

def canny(image):
    """Apply Canny edge detection."""
    return cv2.Canny(image, 100, 200)

def region_of_interest(image):
    """Mask the image to keep only the region of interest (ROI)."""
    height = image.shape[0]
    width = image.shape[1]
    polygons = np.array([[(0, height / 2 + 100), (width, height / 2 + 100), (width, height), (0, height)]], dtype=np.int32)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    return cv2.bitwise_and(image, mask)

def detect_lines(image):
    """Detect lines using Hough Transform."""
    return cv2.HoughLinesP(
        image,
        rho=2,
        theta=np.pi / 180,
        threshold=100,
        minLineLength=40,
        maxLineGap=5
    )

def average_slope_intercept(image, lines):
    """Average out left and right lines."""
    if lines is None:
        return None

    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        if x2 == x1:
            continue  # avoid divide by zero
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1
        if slope < 0:  # left lane
            left_fit.append((slope, intercept))
        else:          # right lane
            right_fit.append((slope, intercept))

    left_line = make_coordinates(image, np.mean(left_fit, axis=0)) if left_fit else None
    right_line = make_coordinates(image, np.mean(right_fit, axis=0)) if right_fit else None

    if left_line is None or right_line is None:
        return None

    return np.array([left_line, right_line])

def make_coordinates(image, line_params):
    """Convert slope/intercept into line coordinates."""
    try:
        slope, intercept = line_params
    except TypeError:
        return None
    y1 = image.shape[0]            # bottom of frame
    y2 = int(y1 * 0.6)             # 60% from bottom
    if slope == 0:
        return None
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])

# ----------------------------
# Optional test runner
# ----------------------------

def process_frame(frame):
    """
    Run the full pipeline on one frame.
    Returns left/right line coordinates or None.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = canny(blur)
    cropped = region_of_interest(edges)
    lines = detect_lines(cropped)
    return average_slope_intercept(frame, lines)

if __name__ == "__main__":
    # Standalone test mode: visualize lane detection
    cap = cv2.VideoCapture(0)  # USB cam
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        lines = process_frame(frame)
        if lines is not None:
            for x1, y1, x2, y2 in lines:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

        cv2.imshow("Lane Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
