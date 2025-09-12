import cv2
import numpy as np
from control.controller import LaneFollowerController
from utils.logger import Logger
import time
import RPi.GPIO as GPIO

# Keep previous fits for smoother lane detection
prev_left_fit_average = prev_right_fit_average = None

# GPIO setup for the button
BUTTON_PIN = 26
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
run_algorithm = False

# Function to toggle the state on button press
def button_callback(channel):
    global run_algorithm
    run_algorithm = not run_algorithm
    if run_algorithm:
        print("Lane following enabled.")
    else:
        print("Lane following disabled.")

# Add an event listener for button presses
GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=200)

def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    return cv2.Canny(blur, 100, 200)

def region_of_interest(image):
    height, width = image.shape[:2]
    polygons = np.array([[
        (0, height // 2 + 100),
        (width, height // 2 + 100),
        (width, height),
        (0, height)
    ]], dtype=np.int32)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    return cv2.bitwise_and(image, mask)

def make_coordinates(image, line_parameters):
    if line_parameters is None:
        return None
    slope, intercept = line_parameters
    if slope == 0 or slope is None or np.isnan(slope) or np.isnan(intercept):
        return None

    y1 = image.shape[0]
    y2 = int(y1 * 0.6)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    x1 = max(0, min(x1, image.shape[1] - 1))
    x2 = max(0, min(x2, image.shape[1] - 1))
    return np.array([x1, y1, x2, y2], dtype=np.int32)

def average_slope_intercept(image, lines):
    global prev_left_fit_average, prev_right_fit_average
    left_fit, right_fit = [], []
    if lines is None:
        return None

    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope, intercept = parameters
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))

    if left_fit:
        left_fit_average = np.average(left_fit, axis=0)
        prev_left_fit_average = left_fit_average
    else:
        left_fit_average = prev_left_fit_average

    if right_fit:
        right_fit_average = np.average(right_fit, axis=0)
        prev_right_fit_average = right_fit_average
    else:
        right_fit_average = prev_right_fit_average

    left_line_coords = make_coordinates(image, left_fit_average)
    right_line_coords = make_coordinates(image, right_fit_average)

    valid_lines = [l for l in [left_line_coords, right_line_coords] if l is not None]
    if not valid_lines:
        return None
    return np.array(valid_lines)

def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 100), 12)
    return line_image

def main():
    global run_algorithm
    cap = cv2.VideoCapture(0)
    ctrl = LaneFollowerController()
    log = Logger()

    # Timestamped output file
    video_name = "lane_run.avi"
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = None

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            if out is None:
                h, w = frame.shape[:2]
                out = cv2.VideoWriter(video_name, fourcc, 20.0, (w, h))

            steer, speed = 0.0, 0.0
            
            if run_algorithm:
                # Lane detection
                canny_img = canny(frame)
                cropped = region_of_interest(canny_img)
                lines = cv2.HoughLinesP(cropped, 2, np.pi/180, 100,
                                        np.array([]), minLineLength=40, maxLineGap=5)
                averaged_lines = average_slope_intercept(frame, lines)
                line_image = display_lines(frame, averaged_lines)
                combo_image = cv2.addWeighted(frame, 0.9, line_image, 1, 1)

                # Controller still runs on offset + heading (simplified: center of lines)
                if averaged_lines is not None and len(averaged_lines) == 2:
                    (xL1, yL1, xL2, yL2), (xR1, yR1, xR2, yR2) = averaged_lines
                    lane_center_bottom = (xL1 + xR1) / 2
                    pixel_offset = lane_center_bottom - (frame.shape[1] / 2)
                    offset_norm = pixel_offset / (frame.shape[1] / 2)

                    dx = ((xL2 + xR2) / 2 - lane_center_bottom)
                    dy = abs(yL2 - frame.shape[0])
                    heading = np.arctan2(dx, dy) if dy > 0 else 0.0
                    conf = 1.0
                else:
                    offset_norm, heading, conf = 0.0, 0.0, 0.2

                steer, speed = ctrl.update(offset_norm, heading, conf)
                log.log(offset_norm, heading, steer, speed, conf)
            else:
                combo_image = frame.copy()
                steer, speed = 0.0, 0.0
                ctrl.update(0, 0, 0.0) # Ensure the controller commands a stop

            # Add telemetry overlay
            status_text = "RUNNING" if run_algorithm else "PAUSED"
            status_color = (0, 255, 0) if run_algorithm else (0, 165, 255)
            cv2.putText(combo_image, f"Status: {status_text}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv2.putText(combo_image, f"Steer: {steer:.2f} deg  Speed: {speed:.2f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            out.write(combo_image)

    except KeyboardInterrupt:
        print("Run stopped by user.")
    finally:
        ctrl.cleanup()
        cap.release()
        if out is not None:
            out.release()
        cv2.destroyAllWindows()
        GPIO.cleanup() # Clean up GPIO settings
        print(f"Video saved: {video_name}")

if __name__ == "__main__":
    main()