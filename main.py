import cv2
import numpy as np
from control.controller import LaneFollowerController
from utils.logger import Logger
# from perception.perception_adapter import detect_lane_params
from perception.perception_adapter import detect_lane_params_sliding as detect_lane_params
import time
import RPi.GPIO as GPIO

# GPIO setup for the button
BUTTON_PIN = 26
RED_LED = 17
GREEN_LED = 27
BLUE_LED = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RED_LED, GPIO.OUT)
GPIO.setup(GREEN_LED, GPIO.OUT)
GPIO.setup(BLUE_LED, GPIO.OUT)

GPIO.output(RED_LED, GPIO.HIGH)
GPIO.output(GREEN_LED, GPIO.HIGH)

run_algorithm = False

# Function to toggle the state on button press
def button_callback(channel):
    global run_algorithm
    run_algorithm = not run_algorithm
    if run_algorithm:
        GPIO.output(GREEN_LED, GPIO.HIGH)
        GPIO.output(RED_LED, GPIO.LOW)  
        print("Lane following enabled.")
    else:
        GPIO.output(GREEN_LED, GPIO.LOW)
        GPIO.output(RED_LED, GPIO.HIGH)
        print("Lane following disabled.")

# Add an event listener for button presses
GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=200)


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
                # Use perception package to detect lane params + debug visualization
                offset_norm, heading, conf, debug_frame = detect_lane_params(frame)

                steer, speed = ctrl.update(offset_norm, heading, conf)
                log.log(offset_norm, heading, steer, speed, conf)

                combo_image = debug_frame
            else:
                combo_image = frame.copy()
                steer, speed = 0.0, 0.0
                ctrl.update(0, 0, 0.0)  # Ensure the controller commands a stop

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
        GPIO.cleanup()  # Clean up GPIO settings
        print(f"Video saved: {video_name}")


if __name__ == "__main__":
    main()
