import cv2
from perception.perception_adapter import detect_lane_params
from control.controller import LaneFollowerController
from utils.logger import Logger

def main():
    cap = cv2.VideoCapture(0)  # USB cam
    ctrl = LaneFollowerController()
    log = Logger()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        offset, heading, conf = detect_lane_params(frame)
        steer, speed = ctrl.update(offset, heading, conf)
        log.log(offset, heading, steer, speed, conf)

if __name__ == "__main__":
    main()
