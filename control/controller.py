import math
from .pid import PID
from actuation import steering, motor

class LaneFollowerController:
    def __init__(self):
        self.pid = PID(1.2, 0.0, 0.2, output_limits=(-30, 30))  # tune later

    def update(self, offset, heading, confidence):
        # composite error
        composite = 0.7*offset + 0.3*heading
        steer_angle = self.pid.update(composite)

        # base motor speed (scale down if low confidence)
        base_speed = 0.5 * confidence  # 50% duty max
        speed = base_speed * (1 - min(abs(steer_angle)/30, 1)*0.5)

        steering.set_steering(steer_angle)
        motor.set_motor(speed)

        return steer_angle, speed
