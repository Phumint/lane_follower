import math
from .pid import PID
from actuation import steering, motor
import pigpio

pi = pigpio.pi()

class LaneFollowerController:
    def __init__(self):
        self.pid = PID(3.0, 0.0, 0.5, output_limits=(-20, 20))  # tune later

    def update(self, offset, heading, confidence):
        # composite error
        composite = 0.8*offset*30 + 0.2*heading*30
        steer_angle = -self.pid.update(composite)

        # base motor speed (scale down if low confidence)
        base_speed = 1.0 * confidence  # 50% duty max
        speed = base_speed * (1 - min(abs(steer_angle)/20, 1)*0.3)

        steering.set_steering(steer_angle)
        motor.set_motor(speed)

        return steer_angle, speed

    def cleanup(self):
        print("Executing controller cleanup...")
        
        # 1. Stop the motor.
        motor.set_motor(0)
        
        # 2. Disable the servo pulse. This allows the servo to be moved freely
        # by hand and reduces power consumption.
        steering.set_steering(0)
        
        # 3. Stop the pigpio daemon connection.
        pi.stop()
        print("Cleanup complete. All hardware disabled.")
