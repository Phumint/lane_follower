import pigpio

SERVO_PIN = 18
NEUTRAL_US = 1500
MIN_US = 1000
MAX_US = 2000
MAX_WHEEL_ANGLE_DEG = 30.0

pi = pigpio.pi()

def angle_to_pulse(angle_deg):
    angle_deg = max(-MAX_WHEEL_ANGLE_DEG, min(MAX_WHEEL_ANGLE_DEG, angle_deg))
    return int(NEUTRAL_US + (angle_deg / MAX_WHEEL_ANGLE_DEG) * (MAX_US - NEUTRAL_US))

def set_steering(angle_deg):
    pulse = angle_to_pulse(angle_deg)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse)
