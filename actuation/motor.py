import pigpio

RPWM = 13
LPWM = 12
R_EN, L_EN = 6 ,5
PWM_FREQ = 1000 # 8 kHz

pi = pigpio.pi()
for pin in (R_EN, L_EN):
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 1)

def set_motor(speed):
    """
    speed ∈ [-1,1] (negative = reverse)
    """
    duty = int(abs(speed) * 1000000)  # range 0-1M scale pigpio uses for duty cycle.
    duty = min(max(duty, 0), 1000000)  # clamp
    if speed > 0:
        pi.hardware_PWM(RPWM, PWM_FREQ, duty)
        pi.hardware_PWM(LPWM, 0, 0)
    elif speed < 0:
        pi.hardware_PWM(LPWM, PWM_FREQ, duty)
        pi.hardware_PWM(RPWM, 0, 0)
    else:
        pi.hardware_PWM(RPWM, 0, 0)
        pi.hardware_PWM(LPWM, 0, 0)
