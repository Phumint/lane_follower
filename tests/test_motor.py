import pigpio
import time

RPWM = 12
LPWM = 13
R_EN, L_EN = 5, 6
PWM_FREQS = [500, 1000, 2000, 5000, 8000, 10000]  # test these

pi = pigpio.pi()
pi.set_mode(R_EN, pigpio.OUTPUT)
pi.set_mode(L_EN, pigpio.OUTPUT)
pi.write(R_EN, 1)
pi.write(L_EN, 1)

def stop():
    pi.hardware_PWM(RPWM, 0, 0)
    pi.hardware_PWM(LPWM, 0, 0)

def forward(freq, duty=0.5):
    print(f"Forward @ {freq} Hz, duty={duty}")
    duty_int = int(duty * 1_000_000)
    pi.hardware_PWM(RPWM, freq, duty_int)
    pi.hardware_PWM(LPWM, 0, 0)

if __name__ == "__main__":
    try:
        for f in PWM_FREQS:
            forward(f, duty=1.0)  # gentle test at 30%
            time.sleep(3)
            stop()
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping motor")
        stop()
        pi.stop()
