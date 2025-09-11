import pigpio
import time

SERVO_PIN = 18
pi = pigpio.pi()

# Typical SG92R works at 50 Hz (20 ms period)
# pigpio handles this automatically for servo pulses
def set_pulse(us):
    pi.set_servo_pulsewidth(SERVO_PIN, us)

def sweep_servo():
    print("Sweeping servo...")
    for us in range(1000, 2001, 100):  # 1000 → 2000 µs
        print(f"Pulse: {us} µs")
        set_pulse(us)
        time.sleep(0.7)
    set_pulse(1500)  # back to center

if __name__ == "__main__":
    try:
        # Neutral (should be straight wheels)
        print("Centering at 1500 µs")
        set_pulse(1500)
        time.sleep(2)

        sweep_servo()
    except KeyboardInterrupt:
        pass
    finally:
        print("Returning servo to center and stopping")
        # Command the servo to move back to the center (1500 µs)
        pi.set_servo_pulsewidth(SERVO_PIN, 1500)
        time.sleep(1) # Give it time to move back to the center
        
        # Now, disable the pulses, allowing it to be moved freely.
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
