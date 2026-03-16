import math
import time

PARAFOIL_LEFT_MOTOR_PIN = 12   # GPIO 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # GPIO 13, physical pin 33

PULSE_PER_DEG = 2000 / 180
LEFT_ZERO = 600
RIGHT_ZERO = 2500
MAX_ANGLE_SCOPE = 120  # degrees

LEFT_NEUTRAL = int(LEFT_ZERO + 60 * PULSE_PER_DEG)
RIGHT_NEUTRAL = int(RIGHT_ZERO - 60 * PULSE_PER_DEG)

def init_parafoil_motor():
    global last_time, pi_integral, wind_crab_est
    import pigpio
    pi = pigpio.pi()

    last_time = time.time()
    pi_integral = 0.0
    wind_crab_est = 0.0

    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_NEUTRAL)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_NEUTRAL)
    return pi

def terminate_parafoil_motor(pi):
    if pi is not None:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_NEUTRAL)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_NEUTRAL)
        time.sleep(0.1)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
        pi.stop()


