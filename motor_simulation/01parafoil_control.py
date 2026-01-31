
def quick_angle(angle: float) -> float:
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def calculate_motor_control(yaw: float, lat: float, lon: float) -> float:
    global last_error
    
    if target_lat == 0.0 and target_lon == 0.0:
        return 0.0
    
    # GPS 유효 → 방위각 계
    if is_gps_valid(lat, lon):
        target_azimuth = math.degrees(math.atan2(target_lat - lat, target_lon - lon))
        error=quick_angle(target_azimuth-yaw)
        last_error = error
        return error
    # GPS 무효 but 이전 방위각 있음 → 유지
    if last_error is not None:
        return quick_angle(yaw - last_error)
    
    # GPS 무효, 이전 방위각 없음 → 직진
    return 0.0


def rotate_parafoil_motor(pi, error: float):
    """
    Control parafoil motor (ON/OFF control).
    
    Args:
        pi: pigpio instance
        turn: Angle difference (-180 ~ +180 degrees)
              - Negative: turn left (left release down + right pull up)
              - Positive: turn right (left pull up + right release down)
    """
    THRESHOLD = 15  # Dead zone (±15 degrees)
    error_to_purse = 2000/180
    purse = error*error_to_purse
    # Within dead zone: straight (both motors release down)
    
    
    if error < -THRESHOLD:  # Turn left: left release down (1500), right pull up (500)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_neutral+purse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_neutral)
    elif error > THRESHOLD: # Turn right
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_neutral + purse)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_neutral)
        return
    
    else:  # Turn right: left pull up (500), right release down (1500)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_neutral)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_neutral)
    
    return