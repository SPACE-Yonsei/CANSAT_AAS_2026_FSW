import math

# Copy functions from 01parafoil_control.py
def quick_angle(angle: float) -> float:
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# Mock global variables
last_error = None
target_lat = 37.5665
target_lon = 126.9780

def is_gps_valid(lat, lon):
    return lat != 0.0 and lon != 0.0

def calculate_motor_control(yaw: float, lat: float, lon: float) -> float:
    global last_error

    if target_lat == 0.0 and target_lon == 0.0:
        return 0.0

    if is_gps_valid(lat, lon):
        target_azimuth = math.degrees(math.atan2(target_lat - lat, target_lon - lon))
        error = quick_angle(target_azimuth - yaw)
        last_error = error
        return error

    if last_error is not None:
        return quick_angle(yaw - last_error)

    return 0.0

# Test quick_angle
print("=== quick_angle 테스트 ===")
print(f"quick_angle(270) = {quick_angle(270)}")  # -90
print(f"quick_angle(-270) = {quick_angle(-270)}")  # 90
print(f"quick_angle(540) = {quick_angle(540)}")  # -180
print(f"quick_angle(90) = {quick_angle(90)}")  # 90

# Test calculate_motor_control
print("\n=== calculate_motor_control 테스트 ===")
print(f"GPS 유효 (현재: 37.5, 126.9, yaw=0): {calculate_motor_control(0, 37.5, 126.9)}")
print(f"GPS 유효 (현재: 37.5, 126.9, yaw=45): {calculate_motor_control(45, 37.5, 126.9)}")
print(f"GPS 무효 (이전 에러 있음): {calculate_motor_control(30, 0, 0)}")
last_error = None
print(f"GPS 무효 (이전 에러 없음): {calculate_motor_control(30, 0, 0)}")

# Test rotate_parafoil_motor logic
print("\n=== rotate_parafoil_motor 로직 테스트 ===")
THRESHOLD = 15
error_to_purse = 2000/180

test_errors = [-50, -10, 0, 10, 50]
for error in test_errors:
    purse = error * error_to_purse
    if error < -THRESHOLD:
        print(f"Error={error:3d}° → 좌회전 (purse={purse:.1f})")
    elif error > THRESHOLD:
        print(f"Error={error:3d}° → 우회전 (purse={purse:.1f})")
    else:
        print(f"Error={error:3d}° → 직진")
