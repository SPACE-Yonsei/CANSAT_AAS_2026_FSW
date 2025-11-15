
import math
import sys
import os

# 프로젝트 루트 경로 추가
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def calculate_distance_haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000  # 지구 반지름 (미터)
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    distance = R * c
    return distance

def calculate_bearing_angle(lat1: float, lon1: float, lat2: float, lon2: float) -> float:

    gps_angle_rad = math.atan2(lat2 - lat1, lon2 - lon1)
    gps_angle_deg = math.degrees(gps_angle_rad)
    
    # 0-360도로 정규화
    if gps_angle_deg < 0:
        gps_angle_deg += 360
    
    return gps_angle_deg

def calculate_turn_angle(target_yaw: float, gps_angle: float) -> float:
    """
    회전해야 할 각도 계산 (-180 ~ +180도)
    
    Args:
        target_yaw: 현재 heading (IMU yaw, 0-360도)
        gps_angle: 목표 방향 (GPS bearing, 0-360도)
    
    Returns:
        회전 각도 (-180 ~ +180도, 음수=왼쪽, 양수=오른쪽)
    """
    angle_diff = target_yaw - gps_angle
    
    while angle_diff > 180:
        angle_diff -= 360
    while angle_diff < -180:
        angle_diff += 360
    
    return angle_diff

def simulate_parafoil_motor(turn: float) -> dict:
    """
    파라포일 모터 제어 시뮬레이션
    
    Args:
        turn: 회전 각도 (-180 ~ +180도, 음수=왼쪽, 양수=오른쪽)
    
    Returns:
        모터 제어 정보 딕셔너리
    """
    TURN_THRESHOLD = 15  # 데드존 (±15도)
    MAX_TURN_ANGLE = 90  # 최대 회전 각도 (전속력)
    PARAFOIL_MOTOR_MIN_PULSE = 500  # 최소 펄스 (실제 코드와 동일)
    PARAFOIL_MOTOR_MAX_PULSE = 2500  # 최대 펄스 (실제 코드와 동일)
    
    result = {
        'turn': turn,
        'left_motor_pulse': 0,
        'right_motor_pulse': 0,
        'left_motor_speed': 0.0,
        'right_motor_speed': 0.0,
        'action': 'STOP'
    }
    
    # 데드존 내부면 모터 정지
    if abs(turn) <= TURN_THRESHOLD:
        result['action'] = 'STRAIGHT'
        return result
    
    # 회전 각도의 절댓값
    turn_magnitude = abs(turn)
    
    effective_turn = min(turn_magnitude - TURN_THRESHOLD, MAX_TURN_ANGLE - TURN_THRESHOLD)
    
    # 속도 비율 계산 (0.0 ~ 1.0)
    speed_ratio = effective_turn / (MAX_TURN_ANGLE - TURN_THRESHOLD)
    
    # 모터 펄스 계산
    base_pulse = PARAFOIL_MOTOR_MIN_PULSE
    speed_range = PARAFOIL_MOTOR_MAX_PULSE - PARAFOIL_MOTOR_MIN_PULSE
    motor_pulse = int(base_pulse + (speed_ratio * speed_range))
    
    if turn < 0:  # 왼쪽 회전
        result['left_motor_pulse'] = motor_pulse
        result['left_motor_speed'] = speed_ratio * 100
        result['action'] = 'TURN_LEFT'
    else:  # 오른쪽 회전
        result['right_motor_pulse'] = motor_pulse
        result['right_motor_speed'] = speed_ratio * 100
        result['action'] = 'TURN_RIGHT'
    
    return result

def print_simulation_result(current_pos: dict, target_pos: dict, current_yaw: float, motor_result: dict):
    """
    시뮬레이션 결과 출력
    
    Args:
        current_pos: 현재 위치 {'lat': float, 'lon': float}
        target_pos: 목표 위치 {'lat': float, 'lon': float}
        current_yaw: 현재 heading (0-360도)
        motor_result: 모터 제어 결과
    """
    distance = calculate_distance_haversine(
        current_pos['lat'], current_pos['lon'],
        target_pos['lat'], target_pos['lon']
    )
    
    gps_bearing = calculate_bearing_angle(
        current_pos['lat'], current_pos['lon'],
        target_pos['lat'], target_pos['lon']
    )
    
    print("\n" + "="*70)
    print("파라포일 조종 알고리즘 시뮬레이션 결과")
    print("="*70)
    print(f"\n📍 현재 위치:")
    print(f"   위도: {current_pos['lat']:.6f}°")
    print(f"   경도: {current_pos['lon']:.6f}°")
    print(f"   Heading (BNO055 IMU Yaw): {current_yaw:.2f}°")
    
    print(f"\n🎯 목표 위치:")
    print(f"   위도: {target_pos['lat']:.6f}°")
    print(f"   경도: {target_pos['lon']:.6f}°")
    
    print(f"\n📊 계산 결과:")
    print(f"   목표까지 거리: {distance:.2f}m")
    print(f"   GPS 방위각 (목표 방향): {gps_bearing:.2f}°")
    print(f"   회전 필요 각도: {motor_result['turn']:+.2f}°", end="")
    
    if abs(motor_result['turn']) <= 15:
        print(" (직진)")
    elif motor_result['turn'] < 0:
        print(f" (왼쪽으로 {abs(motor_result['turn']):.2f}° 회전 필요)")
    else:
        print(f" (오른쪽으로 {motor_result['turn']:.2f}° 회전 필요)")
    
    print(f"\n🔧 모터 제어:")
    print(f"   동작: {motor_result['action']}")
    
    if motor_result['action'] == 'TURN_LEFT':
        print(f"   왼쪽 모터: 펄스={motor_result['left_motor_pulse']}µs, 속도={motor_result['left_motor_speed']:.1f}%")
        print(f"   오른쪽 모터: 정지 (0)")
    elif motor_result['action'] == 'TURN_RIGHT':
        print(f"   왼쪽 모터: 정지 (0)")
        print(f"   오른쪽 모터: 펄스={motor_result['right_motor_pulse']}µs, 속도={motor_result['right_motor_speed']:.1f}%")
    else:
        print(f"   왼쪽 모터: 정지 (0)")
        print(f"   오른쪽 모터: 정지 (0)")
    
    print("="*70 + "\n")

def interactive_simulation():
    """
    대화형 시뮬레이션 모드
    """
    print("\n" + "="*70)
    print("파라포일 조종 알고리즘 시뮬레이션")
    print("="*70)
    print("\nGPS 좌표와 IMU heading을 입력하여 모터 제어를 시뮬레이션합니다.\n")
    
    # 목표 GPS 좌표 입력
    print("🎯 목표 GPS 좌표를 입력하세요:")
    try:
        target_lat = float(input("   목표 위도 (예: 37.5665): "))
        target_lon = float(input("   목표 경도 (예: 126.9780): "))
    except ValueError:
        print("❌ 잘못된 입력입니다. 숫자를 입력해주세요.")
        return
    
    target_pos = {'lat': target_lat, 'lon': target_lon}
    
    # 현재 GPS 좌표와 heading 입력 (반복)
    print("\n📍 현재 위치와 heading을 입력하세요 (종료: Ctrl+C):\n")
    
    step = 1
    while True:
        try:
            print(f"\n--- 시뮬레이션 단계 {step} ---")
            current_lat = float(input("   현재 위도: "))
            current_lon = float(input("   현재 경도: "))
            current_yaw = float(input("   현재 Heading (BNO055 IMU Yaw, 0-360°): "))
            
            # 0-360도로 정규화 (BNO055 quaternion에서 계산된 yaw)
            current_yaw = current_yaw % 360
            
            current_pos = {'lat': current_lat, 'lon': current_lon}
            
            # GPS 방위각 계산
            gps_bearing = calculate_bearing_angle(
                current_pos['lat'], current_pos['lon'],
                target_pos['lat'], target_pos['lon']
            )
            
            # 회전 각도 계산
            turn_angle = calculate_turn_angle(current_yaw, gps_bearing)
            
            # 모터 제어 시뮬레이션
            motor_result = simulate_parafoil_motor(turn_angle)
            
            # 결과 출력
            print_simulation_result(current_pos, target_pos, current_yaw, motor_result)
            
            step += 1
            
            # 목표 도달 확인
            distance = calculate_distance_haversine(
                current_pos['lat'], current_pos['lon'],
                target_pos['lat'], target_pos['lon']
            )
            
            if distance <= 50.0:
                print("✅ 목표 지점에 도달했습니다! (50m 반경 내)")
                break
            
        except ValueError:
            print("❌ 잘못된 입력입니다. 숫자를 입력해주세요.")
        except KeyboardInterrupt:
            print("\n\n시뮬레이션을 종료합니다.")
            break

def quick_test():
    """
    빠른 테스트 모드 (예제 데이터)
    """
    print("\n" + "="*70)
    print("파라포일 조종 알고리즘 빠른 테스트")
    print("="*70)
    
    # 예제 데이터
    test_cases = [
        {
            'name': '직진 상황',
            'current': {'lat': 37.5665, 'lon': 126.9780},
            'target': {'lat': 37.5666, 'lon': 126.9781},
            'yaw': 45.0
        },
        {
            'name': '왼쪽 회전 필요',
            'current': {'lat': 37.5665, 'lon': 126.9780},
            'target': {'lat': 37.5670, 'lon': 126.9780},
            'yaw': 90.0
        },
        {
            'name': '오른쪽 회전 필요',
            'current': {'lat': 37.5665, 'lon': 126.9780},
            'target': {'lat': 37.5670, 'lon': 126.9780},
            'yaw': 270.0
        },
        {
            'name': '대각선 회전',
            'current': {'lat': 37.5665, 'lon': 126.9780},
            'target': {'lat': 37.5675, 'lon': 126.9790},
            'yaw': 0.0
        }
    ]
    
    for i, test in enumerate(test_cases, 1):
        print(f"\n{'='*70}")
        print(f"테스트 케이스 {i}: {test['name']}")
        print(f"{'='*70}")
        
        # GPS 방위각 계산
        gps_bearing = calculate_bearing_angle(
            test['current']['lat'], test['current']['lon'],
            test['target']['lat'], test['target']['lon']
        )
        
        # 회전 각도 계산
        turn_angle = calculate_turn_angle(test['yaw'], gps_bearing)
        
        # 모터 제어 시뮬레이션
        motor_result = simulate_parafoil_motor(turn_angle)
        
        # 결과 출력
        print_simulation_result(test['current'], test['target'], test['yaw'], motor_result)

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--test':
        quick_test()
    else:
        interactive_simulation()

