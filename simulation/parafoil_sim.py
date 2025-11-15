
import math
import sys
import os
import time

# 프로젝트 루트 경로 추가
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# BNO055 IMU 센서 읽기
try:
    from Sensor_Imu import imu
    IMU_AVAILABLE = True
except ImportError:
    IMU_AVAILABLE = False

# 파라포일 모터 제어
try:
    from Sensor_Motor import parafoil_motor
    MOTOR_AVAILABLE = True
except ImportError:
    MOTOR_AVAILABLE = False

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

    angle_diff = target_yaw - gps_angle
    
    while angle_diff > 180:
        angle_diff -= 360
    while angle_diff < -180:
        angle_diff += 360
    
    return angle_diff

def simulate_parafoil_motor(turn: float) -> dict:
    TURN_THRESHOLD = 10  # 데드존 (±15도)
    MAX_TURN_ANGLE = 90  # 최대 회전 각도 (전속력)
    PARAFOIL_MOTOR_MIN_PULSE = 1250  # 최소 펄스 (실제 코드와 동일)
    PARAFOIL_MOTOR_MAX_PULSE = 1750  # 최대 펄스 (실제 코드와 동일)
    
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
    print(f"\n 현재 위치:")
    print(f"   위도: {current_pos['lat']:.6f}°")
    print(f"   경도: {current_pos['lon']:.6f}°")
    print(f"   Heading (BNO055 IMU Yaw): {current_yaw:.2f}°")
    
    print(f"\n 목표 위치:")
    print(f"   위도: {target_pos['lat']:.6f}°")
    print(f"   경도: {target_pos['lon']:.6f}°")
    
    print(f"\n 계산 결과:")
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
    print("\n" + "="*70)
    print("파라포일 조종 알고리즘 시뮬레이션 (실제 모터 제어 포함)")
    print("="*70)
    print("\nGPS 좌표를 입력하고 BNO055 센서에서 IMU heading을 읽어 실제 모터를 제어합니다.\n")
    
    # BNO055 IMU 센서 초기화
    i2c_instance = None
    imu_sensor = None
    
    if IMU_AVAILABLE:
        try:
            print(" BNO055 IMU 센서 초기화 중...")
            i2c_instance, imu_sensor = imu.init_imu()
            print(" BNO055 IMU 센서 초기화 완료")
        except Exception as e:
            print(f" BNO055 IMU 센서 초기화 실패: {e}")
            print("  센서 없이 시뮬레이션을 계속하려면 수동 입력 모드로 전환됩니다.")
            use_sensor = False
        else:
            use_sensor = True
    else:
        print("  BNO055 센서 모듈이 없습니다. 수동 입력 모드로 진행합니다.")
        use_sensor = False
    
    # 파라포일 모터 초기화
    motor_pi = None
    use_motor = False
    
    if MOTOR_AVAILABLE:
        try:
            print(" 파라포일 모터 초기화 중...")
            motor_pi = parafoil_motor.init_parafoil_motor()
            if motor_pi is None or not motor_pi.connected:
                print(" pigpio 데몬에 연결할 수 없습니다.")
                print("   sudo pigpiod 명령으로 pigpio 데몬을 시작해주세요.")
            else:
                print(" 파라포일 모터 초기화 완료")
                use_motor = True
        except Exception as e:
            print(f" 파라포일 모터 초기화 실패: {e}")
            print("  모터 없이 시뮬레이션만 진행합니다.")
            use_motor = False
    else:
        print("  파라포일 모터 모듈이 없습니다. 시뮬레이션만 진행합니다.")
        use_motor = False
    
    # 목표 GPS 좌표 입력
    print("\n 목표 GPS 좌표를 입력하세요:")
    try:
        target_lat = float(input("   목표 위도 (예: 37.5665): "))
        target_lon = float(input("   목표 경도 (예: 126.9780): "))
    except ValueError:
        print(" 잘못된 입력입니다. 숫자를 입력해주세요.")
        if i2c_instance is not None:
            imu.imu_terminate(i2c_instance)
        return
    
    target_pos = {'lat': target_lat, 'lon': target_lon}
    
    # 현재 GPS 좌표 입력 (반복)
    if use_sensor:
        print("\n 현재 GPS 좌표를 입력하고 Enter를 누르면 센서에서 IMU yaw 값을 읽습니다 (종료: Ctrl+C):\n")
    else:
        print("\n 현재 위치와 heading을 입력하세요 (종료: Ctrl+C):\n")
    
    step = 1
    while True:
        try:
            print(f"\n--- 시뮬레이션 단계 {step} ---")
            current_lat = float(input("   현재 위도: "))
            current_lon = float(input("   현재 경도: "))
            
            # BNO055 센서에서 yaw 값 읽기
            if use_sensor:
                print("    BNO055 센서에서 yaw 값을 읽는 중...")
                try:
                    sensor_data = imu.read_sensor_data(imu_sensor)
                    # read_sensor_data는 (roll, pitch, yaw, ...) 튜플 반환
                    current_yaw = sensor_data[2]  # yaw는 인덱스 2
                    print(f"    센서에서 읽은 Yaw: {current_yaw:.2f}°")
                    time.sleep(0.1)  # 센서 안정화 대기
                except Exception as e:
                    print(f"    센서 읽기 오류: {e}")
                    print("   수동 입력 모드로 전환합니다.")
                    current_yaw = float(input("   현재 Heading (0-360°): "))
                    current_yaw = current_yaw % 360
            else:
                # 수동 입력 모드
                current_yaw = float(input("   현재 Heading (0-360°): "))
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
            
            # 실제 모터 제어 (가능한 경우)
            if use_motor and motor_pi is not None:
                try:
                    parafoil_motor.rotate_parafoil_motor(motor_pi, turn_angle)
                    print(f"    실제 모터 제어 완료: {motor_result['action']}")
                except Exception as e:
                    print(f"    모터 제어 오류: {e}")
            
            # 결과 출력
            print_simulation_result(current_pos, target_pos, current_yaw, motor_result)
            
            step += 1
            
            # 목표 도달 확인
            distance = calculate_distance_haversine(
                current_pos['lat'], current_pos['lon'],
                target_pos['lat'], target_pos['lon']
            )
            
            if distance <= 50.0:
                print(" 목표 지점에 도달했습니다! (50m 반경 내)")
                break
            
        except ValueError:
            print(" 잘못된 입력입니다. 숫자를 입력해주세요.")
        except KeyboardInterrupt:
            print("\n\n시뮬레이션을 종료합니다.")
            break
    
    # 센서 및 모터 종료
    if i2c_instance is not None:
        try:
            imu.imu_terminate(i2c_instance)
            print(" BNO055 IMU 센서 종료 완료")
        except:
            pass
    
    if motor_pi is not None and use_motor:
        try:
            print("\n모터 정지 중...")
            parafoil_motor.rotate_parafoil_motor(motor_pi, 0)  # 모터 정지
            time.sleep(0.5)
            parafoil_motor.terminate_parafoil_motor(motor_pi)
            print(" 파라포일 모터 종료 완료")
        except:
            pass

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

