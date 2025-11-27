for error in list(range(10, 100, 10)):
            # 왼쪽 모터는 530에서 2470으로, 오른쪽 모터는 2470에서 530으로 움직이도록 계산
            left_pulse = int(2500 - (10/9) * error)
            right_pulse = int(500 + (10/9) * error)
            
            print(f"왼쪽 모터 펄스: {left_pulse}")                
            print(f"Error: {error:2d} -> Left Pulse: {left_pulse}, Right Pulse: {right_pulse}")
            pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
            pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
            time.sleep(0.1)
