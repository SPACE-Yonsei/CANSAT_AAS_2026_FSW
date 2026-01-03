import serial
import threading

# Xbee 포트 설정
XBEE_PORT = 'COM6'
XBEE_BAUDRATE = 9600

'''
ports = serial.tools.list_ports.comports()
port_list = [f"{port.device}" for port in ports]
print("port_list: " + str(port_list))
port_num = int(input("Choose Index of port_number (starting from 0): "))
print(f"port:{port_list[port_num]}")


XBEE_PORT = port_list[port_num]
XBEE_BAUDRATE = 9600
'''
# =======================================================
# 센서 데이터를 GROUND STATION으로 보내기. 
# =======================================================
def sender(xbee_serial):
    # 저번 로그 파일에서 데이터 읽기 (검증용)
    with open('test_log.csv', 'r') as f:
        f.readline()  # 첫 번째 줄 건너뛰기
        lines = f.readlines()
    
    # 실제 Payload 에서 사용할 때
    # 1. for 문 삭제 (검증용으로 사용됨)
    # 2. data = 에다가 telemetry packet 넣기기
    for line in lines:
        try:
            data = line
            xbee_serial.write((data + '\n').encode('utf-8'))
    #        time.sleep(0.1)
    
        except Exception as e:
            print(f"[Command Thread] Error: {e}")
            break

# =======================================================
# GROUND STATION의 커맨드 받기.
# =======================================================
def receiver(xbee_serial):
    while True:
        try:
            if xbee_serial.in_waiting:
                # 여기서 command 가 gcs로 부터 받아온 커맨드
                command = xbee_serial.readline().decode('utf-8').strip()
                
                print(f"Received from Xbee: {command}")
                #client_socket.sendall((data + '\n').encode('utf-8'))
        except Exception as e:
            print(f"[Sensor Thread] Error: {e}")
            break
# ======================================================
# 메인 함수 (멀티스레딩 목적)
# 1. Xbee 주소 정의
# 2. (Payload --> GROUND STATION) 센서 데이터 sender    스레드 시작
# 3. (GROUND STATION --> Payload) 커맨드 receiver  스레드 시작
# ======================================================

def main():
    # Xbee 시리얼 포
    xbee_serial = serial.Serial(XBEE_PORT, XBEE_BAUDRATE, timeout=1)

    # 두 개의 스레드 시작
    threading.Thread(target = receiver, args=(xbee_serial,), daemon=True).start()
    threading.Thread(target = sender, args=(xbee_serial,), daemon=True).start()
    # 메인 스레드는 계속 살아 있어야 함
    while True:
        pass

if __name__ == "__main__":
    main()
