import serial
import socket
import threading
import time
import serial.tools.list_ports
import Command_function

# Xbee 포트 설정

ports = serial.tools.list_ports.comports()
port_list = [f"{port.device}" for port in ports]

# 포트가 없으면 종료
if len(port_list) == 0:
    print("Error: No serial ports found. Please connect XBee and try again.")
    exit(1)

# 포트 목록 출력
print("\n사용 가능한 포트 목록:")
for i, port in enumerate(port_list):
    print(f"  {i}: {port}")

# 유효한 포트 번호 입력 받기
while True:
    try:
        port_num = int(input(f"\n포트 번호를 선택하세요 (0-{len(port_list)-1}): "))
        if 0 <= port_num < len(port_list):
            break
        else:
            print(f"Error: 잘못된 번호입니다. 0부터 {len(port_list)-1} 사이의 숫자를 입력하세요.")
    except ValueError:
        print("Error: 숫자를 입력하세요.")
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
        exit(0)

XBEE_PORT = port_list[port_num]
XBEE_BAUDRATE = 9600
print(f"\n선택된 포트: {XBEE_PORT}")

# TCP 서버 설정 (Serial Studio는 기본적으로 TCP Client)
TCP_IP = '127.0.0.1'
TCP_PORT = 12345

# Command 리스트 (Payload에서 사용할 수 있는 커맨드)
Command_list = ["CX","ST","SIM","SIMP","CAL","MEC"]

# ======================================================
# TCP 서버 열기 및 Serial Studio 연결 기다리기
# ======================================================
def start_tcp_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((TCP_IP, TCP_PORT))
    server_socket.listen(1)
    print(f"TCP server started at {TCP_IP}:{TCP_PORT}. Waiting for Serial Studio...")

    client_socket, addr = server_socket.accept()
    print(f"Serial Studio connected from {addr}")
    return client_socket

# ======================================================
# Xbee를 통해서 받은 센서 데이터를 Serial Studio 로 보내기. (정확힌 서버에 뿌리기)
# ======================================================
def sensor_data_receiver(xbee_serial, client_socket):
    while True:
        try:
            if xbee_serial.in_waiting:
                data = xbee_serial.readline().decode('utf-8').strip()
                #print(f"Received from Xbee: {data}")
                client_socket.sendall((data + '\n').encode('utf-8'))
        except Exception as e:
            print(f"[Sensor Thread] Error: {e}")
            break

# ======================================================
# 커맨드를 입력받아서 Xbee를 통해 Payload에 전송.
# ======================================================
def command_sender(xbee_serial):
    global Command_list
    team_number = 3139
    print("Command List: " + str(Command_list))

    while True:

        try:
            command = input("Enter command to send to payload: ")
            
            # Xbee로 커맨드 전송
            
            # CX Command
            if command == "CX":
                # CX 커맨드에 대한 추가 입력 받기
                command = Command_function.CX_command()
                if command is None:
                    continue
                xbee_serial.write((command + '\n').encode('utf-8'))

            # Set time command
            elif command == "ST":
                    # CX 커맨드에 대한 추가 입력 받기
                command = Command_function.ST_command()
                if command is None:
                    continue
                xbee_serial.write((command + '\n').encode('utf-8'))

            # SIM Command
            elif command == "SIM":
                send_hertz = 1 # Hz
                command = Command_function.SIM_command()
                if command is None:
                    continue
                for each_command in command:
                    xbee_serial.write((each_command + '\n').encode('utf-8'))
                    time.sleep(1/send_hertz)

            elif command == "CAL":
                command = Command_function.CAL_command()
                xbee_serial.write((command + '\n').encode('utf-8'))
            
            elif command == "MEC":
                command = Command_function.MEC_command()
                if command is None:
                    continue
                xbee_serial.write((command + '\n').encode('utf-8'))
            else:
                print("Unknown command. Please enter a valid command.")
                continue

            # Xbee로 커맨드 전송
            
            
        except Exception as e:
            print(f"[Command Thread] Error: {e}")
            break
# ======================================================
# 메인 함수 (멀티스레딩 목적)
# 1. Xbee 주소 정의
# 2. TCP 서버 열기
# 3. (Payload --> GROUND STATION) 센서 데이터 recieve 스레드 시작
# 4. (GROUND STATION --> Payload) 커맨드      sender 스레드 시작
# ======================================================

def main():
    # Xbee 시리얼 포트 열기
    xbee_serial = serial.Serial(XBEE_PORT, XBEE_BAUDRATE, timeout=1)

    # TCP 서버 열기 및 Serial Studio 연결 기다리기
    client_socket = start_tcp_server()

    # 두 개의 스레드 시작
    threading.Thread(target=sensor_data_receiver, args=(xbee_serial, client_socket), daemon=True).start()
    threading.Thread(target=command_sender, args=(xbee_serial,), daemon=True).start()

    # 메인 스레드는 계속 살아 있어야 함
    while True:
        pass

# 수동 or import 용 구분
if __name__ == "__main__":
    main()
