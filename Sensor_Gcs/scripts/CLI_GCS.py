import serial
import socket
import threading
import time
import re

# Xbee 포트 설정
XBEE_PORT = 'COM7'
XBEE_BAUDRATE = 9600

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
    while True:
        try:
            command = input("Enter command to send to payload: ")
            
            # Xbee로 커맨드 전송
            
            # CX Command
            if command == "CX":
                # CX 커맨드에 대한 추가 입력 받기
                inp = input("Input the ON/OFF : ")
                command_pattern = r'[ON|OFF]'
                if re.search(command_pattern, inp):
                    command = f"CMD,{team_number},CX,{inp}"
                else:
                    print("Invalid input. Please enter ON or OFF.")
                    continue
                xbee_serial.write((command + '\n').encode('utf-8'))
            # Set time command
            elif command == "ST":
                inp = input(" UTC : put the time that you want (hh:mm:ss)\n GPS : put just 'GPS' \nInput How to set the time : ")
                command_pattern = r"([01]\d|2[0-3])(:[0-5]\d){2}|GPS"
                
                if inp == "GPS":
                        command = f"CMD,{team_number},CX,{inp}"
                elif re.search(command_pattern, inp):
                    command = f"CMD,{team_number},CX,{inp}"
                else:
                    print("Invalid input. Please enter a valid time format. or 'GPS'")
                    continue
                xbee_serial.write((command + '\n').encode('utf-8'))
            # Simulation mode
            elif command == "SIM":
                inp = input("Input the ENABLE, ACTIVATE, or DISABLE : ")
                if inp == "ENABLE":
                    sim_status = 1
                elif inp == "DISABLE":
                    sim_status = 0
                elif inp == "ACTIVATE":
                    if sim_status == 0:
                        print("Simulation mode is not enabled. Please enable it first.")
                        continue

                    file_path = input("Enter the path to the CSV file: ")
                    # 파일이 존재하는지 확인
                    try:
                        with open(file_path, 'r') as f:
                            f.readline()  # 첫 번째 줄 건너뛰기
                            lines = f.readlines()
                    except FileNotFoundError:
                        print(f"File {file_path} not found.")
                        continue
                    
                    print("File read successfully. Sending data to payload...")
                    
                    # GCS에서 PAYLOAD로 pressure data 를 주기적으로 보내기
                    for line in lines:
                        line_pattern = r"CMD,\$,SIMP,\d{5,6}"
                        if re.search(line_pattern, line):
                            line = line[0:4] + str(team_number) + line[5:]
                            xbee_serial.write((line + '\n').encode('utf-8'))
                            time.sleep(1)
                        
                else:
                    print("Invalid input. Please enter ENABLE, DISABLE, or ACTIVATE.")
                    continue
                    
            elif command == "CAL":
                command = f"CMD,{team_number},CAL"
                xbee_serial.write((command + '\n').encode('utf-8'))
            
            elif command == "MEC":
                inp1 = input("Input Device (MOTOR, CAMERA) : ")
                inp2 = input("Input the ON/OFF : ")
                command_pattern = r'[ON|OFF]'

                if re.search(command_pattern, inp2) and inp1 in ["MOTOR", "CAMERA"]:
                    command = f"CMD,{team_number},MEC,{inp1},{inp2}"
                    xbee_serial.write((command + '\n').encode('utf-8'))
                else:
                    print("Invalid input. Please enter correct Device / enter ON or OFF.")
                    continue
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
