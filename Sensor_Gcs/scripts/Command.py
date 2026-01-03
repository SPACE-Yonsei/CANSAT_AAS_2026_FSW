import socket
import time
import serial
import serial.tools.list_ports
import threading

# Xbee 포트 설정
XBEE_PORT = 'COM6'
XBEE_BAUDRATE = 9600

ports = serial.tools.list_ports.comports()
port_list = [f"{port.device}" for port in ports]
print("port_list: " + str(port_list))
port_num = int(input("Choose Index of port_number (starting from 0): "))
print(f"port:{port_list[port_num]}")


XBEE_PORT = port_list[port_num]
XBEE_BAUDRATE = 9600

# TCP 서버 설정 (Serial Studio는 기본적으로 TCP Client)
TCP_HOST = '127.0.0.1'
TCP_PORT = 12345

command_list = ["CX","ST","SIM","SIMP","CAL","MEC"]

# 저번 로그 파일에서 데이터 읽기 (검증용)
with open('test_log.csv', 'r') as f:
    f.readline()  # 첫 번째 줄 건너뛰기
    lines = f.readlines()
'''
#ports = serial.tools.list_ports.comports()
port_list = [f"{port.device}" for port in ports]
print("port_list: " + str(port_list))
port_num = int(input("Choose port_number: "))
print(f"port:{port_list[port_num-1]}")
'''
#=====================================================
def start_tcp_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((TCP_HOST, TCP_PORT))
    server_socket.listen(1)
    print(f"TCP server started at {TCP_HOST}:{TCP_PORT}. Waiting for Serial Studio...")

    client_socket, addr = server_socket.accept()
    print(f"Serial Studio connected from {addr}")
    return client_socket

def sensor_data_receiver(xbee_serial, client_socket):
    while True:
        try:
            if xbee_serial.in_waiting:
                data = xbee_serial.readline().decode('utf-8').strip()
                print(f"Received from Xbee: {data}")
                client_socket.sendall((data + '\n').encode('utf-8'))
        except Exception as e:
            print(f"[Sensor Thread] Error: {e}")
            break


def main():
    # Xbee 시리얼 포트 열기
    xbee_serial = serial.Serial(XBEE_PORT, XBEE_BAUDRATE, timeout=1)

    # TCP 서버 열기 및 Serial Studio 연결 기다리기
    client_socket = start_tcp_server()

    # 두 개의 스레드 시작
    threading.Thread(target=sensor_data_receiver, args=(xbee_serial, client_socket), daemon=True).start()
    threading.Thread(target=command_sender, args=(xbee_serial), daemon=True).start()

    # 메인 스레드는 계속 살아 있어야 함
    while True:
        pass

def command_sender(xbee_serial):
    while True:
        try:
            command = input("Enter command to send to payload: ")
            xbee_serial.write((command + '\n').encode('utf-8'))
        except Exception as e:
            print(f"[Command Thread] Error: {e}")
            break

def open_localhost():
    i = 0
    global TCP_HOST
    global TCP_PORT
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((TCP_HOST, TCP_PORT))
        server_socket.listen(1)  # 한 개의 연결만 허용
        print(f"서버가 {TCP_HOST}:{TCP_PORT}에서 대기 중입니다.")
        
        conn, addr = server_socket.accept()
        with conn:
            print(f"{addr}에서 연결됨")
            while True:
                # 전송할 데이터 생성 (원하는 데이터 형식에 맞게 수정 가능)
                ''' 여기가 수정해야 할 부분 (UART 포트에서 받아오는 data 들어갈 부분)'''
                #data = lines[i].encode('utf-8')
                data = read_data()
                conn.sendall(data)
                print("데이터 전송:", data)
                time.sleep(1)  # 1초 대기 후 재전송
def read_data():
    # UART 포트에서 데이터 읽기
    global port_list
    global port_num
    
    try:
        ser = serial.Serial(ports, 9600, timeout=1)
        # data = ser.readline().decode('utf-8').strip()
        data = ser.readline()
        ser.close()
        return data
    except serial.SerialException as e:
        print(f"Error: {e}")
        return None

#=====================================================
def send_command(command):
    # UART 포트로 명령어 전송
    
    global command_list
    global XBEE_PORT

    if command in command_list:
        # UART 포트 열기
        try:
            ser = serial.Serial(XBEE_PORT, 9600, timeout=1)
            ser.write(command.encode())
            ser.close()
            print(f"Command '{command}' sent successfully.")
        except serial.SerialException as e:
            print(f"Error: {e}")
    else:
        print("Unknown command")
    return

open_localhost()

send_thread = threading.Thread(target=open_localhost, args=())
read_thread = threading.Thread(target=read_data, args=())
send_thread.start()
while True:
    read_thread.start()


# inp = input(f"Enter command : {command_list} : ")
# send_command(inp)

