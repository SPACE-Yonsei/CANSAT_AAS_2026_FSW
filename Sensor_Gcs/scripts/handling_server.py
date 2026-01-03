import serial
import socket
import threading
import time
import re

# TCP 서버 설정 (Serial Studio는 기본적으로 TCP Client)
TCP_IP = '127.0.0.1'
TCP_PORT = 12345

TCP_SERVER_ISOPEN = False

# ======================================================
# TCP 서버 열기 및 Serial Studio 연결 기다리기
# ======================================================
def start_tcp_server(port):
    global TCP_SERVER_ISOPEN

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((TCP_IP, port))
    server_socket.listen(1)
    print(f"TCP server started at {TCP_IP}:{port}. Waiting for Serial Studio...")

    server_socket.settimeout(10)  # Set timeout to 10 seconds
    try:
        client_socket, addr = server_socket.accept()

    except socket.timeout:
        print("Timeout occurred while waiting for Serial Studio.")
        return None
    
    print(f"Serial Studio connected from {addr}")
    TCP_SERVER_ISOPEN = True
    return server_socket, client_socket

# ======================================================
# Xbee를 통해서 받은 센서 데이터를 Serial Studio 로 보내기. (정확힌 서버에 뿌리기)
# ======================================================
def sensor_data_receiver(xbee_serial:serial.Serial, client_socket:socket.socket):
    global TCP_SERVER_ISOPEN
    while TCP_SERVER_ISOPEN:
        if not xbee_serial.is_open:
            print("Serial port not opened! Open Serial port")
            time.sleep(1)
            continue

        try:
            if xbee_serial.in_waiting:
                data = xbee_serial.readline().decode('utf-8', errors="ignore").strip()
                
                print(f"Received from Xbee: {data}")
                client_socket.sendall((data + '\n').encode('utf-8', errors="ignore"))
                
        except Exception as e:
            print(f"[Sensor Thread] Error: {e}")
            break
    print("Terminating Telemetry Receiver Thread")

# ======================================================
# TCP 연결 종료
# ======================================================
def terminate_connection(client_socket: socket.socket):
    global TCP_SERVER_ISOPEN
    try:
        TCP_SERVER_ISOPEN = False
        client_socket.close()
        print("Connection with Serial Studio terminated.")
    except Exception as e:
        print(f"Error while terminating connection: {e}")

# ======================================================
# TCP 서버 소켓 종료
# ======================================================
def terminate_server_socket(server_socket: socket.socket):
    global TCP_SERVER_ISOPEN
    try:
        TCP_SERVER_ISOPEN = False
        server_socket.close()
        print("TCP server socket closed.")
    except Exception as e:
        print(f"Error while closing server socket: {e}")

issendingsimpdata = False

def simp_data_sender_thread(serial_port:serial.Serial, simp_file_path, team_name):
        global issendingsimpdata

        # 파일이 존재하는지 확인
        try:
            with open(simp_file_path, 'r') as f:
                f.readline()  # 첫 번째 줄 건너뛰기
                lines = f.readlines()
        except FileNotFoundError:
            print(f"File {simp_file_path} not found.")
            return
        
        print("File read successfully. Sending data to payload...")
        
        # GCS에서 PAYLOAD로 pressure data 를 주기적으로 보내기
        for line in lines:
            if issendingsimpdata == False:
                print("SIMP sender thread : Stopping SIMP sending")
                break
            
            if line[0] == "#" or not line.strip():
                continue

            line_pattern = r"CMD,\$,SIMP,\d{5,6}"
            if re.search(line_pattern, line):
                simp_data = re.search(r"\d{5,6}", line).group()
                line = line[0:4] + str(team_name) + line[5:] + "\n"

                simp_to_send = line.encode()
                if serial_port.is_open:
                    serial_port.write(simp_to_send)
                    print(f"Sent SIMP DATA : {simp_data}, command : {simp_to_send.hex()}")
                else:
                    print("ERROR sending simp data : Port not opened!")
            else:
                    print("Error reading line!")
            time.sleep(1)

        return