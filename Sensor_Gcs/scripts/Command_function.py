import re
team_number = 3139
sim_status = 0

def CX_command():
    global team_number
        # CX 커맨드에 대한 추가 입력 받기
    inp = input("Input the ON/OFF : ")
    command_pattern = r'[ON|OFF]'
    if re.search(command_pattern, inp):
        command = f"CMD,{team_number},CX,{inp}"
    else:
        print("Invalid input. Please enter ON or OFF.")
        return None
    return command
        
def ST_command():
    global team_number
    inp = input(" UTC : put the time that you want (hh:mm:ss)\n GPS : put just 'GPS' \nInput How to set the time : ")
    command_pattern = r"([01]\d|2[0-3])(:[0-5]\d){2}|GPS"
    
    if inp == "GPS":
            command = f"CMD,{team_number},ST,{inp}"
    elif re.search(command_pattern, inp):
        command = f"CMD,{team_number},ST,{inp}"
    else:
        print("Invalid input. Please enter a valid time format. or 'GPS'")
        return None
    return command

def CAL_command():
    global team_number
    command = f"CMD,{team_number},CAL"
    return command

def MEC_command():
    global team_number
    inp1 = input("Input Device (MOTOR, CAMERA) : ")
    inp2 = input("Input the ON/OFF : ")
    command_pattern = r'[ON|OFF]'

    if re.search(command_pattern, inp2) and inp1 in ["MOTOR", "CAMERA"]:
        command = f"CMD,{team_number},MEC,{inp1},{inp2}"
    else:
        print("Invalid input. Please enter correct Device / enter ON or OFF.")
        return None
    return command

def SIM_command():
    global team_number
    global sim_status
    result = []
    inp = input("Input the ENABLE(E), ACTIVATE(A), or DISABLE(D) : ")
    if inp == "ENABLE" or inp == "E":
        sim_status = 1
    elif inp == "DISABLE" or inp == "D":
        sim_status = 0
    elif inp == "ACTIVATE" or inp == "A":
        if sim_status == 0:
            print("Simulation mode is not enabled. Please enable it first.")
            return None

        file_path = input("Enter the path to the CSV file: ")
        # 파일이 존재하는지 확인
        try:
            with open(file_path, 'r') as f:
                f.readline()  # 첫 번째 줄 건너뛰기
                lines = f.readlines()
        except FileNotFoundError:
            print(f"File {file_path} not found.")
            return None
        
        print("File read successfully. Sending data to payload...")
        # 데이터 파일을 읽고 array로 저장 1초에 한번씩 보내기 구현해야 됨!!
        
        # GCS에서 PAYLOAD로 pressure data 를 주기적으로 보내기
        for line in lines:
            if line[0:3] == "CMD":
                line = line[0:4] + str(team_number) + line[5:]
                result.append(line.strip())
        return result