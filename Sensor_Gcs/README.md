# CANSAT_AAS_2025_GCS
2025 CANSAT COMPETITION Ground station

### Prerequisities
***Must Create*** a new virtual environment
***Must use*** Python 3.9

### Library
pip3 install PyQt6
pip3 install pyserial

반드시 pyserial (serial 패키지가 깔렸다면 삭제 후 다시 설치해야 됨.)

### Usage
1. GCS.py 실행
2. Serial Port 지정, Port 연결
3. Serial Studio 실행
4. TCP port 지정, Connect to serial studio 버튼 누르기
5. 10초 안에 Serial Studio에서 TCP 연결 하기
6. Good to go!

### Editing Command / Device List
commands 폴더 안에 지상국에서 사용하는 command, device 목록이 적혀있는 .txt 파일들이 있습니다.
파일 안에 쓰여있는 규칙에 따라 원하는 command, device를 추가하고, GCS의 Refresh command, device 버튼을 누르면 적용됩니다.

### Sending Command
GCS에서는 편리하게 미리 지정된 Command를 보낼 수 있습니다.
1. device list에서 device 지정
2. command header list에서 command header 지정
3. option list에서 option 지정
4. send 버튼 누르기
