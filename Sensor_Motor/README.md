## 공지사항 ##
https://github.com/jmpark3972/CANSAT_AAS_2025_Motor  의 파일들을
https://github.com/SPACE-Yonsei/CANSAT_AAS_2025_Motor   로 그대로 fork했습니다.
수정 시에는 https://github.com/jmpark3972/CANSAT_AAS_2025_Motor 수정 후 https://github.com/SPACE-Yonsei/CANSAT_AAS_2025_Motor에서 update branch 부탁드립니다.

## 하드웨어 연결 ##
모터에서 갈색은 gnd 주황은 5v 노랑은 gpio 연결하기(ex gpio18)

## 할 일 ##
모터 정확도 확인 결과 10회전에 약 1도 이하로 오차 생길정도로 정확합니다.

돌아간다.py에서 자기장에 따라 모터가 잘 작동함을 확인하였으며,
motor.py가 자기장 값으로 작동하는 코드입니다.
motor_desc.py는 같은 코드에 상세한 설명이 있습니다.



######
페이로드의 모터 motor_zero.py
컨테이너의 모터 motor_contain.py
로켓의 모터 motor_rocket_payload.py
