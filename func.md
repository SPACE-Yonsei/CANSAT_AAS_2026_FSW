================================================================
CANSAT AAS 2026 FSW - 함수 기능 정리
생성일: 2026-04-30
================================================================

----------------------------------------------------------------
[main.py]  오케스트레이터 / 멀티프로세스 관리
----------------------------------------------------------------

run_app(AppID)
  - app_dict에서 해당 AppID의 프로세스를 꺼내 start()
  - AppID 미등록 시 에러 로그

terminate_FSW()
  - MAINAPP_RUNSTATUS = False 세팅
  - 전체 앱에 MID_TerminateProcess 메시지를 Pipe로 전송
  - 3초 타임아웃으로 join, 응답 없으면 terminate() → kill() 순으로 강제 종료
  - prevstate 초기화, 로깅 시스템 shutdown, sys.exit()

restart_app(appID)
  - 죽은 프로세스를 재시작
  - 기존 Pipe 닫고 새 Pipe 생성
  - app_launchers 딕셔너리에서 launcher 함수를 꺼내 새 Process 생성
  - app_dict 업데이트 후 start()
  - 반환값: 성공 True / 실패 False

checkrunstatus()
  - 전체 app_dict 순회
  - pid != None이고 is_alive() == False이면 restart_app() 호출
  - 재시작 실패 시 에러 로그

process_monitor()
  - 3초 주기로 checkrunstatus() 호출하는 백그라운드 스레드 함수
  - MAINAPP_RUNSTATUS == False가 되면 루프 탈출

runloop(Main_Queue)
  - Main_Queue.get()으로 메시지 대기
  - unpack_msg()로 receiver_app 파악
  - app_dict[receiver_app].pipe.send()로 해당 앱 Pipe에 라우팅
  - KeyboardInterrupt 감지 시 terminate_FSW() 호출


----------------------------------------------------------------
[lib/appargs.py]  AppID / MID 정의
----------------------------------------------------------------

  AppID 매핑:
    Main=10, FlightLogic=11, Comm=12, Barometer=13
    IMU=14, GPS=15, Distance=16, Electro=17, Camera=18, Motor=19

  MID 네이밍 규칙: <SenderAppID><ReceiverAppID_2자리><순번_2자리>
    예) BarometerAppArg.MID_motor_alt = 1301901
        (Barometer=13, Motor=19, 순번=01)

  주요 MID:
    MainAppArg.MID_TerminateProcess = 1001001
    FlightlogicAppArg.MID_motor_burnwire = 1101903
    FlightlogicAppArg.MID_motor_EggDrop = 1101904
    FlightlogicAppArg.MID_motor_PullArms = 1101905
    BarometerAppArg.MID_motor_alt = 1301901
    ImuAppArg.MID_motor_imu = 1401901
    GpsAppArg.MID_motor_gps = 1501901


----------------------------------------------------------------
[lib/msgstructure.py]  메시지 직렬화 / 역직렬화
----------------------------------------------------------------

fill_msg(sender, receiver, MsgID, data) → MsgStructure
  - MsgStructure 객체 생성 후 4개 필드 할당
  - data에 '|' 포함 시 에러 로그 후 None 반환

pack_msg(target) → str
  - "sender|receiver|MsgID|data" 문자열로 직렬화
  - 필드 미설정 시 "ERROR" 반환

unpack_msg(msg) → MsgStructure | False
  - '|' 기준 split 후 4 필드 확인
  - fill_msg()로 MsgStructure 복원
  - 파싱 실패 시 False 반환

send_msg(Main_Queue, sender, receiver, MsgID, data) → bool
  - fill_msg → pack_msg → Queue.put() 원스텝 헬퍼
  - 에러 시 False 반환


----------------------------------------------------------------
[lib/events.py]  멀티프로세스 로깅 시스템
----------------------------------------------------------------

init_events_main_process() → log_queue
  - 메인 프로세스 전용 로깅 초기화
  - RotatingFileHandler event의 콘솔 출력 전체를 로그로 남기는 dbg() + 콘솔 핸들러 + 센서별 개별 핸들러 설정
  - QueueListener 시작, log_queue 반환 (서브프로세스에 인자로 전달)

init_events_subprocess(log_queue)
  - 서브프로세스에서 호출
  - QueueHandler를 root logger에 붙여 메인 프로세스 log_queue로 전달

shutdown_events()
  - QueueListener 정지 (메인 프로세스 종료 시 호출)

LogEvent(app_name, event_msg) i
  - logger.dbg
  - 멀티프로세스 환경에서 안전한 유일한 로그 호출 인터페이스


----------------------------------------------------------------
[lib/config.py]  FSW 실행 모드 / 상태 오버라이드 / Yaw 오프셋
----------------------------------------------------------------

  - `lib/config.py`: 
        - GPIO 정의(5: release 솔레노이드, 6: egg 모터, 12, 13: 파라포일 조종 모터 left, right)
        - 센서 통신 속도 정의(I2c hz)
            bmp390: 10 
            bno085: 10(100hz로 읽고 평균내서 10번 -> 10hz)
            gnss 7 click: 10
            ina228: 1
            tf-luna: 10
            xbee: 10
            camera: 30fps

RELAY_ACTIVATE_LEVEL   = GPIO.HIGH  (번와이어/솔레노이드 동일 릴레이 모듈)
RELAY_DEACTIVATE_LEVEL = GPIO.LOW



----------------------------------------------------------------
[lib/prevstate.py]  비행 상태 영속성 (재부팅 복원)
----------------------------------------------------------------

init_prevstate()
  - lib/prevstate.txt 읽어 전역변수 복원
    (PREV_STATE, PREV_ALT_CAL, PREV_MAX_ALT, Target_lat, Target_lon,
     PREV_PACKET_COUNT, PREV_ST_TIMEDELTA)
  - 파일 없으면 기본값으로 새로 생성

STATE_OVERRIDE
  - None: 정상 상태 복원 (prevstate.txt 참조)
  - 0~5 정수: 해당 상태로 강제 초기화
  - lib/config.txt 의 STATE_OVERRIDE= 항목으로 결정

YAW_OFFSET
  - float (도 단위): IMU yaw 0도 기준 보정값
  
`lib/prevstate.py`: 상태/보정값//IMU yaw 오프셋/타깃좌표/카운터 영속화
- 운용 모드(0: LAUNCH_PAD, 1: ASCENT, 2: APOGEE, 3: RELEASE, 4: EGG, 5: LANDED)
- 상태 오버라이드: STATE를 상태로 강제 구동하는 기능이다. 고도로 STATE를 구별하고, 변환하는 기능을 끈다.

reset_control()
  test용


update_prevstate(state)       → PREV_STATE 갱신 후 파일 저장
update_altcal(alt)            → PREV_ALT_CAL 갱신 후 파일 저장
update_maxalt(alt)            → PREV_MAX_ALT 갱신 후 파일 저장
update_target_gps(lat, lon)   → Target_lat/lon 갱신 후 파일 저장 + 로그
update_packet_count(count)    → PREV_PACKET_COUNT 갱신 후 파일 저장 (F1 요구사항)
update_st_timedelta(seconds)  → PREV_ST_TIMEDELTA 갱신 후 파일 저장 (F2 요구사항)
reset_prevstate()             → 모든 값을 NONE/0으로 초기화 후 파일 저장


----------------------------------------------------------------
[comm/commapp.py]  지상국 명령 수신 / TLM 송신
----------------------------------------------------------------

command_handler(recv_msg)
  - MID별 tlm_data 필드 업데이트
  - MID_TerminateProcess → COMMAPP_RUNSTATUS = False
  - MID_comm_alt: pressure, temperature, altitude (F모드에서만 alt 갱신)
  - MID_comm_euler: roll/pitch/yaw (filtered, acc, mag, gyro) 12개 필드
  - MID_comm_gga: gps_time, gps_alt, gps_lat, gps_lon, gps_sats
  - MID_comm_volt: voltage, current, power
  - MID_comm_dis: distance
  - MID_comm_state: state 문자열
  - MID_comm_sim: mode ("S" or "F")

send_tlm(serial_instance)
  - 1Hz 주기로 CSV 텔레메트리 패킷 직렬화 후 UART 송신
  - 패킷 형식: $TEAMID,time,count,mode,state,alt,temp,kPa,volt,curr,pwr,
               gyro_r,gyro_p,gyro_y,acc_r,acc_p,acc_y,mag_r,mag_p,mag_y,
               gps_time,gps_alt,gps_lat,gps_lon,gps_sats,cmd_echo,
               filtered_r,filtered_p,filtered_y
  - TELEMETRY_ENABLE=False 이면 count 증가 없이 전송 생략

read_cmd(Main_Queue, serial_instance)
  - UART에서 한 줄 읽어 정규식 fullmatch로 명령 검증
  - 인식된 명령을 set_cmdecho() 후 cmd_*() 함수로 분기

cmd_cx(option, Main_Queue)    → TELEMETRY_ENABLE ON/OFF 토글
cmd_st(option, Main_Queue)    → set_timedelta() 로 미션 시간 기준 설정
cmd_sim(option, Main_Queue)   → FlightLogic으로 SIM ENABLE/ACTIVATE/DISABLE 라우팅
cmd_simp(option, Main_Queue)  → 기압값 → 고도 변환 후 tlm_data 갱신 + FlightLogic 라우팅
cmd_cal(option, Main_Queue)   → F모드: Barometer CAL 라우팅, S모드: SIMP 오프셋 보정
cmd_mec(option, Main_Queue)   → Motor로 MEC ON/OFF 라우팅
cmd_ss(option, Main_Queue)    → FlightLogic으로 SS 라우팅
cmd_rbt(option, Main_Queue)   → systemctl reboot -i 실행
cmd_cam(option, Main_Queue)   → Camera로 CAM ON/OFF 라우팅
cmd_tc(option, Main_Queue)    → FlightLogic으로 TC lat,lon 라우팅

set_cmdecho(cmd_str)          → 콤마 제거 후 tlm_data.cmd_echo 갱신
get_current_time()            → datetime.now() - ST_timedelta (보정된 미션 시각)
set_timedelta(timestr)        → "HH:MM:SS" 또는 "GPS" → ST_timedelta 갱신

commapp_main(Main_Queue, Main_Pipe)
  - commapp_init() → TlmSender_Thread, CmdReader_Thread 생성/시작
  - Pipe 수신 루프에서 command_handler() 호출


----------------------------------------------------------------
[comm/uartserial.py]  UART I/O (/dev/serial0, 9600 baud)
----------------------------------------------------------------

init_serial() → ser
  - serial.Serial("/dev/serial0", 9600, timeout=1) 오픈 후 반환

send_serial_data(ser, string_to_write)
  - ser.write(string.encode()) 실행
  - is_open 확인 후 각종 예외 조용히 처리

receive_serial_data(ser) → str | None
  - ser.readline() 후 UTF-8 decode (errors='ignore')
  - 빈 데이터 / 포트 닫힘 / 디코딩 실패 → None 반환

terminate_serial(ser)
  - ser.close()


----------------------------------------------------------------
[comm/xbeereset.py]  XBee 하드웨어 리셋
----------------------------------------------------------------

send_reset_pulse()
  - pigpio로 GPIO18을 INPUT(플로팅) → OUTPUT(LOW 100ms) → INPUT(플로팅) 순서로 조작
  - XBee 리셋 펄스 전송 후 pigpio 연결 해제


----------------------------------------------------------------
[flight_logic/flightlogicapp.py]  비행 상태 관리 / 상태 전이
----------------------------------------------------------------

dispatch(msg, queue)
  - MSG_HANDLERS 딕셔너리에서 MID로 핸들러 함수 조회 후 호출

handle_sim(data, queue)       → sim_enable/sim_active 토글, Comm에 "S"/"F"/"A" 전송
handle_simp(data, queue)      → SIM 활성 상태에서만 barometer_logic() 호출
"A"모드가 활성화 되면 모든 app들간의 연결을 검증
"S"시뮬레이션 모드가 되었을 때는 xbee를 통해 아래 명령어들을 실행할 수 있음.
handle_barometer(data, queue) → SIM 비활성 상태에서만 barometer_logic() 호출
handle_distance(data, queue)  → distance_mm 갱신, EGG 상태에서 solenoid_logic() 호출
handle_ss(data, queue)        → xbee로 state를 지정할 수 있도록 설정
handle_reset_alt(data, queue) → max_alt=0, recent_alt 초기화
handle_target_coord(data, queue)
  - "lat,lon" 파싱 후 범위 검증
  - prevstate 저장 + Motor로 MID_motor_TargetCor 전송

barometer_logic(queue, alt)
  - recent_alt 3개 슬라이딩 윈도우로 max_alt 갱신 (2번째 최댓값 기준)
  - 상태별 카운터 증감 및 임계 도달 시 to_*() 호출
    LAUNCH_PAD: alt>200 → cnt_ascent+1, 3회 → to_ascent
    ASCENT:     alt<=max*0.8 → cnt_release+1, 3회 → to_release
                max*0.8<alt<max-0.25 → cnt_apogee+1, 2회 → to_apogee
    APOGEE:     alt<=max*0.8 → cnt_release+1, 3회 → to_release
    RELEASE:    alt<=50m → cnt_release+1, 3회 → to_egg
    EGG:        alt<=4m → cnt_egg_drop+1, 2회 → MID_motor_EggDrop
                alt<=10m → cnt_landed+1, 100회 → to_landed

solenoid_logic(queue, distance_mm)
  - distance_mm <= 2500mm(250cm)이고 solenoid_count < 3이면 MID_motor_EggDrop 전송
  - 3회 완료 후 solenoid_done = True

to_launch_pad(queue, force=False) → state=0, max_alt 초기화, Motor에 state 전송
to_ascent(queue, force=False)     → state=1, Camera MID_cam_activate 전송
to_apogee(queue, force=False)     → state=2
to_release(queue, force=False)    → state=3, MID_motor_burnwire + target 좌표 재전송
to_egg(queue, force=False)        → state=4
to_landed(queue, force=False)     → state=5

send_current_state_thread(queue)
  - 1Hz 주기로 FlightLogic→Comm MID_comm_state 전송

flightlogicapp_main(main_queue, main_pipe)
  - init() 후 send_current_state_thread 스레드 시작
  - Pipe 수신 루프에서 dispatch() 호출

- GPS 속도 타당성 게이트 (>15 m/s 시 마지막 유효값 유지)
----------------------------------------------------------------
[Sensor_Motor/motorapp.py]  모터 앱 메인 / FDIR / 제어 루프
----------------------------------------------------------------

dispatch(msg)
  - MSG_HANDLERS 딕셔너리에서 MID별 handle_* 호출

handle_gps(data)
  - 7개 필드(lat,lon,speed,course,fix_quality,sats,rmc_status, health) 파싱
  
handle_imu(data)
  - 3개 필드(yaw, gyrz, health) 파싱
  - altitude 네임스페이스 업데이트 (update_lock 보호)

handle_barometer(data)
  - 첫 번째 필드를 float로 파싱 → baro_m 업데이트

handle_target_coord(data)
  - "lat,lon" 파싱, 범위 검증, target 네임스페이스 업데이트
  - motor_guidance.set_target_coord() 호출

handle_flight_state(data)
  - 정수 파싱 후 state 변수 업데이트
  - state==3 진입 시 _start_point_locked 리셋 + GPS 유효하면 start_point 확정

handle_release()              → Motor_Release.activate_burnwire() 호출
handle_egg_drop()             → Motor_Egg.activate_solenoid() 호출
handle_mec(data)              → motor_enabled ON/OFF 토글

_snapshot_sensors() → SimpleNamespace
  - update_lock 획득 후 현재 센서 데이터 스냅샷 복사본 반환

_check_fdir(snap) → str | None
  - FDIR-0: GPS/IMU/BARO NaN·Inf·None 체크
  - FDIR-1: IMU 건강 상태 (healthy 플래그)
  - FDIR-2: GPS 수신 freshness (GPS_STALE_TIMEOUT=10s)
  - FDIR-2b: GPS 무결성 (is_gps_valid)
  - FDIR-2c: GPS 순간 이동 (jump_rejected)
  - FDIR-3: 극한 회전 (|gyrz| > 100°/s)
  - FDIR-4: 기압 고도 <= 0
  - FDIR-5: target 좌표 미수신
  - 문제 없으면 None 반환
 
ctrl_paragldr()
  - 0.1s 주기 제어 루프 스레드 함수
  - state>=3이고 motor_enabled일 때만 동작
  - state==5: set_motors_off()
  - _check_fdir() → 이상 시 set_neutral(), 정상 시 guidance() → control()

motorapp_main(main_pipe)
  - init() 후 ctrl_paragldr 스레드 시작
  - Pipe 수신 루프에서 dispatch() 호출


----------------------------------------------------------------
[Sensor_Motor/motor_guidance.py]  L1 Carrot Guidance + Cascaded PI
----------------------------------------------------------------

init_guidance(logger)
  - 모든 상태 변수 초기화

reset_control()
  test용

is_gps_valid(gps_vector, gps_fidelity) → bool
  - lat/lon 비 0, 범위 내, fix_quality>=1, sats>=4, rmc_status=="A" 모두 충족 시 True

is_gps_jump(lat, lon) → bool
  - Haversine 거리 / dt로 속도 계산, GPS_JUMP_MAX_SPEED(200 m/s) 초과 시 True
  - 첫 Fix는 무조건 True (안정화 대기)
  - GPS_STABLE_COUNT_REQUIRED(2회) 누적 후 정상 판정

calculate_distance_haversine(lat1,lon1,lat2,lon2) → float (m)
  - Haversine 공식 거리 계산

set_start_coordinates(lat, lon) → start_point 갱신
set_target_coord(lat, lon)      → target 갱신

_llh_to_en(lat, lon) → (E, N) (m)
  - start_point 기준 East/North 로컬 좌표 변환

_carrot(my_E, my_N, tgt_E, tgt_N) → (carrot_E, carrot_N)
  - L1 경로추적: start_point → target 기준선 위 투영점에서 L_DISTANCE 전방에 carrot 배치
  - s < 0 (기체가 원점 뒤) 시 s_carrot 클램프

_eight(my_E, my_N, tgt_E, tgt_N) → (guide_E, guide_N)
  - figure-eight: LOBE_PERIOD(25s)마다 lobe_sign 반전
  - target 기준으로 수직 방향 RADIUS(25m) 오프셋 배치

_outer_loop(angl_to_turn, V, L) → desired_yaw_rate
  - tanh 기반 포화: 소각에서 L1 감도 매칭, 대각도에서 ±YR_MAX(45°/s) 포화

_yaw_rate_pi_control(desired_yr, measured_yr, dt) → commanded_yaw_rate
  - 조건부 anti-windup 적분 (|u|<MAX_CMD 또는 반대 부호일 때만 누적)
  - 적분 클램프: ±MAX_INTEGRAL(10°/s²)
  - Slew Rate Limiter: MAX_ACCEL(150°/s²) 제한

guidance(imu_data, gps_vector, gps_fidelity, target, baro_m) → SimpleNamespace
  - 고도별 L_DISTANCE 결정: >300m=40m, 150~300m=30m, <150m=15m
  - 고도별 페이즈: >50m HOMING(carrot), 10~50m + 근접 시 PATTERN(figure-8), <10m HOMING
  - |angl_to_turn|>CAPTURE_THRESHOLD(45°): integral 0 리셋
  - 저고도(<=20m): LANDING_YR_MAX(20°/s) 클램프
  - STRAIGHT 페이즈에서 wind_effect EMA 학습 (alpha=0.15)
  - 반환: {state, distance, commanded_yaw_rate}


----------------------------------------------------------------
[Sensor_Motor/motor_control.py]  서보 믹서 / PWM 출력
----------------------------------------------------------------

init_control(logger) → pi
  - pigpio 연결, LEFT(GPIO13)/RIGHT(GPIO12) 서보 중립 위치로 초기화

actuator_mixer(commanded_yaw_rate) → (left_pulse, right_pulse, ...)
  - 동일 방향 믹서: pulse_offset = cmd_yr/2 * K_pulse
  - LEFT/RIGHT 모두 중립에서 동일 방향으로 이동
  - 클램프: LEFT [LEFT_ZERO, LEFT_MAX], RIGHT [RIGHT_MIN, PULSE_MAX]

control(pi, commanded_yaw_rate) → SimpleNamespace
  - actuator_mixer() 호출 후 pigpio PWM 설정
  - 반환: {left_cmd_deg, right_cmd_deg, actual_delta_deg, expected_yaw_rate, left_pulse, right_pulse}

set_neutral(pi)
  - LEFT_NEUTRAL, RIGHT_NEUTRAL로 서보 복귀 (failsafe 위치)

set_motors_off(pi)
  - PWM 폭 0 (신호 완전 차단, LANDED state용)

terminate_parafoil_motor(pi)
  - 중립 → 0 → pigpio.stop()


----------------------------------------------------------------
[Sensor_Motor/Motor_Egg.py]  솔레노이드 (계란 사출)
----------------------------------------------------------------

init_solenoid()
  - GPIO BCM6 OUT, 초기 비활성 레벨 설정
  - atexit에 terminate_solenoid 등록

activate_solenoid()
  - SOLENOID_REPEAT(3)회 반복: HIGH 0.5s → LOW 0.5s

terminate_solenoid()
  - GPIO 비활성 레벨 출력 후 GPIO.cleanup(6)


----------------------------------------------------------------
[Sensor_Motor/Motor_Release.py]  번와이어 (페이로드 사출)
----------------------------------------------------------------

init_burnwire()
  - GPIO BCM5 OUT, 초기 비활성 레벨 설정
  - atexit에 terminate_burnwire 등록

activate_burnwire()
  - HIGH 5초 → LOW (번와이어 가열로 와이어 절단)

terminate_burnwire()
  - GPIO 비활성 레벨 출력 후 GPIO.cleanup(5)


----------------------------------------------------------------
[Sensor_Camera/cameraapp.py]  카메라 앱
----------------------------------------------------------------

command_handler(recv_msg)
  - MID_TerminateProcess: CAMERAAPP_RUNSTATUS = False
  - MID_RouteCmd_CAM ON/OFF: picam_start/stop_recording()
  - MID_cam_activate: picam_start_recording()

picam_start_recording()   → PICAM_RECORDING = True (picam 모듈 공유 플래그 포함)
picam_stop_recording()    → PICAM_RECORDING = False

picam_record_thread(picam_instance, picamencoder_instance)
  - PICAM_RECORDING==True 이면 picam.record(instance, encoder, 7초) 반복 호출
  - False 이면 0.1s sleep으로 대기

cameraapp_main(Main_Pipe)
  - cameraapp_init() → PicamRecorder_Thread 시작
  - Pipe 수신 루프에서 command_handler() 호출


----------------------------------------------------------------
[Sensor_Camera/picam.py]  Picamera2 드라이버
----------------------------------------------------------------

init_cam() → (cam, enc) | (None, None)
  - Picamera2() 초기화 (카메라 미감지 시 None 반환)
  - 640x480 RGB888, 30fps VideoConfiguration 설정
  - H264Encoder 생성 후 cam.start()

record(cam, enc, sec)
  - PICAM_Video/ 폴더에 MP4 파일 생성 (파일명: P_MMDD_HHMMSS.mp4)
  - FfmpegOutput 우선, 없으면 FileOutput (raw H.264)
  - sec초 동안 녹화, PICAM_RECORDING==False 이면 조기 종료

terminate(cam)
  - cam.close()


----------------------------------------------------------------
[Sensor_Barometer/barometerapp.py]  기압 고도계 앱
----------------------------------------------------------------

command_handler(Main_Queue, recv_msg, barometer_instance)
  - MID_TerminateProcess: BAROMETERAPP_RUNSTATUS = False
  - MID_RouteCmd_CAL: CAL 옵션 없으면 현재 고도를 0m 기준으로 BAROMETER_OFFSET 세팅
    + 옵션(ex: CAL,20) 있으면 OFFSET에 해당 값만큼 추가
    + 보정값을 prevstate에 저장, FlightlogicApp에 MaxAlt 리셋 메시지 전송

barometerapp_init() → (i2c_instance, barometer_instance)
  - SIGINT 무시, barometer.init_barometer() 호출
  - prevstate.PREV_ALT_CAL를 BAROMETER_OFFSET 초기값으로 적용
  -현재값을 0m로 설정

barometerapp_terminate(i2c_instance)
  - RUNSTATUS = False, barometer.terminate_barometer() 호출
  - 모든 스레드 join

read_barometer_data(barometer_instance)  [스레드]
  - OFFSET_MUTEX 획득 후 barometer.read_barometer() 10Hz 호출
  - PRESSURE, TEMPERATURE, ALTITUDE 전역 변수 갱신
  - I2C 에러 시 0.1s 대기 후 재시도

send_barometer_data(Main_Queue)  [스레드]
  - 10Hz: FlightlogicApp(MID_flight_alt), MotorApp(MID_motor_alt)에 고도 전송
  - 1Hz(10회마다): CommApp(MID_comm_alt)에 압력/온도/고도 TLM 전송

barometerapp_main(Main_Queue, Main_Pipe)
  - init → 스레드 시작 → Pipe 수신 루프 → command_handler → terminate


----------------------------------------------------------------
[Sensor_Barometer/barometer.py]  BMP390 I2C 드라이버
----------------------------------------------------------------

I2CLock (context manager)
  - fcntl.LOCK_EX|LOCK_NB로 /tmp/i2c-1.lock 파일 잠금
  - I2C_LOCK_TIMEOUT_SEC(2.0s) 초과 시 TimeoutError

_sanitize(value, min_val, max_val) → float|None
  범위 설정

_median(values) → float|None
  - 값 리스트의 중앙값 반환 (홀수 인덱스 기준)

log_barometer(text)
  - sensorlogs/barometer.txt에 타임스탬프와 함께 기록

init_barometer() → (i2c, bmp)
  - BMP3XX I2C 탐색 우회 (zero-write NACK 회피)
  - pressure_oversampling=8, temperature_oversampling=2 설정
  -현재값을 0m로 설정

read_barometer(bmp, offset) → (pressure, temperature, altitude)
  - I2CLock 내에서 읽기, _sanitize로 유효성 검사
  - PRESSURE_WINDOW/TEMPERATURE_WINDOW/ALTITUDE_WINDOW(5샘플) 중앙값 필터
  - altitude -= offset 후 반환

terminate_barometer(i2c)
  - i2c.deinit()
raw값의 보정을 노이즈 없이 확실하게 코딩해줘

----------------------------------------------------------------
[Sensor_Gps/gpsapp.py]  GPS 앱
----------------------------------------------------------------

command_handler(recv_msg)
  - MID_TerminateProcess만 처리: GPSAPP_RUNSTATUS = False

gpsapp_init() → gps_instance
  - SIGINT 무시, gps.init_gps() 호출

gpsapp_terminate()
  - RUNSTATUS = False, 스레드 join, gps.terminate_gps() 호출

read_and_send_gps_data(Main_Queue, gps_instance)  [스레드]
  - 10Hz gps.gps_readdata() 호출 (None이면 이전 값 유지)
  - 새 NMEA 수신 시마다 MotorApp에 즉시 전송(MID_motor_gps):
      GpsVector(LAT,LON,SPEED_MS,COURSE),GpsFidelity(FIX_QUALITY,SATS,RMC_STATUS)
  - 10회마다 CommApp에 TLM 전송(MID_comm_gga):
      TIME,ALT,LAT,LON,SATS

gpsapp_main(Main_Queue, Main_Pipe)
  - init → ReadAndSendGpsData_Thread 시작 → Pipe 수신 루프 → terminate

gps, imu
stale sensor 공통사항
health라는 변수로 stale을 판단
----------------------------------------------------------------
[Sensor_Gps/gps.py]  GNSS 7 Click I2C 드라이버
----------------------------------------------------------------

I2CLock (context manager)  → barometer.py와 동일 구조

log_gps(text)
  - sensorlogs/gps.txt에 타임스탬프 기록

init_gps() → SMBus(1)
  - smbus2 I2C 버스 1 열기 (GNSS 7 Click, addr 0x42)

_i2c_read_block(bus) → bytes
  - 32바이트 블록 읽기 (reg 0xFF 실패 시 0x00으로 재시도)

read_gps(pi, timeout=0.2) → [NMEA_lines]
  - 버퍼에서 '\n' 기준 NMEA 문장 추출
  - 유효 데이터 수신 후 버퍼 소진될 때까지 반복

_validate_nmea_checksum(sentence) → bool
  - '$'~'*' 사이 XOR vs '*' 뒤 2자리 hex 비교

unit_convert_deg(raw_angle) → decimal_degrees
  - DDMM.MMMM → DD.DDDD 변환

gps_readdata(pi) → [time, alt, lat, lon, sats, fix_quality, rmc_status, speed_ms, course] | None
  - read_gps → parse_gps_data → 단위 변환 파이프라인
  - LAT/LON 범위 이탈, fix_quality>8, RMC 상태 이상 시 방어 처리
  - fix 없으면 None 반환 (gpsapp이 이전 값 유지)

terminate_gps(pi)
  - SMBus.close()


----------------------------------------------------------------
[Sensor_Imu/imuapp.py]  IMU 앱
----------------------------------------------------------------

command_handler(recv_msg)
  - MID_TerminateProcess만 처리: IMUAPP_RUNSTATUS = False

imuapp_init() → (i2c_instance, imu_instance)
  - SIGINT 무시, imu.init_imu() 호출
  - 켰을 때 보는 방향을 북쪽(yaw0도)으로 하여금 초기화

imuapp_terminate(i2c_instance)
  - RUNSTATUS = False, imu.imu_terminate() 호출, 스레드 join

read_imu_data(imu_instance)  [스레드]
  - 100Hz imu.read_sensor_data() 호출. 
  - 실패 시 IMU_ERROR_COUNT 증가, 3회 연속 실패 시 imu.reinit_imu() 호출
  - 성공 시 ROLL/PITCH/YAW/ACC/MAG/GYR 전역 변수 갱신 + IPC 업데이트

send_imu_data(Main_Queue)  [스레드]
  - 10Hz: MotorApp(MID_motor_imu)에 YAW, GYRZ, stale 플래그 전송
  - 10회마다: CommApp(MID_comm_euler)에 12축 전체 TLM 전송

imuapp_main(Main_Queue, Main_Pipe)
  - init → ReadImuData_Thread/SendImuData_Thread 시작 → Pipe 수신 루프 → terminate

gps, imu
stale sensor 공통사항
health라는 변수로 stale을 판단
----------------------------------------------------------------
[Sensor_Imu/imu.py]  BNO085 I2C 드라이버
----------------------------------------------------------------

I2CLock (context manager)  → 동일 구조

pulse_bno085_reset()
  - GPIO22 디지털 출력: True(2ms) → False(10ms) → True(650ms)
  - IMU_BNO085_RST_ENABLE=0 환경변수로 비활성화 가능

_apply_gyrz_filter(raw) → filtered_gyrZ
  - 45°/s 이상 순간 변화 시 스파이크로 판정, 이전 EMA 반환
  - EMA alpha=0.3 적용

_wrap_angle_deg(angle) → 0~360 정규화
_angle_diff_deg(target, current) → -180~+180 차이

_circular_mean_deg(values) → 원형 평균 (sin/cos 합산, 0/360 랩어라운드 안전)

_mag_norm_is_valid(mag_norm) → bool
  - MAG_FIELD_MIN(5.0)~MAG_FIELD_MAX(150.0) μT 범위 및 spike ratio 검사

_hampel_filter_angle(window, new_value) → filtered_angle
  - Hampel identifier: 창 내(윈도우=10) MAD 기반 이상값 중앙값으로 대체 후 평균

_filter_mag(mx, my, mz) → (mx, my, mz)
  - MAG_FILTER_ALPHA(0.2) EMA 저역통과 필터

init_imu(i2c=None) → (i2c, sensor)
  - pulse_bno085_reset() 후 최대 5회 재시도
  - BNO085 주소 0x4A/0x4B 자동 탐색
  - 켰을 때 보는 방향을 북쪽(yaw0도)으로 하여금 초기화

read_sensor_data(sensor) → (roll,pitch,yaw,accX..Z,magX..Z,gyrX..Z) | False
  - Quaternion → Euler 변환 (BNO085 i,j,k,real 순서 주의)
  - IMU_MOUNTED_ON_BOTTOM=True: yaw = -yaw
  - IMU_FORWARD_AXIS='Y': yaw += 90
  - config.YAW_OFFSET 적용
  - 자기장 기반 yaw drift 보정 (YAW_CORRECTION_GAIN=0.02)
  - Hampel 필터로 roll/pitch/yaw 출력

imu_terminate(i2c)
  - i2c.deinit()

reset_angle_window()
  - Hampel 필터 슬라이딩 윈도우 초기화

reinit_imu(i2c, sensor) → (i2c, sensor)
  - sensor 해제, 1s 대기, reset_angle_window()
  - init_imu()를 최대 3회 재시도


----------------------------------------------------------------
[Sensor_Imu/Calibrator.py]  BNO055 캘리브레이션 유틸리티
----------------------------------------------------------------

Cal()
  - BNO055를 NDOF_MODE로 설정
  - Magnetometer: figure-eight 동작으로 calibration_status[3]==3 대기
  - Accelerometer: 6면 정적 캘리브레이션 calibration_status[2]==3 대기
  - Gyroscope: 정지 유지로 calibration_status[1]==3 대기
  - 완료 후 offsets_magnetometer/gyroscope/accelerometer를 offset.txt에 기록
  - 반환값: (mag_offsets, gyr_offsets, acc_offsets)
  (현재 BNO085로 교체됨; 이 파일은 구형 BNO055 캘리브레이션 참조용)


----------------------------------------------------------------
[Sensor_Electro/electroapp.py]  전원 모니터 앱
----------------------------------------------------------------

command_handler(recv_msg)
  - MID_TerminateProcess만 처리: ELECTROAPP_RUNSTATUS = False

electroapp_init() → electro_reader | None
  - SIGINT 무시, electro.init_INA228() 호출
  - 실패 시 RUNSTATUS = False

electroapp_terminate()
  - RUNSTATUS = False, 스레드 join

read_electro_data(electro_reader)  [스레드]
  - 1Hz: electro.read_voltage/current/power() 호출
  - ELECTRO_VOLTAGE, ELECTRO_CURRENT, ELECTRO_POWER 갱신
  - I2C 에러 시 1s 대기 후 재시도

send_electro_data(Main_Queue)  [스레드]
  - 1Hz: CommApp(MID_comm_volt)에 V/A/W TLM 전송

electroapp_main(Main_Queue, Main_Pipe)
  - init → ElectroReader_Thread/ElectroSender_Thread 시작 → Pipe 수신 루프 → terminate


----------------------------------------------------------------
[Sensor_Electro/electro.py]  INA228 I2C 드라이버
----------------------------------------------------------------

I2CLock (context manager)  → 동일 구조

log_Electro(text)
  - sensorlogs/electro_ina228.txt에 타임스탬프 기록

init_INA228(address=0x40) → INA228 인스턴스
  - busio.I2C로 연결, INA228 내부 클래스 생성
  - _read_register(reg, length): writeto_then_readfrom으로 레지스터 읽기
  - bus_voltage property: reg 0x02, LSB=195.3125μV
  - shunt_voltage property: reg 0x04 (24bit signed), LSB=312.5nV
  - current property: reg 0x07 (24bit signed), LSB=10nA
  - power property: reg 0x08 (24bit), LSB=3.2μW

read_voltage(dev) → float (V)
read_current(dev) → float (A)
read_power(dev)   → float (W)


----------------------------------------------------------------
[Sensor_Distance/distanceapp.py]  거리 센서 앱
----------------------------------------------------------------

command_handler(recv_msg)
  - MID_TerminateProcess만 처리: DISTANCEAPP_RUNSTATUS = False

distanceapp_init() → tof_sensor | None
  - SIGINT 무시, distance.init_TFLuna() 호출

distanceapp_terminate(tof_sensor)
  - RUNSTATUS = False, distance.terminate_TFLuna() 호출, 스레드 join

read_distance_data(tof_sensor)  [스레드, 10Hz]
  - distance.read_distance() 호출, DISTANCE_MM 갱신
  - 에러 시 0.5s 대기 후 재시도

send_distance_data(Main_Queue)  [스레드, 5Hz]
  - FlightlogicApp(MID_flight_dis): EGG_RELEASE 트리거용 거리 전송
  - CommApp(MID_comm_dis): TLM 전송

distanceapp_main(Main_Queue, Main_Pipe)
  - init → DistanceReader_Thread/DistanceSender_Thread 시작 → Pipe 수신 루프 → terminate


----------------------------------------------------------------
[Sensor_Distance/distance.py]  TF-Luna I2C 드라이버
----------------------------------------------------------------

I2CLock (context manager)  → 동일 구조

_median_mm(values) → int  중앙값 (정수 거리용)

init_TFLuna(address=0x10) → TFLuna 인스턴스 | None
  - busio.I2C 연결, 내부 TFLuna 클래스 생성
  - _read_register(reg, length): writeto_then_readfrom 래퍼
  - read_distance_cm(): reg 0x00~0x01 리틀엔디안 16bit
  - read_distance_mm(): cm × 10
  - read_flux(): 신호 강도 (reg 0x02~0x03)
  - read_temperature(): reg 0x04~0x05, 0.01°C 단위

read_distance(sensor) → int (mm)
  - 유효 범위 [200, 8000] mm 외 무효 샘플 거부
  - DISTANCE_FILTER_WINDOW(5) 슬라이딩 중앙값 필터 적용

terminate_TFLuna(sensor)
  - sensor.unlock() 후 _sensor = None

init_distance_sensor / terminate_distance_sensor
  - init_TFLuna / terminate_TFLuna 하위 호환 별칭


----------------------------------------------------------------
[Sensor_Motor/motor_logger.py]  제어 로깅 유틸리티
----------------------------------------------------------------
모터 제어에 필요한 변수들을 feedback이 가능한 형태로 남긴다.


----------------------------------------------------------------
[Sensor_Motor/set_motor_angle.py]  서보 각도 테스트 스크립트
----------------------------------------------------------------

(독립 실행 스크립트, FSW에 import 되지 않음)
  - LEFT(GPIO13) 0→120도, RIGHT(GPIO12) 0→120도 순서로 7s 간격 이동 확인
  - LEFT_ZERO=600μs, RIGHT_ZERO=2500μs, pulse_per_degree=2000/180
  - 완료 후 PWM 신호 차단 (pulsewidth=0) 및 pigpio.stop()


----------------------------------------------------------------
[startup.sh]
----------------------------------------------------------------
  1. sudo pigpiod 시작 (GPIO PWM 데몬)
  2. venv 활성화 (source .venv/bin/activate)
  3. python3 main.py 실행


----------------------------------------------------------------
[cansat-fsw.service]
----------------------------------------------------------------
  - Type=simple, ExecStart=python3 main.py
  - Restart=on-failure 정책
  - 비정상 종료 시 자동 재시작


================================================================
END
================================================================
