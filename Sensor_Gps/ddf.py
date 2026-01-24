# main.py
  from multiprocessing import Manager

  manager = Manager()
  app_queues = manager.dict({
      appargs.BarometerAppArg.AppID: manager.Queue(maxsize=100),
      appargs.FlightlogicAppArg.AppID: manager.Queue(maxsize=100),
      appargs.MotorAppArg.AppID: manager.Queue(maxsize=100),
      # ... 모든 앱
  })

  # 각 앱 시작 시 전달
  def barometerapp_launcher(my_queue, app_queues, log_queue):
      barometerapp.barometerapp_main(my_queue, app_queues)

  barometerapp_elements.process = Process(
      target=barometerapp_launcher,
      args=(app_queues[BarometerAppArg.AppID], app_queues, log_queue)
  )

  ---
  barometerapp.py:

  # 전역 변수
  APP_QUEUES = None

  def barometerapp_main(My_Queue, App_Queues):
      global APP_QUEUES
      APP_QUEUES = App_Queues

      # 받기 스레드
      thread_dict["Receiver"] = threading.Thread(target=receive_messages, args=(My_Queue,))
      # 기존 스레드들...

  def send_barometer_data():
      msg = msgstructure.fill_msg(
          appargs.BarometerAppArg.AppID,
          appargs.FlightlogicAppArg.AppID,
          MID_SendBarometerFlightLogicData,
          f"{ALTITUDE}"
      )
      packed = msgstructure.pack_msg(msg)

      try:
          # ✅ 직접 전송, non-blocking, timeout
          APP_QUEUES[appargs.FlightlogicAppArg.AppID].put(
              packed,
              block=False  # ❗ 중요: 안 막힘
          )
      except Full:
          # FlightLogic이 멈춰서 Queue 가득 참
          events.LogEvent("Baro", EventType.warning, "FlightLogic queue full, dropping message")
          # ✅ Baro는 계속 동작! (메시지만 버림)

  def receive_messages(My_Queue):
      while BAROMETERAPP_RUNSTATUS:
          try:
              msg = My_Queue.get(timeout=1)
              recv_msg = msgstructure.unpack_msg(msg)
              if recv_msg:
                  command_handler(recv_msg)
          except Empty:
              continue

  ---
  장점 ✅

  1. 가독성 최고:
  APP_QUEUES[FlightlogicAppArg.AppID].put(msg)  # 명확!
  2. Fault Isolation 유지:
    - block=False → Queue full이면 메시지 버림
    - FlightLogic 멈춰도 Baro 계속 동작
  3. main.py 단순화:
  # main.py는 termination만 처리
  for appID in app_dict:
      app_queues[appID].put(termination_msg)
  4. 성능 개선:
    - main.py 우회
    - 직접 통신

  ---
  msgstructure.py 수정 (선택):

  # 기존
  def send_msg(Main_Queue, _sender, _receiver, _MsgID, _data):
      target = fill_msg(_sender, _receiver, _MsgID, _data)
      msg_to_send = pack_msg(target)
      Main_Queue.put(msg_to_send)  # ❌ 하나의 queue만

  # 새로운
  def send_msg(App_Queues, _sender, _receiver, _MsgID, _data):
      target = fill_msg(_sender, _receiver, _MsgID, _data)
      msg_to_send = pack_msg(target)
      try:
          App_Queues[_receiver].put(msg_to_send, block=False)  # ✅ 직접 전송
          return True
      except Full:
          events.LogEvent("MsgStructure", EventType.warning, f"Queue full for {_receiver}")
          return False