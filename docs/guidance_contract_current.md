# guidance.py current contract

이 문서는 현재 `Sensor_Motor/guidance.py` 구현을 정적 분석한 기준 계약이다.
테스트와 replay harness는 이 문서의 cycle ownership을 따라야 한다.

## 1. `DecideControlMode` 실제 호출 순서

`DecideControlMode(gps, imu, baro, now)`는 현재 guidance cycle의 단일 진입점이다.
실제 호출 순서는 다음과 같다.

1. `UpdateRaw(gps, imu, baro, now)`
2. `flags = ComputeFreshFlags(now)`
3. `FillNav(flags, now)`
4. `return _STATE_t.nav.control_mode`

호환 경로로 `DecideControlMode(now)` 형태도 허용된다. 이 경우 `gps`, `imu`,
`baro`는 모두 `None`으로 처리되고, 기존 raw state의 timestamp freshness만으로
`ComputeFreshFlags`와 `FillNav`가 수행된다.

## 2. `SelectControlMode` 제거 상태

`SelectControlMode(flags, now)`는 현재 production API에서 제거되었다. 이 함수는
독립적인 selector가 아니라 `FillNav(flags, now)`를 다시 호출한 뒤
`_STATE_t.nav.control_mode`를 반환하던 compatibility wrapper였다.

실제 FSW cycle의 단일 진입점은 `DecideControlMode(gps, imu, baro, now)`다.
`DecideControlMode` 내부에서 이미 `UpdateRaw -> ComputeFreshFlags -> FillNav`가
수행되므로, 같은 cycle에서 외부 코드가 `FillNav(flags, now)`를 다시 호출하면
mode/nav 계산이 중복 수행된다.

특히 GPS position이 stale이고 DR current가 valid인 `DR_PM_*` 경로에서는
`_fill_nav_for_dr_pm_mode`가 `dr.current_E/N`을 적분하므로, 같은 cycle의 중복
`FillNav` 호출은 DR position을 이중 적분할 수 있다.

cycle pipeline에서는 `DecideControlMode` 이후 `FillNav`를 다시 호출하지 않는다.

## 3. `FillNav` 역할

`FillNav(flags, now)`는 mode 선택과 `NavState` 작성을 한 번에 수행하는 함수다.
현재 구조에서 mode와 nav는 분리된 단계가 아니라 같은 함수 안에서 결정된다.

주요 책임은 다음과 같다.

- cycle 시작 시 `_reset_nav_for_cycle(now)`로 nav를 초기화한다.
- origin/target readiness를 확인한다.
- origin 또는 target이 없으면 `_set_nav_fail("NO_ORIGIN" | "NO_TARGET", flags)`로
  실패 상태를 설정한다.
- GPS position과 GPS motion이 모두 fresh이면 `GPS_TRACKING_*` nav를 작성한다.
  이 경로에서는 GPS E/N, GPS V/course, vE/vN을 nav에 채우고 `dr_lock`으로 DR
  anchor/current를 GPS 기준으로 재고정한다.
- GPS position은 fresh이고 GPS motion은 stale이면 `DR_M_*` nav를 작성한다.
  이 경로에서는 position은 GPS E/N을 사용하고, course/speed/vE/vN motion만 DR
  추정으로 보완한다.
- GPS position이 stale이고 DR current가 valid이면 `DR_PM_*` nav를 작성한다.
  이 경로에서는 position과 motion이 모두 `dr.current`에서 누적/갱신된다.
- 각 실패 조건에서 `NO_GUIDANCE_SOURCE`, `NO_COURSE_SOURCE`,
  `NO_SPEED_SOURCE`, `DR_TIMEOUT`, `DR_POSITION_JUMP`, `GPS_POS_NAN` 등의
  fail reason을 `_set_nav_fail`로 설정한다.
- 성공 경로에서는 `_finish_nav(mode, reason, flags, now)`를 통해
  `nav.control_mode`, `nav.valid`, `flags.nav_valid`, DR valid flags를 함께
  확정한다.

즉, `FillNav`는 `nav.control_mode`와 `nav.valid`를 같은 cycle에서 atomic하게
설정하는 소유자다.

## 4. `ProduceL1Input` 역할

`ProduceL1Input(now)`는 더 이상 `DR_M_*` 또는 `DR_PM_*` 계산을 수행하지 않는다.
DR course, speed, position 적분은 `FillNav` 내부에서 이미 끝난 상태여야 한다.

현재 역할은 다음과 같다.

- `_STATE_t.nav.control_mode`를 읽는다.
- mode가 `FAIL`이면 `L1Input(valid=False, reason=nav.fail_reason or "FAIL")`을
  반환한다.
- origin이 없으면 `NO_ORIGIN`, target이 없으면 `NO_TARGET` reason으로 invalid
  `L1Input`을 반환한다.
- GPS tracking mode이면 reason을 `GPS_NAV`로 둔다.
- DR mode이면 reason을 mode value로 둔다.
- 최종 변환은 `_make_l1input_from_nav(mode, reason)`에 위임한다.

`_make_l1input_from_nav`는 이미 채워진 `NavState`를 `L1Input`으로 복사하고,
target NE 좌표, confidence, DR method를 채운다. nav E/N/V/course 또는 target이
유효하지 않거나 `V < config.V_MIN_MPS`이면 `NAV_INVALID`를 반환한다.

중요한 테스트 계약:

- `ProduceL1Input`이 DR을 다시 계산한다고 가정하지 않는다.
- `ProduceL1Input` 호출 전에는 같은 cycle의 `DecideControlMode`가 이미
  `FillNav`를 통해 nav를 채웠어야 한다.
- `ProduceL1Input` 호출은 DR current를 적분하거나 mode를 새로 선택하지 않아야
  한다.

## 5. `ProduceL1Output` 역할

`ProduceL1Output(l1in)`은 `L1Input`을 L1 guidance output으로 변환한다.

현재 역할은 다음과 같다.

- 출력 생성 직후 mode별 yaw-rate limit을 채운다.
  - `yaw_rate_limit`은 rad/s
  - `yaw_rate_limit_dps`는 deg/s
  - invalid 경로에서도 control layer가 finite한 limit을 읽을 수 있게 먼저
    설정된다.
- `l1in.valid == False`이면 해당 reason으로 invalid output을 반환한다.
- `ControlMode.FAIL`이면 `FAIL` reason으로 invalid output을 반환한다.
- E/N/target_E/target_N/course/V 중 하나라도 non-finite이면
  `NAN_NAV_STATE`를 반환한다.
- `V < config.V_MIN_MPS`이면 `V_TOO_SMALL`을 반환한다.
- DR mode에서 confidence가 `DR_MIN_CONFIDENCE_FOR_CONTROL`보다 낮으면
  `LOW_DR_CONFIDENCE`를 반환한다.
- 유효한 입력에서는 다음 값을 계산한다.
  - `distance_to_target`, `dist_to_target`
  - `target_bearing`
  - `nu`
  - `yaw_rate_cmd`
- yaw-rate command는 `2 * V_eff / L_GAIN_M * sin(nu)` 형태의 L1 command이며,
  mode별 yaw-rate limit으로 clamp된다.
- DR mode에서는 `confidence` scaling이 yaw-rate command에 적용된다.
  GPS tracking mode에서는 confidence를 1.0으로 사용한다.
- `pid_enabled`는 `_mode_uses_gyro_feedback(l1in.control_mode)`로 결정된다.
  결과적으로 GPS closed 및 DR closed mode는 PID feedback enabled이고,
  open mode는 disabled다.

## 6. 기존 테스트 수정 지침

현재 `guidance.py` 구조 기준으로 기존 테스트와 replay harness는 다음 원칙을
따라야 한다.

- `ProduceL1Input`이 DR_M/DR_PM 계산을 다시 수행한다고 가정하지 않는다.
  DR 계산 검증은 `DecideControlMode -> FillNav` 이후 채워진 `NavState`와
  `DRState`를 기준으로 한다.
- 같은 cycle에서 `DecideControlMode` 이후 `FillNav`를
  중복 호출하지 않는다. `DR_PM_*`에서는 같은 timestamp라도 중복 적분 위험이
  있다.
- 각 cycle의 mode를 테스트 코드에서 강제로 설정하지 않는다.
  `guidance._STATE_t.nav.control_mode = ...` 방식은 현재 계약과 맞지 않는다.
- DR trace나 scenario test에서는 mode를 직접 만들지 말고, GPS/IMU/baro sensor
  schedule을 조작해 `DecideControlMode`가 자연스럽게 mode를 선택하게 한다.
- `FillNav`, `DecideControlMode`, `ProduceL1Input`의 내부
  production 로직을 테스트 코드에 복사하지 않는다.
- 검증 pipeline은 다음 순서를 유지한다.
  1. sensor schedule에서 GPS/IMU/baro mock 생성
  2. `mode = guidance.DecideControlMode(gps, imu, baro, now)`
  3. `l1_in = guidance.ProduceL1Input(now)`
  4. `l1_out = guidance.ProduceL1Output(l1_in)`
  5. `ctrl_in = control.ProduceCtrlInput(l1_out, now)`
  6. `ctrl_out = control.ProduceCtrlOutput(ctrl_in, gyrz_meas_deg_s, now)`

invalid 경로에서는 `ProduceL1Input` 또는 `ProduceL1Output`의 reason을 기록하고
`control.WriteNeutral(now, mode)`로 중립 출력을 만든다.
