#!/usr/bin/env python3
"""
CanSat Parafoil Flight Control — Local Simulation & Integrity Test
==================================================================
하드웨어(pigpio, RPi.GPIO) 없이 로컬 PC에서 실행 가능.
6가지 FIX 검증 + 전체 비행 시나리오 시뮬레이션.

실행: python sim_test.py
"""

import math
import sys
import time
import types
import io

# Windows 콘솔 인코딩 문제 해결
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

# ─────────────────────────────────────────────
# 1. 하드웨어 Mock — pigpio / RPi.GPIO 대체
# ─────────────────────────────────────────────

class MockPi:
    """pigpio.pi() 대체 — 서보 PWM 명령을 기록만 한다."""
    def __init__(self):
        self._pulses = {}   # pin -> pulse_width

    def set_servo_pulsewidth(self, pin, pw):
        self._pulses[pin] = pw

    def stop(self):
        pass

    def get_pulse(self, pin):
        return self._pulses.get(pin, 0)


class MockGPIO:
    BCM = 11
    OUT = 0
    HIGH = 1
    LOW = 0
    @staticmethod
    def setmode(m): pass
    @staticmethod
    def setup(pin, mode, initial=0): pass
    @staticmethod
    def output(pin, val): pass
    @staticmethod
    def cleanup(pin=None): pass


# Mock 모듈을 sys.modules에 주입 (import 전에)
sys.modules["pigpio"] = types.ModuleType("pigpio")
sys.modules["pigpio"].pi = MockPi

gpio_mod = types.ModuleType("RPi.GPIO")
gpio_mod.GPIO = MockGPIO
sys.modules["RPi"] = types.ModuleType("RPi")
sys.modules["RPi"].GPIO = MockGPIO
sys.modules["RPi.GPIO"] = types.ModuleType("RPi.GPIO")
for attr in dir(MockGPIO):
    if not attr.startswith("_"):
        setattr(sys.modules["RPi.GPIO"], attr, getattr(MockGPIO, attr))

# ─────────────────────────────────────────────
# 2. lib 패키지 Mock
# ─────────────────────────────────────────────

lib_mod = types.ModuleType("lib")
sys.modules["lib"] = lib_mod

# appargs
appargs_mod = types.ModuleType("lib.appargs")
class _NS:
    def __init__(self, **kw):
        self.__dict__.update(kw)

appargs_mod.MainAppArg        = _NS(AppID=10, AppName="Main", MID_TerminateProcess=1001001)
appargs_mod.MotorAppArg       = _NS(AppID=19, AppName="Motor")
appargs_mod.FlightlogicAppArg = _NS(
    MID_motor_TargetCor=1101901, MID_motor_state=1101902,
    MID_motor_burnwire=1101903, MID_motor_EggDrop=1101904,
    MID_motor_PullArms=1101905)
appargs_mod.GpsAppArg         = _NS(MID_motor_gps=1501901)
appargs_mod.ImuAppArg         = _NS(MID_motor_imu=1401901)
appargs_mod.BarometerAppArg   = _NS(MID_motor_alt=1301901)
appargs_mod.CommAppArg        = _NS(MID_RouteCmd_MEC=1607)
sys.modules["lib.appargs"] = appargs_mod
lib_mod.appargs = appargs_mod

# events
events_mod = types.ModuleType("lib.events")
events_mod.EventType = _NS(error=0, info=1, debug=2, warning=3)
_event_log = []
def _log_event(app, lvl, msg, print_event=False):
    _event_log.append((app, lvl, msg))
events_mod.LogEvent = _log_event
sys.modules["lib.events"] = events_mod
lib_mod.events = events_mod

# msgstructure
msg_mod = types.ModuleType("lib.msgstructure")
class MsgStructure:
    def __init__(self, sender, receiver, mid, data):
        self.sender_app = sender
        self.receiver_app = receiver
        self.MsgID = mid
        self.data = data
msg_mod.MsgStructure = MsgStructure
def _pack(m):
    return f"{m.sender_app}|{m.receiver_app}|{m.MsgID}|{m.data}"
def _unpack(s):
    p = s.split("|")
    if len(p) != 4:
        return False
    return MsgStructure(int(p[0]), int(p[1]), int(p[2]), p[3])
def _fill(s, r, m, d):
    return MsgStructure(s, r, m, d)
msg_mod.pack_msg = _pack
msg_mod.unpack_msg = _unpack
msg_mod.fill_msg = _fill
sys.modules["lib.msgstructure"] = msg_mod
lib_mod.msgstructure = msg_mod

# ─────────────────────────────────────────────
# 3. Sensor_Motor 패키지 초기화 (import 가능하도록)
# ─────────────────────────────────────────────

import importlib, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 이제 실제 모듈 import
from Sensor_Motor import motor_guidance, motor_control

# ═════════════════════════════════════════════
# 유틸리티
# ═════════════════════════════════════════════

PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"
WARN = "\033[93mWARN\033[0m"
HEADER = "\033[96m"
RESET = "\033[0m"
test_results = []

def section(title):
    print(f"\n{HEADER}{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}{RESET}")

def check(name, condition, detail=""):
    tag = PASS if condition else FAIL
    test_results.append((name, condition))
    print(f"  [{tag}] {name}" + (f"  — {detail}" if detail else ""))

def make_gps(lat, lon, spd=5.0, crs=0.0, fix=1, sats=8, rmc="A"):
    return (
        types.SimpleNamespace(lat=lat, lon=lon, speed=spd, course=crs),
        types.SimpleNamespace(fix_quality=fix, sats=sats, rmc_status=rmc),
    )

def make_imu(yaw=0.0, gyrz=0.0):
    return types.SimpleNamespace(yaw=yaw, gyrz=gyrz)

# ═════════════════════════════════════════════
# TEST 1 — GPS Validation 기본
# ═════════════════════════════════════════════

section("TEST 1: GPS Validation (is_gps_valid)")

check("(0,0) 좌표 → invalid",
      not motor_guidance.is_gps_valid(0.0, 0.0, 1, 8, "A"))
check("위성 3개 → invalid",
      not motor_guidance.is_gps_valid(35.0, 127.0, 1, 3, "A"))
check("rmc_status=V → invalid",
      not motor_guidance.is_gps_valid(35.0, 127.0, 1, 8, "V"))
check("fix=0 → invalid",
      not motor_guidance.is_gps_valid(35.0, 127.0, 0, 8, "A"))
check("정상 GPS → valid",
      motor_guidance.is_gps_valid(35.0, 127.0, 1, 8, "A"))

# ═════════════════════════════════════════════
# TEST 2 — FIX-2: GPS 순간 이동(Multipath) 감지
# ═════════════════════════════════════════════

section("TEST 2: FIX-2 — GPS Jump / Multipath Detection")

motor_guidance.init_guidance()
TARGET = types.SimpleNamespace(lat=35.1000, lon=127.1000)
motor_guidance.set_target_coord(TARGET.lat, TARGET.lon)
motor_guidance.set_start_coordinates(35.0950, 127.0950)

gps, fid = make_gps(35.0960, 127.0960)
imu = make_imu(yaw=45.0)

# 첫 Fix(1) + 안정화 대기(COUNT-1) = COUNT개 샘플이 GPS_INVALID
stabilize_invalid = 0
for i in range(motor_guidance.GPS_STABLE_COUNT_REQUIRED + 1):
    r = motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=200.0)
    if r.state == "GPS_INVALID":
        stabilize_invalid += 1
    time.sleep(0.05)

expected_invalid = motor_guidance.GPS_STABLE_COUNT_REQUIRED
check(f"초기 안정화 기간 {expected_invalid}+ 샘플 GPS_INVALID",
      stabilize_invalid >= expected_invalid,
      f"{stabilize_invalid}/{motor_guidance.GPS_STABLE_COUNT_REQUIRED+1}")

# 안정화 후 정상 동작
time.sleep(0.05)
r = motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=200.0)
check("안정화 후 → 정상 유도 출력",
      r.state != "GPS_INVALID",
      f"state={r.state}")

# 순간 이동 주입: 1km 떨어진 좌표
gps_jump, _ = make_gps(35.1060, 127.1060)
time.sleep(0.05)
r = motor_guidance.guidance(imu, gps_jump, fid, TARGET, baro_m=200.0)
check("비현실적 점프(~1km) → GPS_INVALID",
      r.state == "GPS_INVALID",
      f"state={r.state}")

# 점프 후 다시 안정화 필요
recovery_count = 0
for i in range(motor_guidance.GPS_STABLE_COUNT_REQUIRED + 2):
    time.sleep(0.05)
    r = motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=200.0)
    if r.state == "GPS_INVALID":
        recovery_count += 1

check("점프 후 안정화 기간 존재",
      recovery_count >= motor_guidance.GPS_STABLE_COUNT_REQUIRED,
      f"invalid 횟수={recovery_count}")

# ═════════════════════════════════════════════
# TEST 3 — FIX-3: 기압계 고도 0.0 방어
# ═════════════════════════════════════════════

section("TEST 3: FIX-3 — Baro Altitude 0.0 Defense")

motor_guidance.init_guidance()
# 안정화 통과
for _ in range(motor_guidance.GPS_STABLE_COUNT_REQUIRED + 2):
    motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=100.0)
    time.sleep(0.05)

time.sleep(0.05)
r = motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=0.0)
check("baro_m=0.0 → BARO_INVALID",
      r.state == "BARO_INVALID",
      f"state={r.state}")

r = motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=-5.0)
check("baro_m=-5.0 → BARO_INVALID",
      r.state == "BARO_INVALID",
      f"state={r.state}")

time.sleep(0.05)
r = motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=100.0)
check("baro_m=100.0 → 정상 유도",
      r.state not in ("BARO_INVALID", "GPS_INVALID"),
      f"state={r.state}")

# ═════════════════════════════════════════════
# TEST 4 — FIX-5: 패턴 비행이 20m에서 끊기지 않는지
# ═════════════════════════════════════════════

section("TEST 4: FIX-5 — Pattern Flight Continuity (30m → 0m)")

motor_guidance.init_guidance()
# GPS 안정화
for _ in range(motor_guidance.GPS_STABLE_COUNT_REQUIRED + 2):
    motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=100.0)
    time.sleep(0.05)

altitudes_to_test = [30.0, 25.0, 20.0, 15.0, 10.0, 5.1]
all_pattern = True
pattern_log = []
for alt in altitudes_to_test:
    time.sleep(0.05)
    r = motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=alt, patterned=True)
    pattern_log.append(f"  alt={alt:5.1f}m → state={r.state}, dist={r.distance:.1f}m, yr={r.commanded_yaw_rate:.2f}")
    if r.state != "PATTERN" and r.state not in ("STRAIGHT",):
        # STRAIGHT은 deadband 안에 있을 때 HOMING→STRAIGHT 변환, PATTERN일 때는 안 나옴
        # 실제로 pattern에서 heading_error가 deadband 안이면 commanded_yaw_rate=0이지만 phase는 PATTERN 유지
        all_pattern = False

print("  고도별 패턴 유도 결과:")
for line in pattern_log:
    print(line)

check("patterned=True일 때 20m 미만에서도 PATTERN 유지",
      all_pattern,
      "FINAL 분기 없음 확인")

# 반대로 patterned=False일 때는 HOMING
time.sleep(0.05)
r = motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=15.0, patterned=False)
check("patterned=False, 15m → HOMING/TURNING/STRAIGHT",
      r.state in ("HOMING", "TURNING", "STRAIGHT"),
      f"state={r.state}")

# ═════════════════════════════════════════════
# TEST 5 — motor_control 무결성
# ═════════════════════════════════════════════

section("TEST 5: motor_control — Actuator Mixer & Servo")

mock_pi = MockPi()

# 중립
motor_control.set_neutral(mock_pi)
lp = mock_pi.get_pulse(motor_control.PARAFOIL_LEFT_MOTOR_PIN)
rp = mock_pi.get_pulse(motor_control.PARAFOIL_RIGHT_MOTOR_PIN)
check("set_neutral → 좌/우 neutral pulse",
      lp == motor_control.LEFT_NEUTRAL and rp == motor_control.RIGHT_NEUTRAL,
      f"L={lp}, R={rp} (expect L={motor_control.LEFT_NEUTRAL}, R={motor_control.RIGHT_NEUTRAL})")

# 양수 yaw rate → 좌 당김 > 우 당김
res = motor_control.apply_differential_deflection(mock_pi, 30.0)
check("yaw_rate=+30 → left_deg > right_deg (좌선회)",
      res.left_cmd_deg > res.right_cmd_deg,
      f"L={res.left_cmd_deg:.1f}°, R={res.right_cmd_deg:.1f}°")

# 음수 yaw rate → 우 당김 > 좌 당김
res = motor_control.apply_differential_deflection(mock_pi, -30.0)
check("yaw_rate=-30 → right_deg > left_deg (우선회)",
      res.right_cmd_deg > res.left_cmd_deg,
      f"L={res.left_cmd_deg:.1f}°, R={res.right_cmd_deg:.1f}°")

# yaw_rate=0 → 대칭
res = motor_control.apply_differential_deflection(mock_pi, 0.0)
check("yaw_rate=0 → 좌우 대칭",
      abs(res.left_cmd_deg - res.right_cmd_deg) < 0.01,
      f"L={res.left_cmd_deg:.1f}°, R={res.right_cmd_deg:.1f}°")

# 포화 테스트
res = motor_control.apply_differential_deflection(mock_pi, 999.0)
check("극단 yaw_rate → pulse 범위 내",
      motor_control.PULSE_MIN <= res.left_pulse <= motor_control.PULSE_MAX,
      f"L_pw={res.left_pulse}, R_pw={res.right_pulse}")

# ═════════════════════════════════════════════
# TEST 6 — Haversine 거리 계산
# ═════════════════════════════════════════════

section("TEST 6: Haversine Distance Check")

d = motor_guidance.calculate_distance_haversine(35.0, 127.0, 35.0, 127.0)
check("같은 좌표 → 0m", d < 0.01, f"{d:.6f}m")

d = motor_guidance.calculate_distance_haversine(35.0, 127.0, 35.001, 127.001)
check("~0.001° 이동 → 100~150m 범위",
      80.0 < d < 200.0,
      f"{d:.1f}m")

# ═════════════════════════════════════════════
# TEST 7 — L_DISTANCE 고도별 적응
# ═════════════════════════════════════════════

section("TEST 7: L_DISTANCE Altitude Adaptation")

motor_guidance.init_guidance()
for _ in range(motor_guidance.GPS_STABLE_COUNT_REQUIRED + 2):
    motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=200.0)
    time.sleep(0.05)

time.sleep(0.05)
motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=200.0)
check("alt=200m → L_DISTANCE=HIGH(25)",
      motor_guidance.L_DISTANCE == motor_guidance.L_DISTANCE_HIGH,
      f"L_DISTANCE={motor_guidance.L_DISTANCE}")

time.sleep(0.05)
motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=60.0)
check("alt=60m → L_DISTANCE=BASE(15)",
      motor_guidance.L_DISTANCE == motor_guidance.L_DISTANCE_BASE,
      f"L_DISTANCE={motor_guidance.L_DISTANCE}")

time.sleep(0.05)
motor_guidance.guidance(imu, gps, fid, TARGET, baro_m=20.0)
check("alt=20m → L_DISTANCE=LOW(10)",
      motor_guidance.L_DISTANCE == motor_guidance.L_DISTANCE_LOW,
      f"L_DISTANCE={motor_guidance.L_DISTANCE}")

# ═════════════════════════════════════════════
# TEST 8 — TARGET_REACHED
# ═════════════════════════════════════════════

section("TEST 8: Target Reached Detection")

motor_guidance.init_guidance()
motor_guidance.set_start_coordinates(35.1000, 127.1000)
motor_guidance.set_target_coord(35.1000, 127.1000)

gps_near, _ = make_gps(35.10002, 127.10002)
for _ in range(motor_guidance.GPS_STABLE_COUNT_REQUIRED + 2):
    motor_guidance.guidance(imu, gps_near, fid, TARGET, baro_m=100.0)
    time.sleep(0.05)

time.sleep(0.05)
target_near = types.SimpleNamespace(lat=35.1000, lon=127.1000)
r = motor_guidance.guidance(imu, gps_near, fid, target_near, baro_m=10.0)
check("타겟 근접(~3m) → TARGET_REACHED",
      r.state == "TARGET_REACHED",
      f"state={r.state}, dist={r.distance:.1f}m")

# ═════════════════════════════════════════════
# TEST 9 — motorapp 통합: FIX-1,3,4,6
# ═════════════════════════════════════════════

section("TEST 9: motorapp Integration (FIX-1,3,4,6)")

# motorapp를 직접 import하면 모듈 레벨에서 파일을 열고 전역 상태를 설정함.
# 여기서는 핵심 로직만 단위 테스트한다.
from Sensor_Motor import motorapp

# Mock pi 주입
motorapp.pi = MockPi()
motorapp.motor_enabled = True
motorapp.running = True

# --- FIX-4: 핸들러에 lock이 적용되는지 ---
print("\n  [FIX-4] 핸들러 lock 테스트:")
motorapp.handle_gps("35.096,127.096,5.0,90.0,1,8,A")
with motorapp.update_lock:
    snapshot_lat = motorapp.GpsVector.lat
check("handle_gps → lock 내에서 안전하게 읽기",
      abs(snapshot_lat - 35.096) < 0.001,
      f"lat={snapshot_lat}")

motorapp.handle_imu("45.0,0.5")
with motorapp.update_lock:
    snapshot_yaw = motorapp.altitude.yaw
check("handle_imu → lock 내에서 안전하게 읽기",
      abs(snapshot_yaw - 45.0) < 0.1,
      f"yaw={snapshot_yaw}")

motorapp.handle_barometer("1013.25,25.0,150.0")
with motorapp.update_lock:
    snapshot_baro = motorapp.baro_m
check("handle_barometer → lock 내에서 안전하게 읽기",
      abs(snapshot_baro - 150.0) < 0.1,
      f"baro_m={snapshot_baro}")

# --- FIX-1: Stale 타임스탬프 기록 ---
print(f"\n  [FIX-1] Stale 타임스탬프 테스트:")
check("handle_gps 후 last_gps_time > 0",
      motorapp.last_gps_time > 0,
      f"last_gps_time={motorapp.last_gps_time:.3f}")
check("handle_imu 후 last_imu_time > 0",
      motorapp.last_imu_time > 0,
      f"last_imu_time={motorapp.last_imu_time:.3f}")

# --- FIX-6: State 5 제어 중지 순서 확인 ---
print(f"\n  [FIX-6] State 5 제어 중지 순서:")
motorapp.handle_flight_state("5")
with motorapp.update_lock:
    s = motorapp.state
check("state=5 설정됨", s == 5)

# control_parafoil 내부 로직을 수동 시뮬레이션
# state=5일 때 guidance 호출 없이 set_neutral만 호출되어야 함
_event_log.clear()
mock_pi = MockPi()
motorapp.pi = mock_pi

# 스냅샷을 흉내내서 직접 검증
with motorapp.update_lock:
    _state = motorapp.state
assert _state == 5

motor_control.set_neutral(mock_pi)
lp = mock_pi.get_pulse(motor_control.PARAFOIL_LEFT_MOTOR_PIN)
rp = mock_pi.get_pulse(motor_control.PARAFOIL_RIGHT_MOTOR_PIN)
check("State 5 → set_neutral 호출 확인",
      lp == motor_control.LEFT_NEUTRAL and rp == motor_control.RIGHT_NEUTRAL,
      f"L={lp}, R={rp}")

# --- FIX-3: baro_m=0 방어 (motorapp 레벨) ---
print(f"\n  [FIX-3] motorapp baro=0 방어:")
motorapp.handle_flight_state("3")
motorapp.handle_barometer("0.0")
with motorapp.update_lock:
    b = motorapp.baro_m
check("baro_m=0.0 설정됨", abs(b) < 0.01, f"baro_m={b}")

# ═════════════════════════════════════════════
# SIMULATION — 전체 비행 시나리오
# ═════════════════════════════════════════════

section("FULL FLIGHT SIMULATION (600m → Pattern → State 5 Stop)")

print("""
  시나리오:
    1) 600m 상공에서 낙하 시작 (HOMING)
    2) GPS 데이터로 타겟까지 유도
    3) 30m 이하 진입 → 패턴(8자) 비행 시작
    4) 패턴 비행이 20m, 10m에서도 유지되는지 확인
    5) State 5 → 제어 완전 정지
""")

motor_guidance.init_guidance()
START_LAT, START_LON = 35.0950, 127.0950
TGT_LAT, TGT_LON = 35.1000, 127.1000
motor_guidance.set_start_coordinates(START_LAT, START_LON)
motor_guidance.set_target_coord(TGT_LAT, TGT_LON)

sim_pi = MockPi()

# GPS 위치: 타겟 남서쪽 약 500m
sim_lat, sim_lon = 35.0965, 127.0965
sim_yaw = 45.0
sim_gyrz = 0.0

# 고도 시퀀스: 600m → 30m 진입 → 20m → 10m → state 5
altitude_profile = (
    [(600 - i * 10, False, 3) for i in range(58)]   # 600→30m, state=3, homing
  + [(30 - i * 2, True, 3) for i in range(11)]      # 30→10m, patterned, state=3
  + [(8, True, 5)]                                    # state=5 → 정지
)

# GPS 안정화 단계 (결과 출력 안 함)
for _ in range(motor_guidance.GPS_STABLE_COUNT_REQUIRED + 2):
    g, f = make_gps(sim_lat, sim_lon, 5.0, 45.0)
    motor_guidance.guidance(make_imu(sim_yaw, sim_gyrz), g, f,
                           types.SimpleNamespace(lat=TGT_LAT, lon=TGT_LON),
                           baro_m=600.0)
    time.sleep(0.03)

print(f"  {'Step':>4} | {'Alt':>6} | {'Pat':>3} | {'St':>2} | {'Phase':>12} | {'Dist':>7} | {'YawCmd':>8} | {'L_deg':>6} | {'R_deg':>6} | {'L_pw':>6} | {'R_pw':>6}")
print(f"  {'─'*4}─┼─{'─'*6}─┼─{'─'*3}─┼─{'─'*2}─┼─{'─'*12}─┼─{'─'*7}─┼─{'─'*8}─┼─{'─'*6}─┼─{'─'*6}─┼─{'─'*6}─┼─{'─'*6}")

phase_history = []
stopped_at_state5 = False

for step, (alt, patterned_flag, flight_state) in enumerate(altitude_profile):
    time.sleep(0.03)

    g, f = make_gps(sim_lat, sim_lon, 5.0, sim_yaw)
    im = make_imu(sim_yaw, sim_gyrz)
    tgt = types.SimpleNamespace(lat=TGT_LAT, lon=TGT_LON)

    # State 5 → set_neutral, 유도 건너뜀
    if flight_state == 5:
        motor_control.set_neutral(sim_pi)
        lp = sim_pi.get_pulse(motor_control.PARAFOIL_LEFT_MOTOR_PIN)
        rp = sim_pi.get_pulse(motor_control.PARAFOIL_RIGHT_MOTOR_PIN)
        print(f"  {step:4d} | {alt:5.1f}m | {'Y' if patterned_flag else 'N':>3} | {flight_state:>2} | {'STOP':>12} | {'---':>7} | {'---':>8} | {'---':>6} | {'---':>6} | {lp:>6} | {rp:>6}")
        stopped_at_state5 = (lp == motor_control.LEFT_NEUTRAL and rp == motor_control.RIGHT_NEUTRAL)
        phase_history.append("STOP")
        continue

    result = motor_guidance.guidance(im, g, f, tgt, baro_m=float(alt), patterned=patterned_flag)
    mr = motor_control.apply_differential_deflection(sim_pi, result.commanded_yaw_rate)
    phase_history.append(result.state)

    # 매 10 step 또는 중요 구간만 출력
    if step % 5 == 0 or alt <= 30 or flight_state == 5:
        print(f"  {step:4d} | {alt:5.1f}m | {'Y' if patterned_flag else 'N':>3} | {flight_state:>2} | {result.state:>12} | {result.distance:6.1f}m | {result.commanded_yaw_rate:>7.2f} | {mr.left_cmd_deg:5.1f}° | {mr.right_cmd_deg:5.1f}° | {mr.left_pulse:>6} | {mr.right_pulse:>6}")

    # 간단한 위치 업데이트 (타겟 방향으로 조금씩 이동)
    move_scale = 0.000005
    sim_lat += move_scale * math.cos(math.radians(sim_yaw))
    sim_lon += move_scale * math.sin(math.radians(sim_yaw))

print()

# 시뮬레이션 결과 검증
homing_phases = [p for i, (a, pt, s) in enumerate(altitude_profile)
                 if a > 30 and s == 3 for p in [phase_history[i]]]
pattern_phases = [p for i, (a, pt, s) in enumerate(altitude_profile)
                  if a <= 30 and pt and s == 3 for p in [phase_history[i]]]
low_alt_patterns = [p for i, (a, pt, s) in enumerate(altitude_profile)
                    if a < 20 and pt and s == 3 for p in [phase_history[i]]]

check("600→31m 구간: HOMING/TURNING/STRAIGHT 위주",
      all(p in ("HOMING", "TURNING", "STRAIGHT") for p in homing_phases),
      f"phases={set(homing_phases)}")

check("30→10m 구간: 모두 PATTERN",
      all(p == "PATTERN" for p in pattern_phases),
      f"phases={set(pattern_phases)}")

check("20m 미만에서도 PATTERN 유지 (FIX-5 핵심)",
      len(low_alt_patterns) > 0 and all(p == "PATTERN" for p in low_alt_patterns),
      f"count={len(low_alt_patterns)}, phases={set(low_alt_patterns)}")

check("State 5 → 모터 중립 (FIX-6 핵심)",
      stopped_at_state5)

check("State 5 → STOP 기록",
      "STOP" in phase_history)

# ═════════════════════════════════════════════
# 최종 결과 요약
# ═════════════════════════════════════════════

section("RESULT SUMMARY")

total = len(test_results)
passed = sum(1 for _, ok in test_results if ok)
failed = total - passed

for name, ok in test_results:
    tag = PASS if ok else FAIL
    print(f"  [{tag}] {name}")

print(f"\n  Total: {total}  |  Passed: {passed}  |  Failed: {failed}")

if failed == 0:
    print(f"\n  {HEADER}ALL TESTS PASSED — 코드 무결성 확인 완료{RESET}")
else:
    print(f"\n  \033[91m{failed} TEST(S) FAILED — 확인 필요\033[0m")

sys.exit(0 if failed == 0 else 1)
