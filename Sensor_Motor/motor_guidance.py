#!/usr/bin/env python3
import math
import time
import types


cascade_pi = types.SimpleNamespace(
    Kp_outer=0.6,
    Kp_inner=1.0,
    Ki_inner=0.1,
    pi_integral=0.0,
    MAX_INTEGRAL=15.0,
    DEADBAND=5.0,
    MAX_CMD=60.0
)

target = types.SimpleNamespace(lat=0.0, lon=0.0)
start_point = types.SimpleNamespace(lat=0.0, lon=0.0)

LAT_TO_METER = 111320.0

L_DISTANCE      = 15.0
L_DISTANCE_BASE = 15.0
L_DISTANCE_HIGH = 25.0
L_DISTANCE_LOW  = 10.0

ALT_HIGH = 100.0
ALT_LOW  = 30.0

wind_effect = 0.0
altitude_m = 0.0
last_time = None

_pattern = types.SimpleNamespace(
    lobe_sign=1,
    last_switch_time=0.0,
    LOBE_PERIOD=25.0,
    RADIUS=25.0,
)

FINAL_APPROACH_ALT = 20.0
MAX_YAW_RATE_FINAL = 15.0


def init_guidance():
    global wind_effect, last_time, altitude_m, L_DISTANCE
    wind_effect = 0.0
    altitude_m = 0.0
    L_DISTANCE = L_DISTANCE_BASE
    cascade_pi.pi_integral = 0.0
    last_time = time.time()
    _pattern.lobe_sign = 1
    _pattern.last_switch_time = time.time()


def reset_control():
    global wind_effect, last_time
    wind_effect = 0.0
    cascade_pi.pi_integral = 0.0
    last_time = time.time()


def _wrap_180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def is_gps_valid(lat: float, lon: float,
                 fix_quality: int = 0, sats: int = 0,
                 rmc_status: str = "V") -> bool:
    coord_ok = (not (lat == 0.0 and lon == 0.0)
                and abs(lat) <= 90.0
                and abs(lon) <= 180.0)
    fidelity_ok = fix_quality >= 1 and sats >= 4 and rmc_status == "A"
    return coord_ok and fidelity_ok


def calculate_distance_haversine(lat1: float, lon1: float,
                                  lat2: float, lon2: float) -> float:
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = (math.sin(d_phi / 2.0) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2)
    return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _llh_to_en(lat: float, lon: float) -> tuple:
    N = (lat - start_point.lat) * LAT_TO_METER
    E = (lon - start_point.lon) * LAT_TO_METER * math.cos(math.radians(start_point.lat))
    return E, N


def _carrot(my_E: float, my_N: float,
            tgt_E: float, tgt_N: float) -> tuple:
    rope_len = math.hypot(tgt_E, tgt_N)
    if rope_len < 0.01:
        return tgt_E, tgt_N

    uE = tgt_E / rope_len
    uN = tgt_N / rope_len

    s = max(0.0, min(my_E * uE + my_N * uN, rope_len))

    carrot_s = s + L_DISTANCE
    return carrot_s * uE, carrot_s * uN


def _figure_eight_target(my_E: float, my_N: float,
                         tgt_E: float, tgt_N: float) -> tuple:
    now = time.time()

    if now - _pattern.last_switch_time > _pattern.LOBE_PERIOD:
        _pattern.lobe_sign *= -1
        _pattern.last_switch_time = now

    bearing_to_tgt = math.atan2(tgt_E - my_E, tgt_N - my_N)

    perp_angle = bearing_to_tgt + math.pi / 2.0

    offset_E = _pattern.RADIUS * _pattern.lobe_sign * math.sin(perp_angle)
    offset_N = _pattern.RADIUS * _pattern.lobe_sign * math.cos(perp_angle)

    return tgt_E + offset_E, tgt_N + offset_N


def update_altitude(alt_m: float):
    global altitude_m, L_DISTANCE
    altitude_m = alt_m

    if alt_m > ALT_HIGH:
        L_DISTANCE = L_DISTANCE_HIGH
    elif alt_m < ALT_LOW:
        L_DISTANCE = L_DISTANCE_LOW
    else:
        L_DISTANCE = L_DISTANCE_BASE


def set_start_coordinates(lat: float, lon: float):
    start_point.lat = lat
    start_point.lon = lon


def set_target_coord(lat: float, lon: float):
    target.lat = lat
    target.lon = lon


def draw_pattern():
    return


def _yaw_rate_pi_control(desired_yaw_rate: float,
                         measured_yaw_rate: float,
                         dt: float,
                         is_final: bool) -> tuple:
    rate_error = desired_yaw_rate - measured_yaw_rate

    u_before_sat = (cascade_pi.Kp_inner * rate_error
                    + cascade_pi.Ki_inner * cascade_pi.pi_integral)

    max_cmd = MAX_YAW_RATE_FINAL if is_final else cascade_pi.MAX_CMD

    u_after_sat = max(-max_cmd, min(max_cmd, u_before_sat))

    is_saturated = abs(u_before_sat) > max_cmd
    error_deepens_sat = (rate_error * u_before_sat > 0.0)

    if is_saturated and error_deepens_sat:
        pass
    else:
        cascade_pi.pi_integral += rate_error * dt
        cascade_pi.pi_integral = max(
            -cascade_pi.MAX_INTEGRAL,
            min(cascade_pi.MAX_INTEGRAL, cascade_pi.pi_integral)
        )

    u_before_sat = (cascade_pi.Kp_inner * rate_error
                    + cascade_pi.Ki_inner * cascade_pi.pi_integral)
    u_after_sat = max(-max_cmd, min(max_cmd, u_before_sat))

    return u_before_sat, u_after_sat, cascade_pi.pi_integral


def guidance(imu_data, gps_data, target_data,
             patterned: bool = False) -> types.SimpleNamespace:
    global wind_effect, last_time

    now = time.time()
    dt = now - last_time if last_time else 0.1
    if dt <= 0.02 or dt > 0.5:
        dt = 0.1
    last_time = now

    def _result(**kw):
        defaults = dict(
            commanded_yaw_rate=0.0, heading_error=0.0,
            desired_course=0.0, desired_heading=0.0,
            desired_yaw_rate=0.0, measured_yaw_rate=imu_data.gyrz,
            u_before_sat=0.0, u_after_sat=0.0,
            integral=cascade_pi.pi_integral,
            wind_effect=wind_effect, distance=0.0,
            altitude=altitude_m, l_distance=L_DISTANCE,
            patterned=patterned,
            pattern_wp_E=0.0, pattern_wp_N=0.0,
            state="UNKNOWN",
            gps_speed=gps_data.speed, gps_course=gps_data.course,
            yaw=imu_data.yaw
        )
        defaults.update(kw)
        return types.SimpleNamespace(**defaults)

    if not is_gps_valid(gps_data.lat, gps_data.lon,
                        gps_data.fix_quality, gps_data.sats,
                        gps_data.rmc_status):
        return _result(state="GPS_INVALID")

    my_E, my_N = _llh_to_en(gps_data.lat, gps_data.lon)
    tgt_E, tgt_N = _llh_to_en(target_data.lat, target_data.lon)
    distance = math.hypot(tgt_E - my_E, tgt_N - my_N)

    if distance < 5.0:
        return _result(distance=distance, state="TARGET_REACHED")

    pattern_wp_E, pattern_wp_N = 0.0, 0.0

    is_final = (altitude_m > 0.0 and altitude_m < FINAL_APPROACH_ALT)

    if patterned and not is_final:
        pattern_wp_E, pattern_wp_N = _figure_eight_target(
            my_E, my_N, tgt_E, tgt_N
        )
        guide_E, guide_N = pattern_wp_E, pattern_wp_N
        phase = "PATTERN"

    elif is_final:
        guide_E, guide_N = tgt_E, tgt_N
        phase = "FINAL"

    else:
        cE, cN = _carrot(my_E, my_N, tgt_E, tgt_N)
        guide_E, guide_N = cE, cN
        phase = "HOMING"

    desired_course = math.degrees(
        math.atan2(guide_E - my_E, guide_N - my_N)
    )

    if gps_data.speed > 1.0 and abs(imu_data.gyrz) < 20.0:
        current_crab = _wrap_180(gps_data.course - imu_data.yaw)
        wind_effect = 0.95 * wind_effect + 0.05 * current_crab

    desired_heading = _wrap_180(desired_course - wind_effect)

    heading_error = _wrap_180(desired_heading - imu_data.yaw)
    V = max(gps_data.speed, 1.0)
    desired_yaw_rate = math.degrees(
        2.0 * (V / L_DISTANCE) * math.sin(math.radians(heading_error))
    )

    if abs(heading_error) <= cascade_pi.DEADBAND:
        cascade_pi.pi_integral = 0.0
        u_before_sat = 0.0
        u_after_sat = 0.0
        if phase == "HOMING":
            phase = "STRAIGHT"
    else:
        u_before_sat, u_after_sat, _ = _yaw_rate_pi_control(
            desired_yaw_rate, imu_data.gyrz, dt, is_final
        )
        if phase == "HOMING":
            phase = "TURNING"

    return _result(
        commanded_yaw_rate=u_after_sat,
        heading_error=heading_error,
        desired_course=desired_course,
        desired_heading=desired_heading,
        desired_yaw_rate=desired_yaw_rate,
        u_before_sat=u_before_sat,
        u_after_sat=u_after_sat,
        integral=cascade_pi.pi_integral,
        distance=distance,
        pattern_wp_E=pattern_wp_E,
        pattern_wp_N=pattern_wp_N,
        state=phase
    )
