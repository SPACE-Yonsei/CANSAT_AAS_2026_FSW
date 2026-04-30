"""Guidance logic baseline (L1-like simplified)."""

from __future__ import annotations

from math import atan2, cos, radians, sin, sqrt
from types import SimpleNamespace


def init_guidance(_logger=None) -> None:
    return


def is_gps_valid(gps_vector, gps_fidelity) -> bool:
    lat = float(gps_vector.lat)
    lon = float(gps_vector.lon)
    fix = int(gps_fidelity.fix_quality)
    sats = int(gps_fidelity.sats)
    status = str(gps_fidelity.rmc_status).upper()
    return -90 <= lat <= 90 and -180 <= lon <= 180 and fix >= 1 and sats >= 4 and status == "A"


def is_gps_jump(_lat: float, _lon: float) -> bool:
    # Placeholder: jump rejection stateful logic to be upgraded in next pass.
    return False


def set_start_coordinates(_lat: float, _lon: float) -> None:
    return


def set_target_coord(_lat: float, _lon: float) -> None:
    return


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    # Simple equirect approximation for short distance
    dlat = (lat2 - lat1) * 111_000.0
    dlon = (lon2 - lon1) * 111_000.0 * cos(radians((lat1 + lat2) / 2.0))
    return sqrt(dlat * dlat + dlon * dlon)


def guidance(imu_data, gps_vector, gps_fidelity, target, baro_m: float):
    if not is_gps_valid(gps_vector, gps_fidelity):
        return SimpleNamespace(state="FDIR", distance=0.0, commanded_yaw_rate=0.0)

    distance = _distance_m(gps_vector.lat, gps_vector.lon, target.lat, target.lon)
    desired_heading = atan2(target.lon - gps_vector.lon, target.lat - gps_vector.lat)
    my_heading = radians(float(imu_data.yaw))
    err = desired_heading - my_heading
    while err > 3.141592:
        err -= 2 * 3.141592
    while err < -3.141592:
        err += 2 * 3.141592

    yr_max = 45.0 if baro_m > 20 else 20.0
    commanded = max(-yr_max, min(yr_max, err * 20.0))
    phase = "HOMING" if baro_m > 10 else "LANDING"
    return SimpleNamespace(state=phase, distance=distance, commanded_yaw_rate=commanded)
