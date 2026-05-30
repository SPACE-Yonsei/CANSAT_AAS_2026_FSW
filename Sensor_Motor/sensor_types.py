"""Sensor_Motor/sensor_types.py — 순수 데이터 컨테이너 (메서드 없음)

motorapp.py와 guidance.py 양쪽에서 임포트 가능한 중립 파일.
순환 임포트를 방지하기 위해 이 파일은 프로젝트 내 다른 모듈을 임포트하지 않는다.

── 두 종류의 타입 ──────────────────────────────────────────────────────────
  _*FromApp  : motorapp.py 전용. 하드웨어에서 읽어온 raw 센서 데이터.
               guidance.py는 duck-typing으로 접근 (임포트 불필요).
  *Anchor    : guidance.py 전용. 마지막 신선한 값만 저장하는 navigation anchor.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import nan
from typing import Optional


# ── motorapp 전용 raw 센서 데이터 ─────────────────────────────────────────────

@dataclass
class _GpsFromApp:
    lat:           Optional[float] = None
    lon:           Optional[float] = None
    course_rad:    Optional[float] = None
    speed_mps:     Optional[float] = None
    pos_ts:        Optional[float] = None
    motion_ts:     Optional[float] = None
    rx_ts:         Optional[float] = None
    pos_health:    int = 0
    motion_health: int = 0


@dataclass
class _ImuFromApp:
    roll_rad:      Optional[float] = None
    pitch_rad:     Optional[float] = None
    yaw_rad:       Optional[float] = None
    accx_mps2:     Optional[float] = None
    accy_mps2:     Optional[float] = None
    accz_mps2:     Optional[float] = None
    gyrx_rad_s:    Optional[float] = None
    gyry_rad_s:    Optional[float] = None
    gyrz_rad_s:    Optional[float] = None   # z축 yaw rate, 오른쪽 회전=양수 (nav 부호)
    ts:            Optional[float] = None
    rx_ts:         Optional[float] = None
    lin_acc_x:     Optional[float] = None   # 중력 제거된 전진 가속도 (m/s²)
    lin_acc_y:     Optional[float] = None   # 중력 제거된 우측 가속도 (m/s²)
    lin_acc_z:     Optional[float] = None
    lin_acc_valid:  bool  = False
    health:         int   = 0
    yaw_offset_deg: float = 0.0


@dataclass
class _BaroFromApp:
    alt_m:     Optional[float] = None
    sink_rate: Optional[float] = None
    rx_ts:     Optional[float] = None
    health:    int = 0


@dataclass
class _Cache:
    """_ctrl_cycle 스레드 간 공유되는 최신 raw 센서 데이터."""
    latest_gps:  _GpsFromApp  = field(default_factory=_GpsFromApp)
    latest_imu:  _ImuFromApp  = field(default_factory=_ImuFromApp)
    latest_baro: _BaroFromApp = field(default_factory=_BaroFromApp)
    target_lat:  Optional[float] = None
    target_lon:  Optional[float] = None


# ── guidance 전용 Navigation Anchor ──────────────────────────────────────────
# lat/lon은 저장하지 않음 — guidance는 E/N(local NE, m)만 사용.
# vE/vN은 저장하지 않음 — V*sin/cos(course)로 언제든 계산 가능.

@dataclass
class GpsAnchor:
    E:            float = nan   # local NE 동쪽 (m), origin 기준
    N:            float = nan   # local NE 북쪽 (m), origin 기준
    V:            float = nan   # 속도 (m/s)
    course:       float = nan   # 진행방향 (rad, North=0, East=+π/2)
    pos_ts:       float = nan   # GPS position 타임스탬프 (monotonic)
    motion_ts:    float = nan   # GPS velocity 타임스탬프 (monotonic)
    pos_valid:    bool  = False
    motion_valid: bool  = False


@dataclass
class ImuAnchor:
    yaw:           float = nan   # 절대 yaw (rad, North=0, East=+π/2)
    gyr_z:         float = nan   # yaw rate (rad/s), 오른쪽 회전=양수
    lin_acc_x:     float = nan   # 전진 가속도 (m/s², 중력 제거)
    lin_acc_y:     float = nan   # 우측 가속도 (m/s², 중력 제거)
    ts:            float = nan   # sample 타임스탬프 (monotonic)
    yaw_valid:     bool  = False
    gyrz_valid:    bool  = False
    lin_acc_valid: bool  = False


@dataclass
class BaroAnchor:
    alt_m:     float = nan
    sink_rate: float = nan
    ts:        float = nan   # rx 타임스탬프 (monotonic)
    valid:     bool  = False
