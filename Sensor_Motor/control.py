"""Sensor_Motor/control.py – yaw-rate controller + motor mixer.

Pipeline: ProduceCtrlInput(g_out) -> ProduceCtrlOutput(ctl, cmd, gyrz_meas, now) -> ProducePulse(pi, cmd)

Arm convention
  0 deg   = arm up
  80 deg  = neutral
  160 deg = full brake/down
  positive delta_arm_deg = right turn (right arm angle ↑, left arm angle ↓)
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Optional

from lib import config, timebase

logger = logging.getLogger(__name__)


# ── Mode / fallback string constants (used by motorapp diag + tests) ─────────
CTRL_MODE_NEUTRAL          = "NEUTRAL"
CTRL_MODE_CLOSED_LOOP      = "CLOSED_LOOP"
CTRL_MODE_FEEDFORWARD_ONLY = "FEEDFORWARD_ONLY"
CTRL_MODE_GUIDANCE_TIMEOUT = "GUIDANCE_TIMEOUT"

CTRL_FALLBACK_NONE                 = ""
CTRL_FALLBACK_GUIDANCE_TIMEOUT     = "GUIDANCE_TIMEOUT"
CTRL_FALLBACK_GUIDANCE_ATTENUATED  = "GUIDANCE_ATTENUATED"
CTRL_FALLBACK_GYRO_SPIKE           = "GYRO_SPIKE"


# ── GPIO + servo calibration (from config) ───────────────────────────────────
PARAFOIL_LEFT_MOTOR_PIN  = config.PARAFOIL_LEFT_GPIO
PARAFOIL_RIGHT_MOTOR_PIN = config.PARAFOIL_RIGHT_GPIO

ARM_MIN_DEG     = config.ARM_MIN_DEG
ARM_MAX_DEG     = config.ARM_MAX_DEG
NEUTRAL_ARM_DEG = config.NEUTRAL_ARM_DEG
DELTA_ARM_MAX_DEG = 2.0 * min(
    NEUTRAL_ARM_DEG - ARM_MIN_DEG,
    ARM_MAX_DEG - NEUTRAL_ARM_DEG,
)

LEFT_ZERO     = config.LEFT_SERVO_ZERO_US
RIGHT_ZERO    = config.RIGHT_SERVO_ZERO_US
PULSE_PER_DEG = config.SERVO_PULSE_PER_DEG

LEFT_NEUTRAL  = int(LEFT_ZERO  - NEUTRAL_ARM_DEG * PULSE_PER_DEG)
RIGHT_NEUTRAL = int(RIGHT_ZERO + NEUTRAL_ARM_DEG * PULSE_PER_DEG)
LEFT_ZERO_PULSE  = int(LEFT_ZERO)
RIGHT_ZERO_PULSE = int(RIGHT_ZERO)
LEFT_MIN_PULSE  = int(LEFT_ZERO  - ARM_MAX_DEG * PULSE_PER_DEG)
LEFT_MAX_PULSE  = int(LEFT_ZERO  - ARM_MIN_DEG * PULSE_PER_DEG)
RIGHT_MIN_PULSE = int(RIGHT_ZERO + ARM_MIN_DEG * PULSE_PER_DEG)
RIGHT_MAX_PULSE = int(RIGHT_ZERO + ARM_MAX_DEG * PULSE_PER_DEG)

# Guidance timeouts / gyro spike / integrator decay (from config)
GUIDANCE_TIMEOUT_ATTENUATE_S = config.GUIDANCE_TIMEOUT_ATTENUATE_S
GUIDANCE_TIMEOUT_FAIL_S      = config.GUIDANCE_TIMEOUT_FAIL_S
GYRO_SPIKE_LIMIT_DEG_S       = config.GYRO_SPIKE_LIMIT_DEG_S
INTEGRAL_DECAY_RATE          = config.INTEGRAL_DECAY_RATE


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ── Dataclasses ──────────────────────────────────────────────────────────────

@dataclass
class ControlConfig:
    # Feedforward shaping
    # DELTA_FF_MAX_DEG 80→160: ConnectRoMo가 ±160° 까지 매핑 가능 (left=0, right=160).
    # 명령 포화 시 FF만으로 하드웨어 최대 권한 사용.
    ANGULAR_VELOCITY_CMD_MAX_DEG_S:  float = config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
    ANGULAR_VELOCITY_DEADBAND_DEG_S: float = config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S
    DELTA_FF_MAX_DEG:                float = 160.0
    DELTA_MIN_EFFECTIVE_DEG:         float = config.CTRL_DELTA_MIN_EFFECTIVE_DEG
    EXPO:                            float = config.CTRL_EXPO

    # PID
    # DELTA_PID_MAX_DEG 25→30: 정상상태에서 트림 권한 약간 확장
    ERROR_DEADBAND_DEG_S: float = config.CTRL_ERROR_DEADBAND_DEG_S
    K_P:                  float = config.KP_GPS_CLOSED
    K_I:                  float = config.CTRL_K_I
    K_D:                  float = config.KD_YAW_RATE
    I_LIMIT_DEG:          float = config.CTRL_I_LIMIT_DEG
    DELTA_PID_MAX_DEG:    float = 30.0

    # Total authority + slew
    # DELTA_TOTAL_MAX_DEG 100→160: 하드웨어 최대(DELTA_ARM_MAX_DEG=160)와 일치.
    # FF+PID 합이 160°에 도달하면 left/right arm이 0°/160° 극값에 도달.
    DELTA_TOTAL_MAX_DEG: float = 160.0
    MAX_ARM_RATE_DEG_S:  float = 60.0


@dataclass
class _PIDState:
    integral_deg:    float = 0.0
    prev_error_deg:  float = 0.0
    prev_time:       float = 0.0


@dataclass
class Ctrler:
    config: ControlConfig = field(default_factory=ControlConfig)
    pid:    _PIDState     = field(default_factory=_PIDState)
    prev_left_angle_deg:  float = NEUTRAL_ARM_DEG
    prev_right_angle_deg: float = NEUTRAL_ARM_DEG


@dataclass
class CtrlInput:
    angular_velocity_cmd_deg_s: float = 0.0
    ground_speed_mps: float = 0.0
    valid: bool = False
    timestamp: float = 0.0
    pid_enabled: bool = True
    control_mode: Optional[str] = None
    dr_method:    Optional[str] = None
    kp_override:  Optional[float] = None


@dataclass
class CtrlOutput:
    timestamp: float = 0.0
    left_pw:  int = LEFT_NEUTRAL
    right_pw: int = RIGHT_NEUTRAL
    left_angle_deg:  float = NEUTRAL_ARM_DEG
    right_angle_deg: float = NEUTRAL_ARM_DEG
    delta_arm_deg: float = 0.0
    delta_ff_deg:  float = 0.0
    delta_pid_deg: float = 0.0
    angular_velocity_cmd_deg_s:   float = 0.0
    angular_velocity_meas_deg_s:  float = float("nan")
    angular_velocity_error_deg_s: float = 0.0
    motor_cmd: float = 0.0
    saturated:    bool = False
    sensor_valid: bool = False
    valid:        bool = False
    mode:          str = CTRL_MODE_NEUTRAL
    fallback_mode: str = CTRL_FALLBACK_NONE
    guidance_command_age_s: float = 0.0


# ── Public factory / reset / neutral ──────────────────────────────────────────

def MakeCtrler(cfg: Optional[ControlConfig] = None) -> Ctrler:
    return Ctrler(config=cfg or ControlConfig())


def controller_reset(ctl: Ctrler) -> None:
    ctl.pid = _PIDState()
    ctl.prev_left_angle_deg  = NEUTRAL_ARM_DEG
    ctl.prev_right_angle_deg = NEUTRAL_ARM_DEG


def WriteNeutral(now: float, mode: str = CTRL_MODE_NEUTRAL) -> CtrlOutput:
    """Build a neutral-PWM CtrlOutput; does not touch hardware."""
    cmd = CtrlOutput(timestamp=now)
    cmd.mode = mode
    cmd.fallback_mode = mode
    return cmd


# ── Input conversion (rad/s → deg/s) ──────────────────────────────────────────

def ProduceCtrlInput(l1_output, now: float) -> CtrlInput:
    """Convert a guidance L1Output into a controller CtrlInput."""
    is_valid = bool(
        getattr(l1_output, "control_valid", False)
        or getattr(l1_output, "nominal", False)
    )
    rad = getattr(l1_output, "angular_velocity_cmd_rad_s", None)
    if rad is None:
        rad = getattr(l1_output, "yaw_rate_cmd", 0.0)
    rad = float(rad or 0.0)

    return CtrlInput(
        angular_velocity_cmd_deg_s=math.degrees(rad),
        ground_speed_mps=float(getattr(l1_output, "ground_speed_mps", 0.0) or 0.0),
        valid=is_valid,
        timestamp=float(getattr(l1_output, "timestamp", now) or now),
        pid_enabled=bool(getattr(l1_output, "pid_enabled", True)),
        control_mode=getattr(l1_output, "control_mode", None),
        dr_method=getattr(l1_output, "dr_method", None),
        kp_override=getattr(l1_output, "kp_override", None),
    )


# ── Feedforward shaping + mixer ───────────────────────────────────────────────

def angular_velocity_to_delta_ff(cmd_dps: float, cfg: ControlConfig) -> float:
    """Map yaw-rate command (deg/s) to feedforward differential arm angle (deg).

    Expo curve, deadband at small inputs, sign preserved (positive = right turn).
    """
    clamped = _clamp(cmd_dps, -cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S, cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S)
    if abs(clamped) < cfg.ANGULAR_VELOCITY_DEADBAND_DEG_S:
        return 0.0
    x = abs(clamped) / cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S
    delta = cfg.DELTA_MIN_EFFECTIVE_DEG + (cfg.DELTA_FF_MAX_DEG - cfg.DELTA_MIN_EFFECTIVE_DEG) * (x ** cfg.EXPO)
    return math.copysign(delta, clamped)


def ConnectRoMo(delta_arm_deg: float):
    """Map differential arm deflection to arm angles and PWM pulses.

    Positive delta → right turn (right arm angle ↑, left arm angle ↓).
    Returns (left_pw, right_pw, left_angle_deg, right_angle_deg, delta_arm_deg).
    """
    delta = _clamp(delta_arm_deg, -DELTA_ARM_MAX_DEG, DELTA_ARM_MAX_DEG)
    left_angle  = _clamp(NEUTRAL_ARM_DEG - delta / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)
    right_angle = _clamp(NEUTRAL_ARM_DEG + delta / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)
    left_pw  = int(_clamp(LEFT_ZERO  - left_angle  * PULSE_PER_DEG, LEFT_MIN_PULSE,  LEFT_MAX_PULSE))
    right_pw = int(_clamp(RIGHT_ZERO + right_angle * PULSE_PER_DEG, RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))
    return left_pw, right_pw, left_angle, right_angle, delta


# ── ProduceCtrlOutput: the main controller step ──────────────────────────────

def ProduceCtrlOutput(
    ctl: Ctrler,
    cmd: CtrlInput,
    angular_velocity_meas_deg_s: float,
    now: float,
) -> CtrlOutput:
    """Compute brake-servo command from a CtrlInput + gyrz measurement.

    Pipeline:
      1. Validate; check guidance command age (fail / attenuate windows).
      2. Clamp yaw-rate command by configured authority.
      3. Expo-shaped feedforward (FF) within DELTA_FF_MAX_DEG.
      4. PID closed-loop trim within DELTA_PID_MAX_DEG (when gyrz valid + pid_enabled).
      5. Sum FF + PID, clamp to DELTA_TOTAL_MAX_DEG.
      6. Arm-angle slew-rate limit before pulse output.
    NaN command → neutral. Invalid → neutral.
    """
    out = CtrlOutput(timestamp=now)
    cfg = ctl.config   # control.py owns motor authority; no guidance overrides

    # ── Invalid → neutral ────────────────────────────────────────────────────
    if not cmd.valid:
        out.mode = CTRL_MODE_NEUTRAL
        out.fallback_mode = config.MOTOR_REASON_GUIDANCE_INACTIVE
        return out

    # ── NaN command → neutral ────────────────────────────────────────────────
    raw_cmd = float(cmd.angular_velocity_cmd_deg_s)
    if not math.isfinite(raw_cmd):
        out.mode = CTRL_MODE_NEUTRAL
        out.fallback_mode = CTRL_FALLBACK_NONE
        return out

    angular_velocity_cmd_deg_s = raw_cmd

    age = timebase.age(now, cmd.timestamp)
    out.guidance_command_age_s = age

    # ── Two-stage guidance timeout ───────────────────────────────────────────
    if age > GUIDANCE_TIMEOUT_FAIL_S:
        out.mode          = CTRL_MODE_GUIDANCE_TIMEOUT
        out.fallback_mode = CTRL_FALLBACK_GUIDANCE_TIMEOUT
        out.valid = False
        out.left_angle_deg  = NEUTRAL_ARM_DEG
        out.right_angle_deg = NEUTRAL_ARM_DEG
        out.left_pw  = LEFT_NEUTRAL
        out.right_pw = RIGHT_NEUTRAL
        return out

    if age > GUIDANCE_TIMEOUT_ATTENUATE_S:
        angular_velocity_cmd_deg_s *= 0.5
        out.fallback_mode = CTRL_FALLBACK_GUIDANCE_ATTENUATED
    else:
        out.fallback_mode = CTRL_FALLBACK_NONE

    # ── Clamp yaw-rate command ───────────────────────────────────────────────
    angular_velocity_cmd_deg_s = _clamp(
        angular_velocity_cmd_deg_s,
        -cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S,
        cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S,
    )
    command_clamped = not math.isclose(
        angular_velocity_cmd_deg_s, raw_cmd, rel_tol=0.0, abs_tol=1.0e-9
    )
    out.angular_velocity_cmd_deg_s = angular_velocity_cmd_deg_s

    # dt shared by PID + slew
    dt = timebase.clamp_dt(now, ctl.pid.prev_time, default_s=0.1, min_s=0.01, max_s=0.2)

    # ── Feedforward ──────────────────────────────────────────────────────────
    delta_ff = angular_velocity_to_delta_ff(angular_velocity_cmd_deg_s, cfg)

    # ── Gyro spike rejection ─────────────────────────────────────────────────
    gyro_finite = math.isfinite(angular_velocity_meas_deg_s)
    gyro_spike  = gyro_finite and abs(angular_velocity_meas_deg_s) > GYRO_SPIKE_LIMIT_DEG_S
    sensor_valid = gyro_finite and not gyro_spike
    out.sensor_valid = sensor_valid
    if gyro_spike:
        out.fallback_mode = CTRL_FALLBACK_GYRO_SPIKE

    # ── PID closed-loop trim ─────────────────────────────────────────────────
    integral  = ctl.pid.integral_deg
    delta_pid = 0.0
    error     = 0.0

    pid_active = bool(cmd.pid_enabled and cfg.DELTA_PID_MAX_DEG > 0.0 and sensor_valid)

    if pid_active:
        out.angular_velocity_meas_deg_s = angular_velocity_meas_deg_s
        error = angular_velocity_cmd_deg_s - angular_velocity_meas_deg_s
        if abs(error) < cfg.ERROR_DEADBAND_DEG_S:
            error = 0.0
        derivative = (error - ctl.pid.prev_error_deg) / dt
        integral_candidate = _clamp(ctl.pid.integral_deg + error * dt, -cfg.I_LIMIT_DEG, cfg.I_LIMIT_DEG)
        kp = float(cmd.kp_override) if cmd.kp_override is not None else cfg.K_P
        delta_pid = _clamp(
            kp * error + cfg.K_I * integral_candidate + cfg.K_D * derivative,
            -cfg.DELTA_PID_MAX_DEG,
            cfg.DELTA_PID_MAX_DEG,
        )
        integral = integral_candidate
        out.angular_velocity_error_deg_s = error
        out.mode = CTRL_MODE_CLOSED_LOOP
    else:
        # No gyro: FF only. Decay integral so stale windup drains.
        integral = ctl.pid.integral_deg * INTEGRAL_DECAY_RATE
        out.mode = CTRL_MODE_FEEDFORWARD_ONLY

    # ── Sum + total clamp ────────────────────────────────────────────────────
    delta_sum = delta_ff + delta_pid
    authority_saturated = abs(delta_sum) > cfg.DELTA_TOTAL_MAX_DEG
    saturated = command_clamped or authority_saturated
    out.saturated = saturated
    delta_total = _clamp(delta_sum, -cfg.DELTA_TOTAL_MAX_DEG, cfg.DELTA_TOTAL_MAX_DEG)

    # ── Desired arm angles from delta_total ──────────────────────────────────
    _, _, left_des, right_des, delta_arm = ConnectRoMo(delta_total)

    # ── Slew-rate limit ──────────────────────────────────────────────────────
    max_step = cfg.MAX_ARM_RATE_DEG_S * dt
    left_angle = _clamp(left_des,
                        ctl.prev_left_angle_deg - max_step,
                        ctl.prev_left_angle_deg + max_step)
    right_angle = _clamp(right_des,
                         ctl.prev_right_angle_deg - max_step,
                         ctl.prev_right_angle_deg + max_step)

    left_pw  = int(_clamp(LEFT_ZERO  - left_angle  * PULSE_PER_DEG, LEFT_MIN_PULSE,  LEFT_MAX_PULSE))
    right_pw = int(_clamp(RIGHT_ZERO + right_angle * PULSE_PER_DEG, RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))

    out.delta_ff_deg    = delta_ff
    out.delta_pid_deg   = delta_pid
    out.delta_arm_deg   = delta_arm
    out.motor_cmd       = delta_arm
    out.left_angle_deg  = left_angle
    out.right_angle_deg = right_angle
    out.left_pw  = left_pw
    out.right_pw = right_pw
    out.valid = True

    # ── State update ─────────────────────────────────────────────────────────
    if pid_active:
        ctl.pid.prev_error_deg = error
        # Conditional anti-windup: block integral growth only when error pushes deeper into saturation.
        error_aggravates = saturated and (
            error != 0.0 and math.copysign(1.0, delta_sum) == math.copysign(1.0, error)
        )
        if not error_aggravates:
            ctl.pid.integral_deg = integral
    else:
        ctl.pid.integral_deg = integral

    ctl.pid.prev_time = now
    ctl.prev_left_angle_deg  = left_angle
    ctl.prev_right_angle_deg = right_angle
    return out


# ── pigpio bindings ──────────────────────────────────────────────────────────

def init_control():
    """Initialize pigpio + zero servos. Returns pi handle or None."""
    try:
        import pigpio
        pi = pigpio.pi()
        if pi.connected:
            pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  LEFT_ZERO_PULSE)
            pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_ZERO_PULSE)
            logger.info("pigpio connected; servos zeroed")
            return pi
        logger.warning("pigpio.pi() not connected; no servo output")
    except Exception as exc:
        logger.warning("pigpio init failed (%s); no servo output", exc)
    return None


def WriteZero(pi) -> None:
    """Set both arms to 0 deg (arm up)."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  LEFT_ZERO_PULSE)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_ZERO_PULSE)


def WriteOff(pi) -> None:
    """Cut servo PWM (pulse width = 0)."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)


def Set180(pi) -> None:
    """Drive both arms to 180 deg (clamped by configured servo limits)."""
    if pi is None:
        return
    left_pw  = int(_clamp(LEFT_ZERO  - 180.0 * PULSE_PER_DEG, LEFT_MIN_PULSE,  LEFT_MAX_PULSE))
    right_pw = int(_clamp(RIGHT_ZERO + 180.0 * PULSE_PER_DEG, RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  left_pw)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pw)


def ProducePulse(pi, cmd: CtrlOutput) -> None:
    """Write a CtrlOutput to the servo PWM channels."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  cmd.left_pw)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, cmd.right_pw)
