"""Closed-loop scenario player for the CANSAT GCS.

Drives the FSW through a SIM-mode trajectory by sending SIMG/SIMP commands at
a configurable cadence (default 7 s wall between SIMG bursts; ``simg_simp_spacing_s``
pauses between SIMG and SIMP on the UART). Each tick advances ``tick_period_s``
simulated seconds while reading back ``left_pulse_us`` / ``right_pulse_us`` from
telemetry to update the simulated cansat heading. The map in
``ground_station.py`` then animates the trail naturally because every SIMG
causes the FSW to publish a new GPS frame in its next TLM packet.

Two consumers share the same core:
  * ``ground_station.py`` Scenario panel — uses the existing serial connection
    and Tk's ``after()`` scheduler.
  * ``scenario_player.py`` CLI — opens its own serial port (GCS must be
    disconnected) and uses ``threading.Timer``.

Physics (intentionally simple, not a flight model):
  * Heading is integrated from a yaw rate proportional to the differential of
    the FSW servo pulse widths (``(left_pw + right_pw - 3100) / PULSE_PER_DEG``
    matches the mixer in ``Sensor_Motor/motor_control.py``).
  * Ground velocity = airspeed vector (heading) + wind vector (meteorological
    convention: ``wind_dir_met`` is the direction the wind blows *from*).
  * Optional sinusoidal gust modulates the wind magnitude.
  * Altitude decreases at a fixed descent rate; landing fires SS,5 +
    SIM,DISABLE.

The FSW receives SIMG via ``cmd_simg`` (``comm/commapp.py``); the FlightLogic
app injects the simulated GPS into the GpsApp pipeline once
``SIM,ACTIVATE`` is in effect. ``cmd_simg`` rejects ``(0,0)`` and out-of-range
coordinates, so the runner clamps before sending.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional

# ----------------------------------------------------------------- constants
# Mixer constants mirror Sensor_Motor/motor_control.py — keep in sync if those
# are ever retuned. ``LEFT_ZERO + RIGHT_ZERO`` (= 600 + 2500 = 3100) is the
# pulse-width sum at ``delta_arm_deg = 0``; ``PULSE_PER_DEG = 2000/180`` is the
# servo calibration.
_PULSE_PER_DEG: float = 2000.0 / 180.0
_PULSE_SUM_NEUTRAL: int = 3100
_NEUTRAL_LEFT_PW: int = 1267
_NEUTRAL_RIGHT_PW: int = 1833

# Earth model — flat-earth approximation good enough for sub-km scenarios.
_M_PER_DEG_LAT: float = 111320.0


def _m_per_deg_lon(lat_deg: float) -> float:
    return _M_PER_DEG_LAT * math.cos(math.radians(lat_deg))


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2.0) ** 2
    )
    return 2.0 * r * math.asin(min(1.0, math.sqrt(a)))


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


# ------------------------------------------------------------- configuration

@dataclass
class ScenarioConfig:
    """Initial conditions and environmental parameters for one run."""

    name: str
    description: str = ""

    start_lat: float = 37.5607
    start_lon: float = 126.9307
    target_lat: float = 37.5670
    target_lon: float = 126.9370
    start_alt_m: float = 200.0

    # If None, heading is initialised toward the target (anti-parallel cases
    # override this to test 180-error recovery).
    init_heading_deg: Optional[float] = None

    airspeed_ms: float = 8.0       # parafoil forward speed through air
    descent_rate_ms: float = 5.0   # constant vertical sink rate
    descent_jitter_ms: float = 0.0 # +/- random variation each tick (uniform)

    wind_speed_ms: float = 0.0
    wind_dir_met_deg: float = 270.0   # wind FROM this bearing (270 = westerly)
    gust_amp_ms: float = 0.0          # additional sinusoidal wind component
    gust_period_s: float = 4.0

    # Wall-clock spacing between SIMG+SIMP pairs (GCS ``after`` / CLI loop) and
    # simulated dt per tick. Default matches ``setup_inter_cmd_delay_s`` for slow UART.
    tick_period_s: float = 7.0
    pulse_to_yaw_gain: float = 0.3    # deg/s yaw per deg of delta_arm
    yaw_rate_max_deg_s: float = 60.0  # safety clamp on simulated yaw

    target_radius_m: float = 8.0      # stop scenario when within this radius
    timeout_s: float = 240.0          # hard cap so runaway runs eventually end

    use_release_state: bool = True    # send SS,3 in the setup phase
    landing_state: int = 5            # SS,5 on completion
    auto_disable_sim: bool = True     # SIM,DISABLE on completion
    # UART/XBee round-trip can be seconds; spacing avoids FlightLogic dropping cmds.
    setup_inter_cmd_delay_s: float = 7.0  # UART spacing during setup / teardown only
    # Wall time between SIMG and SIMP on each glide tick (CLI ``time.sleep`` / Tk ``after``).
    simg_simp_spacing_s: float = 7.0
    teardown_inter_cmd_delay_s: float = 7.0  # SS / SIM,DISABLE spacing (match setup on slow links)


# ----------------------------------------------------------------- presets


def _bearing_offset(start_lat: float, start_lon: float, bearing_deg: float, dist_m: float) -> tuple[float, float]:
    """Compute a destination point at (bearing, distance) from a start point."""
    lat_rad = math.radians(start_lat)
    dn = dist_m * math.cos(math.radians(bearing_deg))
    de = dist_m * math.sin(math.radians(bearing_deg))
    dlat = dn / _M_PER_DEG_LAT
    dlon = de / (_M_PER_DEG_LAT * math.cos(lat_rad))
    return start_lat + dlat, start_lon + dlon


_BASE_START_LAT = 37.5607
_BASE_START_LON = 126.9307


def _make_target(bearing_deg: float, distance_m: float) -> tuple[float, float]:
    return _bearing_offset(_BASE_START_LAT, _BASE_START_LON, bearing_deg, distance_m)


def _build_presets() -> dict[str, ScenarioConfig]:
    """Built-in scenarios. Same start; target/wind/descent vary per case."""
    common = dict(start_lat=_BASE_START_LAT, start_lon=_BASE_START_LON)
    tgt_calm = _make_target(45.0, 350.0)
    tgt_long = _make_target(45.0, 1000.0)

    return {
        "calm": ScenarioConfig(
            name="calm",
            description="No wind, nominal 5 m/s descent, 350 m target NE.",
            **common,
            target_lat=tgt_calm[0], target_lon=tgt_calm[1],
            start_alt_m=200.0,
            airspeed_ms=8.0, descent_rate_ms=5.0,
            wind_speed_ms=0.0, wind_dir_met_deg=270.0,
        ),
        "west8": ScenarioConfig(
            name="west8",
            description="Strong westerly 8 m/s wind (constant), 350 m target NE.",
            **common,
            target_lat=tgt_calm[0], target_lon=tgt_calm[1],
            start_alt_m=200.0,
            airspeed_ms=8.0, descent_rate_ms=5.0,
            wind_speed_ms=8.0, wind_dir_met_deg=270.0,
        ),
        "gust12": ScenarioConfig(
            name="gust12",
            description="Gusty 12 m/s westerly with +/-3 m/s sinusoidal gust.",
            **common,
            target_lat=tgt_calm[0], target_lon=tgt_calm[1],
            start_alt_m=250.0,
            airspeed_ms=8.0, descent_rate_ms=5.0, descent_jitter_ms=0.6,
            wind_speed_ms=12.0, wind_dir_met_deg=270.0,
            gust_amp_ms=3.0, gust_period_s=4.0,
        ),
        "fast_descent": ScenarioConfig(
            name="fast_descent",
            description="Calm wind, fast 8 m/s descent — short time aloft.",
            **common,
            target_lat=tgt_calm[0], target_lon=tgt_calm[1],
            start_alt_m=180.0,
            airspeed_ms=8.0, descent_rate_ms=8.0,
            wind_speed_ms=0.0,
        ),
        "slow_descent": ScenarioConfig(
            name="slow_descent",
            description="Calm wind, slow 3 m/s descent — long glide.",
            **common,
            target_lat=tgt_calm[0], target_lon=tgt_calm[1],
            start_alt_m=180.0,
            airspeed_ms=8.0, descent_rate_ms=3.0,
            wind_speed_ms=0.0,
        ),
        "anti_parallel": ScenarioConfig(
            name="anti_parallel",
            description="180 deg initial heading error — guidance recovery test.",
            **common,
            target_lat=tgt_calm[0], target_lon=tgt_calm[1],
            start_alt_m=300.0,
            init_heading_deg=225.0,   # target bearing 45 + 180 = 225
            airspeed_ms=8.0, descent_rate_ms=4.0,
            wind_speed_ms=2.0, wind_dir_met_deg=270.0,
        ),
        "long_range": ScenarioConfig(
            name="long_range",
            description="Long range 1 km target, 600 m start altitude.",
            **common,
            target_lat=tgt_long[0], target_lon=tgt_long[1],
            start_alt_m=600.0,
            airspeed_ms=8.0, descent_rate_ms=5.0,
            wind_speed_ms=2.0, wind_dir_met_deg=270.0,
        ),
    }


PRESETS: dict[str, ScenarioConfig] = _build_presets()


def list_preset_names() -> list[str]:
    return list(PRESETS.keys())


def get_preset(name: str) -> ScenarioConfig:
    """Return a fresh copy of a built-in preset.

    A copy is returned so callers (GCS panel overrides, CLI overrides, tests)
    can mutate without polluting the registry shared across the process.
    """
    if name not in PRESETS:
        raise KeyError(f"unknown preset {name!r} — choose from {list_preset_names()}")
    from copy import deepcopy
    return deepcopy(PRESETS[name])


# --------------------------------------------------------------- runner

SendFn = Callable[[str], bool]
TlmFn = Callable[[], Optional[dict]]
LogFn = Callable[[str], None]


@dataclass
class ScenarioState:
    """Mutable per-tick state. Inspect from the panel for live readout."""
    elapsed_s: float = 0.0
    tick_count: int = 0
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0
    heading_deg: float = 0.0
    course_deg: float = 0.0
    ground_speed_ms: float = 0.0
    yaw_rate_deg_s: float = 0.0
    distance_to_target_m: float = math.inf
    finished: bool = False
    finish_reason: str = ""
    extras: dict[str, float] = field(default_factory=dict)


class ScenarioRunner:
    """Closed-loop scenario driver. Caller owns scheduling.

    Usage:
        runner = ScenarioRunner(send_cb, get_tlm_cb, log_cb, config)
        runner.start()                    # sends setup
        while not runner.state.finished:  # or Tk: alternate integrate+simg /
            runner.tick()                 # simp after ``simg_simp_spacing_s``
            ...
        runner.stop()                     # idempotent; safe to call twice
    """

    def __init__(
        self,
        send_cb: SendFn,
        get_tlm_cb: TlmFn,
        log_cb: LogFn,
        config: ScenarioConfig,
        rng_seed: Optional[int] = None,
    ) -> None:
        self._send = send_cb
        self._get_tlm = get_tlm_cb
        self._log = log_cb
        self.config = config
        self.state = ScenarioState()
        self._started = False
        self._stopped = False
        self._t0_mono: Optional[float] = None
        # Use a deterministic seedable RNG so jitter / gust are reproducible
        # across CLI / GCS runs at the same seed.
        import random
        self._rng = random.Random(rng_seed if rng_seed is not None else 0)
        self._async_phase: bool = False
        self._await_simp: bool = False

    # ----------------------------------------------------------- setup
    def setup_command_sequence(self) -> list[str]:
        """Setup commands sent in start(), in order."""
        cfg = self.config
        cmds = [
            "CX,ON",
            "SIM,ENABLE",
            "SIM,ACTIVATE",
            f"TC,{cfg.target_lat:.7f},{cfg.target_lon:.7f}",
            self._format_simg(
                cfg.start_lat, cfg.start_lon,
                self._initial_heading(),
                cfg.airspeed_ms,
                cfg.start_alt_m,
            ),
            f"SIMP,{cfg.start_alt_m:.1f}",
        ]
        if cfg.use_release_state:
            cmds.append("SS,3")
        return cmds

    def teardown_command_sequence(self, reason: str) -> list[str]:
        cfg = self.config
        cmds: list[str] = []
        if cfg.landing_state is not None:
            cmds.append(f"SS,{cfg.landing_state}")
        if cfg.auto_disable_sim:
            cmds.append("SIM,DISABLE")
        return cmds

    def _init_run_state(self) -> None:
        cfg = self.config
        self._t0_mono = time.monotonic()
        self.state.lat = cfg.start_lat
        self.state.lon = cfg.start_lon
        self.state.alt_m = cfg.start_alt_m
        self.state.heading_deg = self._initial_heading()
        self.state.course_deg = self.state.heading_deg
        self.state.ground_speed_ms = cfg.airspeed_ms
        self.state.distance_to_target_m = _haversine_m(
            cfg.start_lat, cfg.start_lon, cfg.target_lat, cfg.target_lon
        )

        self._log(f"[scenario:{cfg.name}] start  alt={cfg.start_alt_m:.0f}m  "
                  f"target_d={self.state.distance_to_target_m:.0f}m  "
                  f"wind={cfg.wind_speed_ms:.1f}m/s @{cfg.wind_dir_met_deg:.0f}deg  "
                  f"descent={cfg.descent_rate_ms:.1f}m/s")
        self._await_simp = False

    # ----------------------------------------------------------- API
    def begin_async_setup(self) -> list[str]:
        """Initialise state and return UART bodies for paced sending (e.g. Tk ``after``).

        Call :meth:`complete_async_setup` after the last command is transmitted.
        """
        if self._started:
            return []
        if self._async_phase:
            raise RuntimeError("begin_async_setup() already in progress")
        self._async_phase = True
        self._init_run_state()
        return self.setup_command_sequence()

    def complete_async_setup(self) -> None:
        """Mark UART setup finished after :meth:`begin_async_setup` commands are sent."""
        if self._started:
            return
        self._started = True
        self._async_phase = False

    def start(self, sleep_fn: Callable[[float], None] = time.sleep) -> bool:
        """Send setup commands and initialise simulation state."""
        if self._started:
            return False
        self._init_run_state()
        cfg = self.config
        sent_any = False
        for cmd in self.setup_command_sequence():
            if self._send(cmd):
                sent_any = True
            sleep_fn(cfg.setup_inter_cmd_delay_s)
        self._started = True
        return sent_any

    def _physics_step(self) -> None:
        """Integrate one simulated period from TLM feedback (no UART)."""
        cfg = self.config
        dt = cfg.tick_period_s

        # 1. Read FSW pulse output, derive simulated yaw rate.
        tlm = self._get_tlm() or {}
        left_pw = _coerce_int(tlm.get("left_pulse_us"), _NEUTRAL_LEFT_PW)
        right_pw = _coerce_int(tlm.get("right_pulse_us"), _NEUTRAL_RIGHT_PW)
        delta_arm_deg = (left_pw + right_pw - _PULSE_SUM_NEUTRAL) / _PULSE_PER_DEG
        yaw_rate = _clamp(
            cfg.pulse_to_yaw_gain * delta_arm_deg,
            -cfg.yaw_rate_max_deg_s,
            cfg.yaw_rate_max_deg_s,
        )

        # 2. Integrate heading.
        self.state.heading_deg = (self.state.heading_deg + yaw_rate * dt) % 360.0
        self.state.yaw_rate_deg_s = yaw_rate

        # 3. Compute wind components (meteorological convention: wind FROM
        #    that bearing, so the actual airmass moves opposite).
        wind_to_bearing = (cfg.wind_dir_met_deg + 180.0) % 360.0
        wind_mag = cfg.wind_speed_ms
        if cfg.gust_amp_ms != 0.0 and cfg.gust_period_s > 0.0:
            phase = 2.0 * math.pi * self.state.elapsed_s / cfg.gust_period_s
            wind_mag = max(0.0, wind_mag + cfg.gust_amp_ms * math.sin(phase))
        wind_E = wind_mag * math.sin(math.radians(wind_to_bearing))
        wind_N = wind_mag * math.cos(math.radians(wind_to_bearing))

        # 4. Ground velocity = airmass-relative + wind.
        hdg_rad = math.radians(self.state.heading_deg)
        v_air_E = cfg.airspeed_ms * math.sin(hdg_rad)
        v_air_N = cfg.airspeed_ms * math.cos(hdg_rad)
        V_E = v_air_E + wind_E
        V_N = v_air_N + wind_N
        self.state.ground_speed_ms = math.hypot(V_E, V_N)
        self.state.course_deg = (math.degrees(math.atan2(V_E, V_N))) % 360.0

        # 5. Update geodetic position.
        m_per_deg_lon = _m_per_deg_lon(self.state.lat)
        if m_per_deg_lon == 0.0:
            m_per_deg_lon = 1.0
        self.state.lat += V_N * dt / _M_PER_DEG_LAT
        self.state.lon += V_E * dt / m_per_deg_lon

        # 6. Update altitude with optional jitter; never negative.
        descent = cfg.descent_rate_ms
        if cfg.descent_jitter_ms > 0.0:
            descent += self._rng.uniform(-cfg.descent_jitter_ms, cfg.descent_jitter_ms)
        self.state.alt_m = max(0.0, self.state.alt_m - descent * dt)

        # 7. Distance to target.
        self.state.distance_to_target_m = _haversine_m(
            self.state.lat, self.state.lon, cfg.target_lat, cfg.target_lon
        )

    def tick_integrate_and_simg(self) -> bool:
        """Run physics, send SIMG. Call :meth:`tick_send_simp` after UART spacing."""
        if not self._started or self._stopped or self.state.finished:
            return False
        if self._await_simp:
            return False

        self._physics_step()

        # (0,0) is rejected by cmd_simg; flat-earth integration won't reach it.
        self._send(self._format_simg(
            self.state.lat, self.state.lon,
            self.state.course_deg, self.state.ground_speed_ms,
            self.state.alt_m,
        ))
        self._await_simp = True
        return True

    def tick_send_simp(self) -> bool:
        """Send SIMP after SIMG; completes the tick (counters + end checks)."""
        if not self._started or self._stopped or self.state.finished:
            return False
        if not self._await_simp:
            return False

        cfg = self.config
        dt = cfg.tick_period_s

        self._send(f"SIMP,{self.state.alt_m:.1f}")
        self._await_simp = False

        self.state.tick_count += 1
        self.state.elapsed_s += dt

        # End conditions.
        if self.state.alt_m <= 0.0:
            self._finish("landed (alt=0)")
            return False
        if self.state.distance_to_target_m <= cfg.target_radius_m:
            self._finish(f"within {cfg.target_radius_m:.0f} m of target")
            return False
        if cfg.timeout_s > 0.0 and self.state.elapsed_s >= cfg.timeout_s:
            self._finish(f"timeout after {cfg.timeout_s:.0f} s")
            return False
        return True

    def awaiting_simp(self) -> bool:
        """True after SIMG until SIMP is sent (for async Tk scheduling)."""
        return self._await_simp

    def tick(self) -> bool:
        """Advance one period (integrate, SIMG, UART pause, SIMP). Blocking CLI/tests."""
        if not self.tick_integrate_and_simg():
            return False
        spacing = self.config.simg_simp_spacing_s
        if spacing > 0.0:
            time.sleep(spacing)
        return self.tick_send_simp()

    def stop(self, send_teardown: bool = True,
             sleep_fn: Callable[[float], None] = time.sleep,
             reason: str = "user-stop") -> None:
        if self._stopped:
            return
        self._stopped = True
        self._await_simp = False
        if not self.state.finished:
            self._finish(reason, log=False)
        if send_teardown:
            td = self.config.teardown_inter_cmd_delay_s
            for cmd in self.teardown_command_sequence(reason):
                self._send(cmd)
                sleep_fn(td)
        self._async_phase = False
        self._log(f"[scenario:{self.config.name}] stop reason={self.state.finish_reason}  "
                  f"ticks={self.state.tick_count}  elapsed={self.state.elapsed_s:.1f}s  "
                  f"final_d={self.state.distance_to_target_m:.1f}m")

    # ----------------------------------------------------------- helpers
    def _finish(self, reason: str, *, log: bool = True) -> None:
        self.state.finished = True
        self.state.finish_reason = reason
        if log:
            self._log(f"[scenario:{self.config.name}] finish reason={reason}  "
                      f"final_d={self.state.distance_to_target_m:.1f}m  "
                      f"alt={self.state.alt_m:.1f}m  "
                      f"elapsed={self.state.elapsed_s:.1f}s")

    def _initial_heading(self) -> float:
        cfg = self.config
        if cfg.init_heading_deg is not None:
            return cfg.init_heading_deg % 360.0
        return _bearing_deg(cfg.start_lat, cfg.start_lon,
                            cfg.target_lat, cfg.target_lon)

    @staticmethod
    def _format_simg(lat: float, lon: float, course: float,
                     speed: float, alt: float) -> str:
        # Latitude / longitude clamped to documented ranges so cmd_simg won't
        # silently reject (we already avoid 0,0 by construction).
        lat = _clamp(lat, -89.999999, 89.999999)
        lon = ((lon + 180.0) % 360.0) - 180.0
        course = course % 360.0
        speed = max(0.0, speed)
        alt = max(0.0, alt)
        return f"SIMG,{lat:.7f},{lon:.7f},{course:.1f},{speed:.2f},{alt:.1f}"


# ----------------------------------------------------------------- helpers


def _clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _coerce_int(value: object, default: int) -> int:
    try:
        v = int(float(value))  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return v


# ----------------------------------------------------------------- iter API

def iterate_scenario(
    config: ScenarioConfig,
    get_tlm_cb: TlmFn,
    send_cb: SendFn,
    log_cb: LogFn = lambda msg: None,
    sleep_fn: Callable[[float], None] = time.sleep,
    rng_seed: Optional[int] = None,
) -> Iterable[ScenarioState]:
    """Generator-style helper for the CLI: yields after each tick."""
    runner = ScenarioRunner(send_cb, get_tlm_cb, log_cb, config, rng_seed=rng_seed)
    runner.start(sleep_fn=sleep_fn)
    try:
        while runner.tick():
            yield runner.state
            # tick() already sleeps ``simg_simp_spacing_s`` between SIMG and SIMP.
            sleep_fn(max(0.0, config.tick_period_s - config.simg_simp_spacing_s))
        yield runner.state
    finally:
        runner.stop(sleep_fn=sleep_fn, reason="generator-exit")
