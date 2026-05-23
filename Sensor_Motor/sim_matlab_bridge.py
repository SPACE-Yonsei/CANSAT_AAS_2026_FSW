"""
sim_matlab_bridge.py  –  CanSat L1 guidance + control verification simulator.

Calls real Sensor_Motor/guidance.py and Sensor_Motor/control.py functions at
20 Hz, integrates a simple kinematic model, and broadcasts every result as a
JSON UDP datagram to MATLAB for real-time visualization.

Usage
-----
    python tests/sim_matlab_bridge.py
    python tests/sim_matlab_bridge.py --batch

Then open MATLAB and run:  matlab/run_l1_viz.m
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Optional

# ---------------------------------------------------------------------------
# Path: make Sensor_Motor and lib importable from anywhere
# ---------------------------------------------------------------------------
FSW_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, FSW_ROOT)

from Sensor_Motor.guidance import (
    ControlMode,
    ProduceL1Input,
    ProduceL1Output,
    latlon_to_ne,
    ne_to_latlon,
)
from Sensor_Motor.control import (
    ControlConfig,
    MakeCtrler,
    ProduceCtrlInput,
    ProduceCtrlOutput,
    NEUTRAL_ARM_DEG,
)

# ===========================================================================
# ── User-configurable parameters ───────────────────────────────────────────
# ===========================================================================
ORIGIN_LAT       = 37.5018   # Drop/start latitude  (deg)
ORIGIN_LON       = 127.0018  # Drop/start longitude (deg)
TARGET_LAT       = 37.5000   # Landing target latitude  (deg)
TARGET_LON       = 127.0000  # Landing target longitude (deg)
GLIDE_SPEED_MPS  = 5.5       # Constant horizontal glide speed (m/s)
DESCENT_RATE_MPS = 3.5      # Vertical descent rate (m/s)
START_ALT_M      = 480.0     # Initial altitude AGL (m)
SIM_RATE_HZ      = 20        # Simulation rate (Hz); 20 Hz = FSW default

# ── Wind disturbance ────────────────────────────────────────────────────────
# Constant base wind (NE frame, m/s). Positive N = northward, positive E = eastward.
WIND_N_MPS       =   0     # base northward wind component
WIND_E_MPS       =  2.0      # base eastward wind component
# Turbulence: random noise added each step (0 = off, ~0.3 = light, ~1.0 = severe)
WIND_TURB_STD    =  0.5      # std-dev of per-step Gaussian turbulence (m/s)
UDP_HOST         = "127.0.0.1"
UDP_PORT         = 14560
DATA_FILE        = os.path.join(FSW_ROOT, "matlab", "viz_data.json")
TRACE_FILE       = os.path.join(FSW_ROOT, "matlab", "viz_trace.json")
MATLAB_EXE       = r"C:\Program Files\MATLAB\R2025b\bin\matlab.exe"
MATLAB_SCRIPT    = os.path.join(FSW_ROOT, "matlab", "run_l1_viz.m")

# Control config (mirrors FSW defaults)
CTRL_CFG = ControlConfig(K_FF=1.0, K_P=0.0, K_I=0.0, K_D=0.0)

# ===========================================================================
# ── Earth geometry helpers ──────────────────────────────────────────────────
# ===========================================================================
def latlon_to_NE(lat: float, lon: float,
                 origin_lat: float, origin_lon: float) -> tuple[float, float]:
    return latlon_to_ne(lat, lon, origin_lat, origin_lon)


def NE_to_latlon(N: float, E: float,
                 origin_lat: float, origin_lon: float) -> tuple[float, float]:
    return ne_to_latlon(N, E, origin_lat, origin_lon)


# ===========================================================================
# ── Mock sensor objects (duck-typed – FillFresh uses getattr throughout) ────
# ===========================================================================
@dataclass
class MockGPS:
    lat: Optional[float] = None
    lon: Optional[float] = None
    course: Optional[float] = None      # radians, NE-heading (0=North)
    speed: Optional[float] = None       # m/s
    pos_ts: Optional[float] = None
    motion_ts: Optional[float] = None


@dataclass
class MockIMU:
    gyrz: Optional[float] = None       # rad/s (positive = right turn)
    ts: Optional[float] = None
    freefall: int = 0   # 1=자유낙하, 0=정상
    tumble:   int = 0   # 1=텀블링,  0=안정


@dataclass
class MockBaro:
    alt: Optional[float] = None        # m AGL
    ts: Optional[float] = None
    sink_rate: Optional[float] = None


# ===========================================================================
# ── Main simulation loop ────────────────────────────────────────────────────
# ===========================================================================
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the CanSat L1 MATLAB bridge simulator.")
    parser.add_argument(
        "--batch",
        action="store_true",
        help="Run at CPU speed, save all frames to viz_trace.json, then auto-open MATLAB.",
    )
    parser.add_argument(
        "--trace-file",
        default=TRACE_FILE,
        help="Output path for batch trace JSON.",
    )
    parser.add_argument(
        "--no-matlab",
        action="store_true",
        help="Skip auto-launching MATLAB (live or batch).",
    )
    parser.add_argument(
        "--matlab-wait",
        type=int,
        default=18,
        help="Seconds to wait for MATLAB to start before beginning live sim (default: 18).",
    )
    return parser.parse_args()


def _launch_matlab(script_path: str) -> bool:
    """Open MATLAB and run script_path. Non-blocking. Returns True if launched."""
    if not os.path.isfile(MATLAB_EXE):
        print(f"  [matlab] Executable not found: {MATLAB_EXE}")
        print(f"  [matlab] Open MATLAB manually and run: run('{script_path}')")
        return False
    script_path_fwd = script_path.replace("\\", "/")
    cmd = [MATLAB_EXE, "-nosplash", "-r", f"run('{script_path_fwd}')"]
    subprocess.Popen(cmd, creationflags=subprocess.DETACHED_PROCESS)
    print(f"  [matlab] Launched MATLAB -> {os.path.basename(script_path)}")
    return True


def _wait_for_matlab(wait_s: int) -> None:
    """Countdown so MATLAB finishes loading before sim data starts."""
    for remaining in range(wait_s, 0, -1):
        print(f"  [matlab] Waiting for MATLAB to load ... {remaining:2d}s ", end="\r", flush=True)
        time.sleep(1)
    print(f"  [matlab] MATLAB should be ready. Starting simulation now.     ")


def main() -> None:
    args = parse_args()
    batch_mode = bool(args.batch)
    dt = 1.0 / SIM_RATE_HZ

    # ── Pre-compute constant geometry ──────────────────────────────────────
    target_N, target_E = latlon_to_NE(TARGET_LAT, TARGET_LON, ORIGIN_LAT, ORIGIN_LON)
    start_N, start_E   = 0.0, 0.0   # CanSat starts at origin

    # ── Simulation state ────────────────────────────────────────────────────
    sim_pos_N   = 0.0
    sim_pos_E   = 0.0
    sim_heading = math.atan2(target_E, target_N)   # NE-heading (0=North)
    sim_alt     = START_ALT_M
    sim_gyrz    = 0.0   # Simulated IMU yaw-rate (1-step delay from command)

    # ── FSW objects ─────────────────────────────────────────────────────────
    ctrler   = MakeCtrler(CTRL_CFG)

    # ── UDP socket ──────────────────────────────────────────────────────────
    sock = None if batch_mode else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    trace_payloads: list[dict] = []

    print("=" * 60)
    print(" CanSat L1 Guidance Simulation -> MATLAB Bridge")
    print("=" * 60)
    print(f"  Origin  : ({ORIGIN_LAT:.4f} N, {ORIGIN_LON:.4f} E)")
    print(f"  Target  : ({TARGET_LAT:.4f} N, {TARGET_LON:.4f} E)")
    print(f"  Target offset: N={target_N:.1f} m, E={target_E:.1f} m")
    print(f"  Speed   : {GLIDE_SPEED_MPS} m/s  |  Descent: {DESCENT_RATE_MPS} m/s")
    print(f"  Alt     : {START_ALT_M} m  |  Rate: {SIM_RATE_HZ} Hz")
    if batch_mode:
        print(f"  Output  : {os.path.abspath(args.trace_file)}")
        print("  Mode    : BATCH (CPU speed) -> viz_trace.json -> MATLAB")
        print("  Running ...\n")
    else:
        # ── Live mode: clear stale batch trace so MATLAB uses live timer ──
        if os.path.exists(TRACE_FILE):
            os.remove(TRACE_FILE)
        # ── Write a stub viz_data.json immediately so MATLAB's wait loop ──
        # exits on the first check instead of blocking for 20 seconds.
        _stub = {
            "t": 0.0, "step": -1, "mode": "INIT",
            "pos_N": 0.0, "pos_E": 0.0,
            "start_N": 0.0, "start_E": 0.0,
            "target_N": target_N, "target_E": target_E,
            "carrot_N": 0.0, "carrot_E": 0.0,
            "angular_velocity_cmd_rad_s": 0.0, "lat_acc_cmd_mps2": 0.0,
            "nu": 0.0, "nu1": 0.0, "nu2": 0.0,
            "crossTrack": 0.0, "alongTrack": 0.0, "L1_distance": 5.0,
            "heading_rad": 0.0,
            "left_angle_deg": 80.0, "right_angle_deg": 80.0,
            "delta_arm_deg": 0.0, "left_pw": 1591, "right_pw": 1524,
            "alt_m": START_ALT_M, "nominal": False,
            "wind_N": 0.0, "wind_E": 0.0, "gnd_speed_mps": 0.0,
        }
        with open(DATA_FILE, "w") as _f:
            json.dump(_stub, _f)
        # ── Auto-launch MATLAB unless suppressed ──────────────────────────
        if not args.no_matlab:
            launched = _launch_matlab(os.path.abspath(MATLAB_SCRIPT))
            if launched:
                _wait_for_matlab(args.matlab_wait)
            else:
                time.sleep(2)
        else:
            print("  Mode    : LIVE (real-time) | MATLAB not launched (--no-matlab)")
            time.sleep(2)
        print("  Running ...\n")

    step    = 0

    try:
        while True:
            t0  = time.monotonic()
            now = step * dt

            # ── Wind model (constant + turbulence) ──────────────────────
            turb_N = random.gauss(0.0, WIND_TURB_STD)
            turb_E = random.gauss(0.0, WIND_TURB_STD)
            wind_N = WIND_N_MPS + turb_N
            wind_E = WIND_E_MPS + turb_E

            # Ground velocity = body airspeed vector + wind
            gnd_vel_N  = GLIDE_SPEED_MPS * math.cos(sim_heading) + wind_N
            gnd_vel_E  = GLIDE_SPEED_MPS * math.sin(sim_heading) + wind_E
            gnd_speed  = math.hypot(gnd_vel_N, gnd_vel_E)
            gnd_course = math.atan2(gnd_vel_E, gnd_vel_N)   # GPS-reported track

            # ── Build sensor snapshot from simulation state ──────────────
            lat, lon = NE_to_latlon(sim_pos_N, sim_pos_E, ORIGIN_LAT, ORIGIN_LON)

            gps = MockGPS(
                lat=lat, lon=lon,
                course=gnd_course,   # GPS sees ground track, not body heading
                speed=gnd_speed,
                pos_ts=now, motion_ts=now,
            )
            imu  = MockIMU(gyrz=sim_gyrz, ts=now)
            baro = MockBaro(alt=sim_alt,   ts=now)

            # ── Call real guidance.py ────────────────────────────────────
            l1_input, mode = ProduceL1Input(
                gps, imu, baro,
                ORIGIN_LAT, ORIGIN_LON,
                TARGET_LAT, TARGET_LON,
                now,
            )
            l1_out = ProduceL1Output(
                l1_input, mode,
                ORIGIN_LAT, ORIGIN_LON,
                TARGET_LAT, TARGET_LON,
                now,
            )

            # ── Call real control.py ─────────────────────────────────────
            ctrl_in  = ProduceCtrlInput(l1_out, now)
            ctrl_out = ProduceCtrlOutput(
                ctrler, ctrl_in,
                math.degrees(sim_gyrz),   # simulated IMU measurement
                now,
            )

            # ── Integrate kinematics (with wind drift) ──────────────────
            yaw_cmd   = l1_out.angular_velocity_cmd_rad_s if l1_out.nominal else 0.0
            sim_gyrz  = yaw_cmd                       # 1-step delay feedback
            sim_heading += yaw_cmd * dt
            sim_heading  = (sim_heading + math.pi) % (2.0 * math.pi) - math.pi
            sim_pos_N   += gnd_vel_N * dt             # actual ground movement (body + wind)
            sim_pos_E   += gnd_vel_E * dt
            sim_alt     -= DESCENT_RATE_MPS * dt


            # ── Build and send UDP payload ───────────────────────────────
            payload = {
                # Time
                "t":    round(now, 3),
                "step": step,
                "mode": l1_out.reason,
                # Positions (m from origin/start)
                "pos_N":    round(sim_pos_N, 3),
                "pos_E":    round(sim_pos_E, 3),
                "start_N":  round(start_N, 3),
                "start_E":  round(start_E, 3),
                "target_N": round(l1_out.target_N, 3),
                "target_E": round(l1_out.target_E, 3),
                "carrot_N": round(l1_out.carrot_N, 3),
                "carrot_E": round(l1_out.carrot_E, 3),
                # L1 guidance outputs
                "angular_velocity_cmd_rad_s": round(l1_out.angular_velocity_cmd_rad_s, 5),
                "lat_acc_cmd_mps2":   round(l1_out.lat_acc_cmd_mps2,   5),
                "nu":          round(l1_out.angle_to_turn,          5),
                "nu1":         round(l1_out.nu1,         5),
                "nu2":         round(l1_out.nu2,         5),
                "crossTrack":  round(l1_out.crossTrack,  3),
                "alongTrack":  round(l1_out.alongTrack,  3),
                "L1_distance": round(l1_out.L1_distance, 3),
                "heading_rad": round(l1_out.current_heading_rad, 5),
                # Control outputs (arm angles: 0° = up, 80° = neutral, 160° = full brake)
                "left_angle_deg":  round(ctrl_out.left_angle_deg,  2),
                "right_angle_deg": round(ctrl_out.right_angle_deg, 2),
                "delta_arm_deg":   round(ctrl_out.delta_arm_deg,   2),
                "left_pw":  ctrl_out.left_pw,
                "right_pw": ctrl_out.right_pw,
                # State
                "alt_m":  round(sim_alt, 2),
                "nominal": l1_out.nominal,
                # Wind
                "wind_N": round(wind_N, 3),
                "wind_E": round(wind_E, 3),
                "gnd_speed_mps": round(gnd_speed, 3),
            }

            if batch_mode:
                trace_payloads.append(payload)
            else:
                # Write to shared file (MATLAB reads this)
                with open(DATA_FILE, "w") as f:
                    json.dump(payload, f)
                # Also send UDP (kept for compatibility)
                try:
                    assert sock is not None
                    sock.sendto(json.dumps(payload).encode(), (UDP_HOST, UDP_PORT))
                except Exception:
                    pass

            if step % SIM_RATE_HZ == 0:
                dist = math.hypot(sim_pos_N - target_N, sim_pos_E - target_E)
                L = ctrl_out.left_angle_deg
                R = ctrl_out.right_angle_deg
                dL = L - NEUTRAL_ARM_DEG
                dR = R - NEUTRAL_ARM_DEG
                print(
                    f"  t={now:6.1f}s | alt={sim_alt:6.1f}m | "
                    f"dist={dist:6.1f}m | "
                    f"yaw={math.degrees(l1_out.angular_velocity_cmd_rad_s):+6.2f} deg/s | "
                    f"L={L:5.1f}°({dL:+5.1f}) R={R:5.1f}°({dR:+5.1f}) | "
                    f"delta={ctrl_out.delta_arm_deg:+5.1f}° | "
                    f"{l1_out.reason}"
                )

            step += 1

            if not batch_mode:
                elapsed = time.monotonic() - t0
                sleep_t = dt - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

            if sim_alt <= 0.0:
                print("\n  [sim] Landed. Simulation complete.")
                break

    except KeyboardInterrupt:
        print("\n  [sim] Stopped by user."  )
    finally:
        if sock is not None:
            sock.close()

    if batch_mode and trace_payloads:
        trace_path = os.path.abspath(args.trace_file)
        os.makedirs(os.path.dirname(trace_path), exist_ok=True)
        with open(trace_path, "w") as f:
            json.dump(trace_payloads, f)
        with open(DATA_FILE, "w") as f:
            json.dump(trace_payloads[-1], f)
        print(f"  [sim] Wrote {len(trace_payloads)} frames to {trace_path}")
        print(f"  [sim] Final frame mirrored to {os.path.abspath(DATA_FILE)}")
        if not args.no_matlab:
            _launch_matlab(os.path.abspath(MATLAB_SCRIPT))


if __name__ == "__main__":
    main()
