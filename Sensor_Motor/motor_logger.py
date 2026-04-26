import os
import types
from datetime import datetime


class MotorLogger:
    def __init__(self, sim_log_path: str, control_log_dir: str = "./sensorlogs"):
        os.makedirs(control_log_dir, exist_ok=True)
        self._sim  = open(sim_log_path, "a", encoding="utf-8")
        self._ctrl = open(os.path.join(control_log_dir, "control.txt"), "a", encoding="utf-8")

    def guidance_in(self, state, baro_m, imu, gps, fidelity, target) -> None:
        self.dbg(
            f"[GUIDANCE IN ] "
            f"state={state} baro={baro_m:.1f}m | "
            f"yaw={imu.yaw:.1f}° gyrz={imu.gyrz:.2f} | "
            f"gps=({gps.lat:.6f},{gps.lon:.6f}) spd={gps.speed:.1f} crs={gps.course:.1f} | "
            f"fix={fidelity.fix_quality} sats={fidelity.sats} rmc={fidelity.rmc_status} | "
            f"target=({target.lat:.6f},{target.lon:.6f})"
        )

    def motor_out(self, m) -> None:
        self.dbg(
            f"[MOTOR] "
            f"L: {m.left_cmd_deg:6.1f}°  pw={m.left_pulse} | "
            f"R: {m.right_cmd_deg:6.1f}°  pw={m.right_pulse} | "
            f"delta={m.actual_delta_deg:+.1f}°  exp_yr={m.expected_yaw_rate:+.2f}°/s"
        )

    def dbg(self, line: str) -> None:
        ts   = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        full = f"[{ts}] {line}"
        print(full)
        self._sim.write(full + "\n")
        self._sim.flush()

    def control(self, g, m) -> None:
        if m is None:
            m = types.SimpleNamespace(
                left_cmd_deg=0.0, right_cmd_deg=0.0,
                actual_delta_deg=0.0, expected_yaw_rate=0.0,
                left_pulse=0, right_pulse=0,
            )
        t = datetime.now().isoformat(sep=" ", timespec="milliseconds")
        self._ctrl.write(
            f"{t},"
            f"state:{g.state},"
            f"dist:{g.distance:.2f},"
            f"cmd_yr:{g.commanded_yaw_rate:.2f},"
            f"L_deg:{m.left_cmd_deg:.1f},"
            f"R_deg:{m.right_cmd_deg:.1f},"
            f"L_pw:{m.left_pulse},"
            f"R_pw:{m.right_pulse}\n"
        )

    def close(self) -> None:
        for f in (self._sim, self._ctrl):
            try:
                f.close()
            except Exception:
                pass
