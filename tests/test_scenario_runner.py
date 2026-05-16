"""Unit tests for ground_station/scenario_runner.py.

These exercise the runner without any serial I/O: send_cb/get_tlm_cb are simple
fakes. The goals are:
  * presets are registered and self-consistent
  * setup / teardown command sequences contain the expected CMDs in order
  * tick() integrates heading, position, altitude correctly under both no-wind
    and pure-wind drift cases
  * closed-loop coupling: a non-neutral pulse pair produces a non-zero yaw rate
    in the simulation
  * landing / target-radius / timeout cause the scenario to finish
  * (0, 0) and out-of-range coordinates never appear in the SIMG payload
"""
from __future__ import annotations

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GS_DIR = os.path.join(ROOT, "ground_station")
sys.path.insert(0, GS_DIR)

from scenario_runner import (  # type: ignore[import-not-found]
    PRESETS,
    ScenarioConfig,
    ScenarioRunner,
    _haversine_m,
    get_preset,
    list_preset_names,
)


def _make_recorder():
    """Capture body strings from ScenarioRunner.send_cb in a list."""
    sent: list[str] = []

    def send(body: str) -> bool:
        sent.append(body)
        return True

    return sent, send


def _const_tlm(left: int = 1511, right: int = 1489):
    """Fake telemetry callback that always returns the same pulse pair.

    Defaults are LEFT_NEUTRAL / RIGHT_NEUTRAL from control.py:
    zero delta_arm -> zero yaw rate.
    """
    def cb():
        return {"left_pulse_us": str(left), "right_pulse_us": str(right)}
    return cb


def _no_sleep(_s):
    pass


class TestPresets(unittest.TestCase):
    def test_all_presets_registered(self):
        expected = {
            "calm", "west8", "gust12",
            "fast_descent", "slow_descent",
            "anti_parallel", "long_range",
        }
        self.assertEqual(set(list_preset_names()), expected)

    def test_get_preset_unknown(self):
        with self.assertRaises(KeyError):
            get_preset("does_not_exist")

    def test_preset_invariants(self):
        for name, cfg in PRESETS.items():
            with self.subTest(name=name):
                self.assertGreater(cfg.start_alt_m, 0.0)
                self.assertGreater(cfg.airspeed_ms, 0.0)
                self.assertGreater(cfg.descent_rate_ms, 0.0)
                self.assertGreaterEqual(cfg.wind_speed_ms, 0.0)
                self.assertGreater(cfg.tick_period_s, 0.0)
                self.assertNotEqual(
                    (cfg.start_lat, cfg.start_lon),
                    (cfg.target_lat, cfg.target_lon),
                    "start and target must differ",
                )


class TestSetupTeardown(unittest.TestCase):
    def test_setup_sequence_order(self):
        cfg = get_preset("calm")
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)

        self.assertEqual(sent[0], "CX,ON")
        self.assertEqual(sent[1], "SIM,ENABLE")
        self.assertEqual(sent[2], "SIM,ACTIVATE")
        self.assertTrue(sent[3].startswith("TC,"))
        self.assertTrue(sent[4].startswith("SIMG,"))
        self.assertTrue(sent[5].startswith("SIMP,"))
        self.assertEqual(sent[6], "SS,3")

    def test_setup_no_release_when_disabled(self):
        cfg = get_preset("calm")
        cfg.use_release_state = False
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        self.assertNotIn("SS,3", sent)

    def test_teardown_sends_landing_state_and_disable(self):
        cfg = get_preset("calm")
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        sent.clear()
        runner.stop(sleep_fn=_no_sleep, reason="test")
        self.assertIn("SS,5", sent)
        self.assertIn("SIM,DISABLE", sent)

    def test_teardown_skip_when_disabled(self):
        cfg = get_preset("calm")
        cfg.auto_disable_sim = False
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        sent.clear()
        runner.stop(sleep_fn=_no_sleep, reason="test")
        self.assertNotIn("SIM,DISABLE", sent)


class TestPhysics(unittest.TestCase):
    def _basic_cfg(self, **overrides) -> ScenarioConfig:
        cfg = ScenarioConfig(
            name="unit",
            start_lat=37.5607, start_lon=126.9307,
            target_lat=37.5650, target_lon=126.9350,
            start_alt_m=200.0,
            init_heading_deg=0.0,    # straight north
            airspeed_ms=8.0,
            descent_rate_ms=5.0,
            wind_speed_ms=0.0,
            tick_period_s=1.0,
            simg_simp_spacing_s=0.0,
            timeout_s=300.0,
        )
        for k, v in overrides.items():
            setattr(cfg, k, v)
        return cfg

    def test_neutral_pulses_no_angular_velocity(self):
        cfg = self._basic_cfg()
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(1511, 1489), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        for _ in range(5):
            runner.tick()
        self.assertAlmostEqual(runner.state.angular_velocity_deg_s, 0.0, places=3)
        self.assertAlmostEqual(runner.state.heading_deg, 0.0, places=3)

    def test_asymmetric_pulses_produce_yaw(self):
        cfg = self._basic_cfg()
        sent, send = _make_recorder()
        # Positive delta_arm (left + right pulse sum > 3000) -> right turn.
        # Use 1700 / 1700 = sum 3400 -> delta ~= (3400-3000)/(2000/180) = +36 deg
        runner = ScenarioRunner(send, _const_tlm(1700, 1700), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        runner.tick()
        self.assertGreater(runner.state.angular_velocity_deg_s, 0.0,
                           "positive delta_arm should turn right (positive yaw rate)")

    def test_altitude_decreases_at_descent_rate(self):
        cfg = self._basic_cfg(start_alt_m=200.0, descent_rate_ms=5.0,
                              tick_period_s=1.0)
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        for _ in range(10):
            runner.tick()
        self.assertAlmostEqual(runner.state.alt_m, 150.0, delta=0.5)

    def test_north_heading_with_no_wind_moves_lat_only(self):
        cfg = self._basic_cfg(init_heading_deg=0.0, wind_speed_ms=0.0,
                              airspeed_ms=8.0, tick_period_s=1.0)
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        lat0, lon0 = runner.state.lat, runner.state.lon
        for _ in range(5):
            runner.tick()
        # 5 s @ 8 m/s due north = 40 m, ~0.000359 deg lat
        dlat = runner.state.lat - lat0
        dlon = runner.state.lon - lon0
        self.assertAlmostEqual(dlat * 111320.0, 40.0, delta=2.0)
        self.assertLess(abs(dlon * 111320.0 * math.cos(math.radians(lat0))), 0.5,
                        "longitude must barely change with pure-north heading")

    def test_west_wind_drifts_east(self):
        # With airspeed=0 and wind FROM west (270), the canister should drift
        # purely east. Set airspeed = 0 by making heading and airspeed match a
        # static condition: easier to just turn airspeed off.
        cfg = self._basic_cfg(
            init_heading_deg=90.0,    # heading east, but ignore airspeed
            airspeed_ms=0.0,
            wind_speed_ms=5.0,
            wind_dir_met_deg=270.0,   # wind FROM west -> drift east
            tick_period_s=1.0,
        )
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        lat0, lon0 = runner.state.lat, runner.state.lon
        for _ in range(4):
            runner.tick()
        dn_m = (runner.state.lat - lat0) * 111320.0
        de_m = (runner.state.lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
        # 4 s @ 5 m/s drift east
        self.assertAlmostEqual(de_m, 20.0, delta=1.0)
        self.assertAlmostEqual(dn_m, 0.0, delta=0.5)


class TestEndConditions(unittest.TestCase):
    def test_landing_finishes_when_altitude_hits_zero(self):
        cfg = ScenarioConfig(
            name="landtest",
            start_lat=37.5607, start_lon=126.9307,
            target_lat=37.5700, target_lon=126.9400,  # far enough not to hit
            start_alt_m=10.0,
            init_heading_deg=0.0,
            descent_rate_ms=5.0,
            tick_period_s=1.0,
            simg_simp_spacing_s=0.0,
            timeout_s=120.0,
        )
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        steps = 0
        while runner.tick() and steps < 50:
            steps += 1
        self.assertTrue(runner.state.finished)
        self.assertIn("landed", runner.state.finish_reason)
        self.assertLessEqual(runner.state.alt_m, 0.0)

    def test_timeout_finishes_runaway(self):
        cfg = ScenarioConfig(
            name="timeout",
            start_lat=37.5607, start_lon=126.9307,
            target_lat=38.0, target_lon=128.0,    # far away
            start_alt_m=1.0e9,                    # never lands in budget
            init_heading_deg=180.0,               # heads opposite of target
            descent_rate_ms=0.001,
            tick_period_s=1.0,
            simg_simp_spacing_s=0.0,
            timeout_s=5.0,
        )
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        steps = 0
        while runner.tick() and steps < 100:
            steps += 1
        self.assertTrue(runner.state.finished)
        self.assertIn("timeout", runner.state.finish_reason)

    def test_target_radius_finishes(self):
        # airspeed=0 + no wind keeps the canister stationary so we can test the
        # radius gate without competing against forward motion.
        cfg = ScenarioConfig(
            name="radius",
            start_lat=37.5607, start_lon=126.9307,
            target_lat=37.5607 + 1e-5, target_lon=126.9307,  # ~1.1 m north
            start_alt_m=1000.0,
            init_heading_deg=0.0,
            airspeed_ms=0.0,
            descent_rate_ms=1.0,
            wind_speed_ms=0.0,
            tick_period_s=1.0,
            simg_simp_spacing_s=0.0,
            target_radius_m=5.0,
        )
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        runner.tick()
        self.assertTrue(runner.state.finished)
        self.assertIn("target", runner.state.finish_reason)


class TestSimgPayloadShape(unittest.TestCase):
    def test_simg_payload_format(self):
        cfg = get_preset("calm")
        cfg.simg_simp_spacing_s = 0.0
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        runner.tick()
        simg_lines = [s for s in sent if s.startswith("SIMG,")]
        self.assertGreater(len(simg_lines), 0)
        for s in simg_lines:
            parts = s.split(",")
            self.assertEqual(parts[0], "SIMG")
            # SIMG,lat,lon,course,speed,alt = 6 tokens
            self.assertEqual(len(parts), 6)
            lat = float(parts[1]); lon = float(parts[2])
            self.assertTrue(-90.0 <= lat <= 90.0)
            self.assertTrue(-180.0 <= lon <= 180.0)
            self.assertFalse(lat == 0.0 and lon == 0.0,
                             "(0,0) is rejected by cmd_simg in commapp.py")


class TestRunnerLifecycle(unittest.TestCase):
    def test_double_start_is_noop(self):
        cfg = get_preset("calm")
        cfg.simg_simp_spacing_s = 0.0
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        self.assertTrue(runner.start(sleep_fn=_no_sleep))
        self.assertFalse(runner.start(sleep_fn=_no_sleep))

    def test_tick_after_stop_is_noop(self):
        cfg = get_preset("calm")
        cfg.simg_simp_spacing_s = 0.0
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        runner.stop(sleep_fn=_no_sleep)
        self.assertFalse(runner.tick())

    def test_stop_idempotent(self):
        cfg = get_preset("calm")
        cfg.simg_simp_spacing_s = 0.0
        sent, send = _make_recorder()
        runner = ScenarioRunner(send, _const_tlm(), lambda _m: None, cfg)
        runner.start(sleep_fn=_no_sleep)
        runner.stop(sleep_fn=_no_sleep)
        # Second stop must not raise nor double-send teardown.
        sent.clear()
        runner.stop(sleep_fn=_no_sleep)
        self.assertEqual(sent, [])


class TestHaversine(unittest.TestCase):
    def test_zero_distance(self):
        self.assertAlmostEqual(_haversine_m(37.5, 127.0, 37.5, 127.0), 0.0, places=6)

    def test_known_distance(self):
        # ~111 km per degree of latitude
        d = _haversine_m(37.0, 127.0, 38.0, 127.0)
        self.assertAlmostEqual(d, 111195.0, delta=200.0)


if __name__ == "__main__":
    unittest.main()
