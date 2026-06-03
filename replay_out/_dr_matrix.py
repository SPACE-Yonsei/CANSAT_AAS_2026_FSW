import os, sys, math
sys.stdout.reconfigure(encoding="utf-8")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Sensor_Motor import guidance as g
from Sensor_Motor.guidance import ControlMode

def setup(gyrz_dps=10.0):
    g.reset(keep_candidate_origin=False)
    m = g._MISSION_t; s = g._STATE_t
    m.origin_ready = True; m.target_ready = True
    m.origin_lat, m.origin_lon = 37.0, 127.0
    m.target_E, m.target_N = 100.0, 100.0
    # DR anchor + current state valid, recent
    d = s.dr
    d.anchor_E=d.current_E=0.0; d.anchor_N=d.current_N=0.0
    d.anchor_V=d.current_V=5.0; d.anchor_course=d.current_course=0.0
    d.anchor_time=99.5; d.current_time=99.5; d.last_step_time=99.5
    d.yaw_at_anchor=0.0
    s.imu.gyr_z=math.radians(gyrz_dps); s.imu.yaw=0.0
    return m, s

def flags(s, **kw):
    f = s.flags
    for k in ("gps_pos_fresh","gps_motion_fresh","imu_gyrz_fresh","imu_yaw_fresh",
              "baro_sink_fresh","acc_fresh"):
        setattr(f, k, kw.get(k, False))
    f.gps_pos_stale = not f.gps_pos_fresh
    f.gps_motion_stale = not f.gps_motion_fresh
    f.dr_current_valid = g.dr_current_valid(s.dr)
    f.dr_anchor_valid = True
    return f

NOW = 100.0
# (label, gps_pos, gps_motion, gyrz, yaw, baro, acc, gyrz_dps) -> expected
CASES = [
    # GPS tracking
    ("GPS both fresh, gyro ok",      dict(gps_pos_fresh=1,gps_motion_fresh=1,imu_gyrz_fresh=1,baro_sink_fresh=1,acc_fresh=1), ControlMode.GPS_TRACKING_CLOSED),
    ("GPS both fresh, gyro stale",   dict(gps_pos_fresh=1,gps_motion_fresh=1,imu_yaw_fresh=1,baro_sink_fresh=1,acc_fresh=1), ControlMode.GPS_TRACKING_OPEN),
    # DR_M (pos fresh, motion stale)
    ("M: gyro+baro+acc",   dict(gps_pos_fresh=1,imu_gyrz_fresh=1,baro_sink_fresh=1,acc_fresh=1), ControlMode.DR_M_GBA_CLOSED),
    ("M: gyro+baro",       dict(gps_pos_fresh=1,imu_gyrz_fresh=1,baro_sink_fresh=1),             ControlMode.DR_M_GB_CLOSED),
    ("M: gyro only",       dict(gps_pos_fresh=1,imu_gyrz_fresh=1),                                ControlMode.DR_M_G_CLOSED),
    ("M: yaw+baro+acc",    dict(gps_pos_fresh=1,imu_yaw_fresh=1,baro_sink_fresh=1,acc_fresh=1),  ControlMode.DR_M_YBA_OPEN),
    ("M: yaw+baro",        dict(gps_pos_fresh=1,imu_yaw_fresh=1,baro_sink_fresh=1),              ControlMode.DR_M_YB_OPEN),
    ("M: yaw only",        dict(gps_pos_fresh=1,imu_yaw_fresh=1),                                 ControlMode.DR_M_Y_OPEN),
    # DR_PM (pos stale)
    ("PM: gyro+baro+acc",  dict(imu_gyrz_fresh=1,baro_sink_fresh=1,acc_fresh=1),                 ControlMode.DR_PM_GBA_CLOSED),
    ("PM: gyro+baro",      dict(imu_gyrz_fresh=1,baro_sink_fresh=1),                             ControlMode.DR_PM_GB_CLOSED),
    ("PM: gyro only",      dict(imu_gyrz_fresh=1),                                                ControlMode.DR_PM_G_CLOSED),
    ("PM: yaw+baro+acc",   dict(imu_yaw_fresh=1,baro_sink_fresh=1,acc_fresh=1),                  ControlMode.DR_PM_YBA_OPEN),
    ("PM: yaw+baro",       dict(imu_yaw_fresh=1,baro_sink_fresh=1),                              ControlMode.DR_PM_YB_OPEN),
    ("PM: yaw only",       dict(imu_yaw_fresh=1),                                                 ControlMode.DR_PM_Y_OPEN),
    # FAIL / guards
    ("FAIL: nothing fresh", dict(),                                                              ControlMode.FAIL),
]

print(f'{"case":24}{"expected":24}{"got":24}{"OK"}')
print("-"*78)
allok=True
for label, kw, exp in CASES:
    m, s = setup(gyrz_dps=10.0)
    flags(s, **kw)
    got = g.SelectControlMode(s.flags, NOW)
    ok = (got == exp)
    allok &= ok
    print(f'{label:24}{exp.value:24}{got.value:24}{"✓" if ok else "✗ FAIL"}')

# guard demos
print("\n--- 안전 가드 데모 ---")
# guard #5: high gyrz -> gyro not used -> falls to yaw (OPEN) even though gyrz fresh
m,s=setup(gyrz_dps=200.0)
flags(s, imu_gyrz_fresh=1, imu_yaw_fresh=1, baro_sink_fresh=1, acc_fresh=1)
print("guard#5 gyrz=200dps (>120):", g.SelectControlMode(s.flags,NOW).value, "(기대: DR_PM_YBA_OPEN — gyro 배제)")
# guard #1: anchor too old -> DR_TIMEOUT FAIL
m,s=setup(); s.dr.anchor_time=10.0  # age 90 > 60
flags(s, imu_gyrz_fresh=1, baro_sink_fresh=1, acc_fresh=1)
r=g.SelectControlMode(s.flags,NOW)
print("guard#1 anchor age=90s (>60):", r.value, "/ reason:", s.nav.fail_reason, "(기대: FAIL/DR_TIMEOUT)")
# no origin
m,s=setup(); m.origin_ready=False
flags(s, imu_gyrz_fresh=1, baro_sink_fresh=1, acc_fresh=1)
r=g.SelectControlMode(s.flags,NOW)
print("no origin:", r.value, "/ reason:", s.nav.fail_reason)

print("\nALL 12 DR CASES + GPS + FAIL MAPPED CORRECTLY:", allok)
