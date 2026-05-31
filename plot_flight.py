import csv, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
import numpy as np
from datetime import datetime

folder = r'C:\Users\ms kang\Desktop\닭\run_20260531_165535'

ref_ts = datetime(2026, 5, 31, 17, 1, 40)
def to_sec(dt): return (dt - ref_ts).total_seconds()
def ts2s(s): return to_sec(datetime.fromisoformat(s))

# --- Barometer altitude (->FlightLogic)
baro_t, baro_alt = [], []
with open(os.path.join(folder, 'Barometer.csv'), newline='', encoding='utf-8-sig') as fp:
    for r in csv.DictReader(fp):
        if r['receiver_name'] != 'FlightLogic':
            continue
        parts = r['data'].split(',')
        try:
            if int(parts[1]) == 1:
                baro_t.append(ts2s(r['timestamp']))
                baro_alt.append(float(parts[0]))
        except:
            pass

# --- GPS (valid fix=12 only)
gps_t, gps_spd = [], []
with open(os.path.join(folder, 'GPS.csv'), newline='', encoding='utf-8-sig') as fp:
    for r in csv.DictReader(fp):
        parts = r['data'].split(',')
        t = ts2s(r['timestamp'])
        if t < -10 or t > 80:
            continue
        try:
            if 'nan' not in r['data'] and int(parts[4]) == 12 and float(parts[2]) != 0.0:
                gps_t.append(t)
                gps_spd.append(float(parts[1]))
        except:
            pass

# --- raw_motor
rm_t, rm_left, rm_right, rm_dist, rm_mode = [], [], [], [], []
with open(os.path.join(folder, 'raw_motor.csv'), newline='', encoding='utf-8-sig') as fp:
    for r in csv.DictReader(fp):
        t = ts2s(r['timestamp'])
        if t < -10 or t > 80:
            continue
        rm_t.append(t)
        rm_left.append(float(r['left_pw']) if r['left_pw'] not in ('nan', '') else float('nan'))
        rm_right.append(float(r['right_pw']) if r['right_pw'] not in ('nan', '') else float('nan'))
        rm_dist.append(float(r['dist_to_target_m']) if r['dist_to_target_m'] not in ('nan', '') else float('nan'))
        rm_mode.append(r['control_mode'])

# State event lines
state_events = [
    (to_sec(datetime(2026,5,31,17,1,43)), 'ASCENT'),
    (to_sec(datetime(2026,5,31,17,1,55)), 'DESCENT'),
    (to_sec(datetime(2026,5,31,17,2,6)),  'DR_START\n(origin set)'),
    (to_sec(datetime(2026,5,31,17,2,15)), 'HOMING'),
    (to_sec(datetime(2026,5,31,17,2,33)), 'GPS_TRACK'),
    (to_sec(datetime(2026,5,31,17,2,43)), 'LANDED'),
]

mode_colors = {
    'ControlMode.FAIL':                '#555577',
    'ControlMode.FAIL_IMU_FALLBACK':   '#e74c3c',
    'ControlMode.DR_TRACKING_CLOSED':  '#1abc9c',
    'ControlMode.DETUMBLING':          '#f39c12',
    'ControlMode.GPS_TRACKING_CLOSED': '#3498db',
}
mode_labels = {
    'ControlMode.FAIL':                'FAIL',
    'ControlMode.FAIL_IMU_FALLBACK':   'FAIL_IMU_FALLBACK',
    'ControlMode.DR_TRACKING_CLOSED':  'DR_TRACKING_CLOSED',
    'ControlMode.DETUMBLING':          'DETUMBLING',
    'ControlMode.GPS_TRACKING_CLOSED': 'GPS_TRACKING_CLOSED',
}

fig = plt.figure(figsize=(16, 13))
fig.patch.set_facecolor('#0f0f1a')
gs = GridSpec(4, 1, figure=fig, hspace=0.06, top=0.93, bottom=0.07)

ax1 = fig.add_subplot(gs[0])
ax2 = fig.add_subplot(gs[1], sharex=ax1)
ax3 = fig.add_subplot(gs[2], sharex=ax1)
ax4 = fig.add_subplot(gs[3], sharex=ax1)

for ax in [ax1, ax2, ax3, ax4]:
    ax.set_facecolor('#16213e')
    ax.tick_params(colors='#ccd6f6', labelsize=9)
    for spine in ax.spines.values():
        spine.set_color('#2d3561')

def shade_modes(ax):
    if not rm_t:
        return
    prev_t = rm_t[0]
    prev_m = rm_mode[0]
    for i in range(1, len(rm_t)):
        if rm_mode[i] != prev_m or i == len(rm_t) - 1:
            c = mode_colors.get(prev_m, '#222244')
            ax.axvspan(prev_t, rm_t[i], alpha=0.22, color=c, linewidth=0)
            prev_t = rm_t[i]
            prev_m = rm_mode[i]

for ax in [ax1, ax2, ax3, ax4]:
    shade_modes(ax)

for t, label in state_events:
    for ax in [ax1, ax2, ax3, ax4]:
        ax.axvline(t, color='#ffffff', lw=0.9, ls='--', alpha=0.55, zorder=5)

# AX1: Altitude
baro_filt_t = [t for t in baro_t if -10 <= t <= 80]
baro_filt_a = [baro_alt[i] for i, t in enumerate(baro_t) if -10 <= t <= 80]
ax1.plot(baro_filt_t, baro_filt_a, color='#00d4ff', lw=2, label='Altitude (m)')
ax1.fill_between(baro_filt_t, 0, baro_filt_a, alpha=0.15, color='#00d4ff')
ax1.set_ylabel('Altitude (m)', color='#ccd6f6', fontsize=9)
ax1.set_ylim(-5, 140)
ax1.legend(loc='upper right', fontsize=8.5, facecolor='#16213e', labelcolor='#ccd6f6', framealpha=0.8)
ax1.set_title('Flight Log — 2026-05-31  17:01:40~17:03:00  |  배경색: Motor 제어 모드', color='#ccd6f6', fontsize=11, pad=6)
ax1.annotate('117m\npeak', xy=(3.1, 117), xytext=(9, 108),
             color='#00d4ff', fontsize=8,
             arrowprops=dict(arrowstyle='->', color='#00d4ff', lw=0.8))
for t, label in state_events:
    ax1.text(t + 0.3, 3, label, color='#ffffff', fontsize=7.5,
             fontweight='bold', va='bottom', rotation=90, alpha=0.9, zorder=10)

# AX2: GPS speed
ax2.plot(gps_t, gps_spd, color='#2ecc71', lw=1.5, marker='o', ms=4.5,
         label='GPS Speed (knots, fix=3D)', zorder=5)
ax2.set_ylabel('GPS Speed\n(knots)', color='#ccd6f6', fontsize=9)
ax2.set_ylim(-5, 130)
ax2.legend(loc='upper right', fontsize=8.5, facecolor='#16213e', labelcolor='#ccd6f6', framealpha=0.8)
ax2.annotate('GPS mostly nan\n(HOMING 28s 중 유효 ~32회)',
             xy=(35, 10), xytext=(45, 65),
             color='#ff6b6b', fontsize=8,
             arrowprops=dict(arrowstyle='->', color='#ff6b6b', lw=0.8))

# AX3: PWM
ax3.plot(rm_t, rm_left,  color='#e67e22', lw=1.5, label='Left PWM (μs)')
ax3.plot(rm_t, rm_right, color='#e74c3c', lw=1.5, label='Right PWM (μs)', alpha=0.85)
ax3.axhline(1500, color='#aaaaaa', lw=0.8, ls=':', alpha=0.5, label='Neutral 1500')
ax3.set_ylabel('PWM (μs)', color='#ccd6f6', fontsize=9)
ax3.set_ylim(400, 2700)
ax3.legend(loc='upper right', fontsize=8.5, facecolor='#16213e', labelcolor='#ccd6f6', framealpha=0.8)
det_t = to_sec(datetime(2026, 5, 31, 17, 2, 21))
ax3.annotate('DETUMBLING\n(PWM~900)', xy=(det_t, 902), xytext=(det_t + 3, 1250),
             color='#f39c12', fontsize=7.5,
             arrowprops=dict(arrowstyle='->', color='#f39c12', lw=0.8))

# AX4: Distance to target
ax4.plot(rm_t, rm_dist, color='#f1c40f', lw=2, label='Dist to Target (m)')
ax4.axhline(0, color='#aaaaaa', lw=0.7, ls=':')
ax4.set_ylabel('Dist to Target (m)', color='#ccd6f6', fontsize=9)
ax4.set_xlabel('Time (sec from 17:01:40)', color='#ccd6f6', fontsize=10)
ax4.set_ylim(-20, 400)
origin_t = to_sec(datetime(2026, 5, 31, 17, 2, 6))
ax4.annotate('Origin set here\n(pos_N=0, pos_E=0)',
             xy=(origin_t, 315), xytext=(origin_t + 5, 365),
             color='#1abc9c', fontsize=7.5,
             arrowprops=dict(arrowstyle='->', color='#1abc9c', lw=0.8))

patches = [mpatches.Patch(color=v, alpha=0.7, label=mode_labels[k]) for k, v in mode_colors.items()]
ax4.legend(handles=patches + [mpatches.Patch(color='#f1c40f', label='Dist to Target')],
           loc='upper right', fontsize=8, facecolor='#16213e', labelcolor='#ccd6f6',
           framealpha=0.85, ncol=2)

plt.setp(ax1.get_xticklabels(), visible=False)
plt.setp(ax2.get_xticklabels(), visible=False)
plt.setp(ax3.get_xticklabels(), visible=False)
ax1.set_xlim(-10, 80)

out = r'C:\Users\ms kang\Desktop\닭\flight_analysis_20260531.png'
plt.savefig(out, dpi=150, bbox_inches='tight', facecolor='#0f0f1a')
print('saved:', out)
