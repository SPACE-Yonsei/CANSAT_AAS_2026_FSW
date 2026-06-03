import csv, os, math, sys
sys.stdout.reconfigure(encoding="utf-8")

ALL_DR = [
    "DR_M_GBA_CLOSED","DR_M_GB_CLOSED","DR_M_G_CLOSED",
    "DR_M_YBA_OPEN","DR_M_YB_OPEN","DR_M_Y_OPEN",
    "DR_PM_GBA_CLOSED","DR_PM_GB_CLOSED","DR_PM_G_CLOSED",
    "DR_PM_YBA_OPEN","DR_PM_YB_OPEN","DR_PM_Y_OPEN",
]
DT_MAX = 1.0
TAGS = ["1cha","2cha","3cha"]

def load(tag):
    with open(f"replay_out/result_{tag}.csv", encoding="utf-8") as f:
        return list(csv.DictReader(f))

def dts(times):
    out=[]
    for i in range(len(times)):
        if i+1 < len(times):
            d = times[i+1]-times[i]; d = d if 0<=d<=DT_MAX else min(max(d,0),DT_MAX)
        else: d=0.0
        out.append(d)
    return out

def ff(x):
    try: return float(x)
    except: return float("nan")

print("="*70)
print("12개 DR 케이스 전수 집계 (state 3+4, 단위 초)")
print("="*70)
hdr = f'{"DR mode":22}' + "".join(f"{t:>10}" for t in TAGS)
print(hdr); print("-"*len(hdr))
agg = {m:{t:0.0 for t in TAGS} for m in ALL_DR}
other = {}  # non-DR modes
integ = {t:{} for t in TAGS}

for t in TAGS:
    rows = load(t)
    times = [ff(r["time"]) for r in rows]
    dd = dts(times)
    # integrity counters
    inv_navleak=0; conf_oob=0; nan_course_valid=0; spd_pos=0
    dur_by_mode={}
    in34_dur=0.0
    for i,r in enumerate(rows):
        st = int(float(r["state"])) if r["state"] not in("",None) else 0
        if st not in (3,4): continue
        dt = dd[i]; in34_dur+=dt
        m = r["control_mode"]
        dur_by_mode[m]=dur_by_mode.get(m,0.0)+dt
        if m in agg: agg[m][t]+=dt
        # integrity: l1output valid but nav NaN
        if int(r["l1output_valid"]) and (r["nav_E"]=="" or r["nav_N"]=="" or r["nav_course_deg"]==""):
            inv_navleak+=1
        c = ff(r["dr_confidence"])
        if math.isfinite(c) and not (-1e-9<=c<=1.0+1e-9): conf_oob+=1
    for m in ALL_DR: pass
    other[t]={m:d for m,d in dur_by_mode.items() if m not in agg}
    integ[t]={"in34_dur":in34_dur,"navleak":inv_navleak,"conf_oob":conf_oob,"rows":len(rows)}

for m in ALL_DR:
    vals = agg[m]
    if sum(vals.values())==0:
        line = f'{m:22}' + "".join(f"{'·':>10}" for t in TAGS)
    else:
        line = f'{m:22}' + "".join(f"{vals[t]:>10.1f}" for t in TAGS)
    print(line)

print("\n" + "="*70)
print("비-DR 모드 (참고)")
print("="*70)
allother = set()
for t in TAGS: allother|=set(other[t])
for m in sorted(allother):
    print(f'{m:22}' + "".join(f"{other[t].get(m,0.0):>10.1f}" for t in TAGS))

print("\n" + "="*70)
print("무결성 점검 (replay 산출물)")
print("="*70)
for t in TAGS:
    g = integ[t]
    print(f'{t}: state3+4 {g["in34_dur"]:.1f}s, rows {g["rows"]}, '
          f'nav-leak(valid&NaN)={g["navleak"]}, conf out-of-[0,1]={g["conf_oob"]}')

# 어떤 DR 케이스가 한 번도 안 나왔는지
never = [m for m in ALL_DR if all(agg[m][t]==0 for t in TAGS)]
seen  = [m for m in ALL_DR if any(agg[m][t]>0 for t in TAGS)]
print("\n관측된 DR 케이스:", seen)
print("미관측 DR 케이스:", never)
