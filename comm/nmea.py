# comm/nmea.py

def parse_lat_lon(lat_str, ns, lon_str, ew):
    # lat: ddmm.mmmm, lon: dddmm.mmmm
    if not lat_str or not lon_str:
        return None, None

    try:
        lat_deg = float(lat_str[:2])
        lat_min = float(lat_str[2:])
        lon_deg = float(lon_str[:3])
        lon_min = float(lon_str[3:])
    except ValueError:
        return None, None

    lat = lat_deg + lat_min / 60.0
    lon = lon_deg + lon_min / 60.0

    if ns == "S":
        lat = -lat
    if ew == "W":
        lon = -lon

    return lat, lon


def parse_gga(sentence: str):
    """
    $GNGGA,time,lat,NS,lon,EW,fix,num_sats,hdop,alt,unit,...
    """
    if not sentence.startswith(("$GNGGA", "$GPGGA")):
        return None

    fields = sentence.split(",")

    if len(fields) < 10:
        return None

    time_utc   = fields[1]
    lat_str    = fields[2]
    ns         = fields[3]
    lon_str    = fields[4]
    ew         = fields[5]
    fix_quality = fields[6]
    num_sats   = fields[7]
    hdop       = fields[8]
    alt        = fields[9]

    lat, lon = parse_lat_lon(lat_str, ns, lon_str, ew)

    return {
        "type": "GGA",
        "time_utc": time_utc,
        "lat": lat,
        "lon": lon,
        "fix_quality": int(fix_quality) if fix_quality.isdigit() else 0,
        "num_sats": int(num_sats) if num_sats.isdigit() else 0,
        "hdop": float(hdop) if hdop else None,
        "alt": float(alt) if alt else None,
    }


def parse_rmc(sentence: str):
    """
    $GNRMC,time,status,lat,NS,lon,EW,speed,track,date,...
    """
    if not sentence.startswith(("$GNRMC", "$GPRMC")):
        return None

    fields = sentence.split(",")

    if len(fields) < 10:
        return None

    time_utc = fields[1]
    status   = fields[2]   # 'A' = valid, 'V' = void
    lat_str  = fields[3]
    ns       = fields[4]
    lon_str  = fields[5]
    ew       = fields[6]
    speed_kn = fields[7]
    date     = fields[9]

    lat, lon = parse_lat_lon(lat_str, ns, lon_str, ew)

    return {
        "type": "RMC",
        "time_utc": time_utc,
        "date": date,
        "status": status,
        "lat": lat,
        "lon": lon,
        "speed_kn": float(speed_kn) if speed_kn else None,
    }
