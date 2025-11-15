import os
from lib import events

# Prevstate.py, store and restore prev state

PREV_ALT_CAL = 0
PREV_STATE  = 0
PREV_MAX_ALT = 0
Target_lat = 38
Target_lon = 128

prevstate_file_path = 'lib/prevstate.txt'

def write_prevstate_file():
    global PREV_ALT_CAL
    global PREV_STATE
    global PREV_MAX_ALT
    global Target_lat
    global Target_lon

    content = f"""# Prevstate.txt
# The Config below stores the prev flight data
# DO NOT EDIT MANUALLY
STATE={PREV_STATE}
ALTCAL={PREV_ALT_CAL}
MAXALT={PREV_MAX_ALT}
TARGET_LAT={Target_lat}
TARGET_LON={Target_lon}"""

    with open(prevstate_file_path, 'w') as file:
        file.write(content)

def reset_prevstate():
    global PREV_STATE
    global PREV_ALT_CAL
    global PREV_MAX_ALT

    PREV_STATE = "NONE"
    PREV_ALT_CAL = "NONE"
    PREV_MAX_ALT = "NONE"
    write_prevstate_file()
    return

def update_prevstate(state:int):
    global PREV_STATE
    PREV_STATE = state
    write_prevstate_file()
    return

def update_altcal(alt:float):
    global PREV_ALT_CAL
    PREV_ALT_CAL = alt
    write_prevstate_file()
    return

def update_maxalt(alt:float):
    global PREV_MAX_ALT
    PREV_MAX_ALT = alt
    write_prevstate_file()

def update_target_gps(lat: float, lon: float):
    """Update target GPS coordinates."""
    global Target_lat
    global Target_lon
    Target_lat = lat
    Target_lon = lon
    write_prevstate_file()
    events.LogEvent("RECOVERY", events.EventType.info, f"Target GPS updated: Lat={lat}, Lon={lon}")
    return

def init_prevstate():

    global PREV_STATE
    global PREV_ALT_CAL
    global PREV_MAX_ALT
    global Target_lat
    global Target_lon
    
    if not os.path.exists(prevstate_file_path):
        print(f"#################################################################\n\nPrevstate file does not exist, Using initial state\n\n#################################################################")
        write_prevstate_file()

    else:
        with open(prevstate_file_path, 'r') as file:
            lines = file.readlines()

        prevstate_lines = [line.strip() for line in lines if not line.strip().startswith('#')]
        for prevstate_line in prevstate_lines:
            prevstate_line = prevstate_line.strip().replace(" ", "").replace("\t", "")

            if "STATE=" in prevstate_line:
                prev_state = prevstate_line.split("=")[1].strip()
                if prev_state == "NONE":
                    events.LogEvent("RECOVERY", events.EventType.info, "Prev state is NONE")
                    PREV_STATE = 0
                    continue
                else:
                    events.LogEvent("RECOVERY", events.EventType.info, f"Prev state is {prev_state}")
                    PREV_STATE = int(prev_state)

            elif "ALTCAL=" in prevstate_line:
                prev_altcal = prevstate_line.split("=")[1].strip()
                if prev_altcal == "NONE":
                    events.LogEvent("RECOVERY", events.EventType.info, "Prev calibration is NONE")
                    PREV_ALT_CAL = 0
                    continue
                else:
                    events.LogEvent("RECOVERY", events.EventType.info, f"Prev calibration is {prev_altcal}")
                    PREV_ALT_CAL = float(prev_altcal)

            elif "MAXALT=" in prevstate_line:
                prev_maxalt = prevstate_line.split("=")[1].strip()
                if prev_maxalt == "NONE":
                    events.LogEvent("RECOVERY", events.EventType.info, "Prev maxalt is NONE")
                    PREV_MAX_ALT = 0
                    continue
                else:
                    events.LogEvent("RECOVERY", events.EventType.info, f"Prev maxalt is {prev_maxalt}")
                    PREV_MAX_ALT = float(prev_maxalt)
            
            elif "TARGET_LAT=" in prevstate_line:
                target_lat_str = prevstate_line.split("=")[1].strip()
                if target_lat_str == "NONE" or target_lat_str == "":
                    events.LogEvent("RECOVERY", events.EventType.info, "Target latitude is NONE")
                    Target_lat = 0
                else:
                    try:
                        Target_lat = float(target_lat_str)
                        events.LogEvent("RECOVERY", events.EventType.info, f"Target latitude restored: {Target_lat}")
                    except ValueError:
                        events.LogEvent("RECOVERY", events.EventType.error, f"Invalid target latitude: {target_lat_str}")
                        Target_lat = 0
            
            elif "TARGET_LON=" in prevstate_line:
                target_lon_str = prevstate_line.split("=")[1].strip()
                if target_lon_str == "NONE" or target_lon_str == "":
                    events.LogEvent("RECOVERY", events.EventType.info, "Target longitude is NONE")
                    Target_lon = 0
                else:
                    try:
                        Target_lon = float(target_lon_str)
                        events.LogEvent("RECOVERY", events.EventType.info, f"Target longitude restored: {Target_lon}")
                    except ValueError:
                        events.LogEvent("RECOVERY", events.EventType.error, f"Invalid target longitude: {target_lon_str}")
                        Target_lon = 0

    return