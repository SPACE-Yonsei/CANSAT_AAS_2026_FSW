import os

# Config.py, Set the flight software operation mode

CONF_NONE = 0
CONF_PAYLOAD = 1
CONF_CONTAINER = 2  # Container mode (for compatibility)
CONF_PAYLOAD_DESCENT = 3  # Payload descent mode (for motor control)

STATE_NAME_TO_ID = {
    "LAUNCHPAD": 0,
    "ASCENT": 1,
    "APOGEE": 2,        # 컨테이너-페이로드 사출
    "DESCENT": 3,       # 파라포일 모터 제어 시작
    "EGG_RELEASE": 4,   # 상공 2m에서 계란 사출
    "LANDED": 5,
}

FSW_CONF = CONF_PAYLOAD
STATE_OVERRIDE: int | None = None

config_file_path = 'lib/config.txt'

if not os.path.exists(config_file_path):
    print(f"Config file does not exist: {config_file_path}, Creating default config...")

    initial_conf_file_content = """# Config.txt
# FSW Configuration for Payload
# SELECTED=PAYLOAD
#
# Optional: Force the initial flight-logic state
# (LAUNCHPAD, ASCENT, APOGEE, DESCENT, PROBE_RELEASE, LANDED, or NONE)
# STATE_OVERRIDE=NONE"""

    with open(config_file_path, 'w') as file:
        file.write(initial_conf_file_content)

    raise FileNotFoundError(f"Required configuration file not found: {config_file_path}")

else:
    with open(config_file_path, 'r') as file:
        lines = file.readlines()

    selected_set = False
    for raw_line in lines:
        stripped = raw_line.strip()
        if not stripped or stripped.startswith('#'):
            continue

        config_line = stripped.replace(" ", "").replace("\t", "")
        upper_line = config_line.upper()

        if upper_line.startswith("SELECTED=") and not selected_set:
            value = upper_line.split("=", 1)[1]
            if value == "NONE":
                FSW_CONF = CONF_NONE
                print("[CONFIG] NONE selected - FSW will not run")
            elif value == "PAYLOAD":
                FSW_CONF = CONF_PAYLOAD
                print("[CONFIG] PAYLOAD mode selected")
            else:
                print(f"[CONFIG] Invalid SELECTED={value}, defaulting to PAYLOAD")
                FSW_CONF = CONF_PAYLOAD
            selected_set = True

        elif upper_line.startswith("STATE_OVERRIDE="):
            value = upper_line.split("=", 1)[1]
            if value in ("", "NONE"):
                STATE_OVERRIDE = None
                print("[CONFIG] STATE_OVERRIDE disabled")
            elif value.isdigit() and int(value) in STATE_NAME_TO_ID.values():
                STATE_OVERRIDE = int(value)
                print(f"[CONFIG] STATE_OVERRIDE set to {STATE_OVERRIDE}")
            elif value in STATE_NAME_TO_ID:
                STATE_OVERRIDE = STATE_NAME_TO_ID[value]
                print(f"[CONFIG] STATE_OVERRIDE set to {value} ({STATE_OVERRIDE})")
            else:
                STATE_OVERRIDE = None
                print(f"[CONFIG] Invalid STATE_OVERRIDE={value}, ignored")
