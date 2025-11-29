import os

# Config.py, Set the flight software operation mode

CONF_NONE = 0
CONF_PAYLOAD = 1
CONF_CONTAINER = 2

STATE_NAME_TO_ID = {
    "LAUNCHPAD": 0,
    "LAUNCH_PAD": 0,
    "ASCENT": 1,
    "APOGEE": 2,
    "DESCENT": 3,
    "PROBE_RELEASE": 4,
    "PROBERELEASE": 4,
    "LANDED": 5,
}

FSW_CONF = CONF_PAYLOAD
STATE_OVERRIDE: int | None = None

config_file_path = 'lib/config.txt'

if not os.path.exists(config_file_path):
    print(f"#################################################################\n\nConfig file does not exist: {config_file_path}, Configure the config file to run FSW!\n\n#################################################################")

    initial_conf_file_content = """# Config.txt
# Select the FSW operation mode
# Currently supports PAYLOAD, CONTAINER
# SELECTED=PAYLOAD
#
# Optional: Force the initial flight-logic state (LAUNCHPAD, ASCENT,
# APOGEE, DESCENT, PROBE_RELEASE, LANDED, or NONE)
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
                print("#################################################################\n\n NONE SELECTED \n\n#################################################################")
            elif value == "PAYLOAD":
                FSW_CONF = CONF_PAYLOAD
                print("#################################################################\n\n PAYLOAD SELECTED \n\n#################################################################")
            elif value == "CONTAINER":
                FSW_CONF = CONF_CONTAINER
                print("#################################################################\n\n CONTAINER SELECTED \n\n#################################################################")
            else:
                print(f"#################################################################\n\n INVALID CONFIG SELECTED={value}, defaulting to PAYLOAD \n\n#################################################################")
                FSW_CONF = CONF_PAYLOAD
            selected_set = True

        elif upper_line.startswith("STATE_OVERRIDE="):
            value = upper_line.split("=", 1)[1]
            if value in ("", "NONE"):
                STATE_OVERRIDE = None
                print("#################################################################\n\n STATE OVERRIDE DISABLED \n\n#################################################################")
            elif value.isdigit() and int(value) in STATE_NAME_TO_ID.values():
                STATE_OVERRIDE = int(value)
                print(f"#################################################################\n\n STATE OVERRIDE SET TO {STATE_OVERRIDE} \n\n#################################################################")
            elif value in STATE_NAME_TO_ID:
                STATE_OVERRIDE = STATE_NAME_TO_ID[value]
                print(f"#################################################################\n\n STATE OVERRIDE SET TO {value} \n\n#################################################################")
            else:
                STATE_OVERRIDE = None
                print(f"#################################################################\n\n INVALID STATE_OVERRIDE={value} (ignored) \n\n#################################################################")