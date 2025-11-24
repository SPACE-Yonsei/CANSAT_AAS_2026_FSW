#!/usr/bin/env python3
import time
from smbus2 import SMBus

I2C_BUS = 1
GNSS_ADDR = 0x42
READ_SIZE = 32

# -------------------------------------
# NMEA Helper Functions
# -------------------------------------

# -------------------------------------
# I2C GNSS Reader
# -------------------------------------
def main():
    bus = SMBus(I2C_BUS)
    buffer = b""

    print("[GNSS I2C] Reading from 0x42 ...\n")

    try:
        while True:
            # 읽기
            try:
                try:
                    data = bus.read_i2c_block_data(GNSS_ADDR, 0xFF, READ_SIZE)
                except OSError:
                    data = bus.read_i2c_block_data(GNSS_ADDR, 0x00, READ_SIZE)
            except:
                time.sleep(0.05)
                continue

            chunk = bytes(data)

            buffer += chunk

            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                line = line + b"\n"

                if b"$" not in line:
                    continue

                line = line[line.find(b"$"):]
                text = line.decode("ascii", errors="ignore").strip()

                parts = text.split(",")

                # -------------------------
                # Pretty Print GGA
                # -------------------------
                if text.startswith(("$GNGGA", "$GPGGA")):
                    gga = parse_gga(parts)
                    if gga:
                        print("📡 GGA Frame")
                        print(f"  ▸ Time (UTC):   {gga['time']}")
                        print(f"  ▸ Latitude:     {gga['lat']}")
                        print(f"  ▸ Longitude:    {gga['lon']}")
                        print(f"  ▸ Altitude:     {gga['alt']} m")
                        print(f"  ▸ Fix Quality:  {gga['fix']}")
                        print(f"  ▸ Satellites:   {gga['sats']}")
                        print("-"*45)
                    continue

                # -------------------------
                # Pretty Print RMC
                # -------------------------
                if text.startswith(("$GNRMC", "$GPRMC")):
                    rmc = parse_rmc(parts)
                    if rmc:
                        print("🧭 RMC Frame")
                        print(f"  ▸ Time (UTC):   {rmc['time']}")
                        print(f"  ▸ Status:       {rmc['status']}")
                        print(f"  ▸ Latitude:     {rmc['lat']}")
                        print(f"  ▸ Longitude:    {rmc['lon']}")
                        print(f"  ▸ Speed(knots): {rmc['speed_knots']}")
                        print(f"  ▸ Date:         {rmc['date']}")
                        print("-"*45)
                    continue

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[EXIT] stop.")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
