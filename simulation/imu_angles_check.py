import time
import board
import busio
import adafruit_bno055

def main():
    # Create I2C bus
    i2c = busio.I2C(board.SCL, board.SDA)

    # Create sensor object
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    # Optional: Set operation mode if needed, default is NDOF
    # sensor.mode = adafruit_bno055.NDOF_MODE

    print("Reading BNO055 3D Euler Angles (Roll, Pitch, Yaw)... Press Ctrl+C to exit.")
    print("Ensure your BNO055 sensor is correctly wired and connected via I2C.")
    print("You might need to run this script with 'sudo' on Linux for I2C access.")

    try:
        while True:
            # Read Euler angles
            # The library returns (yaw, roll, pitch) by default for Euler angles
            # We'll reorder them to (roll, pitch, yaw) for consistency with the simulation
            yaw, roll, pitch = sensor.euler

            if roll is not None and pitch is not None and yaw is not None:
                print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
            else:
                print("Sensor data not available. Check sensor connection and calibration.")

            time.sleep(0.1) # Read every 100ms
    except RuntimeError as e:
        print(f"Runtime Error: {e}. Ensure the sensor is connected and working.")
    except KeyboardInterrupt:
        print("\nSensor reading stopped.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    main()

