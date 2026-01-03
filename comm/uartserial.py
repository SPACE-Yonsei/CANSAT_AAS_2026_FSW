import threading
import time

SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 9600
SERIAL_TIMEOUT = 1

def init_serial():
    import serial

    # Open serial port (adjust the port and baudrate as needed)
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)

    return ser

def send_serial_data(ser, string_to_write:str):
    if ser == None:
        return
    
    try:
        # Check if serial port is still open
        if not hasattr(ser, 'is_open') or not ser.is_open:
            return
        
        ser.write(string_to_write.encode())
    except (AttributeError, OSError, ValueError, TypeError, IOError) as e:
        # Serial port may be closed or in invalid state
        return
    except Exception as e:
        # Catch any other unexpected errors
        return

def receive_serial_data(ser) -> str:

    if ser == None:
        return None
    
    try:
        # Check if serial port is still open (this may raise AttributeError if ser is being deinitialized)
        try:
            if not hasattr(ser, 'is_open') or not ser.is_open:
                return None
        except (AttributeError, OSError, ValueError, TypeError):
            # Port may be in the process of being closed or invalid
            return None
        
        # Try to read data, but catch any errors that might occur during termination
        try:
            read_data = ser.readline()
            if read_data is None or len(read_data) == 0:
                return None
        except (AttributeError, OSError, ValueError, TypeError, IOError):
            # readline() may fail if port is being closed or is in invalid state
            return None
            
        try:
            decoded_data = read_data.decode('utf-8', errors='ignore').strip()
            return decoded_data if decoded_data else None
        except (UnicodeDecodeError, AttributeError, TypeError):
            # Decoding may fail if read_data is in unexpected format
            return None
    except Exception as e:
        # Catch any other unexpected errors (including errors from ser.is_open or ser.readline())
        return None

def terminate_serial(ser):
    if ser == None:
        return
    # Close the serial port
    ser.close()

DEBUG_RUNSTATUS = True

def debug_send_tlm_thread(ser):
    global DEBUG_RUNSTATUS
    while DEBUG_RUNSTATUS:
        send_serial_data(ser, "Hello World!")
        time.sleep(1)
    return

if __name__ == "__main__":
    print("DEBUG MODE")
    serial_instance = init_serial()

    try:
        threading.Thread(target = debug_send_tlm_thread, args=(serial_instance,), daemon=True ).start()

        while DEBUG_RUNSTATUS:
            rcv_data = receive_serial_data(serial_instance)
            if rcv_data == "":
                print("TIMEOUT")
            else:
                print(rcv_data)
            time.sleep(1)

    except KeyboardInterrupt:
        print("Keyboard Interrupt!")
        
    finally:
        DEBUG_RUNSTATUS = False
        terminate_serial(serial_instance)