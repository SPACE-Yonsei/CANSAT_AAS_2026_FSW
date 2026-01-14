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

def enter_command_mode(ser):
    """Enter XBee command mode by sending +++"""
    if ser is None:
        return False
    
    try:
        ser.reset_input_buffer()
        time.sleep(1.0)  # Guard time before +++
        ser.write("+++".encode())
        time.sleep(1.0)  # Guard time after +++
        
        # Wait for OK response
        start_time = time.time()
        while time.time() - start_time < 2.0:
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                if "OK" in response:
                    return True
            time.sleep(0.1)
        return False
    except Exception as e:
        print(f"Error entering command mode: {e}")
        return False

def exit_command_mode(ser):
    """Exit XBee command mode by sending ATCN"""
    if ser is None:
        return
    try:
        ser.write("ATCN\r".encode())
        time.sleep(0.5)
    except:
        pass

def send_at_command(ser, command, use_command_mode=False):
    """Send an AT command to XBee and return the response"""
    if ser is None:
        return None
    
    try:
        # Clear any pending data
        ser.reset_input_buffer()
        time.sleep(0.1)
        
        # Enter command mode if needed
        if use_command_mode:
            if not enter_command_mode(ser):
                return None
        
        # Send AT command with carriage return
        cmd = command + "\r"
        ser.write(cmd.encode())
        
        # Wait a bit for response
        time.sleep(0.5)
        
        # Read response
        response = ""
        start_time = time.time()
        while time.time() - start_time < 2.0:  # 2 second timeout
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                response += data
                # XBee typically responds with command echo + value + \r\n
                # Look for OK or a value on a new line
                if '\r' in response and len(response) > len(cmd):
                    break
            time.sleep(0.1)
        
        # Exit command mode if we entered it
        if use_command_mode:
            exit_command_mode(ser)
        
        return response.strip() if response else None
    except Exception as e:
        print(f"Error sending AT command: {e}")
        return None

if __name__ == "__main__":
    print("DEBUG MODE - XBee AT Command Query")
    serial_instance = init_serial()
    
    if serial_instance is None:
        print("Failed to initialize serial port")
        exit(1)

    def parse_at_response(response, command):
        """Parse AT command response to extract the value"""
        if not response:
            return None
        
        # Remove command echo
        response = response.replace(command, "").strip()
        
        # Split by newlines and carriage returns
        lines = response.replace('\r', '\n').split('\n')
        
        # Find the value (usually a hex number or decimal)
        for line in lines:
            line = line.strip()
            if line and line != "OK" and not line.startswith("AT"):
                return line
        
        # If no value found, return the whole response
        return response if response else None
    
    try:
        print("\nQuerying XBee configuration...")
        print("-" * 50)
        
        # Try without command mode first (for API mode or transparent mode)
        # Query Baud Rate (BD)
        print("Baud Rate (BD): ", end="", flush=True)
        bd_response = send_at_command(serial_instance, "ATBD", use_command_mode=False)
        bd_value = parse_at_response(bd_response, "ATBD")
        if bd_value:
            print(bd_value)
        else:
            # Try with command mode
            print("(trying command mode...) ", end="", flush=True)
            bd_response = send_at_command(serial_instance, "ATBD", use_command_mode=True)
            bd_value = parse_at_response(bd_response, "ATBD")
            if bd_value:
                print(bd_value)
            else:
                print("No response")
        
        # Query MY Address
        print("자신의 주소 (MY): ", end="", flush=True)
        my_response = send_at_command(serial_instance, "ATMY", use_command_mode=False)
        my_value = parse_at_response(my_response, "ATMY")
        if my_value:
            print(my_value)
        else:
            print("(trying command mode...) ", end="", flush=True)
            my_response = send_at_command(serial_instance, "ATMY", use_command_mode=True)
            my_value = parse_at_response(my_response, "ATMY")
            if my_value:
                print(my_value)
            else:
                print("No response")
        
        # Query PAN ID
        print("PAN ID (ID): ", end="", flush=True)
        id_response = send_at_command(serial_instance, "ATID", use_command_mode=False)
        id_value = parse_at_response(id_response, "ATID")
        if id_value:
            print(id_value)
        else:
            print("(trying command mode...) ", end="", flush=True)
            id_response = send_at_command(serial_instance, "ATID", use_command_mode=True)
            id_value = parse_at_response(id_response, "ATID")
            if id_value:
                print(id_value)
            else:
                print("No response")
        
        # Query Destination Address Low
        print("Destination Address Low (DL): ", end="", flush=True)
        dl_response = send_at_command(serial_instance, "ATDL", use_command_mode=False)
        dl_value = parse_at_response(dl_response, "ATDL")
        if dl_value:
            print(dl_value)
        else:
            print("(trying command mode...) ", end="", flush=True)
            dl_response = send_at_command(serial_instance, "ATDL", use_command_mode=True)
            dl_value = parse_at_response(dl_response, "ATDL")
            if dl_value:
                print(dl_value)
            else:
                print("No response")
        
        # Query Coordinator Enable
        print("Coordinator Enable (CE): ", end="", flush=True)
        ce_response = send_at_command(serial_instance, "ATCE", use_command_mode=False)
        ce_value = parse_at_response(ce_response, "ATCE")
        if ce_value:
            print(ce_value)
        else:
            print("(trying command mode...) ", end="", flush=True)
            ce_response = send_at_command(serial_instance, "ATCE", use_command_mode=True)
            ce_value = parse_at_response(ce_response, "ATCE")
            if ce_value:
                print(ce_value)
            else:
                print("No response")
        
        print("-" * 50)
        print("\nEntering continuous read mode. Press Ctrl+C to exit.")
        print("Waiting for incoming data...\n")
        
        while DEBUG_RUNSTATUS:
            rcv_data = receive_serial_data(serial_instance)
            if rcv_data:
                print(f"Received: {rcv_data}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nKeyboard Interrupt!")
        
    finally:
        DEBUG_RUNSTATUS = False
        terminate_serial(serial_instance)