# Fixes Needed for CANSAT Flight Software

## Summary of Issues from Logs

1. **Motor App Initialization Error** - Motor app fails during initialization
2. **Barometer App MID 100 Error** - Barometer app doesn't handle termination message
3. **Communication App NoneType Error** - Communication app has NoneType integer error during termination

---

## Fix 1: Motor App - Better Error Handling in Initialization

**Location:** `Sensor_Motor/motorapp.py` in `motorapp_init()` function

**Current Pattern (from hkapp.py):**
```python
def motorapp_init():
    global MOTORAPP_RUNSTATUS
    try:
        # Disable Keyboardinterrupt since Termination is handled by parent process
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "Initializating motorapp")
        ## User Defined Initialization goes HERE
        
        # TODO: Add specific initialization code here
        # If hardware is not connected, handle gracefully
        
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "motorapp Initialization Complete")
    
    except Exception as e:
        # IMPROVED: Log the specific error instead of generic message
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.error, f"Error during initialization: {e}")
        # Optionally, set a flag to indicate hardware is not available
        # MOTORAPP_HARDWARE_AVAILABLE = False
        raise  # Re-raise if you want the app to terminate, or remove this to continue
```

**Key Changes:**
- Change `except:` to `except Exception as e:` to capture the error
- Log the specific error message: `f"Error during initialization: {e}"`
- Consider if the motor app should continue running even if hardware isn't available

---

## Fix 2: Barometer App - Handle Termination Message

**Location:** `Sensor_Barometer/barometerapp.py` in `command_handler()` function

**Add to command_handler function:**
```python
def command_handler(recv_msg : msgstructure.MsgStructure):
    global BAROMETERAPP_RUNSTATUS
    
    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        # Handle termination message (MID 100)
        events.LogEvent(appargs.BarometerAppArg.AppName, events.EventType.info, "BAROMETERAPP TERMINATION DETECTED")
        BAROMETERAPP_RUNSTATUS = False
        return
    
    # Handle other message IDs here
    # elif recv_msg.MsgID == appargs.BarometerAppArg.MID_SomeOtherMessage:
    #     ...
    # else:
    #     events.LogEvent(appargs.BarometerAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
```

**Key Changes:**
- Add check for `appargs.MainAppArg.MID_TerminateProcess` (which is 100)
- Set `BAROMETERAPP_RUNSTATUS = False` when termination is received
- This will prevent the "MID 100 not handled" error

---

## Fix 3: Communication App - Fix NoneType Integer Error

**Location:** `comm/commapp.py` in command reader thread/function

**The error occurs when:**
```
Error receiving command, Sleeping 1 second : 'NoneType' object cannot be interpreted as an integer
```

**Likely problematic code pattern:**
```python
def cmd_reader_thread():
    global COMMAPP_RUNSTATUS
    
    while COMMAPP_RUNSTATUS:
        try:
            # This line likely has the issue
            cmd_time = some_function_that_returns_none()
            time.sleep(cmd_time)  # cmd_time is None, causing the error
            
        except Exception as e:
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, 
                          f"Error receiving command, Sleeping 1 second : {e}")
            time.sleep(1)  # Good fallback
```

**Fixed version:**
```python
def cmd_reader_thread():
    global COMMAPP_RUNSTATUS
    
    while COMMAPP_RUNSTATUS:
        try:
            # Get command timeout or sleep duration
            cmd_time = some_function_that_returns_none()
            
            # FIX: Check if cmd_time is None or not a valid number
            if cmd_time is None:
                cmd_time = 1.0  # Default sleep time
            else:
                try:
                    cmd_time = float(cmd_time)  # Convert to float to handle both int and float
                except (ValueError, TypeError):
                    cmd_time = 1.0  # Default if conversion fails
            
            time.sleep(cmd_time)
            
        except Exception as e:
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, 
                          f"Error receiving command, Sleeping 1 second : {e}")
            time.sleep(1)  # Always use 1 second as fallback
```

**Key Changes:**
- Always check if the value is `None` before using it
- Provide a default value (typically 1.0 second)
- Convert to float to handle both int and string inputs
- Add try-except around conversion in case it's not a number

**Alternative fix if using timeout parameter:**
```python
# If the issue is with socket/timeout parameters:
timeout_val = get_timeout_value()  # This might return None
if timeout_val is None:
    timeout_val = 1.0  # Default timeout
# Then use timeout_val instead of None
```

---

## Testing

After applying fixes:

1. **Motor App**: Check if initialization error provides more detailed information
2. **Barometer App**: Test graceful shutdown - should not show "MID 100 not handled"
3. **Communication App**: Test termination - should not show NoneType error

---

## Additional Recommendations

1. **Motor App**: Consider making motor hardware optional - allow app to continue if hardware isn't connected
2. **All Apps**: Ensure all command handlers check for `MID_TerminateProcess` for graceful shutdown
3. **Communication App**: Review all places where timeout/sleep values are used and ensure None checks

