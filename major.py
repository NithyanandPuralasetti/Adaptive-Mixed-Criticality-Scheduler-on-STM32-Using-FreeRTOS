import serial

# ═══════════════════════════════════════════════
#  CONFIG — edit these lines
# ═══════════════════════════════════════════════
SERIAL_PORT   = "COM6"            # Ensure this is your STM32 COM port
BAUD_RATE     = 115200
# ═══════════════════════════════════════════════

def main():
    print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud...")

    try:
        # Open the serial port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Connected. Reading RTOS data from STM32...")
        print("-" * 50)
        
        while True:
            try:
                # Read a line from the serial port
                raw = ser.readline()
                if not raw: 
                    continue
                
                # Decode and print directly to the terminal
                line = raw.decode("utf-8", errors="replace").strip()
                
                # Only print if the line isn't completely empty
                if line:
                    print(line)

            except UnicodeDecodeError:
                # Ignore garbage bytes that sometimes happen on startup
                pass
                
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nMonitor stopped by user.")

if __name__ == "__main__":
    main()