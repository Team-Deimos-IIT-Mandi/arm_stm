import serial
import struct
import time
import sys

COM_PORT = 'COM10'
BAUD_RATE = 115200
PACKET_SIZE = 27  # 2s + 6i + c = 2 + 24 + 1 = 27

def pack_debug_command():
    # DT mode: firmware responds with raw ADC + raw encoder values
    # (no HOME_OFFSET or direction correction applied)
    return struct.pack('<2s 6i c', b'DT', 0, 0, 0, 0, 0, 0, b'\n')

def find_and_parse_packet(ser):
    """Read bytes and scan for a valid FB packet using header sync."""
    # Read all available bytes
    if ser.in_waiting < PACKET_SIZE:
        return None
    
    data = ser.read(ser.in_waiting)
    
    # Scan for 'FB' header and try to parse a full packet from there
    for i in range(len(data) - PACKET_SIZE + 1):
        if data[i:i+2] == b'FB':
            candidate = data[i:i+PACKET_SIZE]
            # Verify footer is newline
            if candidate[-1:] == b'\n':
                unpacked = struct.unpack('<2s 6i c', candidate)
                return list(unpacked[1:7])
    return None

def main():
    print("=== RAW SENSOR FEEDBACK TOOL ===")
    print("Reads absolute raw encoder/ADC values (no HOME_OFFSET correction)")
    print("Use this to find HOME_OFFSETS for RobotConfig.h\n")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {COM_PORT}!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Flush any stale data
    ser.reset_input_buffer()
    
    # Warmup: send several DT packets so firmware sets comms_ok = true
    # and starts reading encoders
    print("Warming up comms...", end="", flush=True)
    for i in range(20):
        ser.write(pack_debug_command())
        time.sleep(0.05)
    print(" done!\n")

    try:
        while True:
            ser.write(pack_debug_command())
            time.sleep(0.05)
            
            fb = find_and_parse_packet(ser)
            if fb is not None:
                sys.stdout.write(
                    f"\rRAW_ADC[M1]: {fb[0]:04d} | RAW_ADC[M2]: {fb[1]:04d} || "
                    f"RAW_ENC[M3]: {fb[2]:04d} | RAW_ENC[Pitch]: {fb[3]:04d} | "
                    f"RAW_ENC[Roll]: {fb[4]:04d} | RAW_ENC[Z]: {fb[5]:04d}   "
                )
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nExiting...")
        if ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
