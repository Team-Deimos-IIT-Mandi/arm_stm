import serial
import struct
import time
import sys

COM_PORT = 'COM10'
BAUD_RATE = 115200
PACKET_SIZE = 27  # 2s + 6i + c = 2 + 24 + 1 = 27

def pack_limp_command():
    # PT mode (Open Loop): Firmware applies direct PWM values.
    # By sending all 0's, we force all motors to 0 PWM (freewheeling/limp).
    # The firmware still reads sensors and responds with RAW values!
    return struct.pack('<2s 6i c', b'PT', 0, 0, 0, 0, 0, 0, b'\n')

def find_and_parse_packet(ser):
    """Read bytes and scan for a valid FB packet using header sync."""
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
    print("=" * 60)
    print("  LIMP SENSOR READER — Move the arm by hand freely!")
    print("=" * 60)
    print("This script continuously pushes PWM=0 commands to force the PID")
    print("loops off. The arm will give zero resistance, while still")
    print("transmitting true RAW encoder and ADC values to the terminal.\n")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {COM_PORT}!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Flush any stale data
    ser.reset_input_buffer()
    
    # Warmup string
    print("Warming up comms...", end="", flush=True)
    for i in range(10):
        ser.write(pack_limp_command())
        time.sleep(0.05)
    print(" done!\n")

    try:
        while True:
            # Continuously overwrite the open-loop command array with 0s
            # to guarantee the motors never wake up and fight you.
            ser.write(pack_limp_command())
            time.sleep(0.05)
            
            fb = find_and_parse_packet(ser)
            if fb is not None:
                # FB[0]=M1_ADC, FB[1]=M2_ADC
                # FB[2]=M3_ENC, FB[3]=Pitch_ENC, FB[4]=Roll_ENC, FB[5]=Z_ENC
                sys.stdout.write(
                    f"\rADC [M1:{fb[0]:04d} M2:{fb[1]:04d}]  ||  "
                    f"I2C [M3:{fb[2]:04d} Pit:{fb[3]:04d} Rol:{fb[4]:04d} Z:{fb[5]:04d}]    "
                )
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n\nExiting... Watchdog will safely stop the arm in ~2.0s.")
        if ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
