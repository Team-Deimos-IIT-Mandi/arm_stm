import serial
import time
try:
    print("Opening COM10...")
    ser = serial.Serial('COM10', 115200, timeout=0.1)
    print("Opened!")
    
    # Enable DTR/RTS to signal terminal readiness to some USB drivers
    ser.dtr = True
    ser.rts = True
    time.sleep(0.5)
    
    print(f"Initial bytes waiting: {ser.in_waiting}")
    # Send a dummy packet
    cmd = b"ST" + b"\x00"*24 + b"\n"
    print(f"Sending {len(cmd)} bytes...")
    ser.write(cmd)
    
    for i in range(10):
        time.sleep(0.2)
        print(f"[{i}] Bytes waiting: {ser.in_waiting}")
        if ser.in_waiting > 0:
            print("DATA:", ser.read(ser.in_waiting))
    ser.close()
    print("Closed.")
except Exception as e:
    print(f"Error: {e}")
