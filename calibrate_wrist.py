import serial
import struct
import time
import sys
import threading

COM_PORT = 'COM10'
BAUD_RATE = 115200

def pack_pwm_command(m1, m2, m3, pit_motorA, roll_motorB, z):
    return struct.pack('<2s 6i c', b'PT', 
                       int(m1), int(m2), int(m3), 
                       int(pit_motorA), int(roll_motorB), int(z), 
                       b'\n')

def pack_debug_command():
    return struct.pack('<2s 6i c', b'DT', 0, 0, 0, 0, 0, 0, b'\n')

feedback = [0]*6
running = True
speed_A = 0
speed_B = 0

def serial_thread():
    global feedback, running, speed_A, speed_B
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {COM_PORT}")
    except Exception as e:
        print("Failed to connect", e)
        return
        
    while running:
        # Exclusively push 'PT' Open-Loop PWM to physically lockout the PIDs.
        # Idling at 0 holds the arm dead still.
        cmd = pack_pwm_command(0, 0, 0, speed_A, speed_B, 0)
            
        try:
            ser.write(cmd)
            while ser.in_waiting >= 27:
                raw_data = ser.read_until(b'\n')
                if len(raw_data) == 27 and raw_data.startswith(b'FB'):
                    feedback = list(struct.unpack('<2s 6i c', raw_data)[1:7])
        except:
            pass
        time.sleep(0.05)
    ser.close()

def main():
    print("=== DANGEROUS WRIST CALIBRATION ROUTINE ===")
    print("This script bypasses safety PIDs and pushes direct PWM.")
    print("Keep hands clear.\n")
    
    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()
    
    import time
    time.sleep(1) # wait for sync
    
    print("\n--- Testing Motor A (Joint 1/Pitch) ---")
    start_pitch_A = feedback[3]
    start_roll_A = feedback[4]
    
    print("Firing Motor A Positively (Speed 150) for 1 second...")
    global speed_A, speed_B
    speed_A = 150
    time.sleep(1)
    speed_A = 0
    time.sleep(0.5)
    
    mid_pitch_A = feedback[3]
    mid_roll_A = feedback[4]
    
    print("Firing Motor A Negatively (Speed -150) for 1 second...")
    speed_A = -150
    time.sleep(1)
    speed_A = 0
    time.sleep(0.5)
    
    end_pitch_A = feedback[3]
    end_roll_A = feedback[4]
    
    # Calculate pos delta
    delta_pitch_Apos = ((mid_pitch_A - start_pitch_A + 2048) % 4096) - 2048
    delta_roll_Apos = ((mid_roll_A - start_roll_A + 2048) % 4096) - 2048
    
    # Calculate neg delta
    delta_pitch_Aneg = ((end_pitch_A - mid_pitch_A + 2048) % 4096) - 2048
    delta_roll_Aneg = ((end_roll_A - mid_roll_A + 2048) % 4096) - 2048
    
    print(f"  Delta A+ -> Pitch: {delta_pitch_Apos} | Roll: {delta_roll_Apos}")
    print(f"  Delta A- -> Pitch: {delta_pitch_Aneg} | Roll: {delta_roll_Aneg}")
    
    # Check consistency
    if (delta_pitch_Apos * delta_pitch_Aneg > 0) or (delta_roll_Apos * delta_roll_Aneg > 0):
        print("WARNING: Motor A did not return backwards nicely. Check gears.")
        
    delta_pitch_A = delta_pitch_Apos
    delta_roll_A = delta_roll_Apos

    
    input("\nPress Enter to test Motor B (Joint 2/Roll)...")
    
    print("\n--- Testing Motor B (Joint 2/Roll) ---")
    start_pitch_B = feedback[3]
    start_roll_B = feedback[4]
    
    print("Firing Motor B Positively (Speed 150) for 1 second...")
    speed_B = 150
    time.sleep(1)
    speed_B = 0
    time.sleep(0.5)
    
    mid_pitch_B = feedback[3]
    mid_roll_B = feedback[4]
    
    print("Firing Motor B Negatively (Speed -150) for 1 second...")
    speed_B = -150
    time.sleep(1)
    speed_B = 0
    time.sleep(0.5)
    
    end_pitch_B = feedback[3]
    end_roll_B = feedback[4]
    
    # Calculate pos delta
    delta_pitch_Bpos = ((mid_pitch_B - start_pitch_B + 2048) % 4096) - 2048
    delta_roll_Bpos = ((mid_roll_B - start_roll_B + 2048) % 4096) - 2048
    
    # Calculate neg delta
    delta_pitch_Bneg = ((end_pitch_B - mid_pitch_B + 2048) % 4096) - 2048
    delta_roll_Bneg = ((end_roll_B - mid_roll_B + 2048) % 4096) - 2048
    
    print(f"  Delta B+ -> Pitch: {delta_pitch_Bpos} | Roll: {delta_roll_Bpos}")
    print(f"  Delta B- -> Pitch: {delta_pitch_Bneg} | Roll: {delta_roll_Bneg}")
    
    if (delta_pitch_Bpos * delta_pitch_Bneg > 0) or (delta_roll_Bpos * delta_roll_Bneg > 0):
        print("WARNING: Motor B did not return backwards nicely. Check gears.")
        
    delta_pitch_B = delta_pitch_Bpos
    delta_roll_B = delta_roll_Bpos
    
    
    print("\n================ KINEMATICS ANALYSIS ================")
    if abs(delta_pitch_A) < 10 and abs(delta_pitch_B) < 10:
        print("ERROR: Extremely low movement delta. Try increasing test Speed or check 12V power!")
    else:
        print("Based on your data, here are your correct RobotConfig variables:\n")
        
        # Base kinematics: mA = pitch + roll, mB = -pitch + roll
        # If mA is positive, pitch AND roll should go UP.
        # If mB is positive, pitch goes DOWN, roll goes UP.
        
        ma_pitch_goes_up = (delta_pitch_A > 0)
        ma_roll_goes_up = (delta_roll_A > 0)
        mb_pitch_goes_up = (delta_pitch_B > 0)
        mb_roll_goes_up = (delta_roll_B > 0)
        # Determine Motor Hardware Polarity (Does +PWM spin it forward or backward relative to gearbox?)
        motor_A_sign = 1 if (ma_pitch_goes_up == ma_roll_goes_up) else -1
        motor_B_sign = 1 if (mb_pitch_goes_up != mb_roll_goes_up) else -1
        
        # Output Constants
        print(f"constexpr int WRIST_MOTOR_A_SIGN = {motor_A_sign};")
        print(f"constexpr int WRIST_MOTOR_B_SIGN = {motor_B_sign};")
        
        # Determine Encoder Polarity
        adj_pitch_A = delta_pitch_A * motor_A_sign
        pitch_pid_sign = 1 if adj_pitch_A > 0 else -1
        
        adj_roll_A = delta_roll_A * motor_A_sign
        roll_pid_sign = 1 if adj_roll_A > 0 else -1
        
        print(f"constexpr int PITCH_PID_SIGN = {pitch_pid_sign};")
        print(f"constexpr int ROLL_PID_SIGN = {roll_pid_sign};")
        
    print("\nCopy these 4 lines into your RobotConfig.h!")
    global running
    running = False

if __name__ == '__main__':
    main()
