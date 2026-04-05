import serial
import struct
import time
import csv

COM_PORT = 'COM10'
BAUD_RATE = 115200

# Must match RobotConfig.h
WRIST_MOTOR_A_SIGN = 1
WRIST_MOTOR_B_SIGN = 1

def pack_pwm_command(pwms):
    return struct.pack('<2s 6i c', b'PT', int(pwms[0]), int(pwms[1]), int(pwms[2]), int(pwms[3]), int(pwms[4]), int(pwms[5]), b'\n')

def main():
    print("=== MATLAB VIRTUAL JOINT ID LOGGER ===")
    print("This script captures a raw Open-Loop Step Response for PID autotuning.")
    
    try:
        actuator = int(input("Enter Virtual Joint to Profile (0=M1, 1=M2, 2=M3, 3=Virtual_Pitch, 4=Virtual_Roll, 5=Z): "))
        pwm_val = int(input("Enter Step PWM Magnitude (e.g., 100): "))
    except ValueError:
        print("Invalid input.")
        return
        
    print(f"\nConnecting to {COM_PORT}...")
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Data array: (Time_sec, PWM_Input, Encoder_Output)
    data_log = []
    
    idle_pwms = [0]*6
    step_pwms = [0]*6
    
    # Differential Kinematics Mixer for Virtual Joints
    if actuator == 3: # Pure Pitch
        step_pwms[3] =  pwm_val * WRIST_MOTOR_A_SIGN
        step_pwms[4] = -pwm_val * WRIST_MOTOR_B_SIGN
        print(f"Virtual Pitch Mixer Engaged: M_A={step_pwms[3]}, M_B={step_pwms[4]}")
    elif actuator == 4: # Pure Roll
        step_pwms[3] = pwm_val * WRIST_MOTOR_A_SIGN
        step_pwms[4] = pwm_val * WRIST_MOTOR_B_SIGN
        print(f"Virtual Roll Mixer Engaged: M_A={step_pwms[3]}, M_B={step_pwms[4]}")
    else:
        step_pwms[actuator] = pwm_val

    print("Standby. Board syncing...")
    # Seed idle
    for _ in range(5): 
        ser.write(pack_pwm_command(idle_pwms))
        time.sleep(0.05)
    ser.reset_input_buffer()
    
    start_time = time.time()
    phase = 0 # 0=Baseline, 1=Step, 2=Rundown
    baseline_sensor = 0
    baseline_set = False
    
    print("\n--- [0.0s] Logging Baseline ---")
    
    while True:
        t = time.time() - start_time
        
        if t < 0.5:
            if phase != 0: phase = 0 
            cmd = pack_pwm_command(idle_pwms)
            current_pwm = 0
        elif t < 1.0:
            if phase == 0:
                print(f"--- [0.5s] FIRING STEP RESPONSE (PWM {pwm_val}) ---")
                phase = 1
            cmd = pack_pwm_command(step_pwms)
            current_pwm = pwm_val
        elif t < 2.0:
            if phase == 1:
                print("--- [1.0s] COASTING RUNDOWN ---")
                phase = 2
            cmd = pack_pwm_command(idle_pwms)
            current_pwm = 0
        else:
            break
            
        ser.write(cmd)
        
        # Poll aggressively (100Hz) to catch 50Hz FB 
        while ser.in_waiting >= 27:
            raw_data = ser.read_until(b'\n')
            if len(raw_data) == 27 and raw_data.startswith(b'FB'):
                unpacked = struct.unpack('<2s 6i c', raw_data)
                fb = list(unpacked[1:7])
                
                # Log actual sensor feedback corresponding to actuator
                enc_out = fb[actuator]
                
                if not baseline_set:
                    baseline_sensor = int(enc_out)
                    baseline_set = True
                    
                # Convert Absolute Position to Relative Deviation (for MATLAB)
                # Handle 4096-rollover for encoders (indices 2, 3, 4, 5)
                if actuator > 1:
                    rel_out = ((enc_out - baseline_sensor + 2048) % 4096) - 2048
                else: # M1, M2 are standard ADCs with no wraparound physics
                    rel_out = enc_out - baseline_sensor
                    
                data_log.append((t, current_pwm, rel_out))
                
        time.sleep(0.01) 
        
    ser.close()
    print("--- [2.0s] FINISHED. ---")
    
    # Save to CSV
    filename = f"step_response_actuator_{actuator}.csv"
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Time (s)", "PWM Input", "Sensor Output"])
        for row in data_log:
            writer.writerow([f"{row[0]:.4f}", row[1], row[2]])
            
    print(f"\nSUCCESS: Saved {len(data_log)} data points to '{filename}'!")
    print("You can now import this CSV into MATLAB's System Identification Toolbox!")

if __name__ == '__main__':
    main()
