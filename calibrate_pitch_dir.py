"""
calibrate_pitch_dir.py
======================
Fires a brief open-loop PWM pulse to the pitch motor pair,
measures the corrected encoder response, and tells you exactly
what ENCODER_DIRECTION[1] and PITCH_PID_SIGN to use.

USAGE:
  1. Make sure no other script is running.
  2. Keep hands clear of the wrist joint.
  3. Run: python calibrate_pitch_dir.py
"""

import serial
import struct
import time

COM_PORT  = 'COM10'
BAUD_RATE = 115200

TEST_PWM  = 80    # Open-loop speed for test pulse (increase if no movement)
PULSE_SEC = 0.5   # Duration of each test pulse

# Must match RobotConfig.h
WRIST_MOTOR_A_SIGN = 1
WRIST_MOTOR_B_SIGN = 1

def pack_idle():
    return struct.pack('<2s 6i c', b'PT', 0, 0, 0, 0, 0, 0, b'\n')

def pack_pitch_pulse(speed):
    """Drive M_A and M_B in opposite directions for a pure pitch motion."""
    a = int(speed * WRIST_MOTOR_A_SIGN)
    b = int(-speed * WRIST_MOTOR_B_SIGN)
    # PT packet: [M1_pwm, M2_pwm, M3_pwm, WristA_pwm, WristB_pwm, Z_pwm]
    return struct.pack('<2s 6i c', b'PT', 0, 0, 0, a, b, 0, b'\n')

def read_pitch_corrected(ser):
    """Returns corrected pitch encoder value from the latest FB packet."""
    ser.reset_input_buffer()
    ser.write(pack_idle())
    time.sleep(0.15)
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if ser.in_waiting >= 27:
            chunk = ser.read_until(b'\n')
            if len(chunk) == 27 and chunk.startswith(b'FB'):
                vals = struct.unpack('<2s 6i c', chunk)
                # FB[4] = Pitch corrected steps (index [3] in motor_cmd = slot 3 = Pitch)
                return vals[1+3]  # slot 0=M1_deg, 1=M2_deg, 2=M3_enc, 3=Pitch, 4=Roll, 5=Z
        ser.write(pack_idle())
        time.sleep(0.05)
    return None

def main():
    print("=" * 55)
    print("  PITCH DIRECTION CALIBRATION")
    print("=" * 55)
    print(f"  Test PWM = {TEST_PWM}  |  Duration = {PULSE_SEC}s")
    print()

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.5)
        print(f"Connected to {COM_PORT}!")
    except Exception as e:
        print(f"Connection failed: {e}")
        return

    # Warm-up: send idle open-loop packets to establish comms
    print("Warming up comms (1s)...")
    for _ in range(20):
        ser.write(pack_idle())
        time.sleep(0.05)

    # --- Read BEFORE ---
    before = read_pitch_corrected(ser)
    if before is None:
        print("ERROR: No FB packet received before pulse. Is firmware running?")
        ser.close()
        return
    print(f"Pitch corrected (before pulse) = {before}")

    # --- FIRE positive pitch pulse ---
    print(f"\nFiring POSITIVE pitch pulse (A={TEST_PWM*WRIST_MOTOR_A_SIGN}, B={-TEST_PWM*WRIST_MOTOR_B_SIGN}) for {PULSE_SEC}s...")
    print("Keep hands CLEAR!")
    start = time.time()
    while time.time() - start < PULSE_SEC:
        ser.write(pack_pitch_pulse(TEST_PWM))
        time.sleep(0.05)

    # Stop motors
    for _ in range(5):
        ser.write(pack_idle())
        time.sleep(0.05)

    # --- Read AFTER ---
    after = read_pitch_corrected(ser)
    if after is None:
        print("ERROR: No FB packet received after pulse.")
        ser.close()
        return
    print(f"Pitch corrected (after pulse)  = {after}")

    delta = after - before
    print(f"\nDelta = {delta}")

    print()
    print("=" * 55)
    print("  ANALYSIS")
    print("=" * 55)

    if abs(delta) < 5:
        print("WARNING: Delta is very small (< 5 steps).")
        print("  The motor may not be moving. Try increasing TEST_PWM.")
        ser.close()
        return

    if delta > 0:
        print("Corrected value INCREASED with positive pulse.")
        print("  => Encoder is counting CORRECTLY (direction OK).")
        print("  => ENCODER_DIRECTION[1] = +1")
        print()
        print("Now check: did the wrist pitch UP or DOWN physically?")
        direction = input("Did the wrist tip move UP (toward max extension)? [y/n]: ").strip().lower()
        if direction == 'y':
            print("\nPerfect! Positive GUI slider = upward = increasing corrected value.")
            print("  => PITCH_PID_SIGN = +1")
        else:
            print("\nMovement was downward but corrected increased. Invert PID sign.")
            print("  => PITCH_PID_SIGN = -1")
    else:
        print("Corrected value DECREASED with positive pulse.")
        print("  => Encoder counting in REVERSE of joint movement.")
        print("  => ENCODER_DIRECTION[1] = -1")
        print()
        direction = input("Did the wrist tip move UP (toward max extension)? [y/n]: ").strip().lower()
        if direction == 'y':
            print("\nEncoder inverted, motor went up. Signs balance correctly.")
            print("  => PITCH_PID_SIGN = +1")
        else:
            print("\nEncoder inverted, motor went down. Need to flip PID sign too.")
            print("  => PITCH_PID_SIGN = -1")

    print()
    print("=" * 55)
    print("  Copy these into RobotConfig.h (lines 84-86 approx):")
    print("=" * 55)
    enc_dir = 1 if delta > 0 else -1
    pid_sign = 1 if direction == 'y' else -1
    print(f"constexpr int ENCODER_DIRECTION[4] = {{1, {enc_dir}, 1, 1}};")
    print(f"constexpr int PITCH_PID_SIGN = {pid_sign};")
    print()
    print("Rebuild and reflash STM32 after changing RobotConfig.h!")

    ser.close()

if __name__ == '__main__':
    main()
