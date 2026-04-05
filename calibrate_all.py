"""
calibrate_all.py
================
Full Interactive Calibration for All Encoder Joints.

This script walks you through each joint, records raw encoder positions
at HOME and at MAX, then auto-generates the exact lines to paste into
RobotConfig.h.

INSTRUCTIONS:
  - Make sure the robot is POWERED and connected via USB.
  - Do NOT run test_usb_arm.py or any other script at the same time.
  - Move joints BY HAND or use the calibrate_wrist.py for open-loop.
"""

import serial
import struct
import time

COM_PORT = 'COM10'
BAUD_RATE = 115200

JOINT_NAMES = ['M3 (Link 3)', 'Pitch', 'Roll', 'Z-Axis']

def pack_debug(vals=(0,0,0,0,0,0)):
    return struct.pack('<2s 6i c', b'DT',
                       int(vals[0]), int(vals[1]), int(vals[2]),
                       int(vals[3]), int(vals[4]), int(vals[5]), b'\n')

def read_raw_encoders(ser):
    """Drain buffer and return the most recent raw encoder array [M3, Pit, Roll, Z]."""
    # Flush stale data
    ser.reset_input_buffer()
    # Send a passive DT packet to keep comms alive and trigger FB response
    ser.write(pack_debug())
    time.sleep(0.15)
    
    enc = None
    adc = None
    deadline = time.time() + 1.5
    while time.time() < deadline:
        if ser.in_waiting >= 27:
            chunk = ser.read_until(b'\n')
            if len(chunk) == 27 and chunk.startswith(b'FB'):
                unpacked = struct.unpack('<2s 6i c', chunk)
                vals = list(unpacked[1:7])
                # vals[0]=M1_deg*1000, vals[1]=M2_deg*1000,
                # vals[2..5] = corrected enc steps (post-HOME_OFFSETS)
                # BUT: we need RAW values. The firmware stores raw in debug_raw_encoders
                # which is transmitted in the DEBUG 'DB' packet (if implemented).
                # Since we only have FB, we approximate: this returns the corrected values.
                enc = vals[2:6]  # corrected (not raw), but useful for comparison
                return enc
        ser.write(pack_debug())
        time.sleep(0.05)
    return None

def read_raw_adc_enc(ser):
    """
    Sends a 'DT' packet and reads back FB packet.
    Returns (adc_m1, adc_m2, enc_m3, enc_pitch, enc_roll, enc_z) all as
    corrected-step values because that's what the firmware sends in FB.
    The user should use read_encoders.py for true raw values.
    """
    ser.reset_input_buffer()
    for _ in range(3):
        ser.write(pack_debug())
        time.sleep(0.08)
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if ser.in_waiting >= 27:
            chunk = ser.read_until(b'\n')
            if len(chunk) == 27 and chunk.startswith(b'FB'):
                unpacked = struct.unpack('<2s 6i c', chunk)
                vals = list(unpacked[1:7])
                return vals
        ser.write(pack_debug())
        time.sleep(0.05)
    return None

def sample_enc(ser, joint_idx, n_samples=5, delay=0.1):
    """Average n_samples consecutive encoder readings for the given joint index (2-5 in FB)."""
    fb_idx = joint_idx + 2  # FB: [M1_deg, M2_deg, M3_enc, Pit, Roll, Z]
    readings = []
    for _ in range(n_samples):
        vals = read_raw_adc_enc(ser)
        if vals is not None:
            readings.append(vals[fb_idx])
        time.sleep(delay)
    if not readings:
        return None
    return int(round(sum(readings) / len(readings)))

def main():
    print("=" * 60)
    print("  FULL ROBOT ARM CALIBRATION")
    print("=" * 60)
    print()
    print("NOTE: This measures CORRECTED encoder values (post-HOME_OFFSET).")
    print("To measure TRUE raw values, use read_encoders.py instead.")
    print("This script tells you the ENCODER_DIRECTION and TRAVEL in steps.")
    print()

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.5)
        print(f"Connected to {COM_PORT}!")
    except Exception as e:
        print(f"Connection failed: {e}")
        return

    # Warm up comms
    for _ in range(10):
        ser.write(pack_debug())
        time.sleep(0.05)
    print("Board synced. Starting calibration...\n")

    results = []

    for joint_idx, name in enumerate(JOINT_NAMES):
        print("=" * 60)
        print(f"  JOINT {joint_idx}: {name}")
        print("=" * 60)

        # ---- HOME ----
        print(f"\nStep 1: Move '{name}' to its HOME (0 deg/rest) position.")
        print("        Use read_encoders.py or move by hand.")
        input("        Press ENTER when the joint is at HOME...")

        home_val = sample_enc(ser, joint_idx)
        if home_val is None:
            print("  ERROR: No feedback received! Skipping joint.")
            results.append(None)
            continue
        print(f"  HOME corrected-steps = {home_val}")

        # ---- MAX ----
        print(f"\nStep 2: Move '{name}' to its MAXIMUM extension position.")
        input("        Press ENTER when the joint is at MAX...")

        max_val = sample_enc(ser, joint_idx)
        if max_val is None:
            print("  ERROR: No feedback received! Skipping joint.")
            results.append(None)
            continue
        print(f"  MAX  corrected-steps = {max_val}")

        # ---- Analyse travel ----
        travel = abs(max_val - home_val)
        direction = "NORMAL (+1)" if max_val > home_val else "INVERTED (-1)"
        dir_val = 1 if max_val > home_val else -1

        # Range hint for GUI slider
        range_deg = round((travel / 4096.0) * 360.0, 1)

        print(f"\n  Travel       = {travel} steps")
        print(f"  Direction    = {direction}")
        print(f"  Approx range = {range_deg} degrees")

        results.append({
            'name': name,
            'home': home_val,
            'max': max_val,
            'travel': travel,
            'dir': dir_val,
            'range_deg': range_deg
        })
        print()

    ser.close()

    # ---- Generate RobotConfig.h snippet ----
    print()
    print("=" * 60)
    print("  RESULTS: Copy these into RobotConfig.h")
    print("=" * 60)
    print()

    dirs = [r['dir'] if r else 1 for r in results]
    print(f"// Encoder directions: +1 = counts UP with movement, -1 = counts DOWN")
    print(f"constexpr int ENCODER_DIRECTION[4] = {{{dirs[0]}, {dirs[1]}, {dirs[2]}, {dirs[3]}}};")
    print()

    travels = [r['travel'] if r else 4096 for r in results]
    pitch_range = results[1]['range_deg'] if results[1] else 161.0
    print(f"// PITCH_LIMIT_MAX: measured travel in encoder steps")
    print(f"constexpr int PITCH_LIMIT_MAX = {travels[1]};  // ~{results[1]['range_deg']}° of travel")
    print()

    print(f"constexpr float PITCH_RANGE_DEG = ({travels[1]}.0f / 4096.0f) * 360.0f;  // = {pitch_range}°")
    print()

    print("// NOTE: HOME_OFFSETS must be set using TRUE RAW encoder values.")
    print("//       Run read_encoders.py with the joint at physical home position.")
    print("//       The values above are CORRECTED steps, not raw.")
    print()
    print("Done! Rebuild and reflash after updating RobotConfig.h.")

if __name__ == '__main__':
    main()
