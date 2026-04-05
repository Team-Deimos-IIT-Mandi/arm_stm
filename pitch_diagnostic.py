"""
pitch_diagnostic.py
====================
DEFINITIVE pitch diagnostic. Reads raw encoder in real-time while you move
the pitch joint by hand. Shows EXACTLY what the firmware sees and why it breaks.

Run with:  python pitch_diagnostic.py
(Use the .venv python if normal python doesn't have pyserial)
"""

import serial
import struct
import time
import sys

COM_PORT  = 'COM10'
BAUD_RATE = 115200

def pack_debug():
    return struct.pack('<2s 6i c', b'DT', 0, 0, 0, 0, 0, 0, b'\n')

def read_raw(ser):
    """Send DT, read FB, return raw encoder values dict or None."""
    ser.reset_input_buffer()
    ser.write(pack_debug())
    time.sleep(0.08)
    deadline = time.time() + 1.0
    while time.time() < deadline:
        if ser.in_waiting >= 27:
            chunk = ser.read_until(b'\n')
            if len(chunk) == 27 and chunk.startswith(b'FB'):
                vals = struct.unpack('<2s 6i c', chunk)
                # In DT mode, FB sends raw values
                return {
                    'adc_m1': vals[1], 'adc_m2': vals[2],
                    'enc_m3': vals[3], 'enc_pitch': vals[4],
                    'enc_roll': vals[5], 'enc_z': vals[6]
                }
        ser.write(pack_debug())
        time.sleep(0.05)
    return None

def main():
    print("=" * 60)
    print("  PITCH DIAGNOSTIC — Move pitch by hand, watch the numbers")
    print("=" * 60)

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.5)
        print(f"Connected to {COM_PORT}!")
    except Exception as e:
        print(f"Connection failed: {e}")
        return

    # Warm up
    for _ in range(10):
        ser.write(pack_debug())
        time.sleep(0.05)

    HOME = 3300  # Current HOME_OFFSETS[1]
    
    raw_min = 99999
    raw_max = -1
    raw_home = None
    
    print(f"\nCurrent HOME_OFFSETS[1] = {HOME}")
    print()
    print("Move pitch joint to FULL range. Press Ctrl+C when done.\n")
    print(f"{'RAW':>6} | {'CORRECTED':>9} | {'WRAP?':>5} | {'RAW_MIN':>7} {'RAW_MAX':>7} | Notes")
    print("-" * 75)
    
    try:
        while True:
            data = read_raw(ser)
            if data is None:
                print("No data...")
                continue
            
            raw = data['enc_pitch']
            if raw == -1:
                print("Encoder read failed (-1)")
                continue
            
            # Track extremes
            if raw < raw_min:
                raw_min = raw
            if raw > raw_max:
                raw_max = raw
            
            # Compute corrected (same as firmware does)
            corrected = raw - HOME
            while corrected < 0:
                corrected += 4096
            while corrected >= 4096:
                corrected -= 4096
            
            # Detect wrap proximity
            wrap_note = ""
            if corrected > 4000:
                wrap_note = "!! NEAR WRAP (corrected close to 4096→0)"
            elif corrected < 96:
                wrap_note = "~~ near home (corrected ≈ 0)"
            
            sys.stdout.write(f"\r{raw:>6} | {corrected:>9} | {'YES' if corrected > 3500 else 'no':>5} | {raw_min:>7} {raw_max:>7} | {wrap_note:<40}")
            sys.stdout.flush()
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        pass
    
    print("\n\n")
    print("=" * 60)
    print("  RESULTS")
    print("=" * 60)
    print(f"  Raw MIN seen:  {raw_min}")
    print(f"  Raw MAX seen:  {raw_max}")
    print(f"  Total travel:  {raw_max - raw_min} steps")
    print(f"  Current HOME:  {HOME}")
    print()
    
    # Calculate the dead zone
    dead_zone_size = 4096 - raw_max + raw_min
    dead_zone_center = (raw_max + dead_zone_size // 2) % 4096
    
    print(f"  Dead zone: raw {raw_max}→{raw_min} through 0 (size={dead_zone_size})")
    print(f"  Dead zone center: {dead_zone_center}")
    print()
    
    # RECOMMENDED HOME: place it in the dead zone center
    new_home = dead_zone_center
    
    # Compute corrected range with new home
    corr_min = raw_min - new_home
    while corr_min < 0: corr_min += 4096
    corr_max = raw_max - new_home
    while corr_max < 0: corr_max += 4096
    
    # User's physical home at raw 3300
    corr_user_home = 3300 - new_home
    while corr_user_home < 0: corr_user_home += 4096
    
    travel = corr_max - corr_min
    range_deg = (travel / 4096.0) * 360.0
    
    # Degrees for GUI: where user's home sits relative to corrected range
    home_fraction = (corr_user_home - corr_min) / float(travel)
    deg_below_home = home_fraction * range_deg
    deg_above_home = (1.0 - home_fraction) * range_deg
    
    print("  With NEW HOME = {} (in dead zone):".format(new_home))
    print(f"    corrected MIN  = {corr_min}  (raw {raw_min})")
    print(f"    corrected MAX  = {corr_max}  (raw {raw_max})")
    print(f"    corrected HOME = {corr_user_home}  (raw 3300 = your 0°)")
    print(f"    Travel         = {travel} steps = {range_deg:.1f}°")
    print(f"    GUI range      = -{deg_below_home:.1f}° to +{deg_above_home:.1f}°")
    print()
    print("  NO WRAPPING — all corrected values are continuous!")
    print()
    
    print("=" * 60)
    print("  PASTE INTO RobotConfig.h:")
    print("=" * 60)
    print()
    print(f"constexpr int HOME_OFFSETS[4] = {{3306, {new_home}, 823, 3813}};")
    print()
    print(f"constexpr int POS_MAX[4] = {{4096, {corr_max}, 4096, 4096}};")
    print(f"constexpr int PITCH_LIMIT_MIN = {corr_min};")
    print(f"constexpr int PITCH_LIMIT_MAX = {corr_max};")
    print()
    print(f"constexpr float PITCH_RANGE_DEG = ({travel}.0f / 4096.0f) * 360.0f;  // = {range_deg:.1f}°")
    print()
    print(f"// Pitch home (0° on slider) = corrected {corr_user_home} steps")
    print(f"constexpr int PITCH_HOME_STEPS = {corr_user_home};")
    print()
    print("  ALSO UPDATE pitchDegToSteps() in RobotCore.cpp:")
    print(f"    float pitchDegToSteps(float deg) {{")
    print(f"        float steps = (float)PITCH_HOME_STEPS + (deg / PITCH_RANGE_DEG) * (float)(PITCH_LIMIT_MAX - PITCH_LIMIT_MIN);")
    print(f"        if (steps < PITCH_LIMIT_MIN) steps = PITCH_LIMIT_MIN;")
    print(f"        if (steps > PITCH_LIMIT_MAX) steps = PITCH_LIMIT_MAX;")
    print(f"        return steps;")
    print(f"    }}")
    print()
    print("  AND UPDATE pitchStepsToDeg() feedback in the FB builder:")
    print(f"    float pitch_deg = ((current_pos[1] - (float)PITCH_HOME_STEPS) / (float)(PITCH_LIMIT_MAX - PITCH_LIMIT_MIN)) * PITCH_RANGE_DEG;")
    print()
    print(f"  GUI slider range: from -{deg_below_home:.0f} to +{deg_above_home:.0f}")
    print()
    print("Done! Run this data through me and I'll make the changes for you.")
    
    ser.close()

if __name__ == '__main__':
    main()
