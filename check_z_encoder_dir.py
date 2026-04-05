"""
check_z_encoder_dir.py
======================
PURPOSE
-------
Verifies that the Z-axis motor drives the encoder in the expected direction.

HOW IT WORKS
------------
1.  Sends a 'DT' (debug/raw) packet — this makes the firmware reply with
    raw encoder counts (0-4095) for all four I2C joints.
    Index 5 in the FB packet => raw encoder[3] => Z-Axis.

2.  Runs the motor open-loop at LOW power in the POSITIVE direction for ~1 s,
    then negative for ~1 s, reading the encoder the whole time.

3.  Prints a clear report:
      - Start encoder value
      - End encoder value after +PWM
      - End encoder value after -PWM
      - Whether the sign is consistent with ZA_PID_SIGN = -1 in firmware.

CONNECTIONS ASSUMED
-------------------
  COM_PORT  : STM32 UART USB-CDC (same as test_usb_arm.py)
  Protocol  : CommandPacketUnified  '<2s 6i c' (27 bytes, '\\n' footer)
              FeedbackPacketUnified '<2s 6i c' (27 bytes, 'FB' header)

PACKET FIELDS (index in FB)
---------------------------
  [0]=M1 ADC  [1]=M2 ADC  [2]=M3 raw  [3]=Pitch raw  [4]=Roll raw  [5]=Z raw

NOTE: firmware sends raw 0-4095 encoder values when the header is 'D' or 'P'.
"""

import serial
import struct
import time

# ── CONFIG ──────────────────────────────────────────────────────────────────
COM_PORT  = 'COM10'   # <-- change if needed
BAUD_RATE = 115200

TEST_PWM      = 80    # low open-loop PWM for safety (0-255)
PULSE_SECONDS = 1.2   # how long to drive each direction

# From RobotConfig.h — used to explain what the expected behaviour is
ZA_PID_SIGN   = -1    # positive PID output → negative PWM → motor winds UP
# ─────────────────────────────────────────────────────────────────────────────

CMD_SIZE = struct.calcsize('<2s 6i c')  # 27 bytes
FB_SIZE  = struct.calcsize('<2s 6i c')  # 27 bytes


def build_debug_packet():
    """DT packet — firmware replies with raw ADC/encoder values, motors hold still."""
    return struct.pack('<2s 6i c', b'DT', 0, 0, 0, 0, 0, 0, b'\n')


def build_ol_packet(z_pwm: int):
    """
    'PT' open-loop packet.  motor_cmd layout (index in array):
      [0]=M1  [1]=M2  [2]=M3  [3]=Pitch  [4]=Roll  [5]=Z-Axis
    We leave all other joints at 0 so they coast.
    """
    return struct.pack('<2s 6i c', b'PT', 0, 0, 0, 0, 0, z_pwm, b'\n')


def flush_read_z(ser: serial.Serial, is_raw: bool = True) -> int | None:
    """
    Drain the serial buffer and return the most recent Z raw encoder value.
    Returns None if no valid FB packet found.
    """
    last_z = None
    while ser.in_waiting >= FB_SIZE:
        raw_data = ser.read_until(b'\n')
        if len(raw_data) == FB_SIZE and raw_data.startswith(b'FB'):
            unpacked = struct.unpack('<2s 6i c', raw_data)
            last_z = unpacked[6]  # motor_pos[5] is index 6 (after header 2-byte field)
    return last_z


def read_fb(ser: serial.Serial, timeout: float = 0.5) -> list | None:
    """Block until we get one valid FB packet or timeout."""
    deadline = time.time() + timeout
    ser.flushInput()
    while time.time() < deadline:
        if ser.in_waiting >= FB_SIZE:
            raw_data = ser.read_until(b'\n')
            if len(raw_data) == FB_SIZE and raw_data.startswith(b'FB'):
                unpacked = struct.unpack('<2s 6i c', raw_data)
                return list(unpacked[1:7])  # [M1,M2,M3,Pitch,Roll,Z]
        time.sleep(0.005)
    return None


def capture_z_stream(ser: serial.Serial, drive_pkt: bytes | None,
                     duration: float) -> list[int]:
    """
    Send drive_pkt at 20 Hz for `duration` seconds and collect raw Z readings.
    If drive_pkt is None, send DT (hold).
    """
    readings: list[int] = []
    t_end = time.time() + duration
    while time.time() < t_end:
        pkt = drive_pkt if drive_pkt else build_debug_packet()
        ser.write(pkt)

        # Read any pending FB packets
        time.sleep(0.01)
        while ser.in_waiting >= FB_SIZE:
            raw_data = ser.read_until(b'\n')
            if len(raw_data) == FB_SIZE and raw_data.startswith(b'FB'):
                unpacked = struct.unpack('<2s 6i c', raw_data)
                readings.append(unpacked[6])  # Z raw encoder

        time.sleep(0.04)  # ~20 Hz
    return readings


def summarise(label: str, readings: list[int]) -> None:
    if not readings:
        print(f"  {label}: NO READINGS")
        return
    span = readings[-1] - readings[0]
    wrap_note = ""
    # simple wrap detection (crossed 0/4095 boundary)
    for a, b in zip(readings, readings[1:]):
        if abs(b - a) > 2000:
            wrap_note = "  ⚠ wrap detected — raw values crossed 0/4095"
            span = None
            break
    if span is not None:
        print(f"  {label}: start={readings[0]:4d}  end={readings[-1]:4d}  Δ={span:+d}")
    else:
        print(f"  {label}: start={readings[0]:4d}  end={readings[-1]:4d}  Δ=n/a{wrap_note}")


def main():
    print("=" * 60)
    print("  Z-Axis Encoder Direction Check")
    print("=" * 60)
    print(f"  Port: {COM_PORT} @ {BAUD_RATE} baud")
    print(f"  Test PWM: ±{TEST_PWM}  |  Duration per direction: {PULSE_SECONDS}s")
    print()

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"ERROR: Cannot open {COM_PORT}: {e}")
        return

    with ser:
        # ── Step 1: Wake the firmware ──────────────────────────────────────
        print("[1/4] Waking firmware with DT packets (raw encoder mode)...")
        for _ in range(20):                 # ~1 s at 20 Hz
            ser.write(build_debug_packet())
            time.sleep(0.05)

        # ── Step 2: Capture baseline ───────────────────────────────────────
        print("[2/4] Reading baseline Z encoder...")
        base_readings = capture_z_stream(ser, None, 1.0)
        if not base_readings:
            print("ERROR: No FB packets received. Check COM port and baud rate.")
            return
        baseline = base_readings[-1]
        print(f"      Baseline Z raw encoder = {baseline}")
        print()

        # ── Step 3: Drive POSITIVE then NEGATIVE ──────────────────────────
        # WARNING: motor will move! Keep arm clear.
        print("[3/4] Driving motor OPEN-LOOP ... arm will move slightly!")
        print()

        # — Positive PWM —
        print(f"  ► Positive PWM (+{TEST_PWM}) for {PULSE_SECONDS}s ...")
        pos_readings = capture_z_stream(ser, build_ol_packet(+TEST_PWM), PULSE_SECONDS)
        # Stop and settle
        capture_z_stream(ser, None, 0.5)
        summarise(f"+{TEST_PWM} PWM", pos_readings)

        # Return to baseline (send 0 for 0.5 s)
        time.sleep(0.1)

        # — Negative PWM —
        print(f"  ► Negative PWM (-{TEST_PWM}) for {PULSE_SECONDS}s ...")
        neg_readings = capture_z_stream(ser, build_ol_packet(-TEST_PWM), PULSE_SECONDS)
        # Stop
        capture_z_stream(ser, None, 0.5)
        summarise(f"-{TEST_PWM} PWM", neg_readings)

        # ── Step 4: Report ─────────────────────────────────────────────────
        print()
        print("[4/4] Analysis")
        print("-" * 60)

        def net_delta(readings: list[int]) -> int | None:
            if len(readings) < 2:
                return None
            d = readings[-1] - readings[0]
            # unwrap
            for a, b in zip(readings, readings[1:]):
                if abs(b - a) > 2000:
                    return None
            return d

        d_pos = net_delta(pos_readings)
        d_neg = net_delta(neg_readings)

        if d_pos is None or d_neg is None:
            print("  Cannot determine direction — wrap or insufficient data.")
            print("  Try reducing TEST_PWM or PULSE_SECONDS if arm travels too far.")
        else:
            print(f"  +PWM drove encoder by  Δ={d_pos:+d}")
            print(f"  -PWM drove encoder by  Δ={d_neg:+d}")
            print()

            # From firmware:
            #   pid_output[3] (positive = approaching target from below)
            #   → DriveMotor(..., pid_output[3] * ZA_PID_SIGN)
            #   ZA_PID_SIGN = -1  →  positive pid_output → negative PWM
            #
            # So if +PWM makes encoder DECREASE → ZA_PID_SIGN=-1 is CORRECT
            #       +PWM makes encoder INCREASE → ZA_PID_SIGN=-1 may be WRONG

            if d_pos < 0:
                enc_vs_pwm = "DECREASES with +PWM"
                sign_match = True
            elif d_pos > 0:
                enc_vs_pwm = "INCREASES with +PWM"
                sign_match = False
            else:
                enc_vs_pwm = "DID NOT MOVE with +PWM — check wiring/power"
                sign_match = None

            print(f"  Encoder {enc_vs_pwm}")
            print()
            print(f"  Firmware ZA_PID_SIGN = {ZA_PID_SIGN}")

            if sign_match is None:
                print("  ⚠ Cannot determine correctness — motor did not move.")
            elif sign_match:
                print("  ✅ Direction is CONSISTENT with ZA_PID_SIGN = -1.")
                print("     (positive error → negative PWM → encoder counts UP → correct)")
            else:
                print("  ❌ Direction MISMATCH — ZA_PID_SIGN should be +1, not -1.")
                print("     OR invert ENCODER_DIRECTION[3] in RobotConfig.h.")
                print()
                print("  Suggested fixes (pick one):")
                print("    A) In RobotConfig.h: change  ZA_PID_SIGN = +1;")
                print("    B) In RobotConfig.h: change  ENCODER_DIRECTION[3] = -1;")

        print()
        print("Done.")


if __name__ == '__main__':
    main()
