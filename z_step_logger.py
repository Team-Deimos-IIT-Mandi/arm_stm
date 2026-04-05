"""
z_step_logger.py
================
Logs the Z-axis step response for PID tuning in MATLAB.

Sequence
--------
  1.  Connect & warmup (same protocol as test_usb_arm.py)
  2.  Hold at 0° for SETTLE_S seconds
  3.  Step to +STEP_DEG° and log for LOG_S seconds
  4.  Return to 0° and hold for SETTLE_S seconds
  5.  Step to -STEP_DEG° and log for LOG_S seconds  (reverse step)
  6.  Return to 0° and save CSV

Output CSV columns: time_ms, target_deg, actual_deg
"""

import serial
import struct
import time
import csv
import sys

# ── CONFIG ────────────────────────────────────────────────────────────────────
COM_PORT   = 'COM10'
BAUD_RATE  = 115200

STEP_DEG   = 60.0    # Step amplitude in output degrees (0 → +60 → 0 → -60)
SETTLE_S   = 2.0     # Seconds to hold at 0 before/between steps (let PID settle)
LOG_S      = 4.0     # Seconds to log each step response
LOOP_HZ    = 20      # Command/log rate

OUTPUT_CSV = 'z_step_response.csv'
# ─────────────────────────────────────────────────────────────────────────────

CMD_SIZE = struct.calcsize('<2s 6i c')   # 27 bytes

def pack_st(z_deg: float) -> bytes:
    """Build an ST packet targeting only Z-axis. Other joints set to hold-zero."""
    return struct.pack('<2s 6i c', b'ST',
                       0, 0, 0, 0, 0, int(z_deg * 1000), b'\n')

def pack_rt() -> bytes:
    return struct.pack('<2s 6i c', b'RT', 0, 0, 0, 0, 0, 0, b'\n')

def read_latest_fb(ser: serial.Serial) -> float | None:
    """Drain buffer, return most recent Z feedback degree. None if no packet."""
    last_z = None
    while ser.in_waiting >= CMD_SIZE:
        raw = ser.read_until(b'\n')
        if len(raw) == CMD_SIZE and raw.startswith(b'FB'):
            unpacked = struct.unpack('<2s 6i c', raw)
            z_raw = unpacked[6]   # motor_pos[5]
            if abs(z_raw) < 990000:
                last_z = z_raw / 1000.0
    return last_z


def warmup(ser: serial.Serial) -> float:
    """Return initial Z feedback degree after warmup."""
    print("Warming up comms...")
    start = time.time()
    z = 0.0
    while time.time() - start < 2.0:
        ser.write(pack_rt())
        time.sleep(0.05)
        v = read_latest_fb(ser)
        if v is not None:
            z = v
    print(f"  Initial Z = {z:.2f}°")
    return z


def run_phase(ser: serial.Serial,
              target_deg: float,
              duration_s: float,
              dt: float,
              label: str,
              rows: list) -> None:
    """Drive at target_deg for duration_s, appending rows to the shared list."""
    print(f"  {label}: target={target_deg:+.1f}°  duration={duration_s:.1f}s")
    t_end = time.time() + duration_s
    while time.time() < t_end:
        t0 = time.time()
        ser.write(pack_st(target_deg))
        z = read_latest_fb(ser)
        elapsed_ms = (time.time() - (t_end - duration_s)) * 1000.0
        if z is not None:
            rows.append({'time_ms': f'{elapsed_ms:.1f}',
                         'target_deg': f'{target_deg:.3f}',
                         'actual_deg': f'{z:.3f}',
                         'phase': label})
        # Sleep for the rest of the loop period
        sleep_t = dt - (time.time() - t0)
        if sleep_t > 0:
            time.sleep(sleep_t)


def main():
    dt = 1.0 / LOOP_HZ
    rows: list[dict] = []

    print("=" * 60)
    print("  Z-Axis Step Response Logger")
    print("=" * 60)
    print(f"  Port: {COM_PORT}  |  Step: 0 → ±{STEP_DEG}°")
    print(f"  Settle: {SETTLE_S}s  |  Log: {LOG_S}s  |  Output: {OUTPUT_CSV}")
    print()

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    with ser:
        warmup(ser)

        # Phase 1 — settle at 0°
        print("[1] Settling at 0°...")
        run_phase(ser,  0.0,      SETTLE_S, dt, "settle_0_pre", rows)

        # Phase 2 — positive step
        print("[2] Positive step...")
        run_phase(ser, +STEP_DEG, LOG_S,    dt, "step_pos", rows)

        # Phase 3 — return to 0°
        print("[3] Return to 0°...")
        run_phase(ser,  0.0,      SETTLE_S, dt, "settle_0_mid", rows)

        # Phase 4 — negative step
        print("[4] Negative step...")
        run_phase(ser, -STEP_DEG, LOG_S,    dt, "step_neg", rows)

        # Phase 5 — return to 0°
        print("[5] Return to 0°...")
        run_phase(ser,  0.0,      SETTLE_S, dt, "settle_0_post", rows)

    if not rows:
        print("ERROR: No feedback packets received. Check COM port and firmware.")
        sys.exit(1)

    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['time_ms', 'target_deg', 'actual_deg', 'phase'])
        writer.writeheader()
        writer.writerows(rows)

    print()
    print(f"Saved {len(rows)} samples → {OUTPUT_CSV}")
    print("Now run tune_z_pid.m in MATLAB.")


if __name__ == '__main__':
    main()
