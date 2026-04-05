"""
calibrate_pots.py — Live Pot Calibration  (LUT Generator)
==========================================================
WORKFLOW:
  1. Move the joint to a known angle
  2. Watch the ADC stabilise on screen (● STABLE)
  3. Press ENTER to stamp that reading
  4. Type the physical angle you used
  5. Repeat 10–15 times across the full range
  6. Type 'done' → get quality report + C code for RobotConfig.h
"""

import serial, struct, time, sys, math, threading, statistics, datetime

# ── Config ────────────────────────────────────────────────────────────────────
COM_PORT   = 'COM10'
BAUD_RATE  = 115200
PACKET_SIZE = 27
STABLE_N    = 10       # rolling window for σ
STABLE_SIG  = 4.0      # counts — below this = STABLE
CAPTURE_N   = 30       # samples averaged per stamped point

JOINTS = [
    (0, "M1  Shoulder / Link-1",  47.4,  74.0),
    (1, "M2  Elbow    / Link-2",   1.0,  61.2),
]

# ── ANSI helpers ──────────────────────────────────────────────────────────────
R='\033[0m'; B='\033[1m'; DIM='\033[2m'
GRN='\033[32m'; YLW='\033[33m'; CYN='\033[36m'; RED='\033[31m'; WHT='\033[97m'
UP  = lambda n: f'\033[{n}A'
CLR = '\r\033[2K'

def c(t, *codes): return ''.join(codes) + str(t) + R
def hbar(v, lo, hi, w=22, full='█', empty='░'):
    f = int(max(0, min(1, (v-lo)/(hi-lo) if hi!=lo else 0)) * w)
    return full*f + empty*(w-f)

# ── Serial / packet ───────────────────────────────────────────────────────────
def pack_debug():
    return struct.pack('<2s 6i c', b'DT', 0,0,0,0,0,0, b'\n')

def parse_fb(data):
    for i in range(len(data) - PACKET_SIZE + 1):
        if data[i:i+2]==b'FB' and data[i+PACKET_SIZE-1:i+PACKET_SIZE]==b'\n':
            try:
                u = struct.unpack('<2s 6i c', data[i:i+PACKET_SIZE])
                return list(u[1:7])
            except: pass
    return None

# ── Shared live state ─────────────────────────────────────────────────────────
class Live:
    def __init__(self):
        self.lock    = threading.Lock()
        self.adc     = [0, 0]
        self.hist    = [[], []]
        self.sigma   = [99.9, 99.9]
        self.stable  = [False, False]
        self.running = True
        self.ser     = None

lv = Live()

def reader():
    while lv.running:
        try:
            if lv.ser and lv.ser.is_open:
                lv.ser.write(pack_debug())
                time.sleep(0.05)
                raw = lv.ser.read(lv.ser.in_waiting or 1)
                fb = parse_fb(raw)
                if fb and 0 < fb[0] < 4096 and 0 < fb[1] < 4096:
                    with lv.lock:
                        for ch in range(2):
                            lv.adc[ch] = fb[ch]
                            h = lv.hist[ch]
                            h.append(fb[ch])
                            if len(h) > STABLE_N: h.pop(0)
                            lv.sigma[ch]  = statistics.pstdev(h) if len(h)>2 else 99.9
                            lv.stable[ch] = lv.sigma[ch] < STABLE_SIG
        except: time.sleep(0.1)

# ── Dashboard (single joint view) ─────────────────────────────────────────────
DASH_LINES = 11

def draw_dash(ch, pts, hint):
    """Redraw the fixed-height dashboard in place."""
    with lv.lock:
        adc, sig, stab = lv.adc[ch], lv.sigma[ch], lv.stable[ch]

    bar   = hbar(adc, 0, 4095, 24)
    bclr  = GRN if stab else YLW
    stlbl = c(' ● STABLE ', GRN, B) if stab else c(' ◌ MOVING ', YLW)
    siglbl= c(f'σ={sig:4.1f}', GRN if stab else YLW)
    ptlbl = c(str(len(pts)), CYN, B)

    rows = [
        '',
        c('  ┌──────────────────────────────────────────────────────────┐', CYN),
        c('  │  LIVE POT CALIBRATION                                    │', CYN),
        c('  └──────────────────────────────────────────────────────────┘', CYN),
        '',
        f'  ADC: {c(f"{adc:5d}", WHT, B)}  [{c(bar[:int(adc/4095*24)], bclr)}{c(bar[int(adc/4095*24):], DIM)}]  {stlbl} {siglbl}',
        f'  Points so far: {ptlbl}',
        '',
        c(f'  {hint}', DIM),
        c('  ─────────────────────────────────────────────────────────────', DIM),
        '',
    ]

    sys.stdout.write(UP(DASH_LINES))
    for row in rows:
        sys.stdout.write(CLR + row + '\n')
    sys.stdout.flush()

# ── Joint calibration ─────────────────────────────────────────────────────────
def calibrate_joint(ch, name, amin, amax):
    pts = []   # [(angle, mean, sigma)]

    print(f'\n{c("  ═"*31, CYN)}')
    print(f'  {c(name, WHT, B)}   {c(f"{amin}° → {amax}°", DIM)}')
    print(f'  {c("Aim for 10–15 points spread evenly across the range.", DIM)}')
    print(f'{c("  ═"*31, CYN)}\n')

    # Reserve dashboard area
    sys.stdout.write('\n' * DASH_LINES)

    def hint_str():
        if not lv.stable[ch]:
            return 'Move joint → wait for ● STABLE, then press ENTER to stamp'
        return 'Press ENTER to stamp reading — or type: done | undo | list | skip'

    # Launch display updater
    stop_disp = threading.Event()
    def disp_loop():
        while not stop_disp.is_set():
            draw_dash(ch, pts, hint_str())
            time.sleep(0.15)
    dt = threading.Thread(target=disp_loop, daemon=True)
    dt.start()

    try:
        while True:
            # --- get user input (display thread keeps running above) ---
            try:
                raw = input()
            except (EOFError, KeyboardInterrupt):
                break

            cmd = raw.strip().lower()

            # ── Commands ─────────────────────────────────────────────
            if cmd == 'done':
                if len(pts) < 3:
                    stop_disp.set(); dt.join(0.4)
                    print(CLR + c('  ⚠  Need at least 3 points — keep going.', YLW))
                    stop_disp.clear()
                    dt = threading.Thread(target=disp_loop, daemon=True); dt.start()
                    continue
                break

            if cmd == 'skip':
                pts.clear(); break

            if cmd == 'undo':
                stop_disp.set(); dt.join(0.4)
                if pts:
                    p = pts.pop()
                    print(CLR + c(f'  ↩  Removed {p[0]:.1f}° (ADC {p[1]:.0f})', YLW))
                else:
                    print(CLR + c('  Nothing to undo.', DIM))
                stop_disp.clear()
                dt = threading.Thread(target=disp_loop, daemon=True); dt.start()
                continue

            if cmd == 'list':
                stop_disp.set(); dt.join(0.4)
                if not pts:
                    print(CLR + c('  (no points yet)', DIM))
                else:
                    print(CLR + f'  {"#":>3}  {"Angle":>8}  {"ADC":>7}  {"σ":>6}')
                    for i,(a,v,s) in enumerate(pts):
                        print(f'  {i+1:>3}  {a:>7.1f}°  {v:>7.0f}  {s:>5.1f}')
                stop_disp.clear()
                dt = threading.Thread(target=disp_loop, daemon=True); dt.start()
                continue

            # ── Blank ENTER = stamp the current ADC ──────────────────
            if cmd == '':
                if not lv.stable[ch]:
                    stop_disp.set(); dt.join(0.4)
                    print(CLR + c('  ◌  Not stable yet — wait a moment and try again.', YLW))
                    stop_disp.clear()
                    dt = threading.Thread(target=disp_loop, daemon=True); dt.start()
                    continue

                # Collect N samples around this moment
                stop_disp.set(); dt.join(0.4)
                sys.stdout.write(CLR + c('  ⏳ Capturing...', CYN))
                sys.stdout.flush()
                with lv.lock:
                    lv.hist[ch].clear()
                readings = []
                t0 = time.time()
                while len(readings) < CAPTURE_N and time.time()-t0 < 3.0:
                    with lv.lock:
                        h = list(lv.hist[ch])
                    if len(h) > len(readings):
                        readings = list(h)
                    time.sleep(0.06)

                if not readings:
                    sys.stdout.write(CLR + c('  ✗ No data — check connection.\n', RED))
                    stop_disp.clear()
                    dt = threading.Thread(target=disp_loop, daemon=True); dt.start()
                    continue

                readings.sort()
                trim = max(1, len(readings)//8)
                trimmed = readings[trim:-trim] if len(readings)>2*trim else readings
                mean  = sum(trimmed)/len(trimmed)
                sigma = statistics.pstdev(trimmed) if len(trimmed)>1 else 0.0

                sys.stdout.write(CLR + c(f'  ADC stamped: {mean:.0f}  σ={sigma:.1f}', WHT, B))
                sys.stdout.write('  → ')
                sys.stdout.flush()

                # Now ask for angle
                try:
                    angle_str = input(f'What angle is the joint at? (°): ').strip()
                except (EOFError, KeyboardInterrupt):
                    stop_disp.clear()
                    dt = threading.Thread(target=disp_loop, daemon=True); dt.start()
                    continue

                try:
                    angle = float(angle_str)
                except ValueError:
                    print(c('  Invalid angle — point discarded.', RED))
                    stop_disp.clear()
                    dt = threading.Thread(target=disp_loop, daemon=True); dt.start()
                    continue

                if sigma > 20:
                    print(c(f'  ⚠  High noise (σ={sigma:.0f}) — consider retaking', YLW))

                pts.append((angle, mean, sigma))
                print(c(f'  ✓  Point #{len(pts)}: {angle:.1f}° → ADC {mean:.0f}', GRN))

                stop_disp.clear()
                dt = threading.Thread(target=disp_loop, daemon=True); dt.start()
                continue

            # ── If they typed a number directly = treat as angle shortcut
            try:
                angle = float(cmd)
                # Same as stamp + angle in one
                stop_disp.set(); dt.join(0.4)
                if not lv.stable[ch]:
                    print(CLR + c('  ◌  Not stable yet.', YLW))
                    stop_disp.clear()
                    dt = threading.Thread(target=disp_loop, daemon=True); dt.start()
                    continue
                sys.stdout.write(CLR + c('  ⏳ Capturing...', CYN))
                sys.stdout.flush()
                with lv.lock: lv.hist[ch].clear()
                readings = []
                t0 = time.time()
                while len(readings) < CAPTURE_N and time.time()-t0 < 3.0:
                    with lv.lock: h = list(lv.hist[ch])
                    if len(h) > len(readings): readings = list(h)
                    time.sleep(0.06)
                if readings:
                    readings.sort()
                    trim = max(1, len(readings)//8)
                    trimmed = readings[trim:-trim] if len(readings)>2*trim else readings
                    mean  = sum(trimmed)/len(trimmed)
                    sigma = statistics.pstdev(trimmed) if len(trimmed)>1 else 0.0
                    pts.append((angle, mean, sigma))
                    print(CLR + c(f'  ✓  Point #{len(pts)}: {angle:.1f}° → ADC {mean:.0f}  σ={sigma:.1f}', GRN))
                else:
                    print(CLR + c('  ✗ No data.', RED))
                stop_disp.clear()
                dt = threading.Thread(target=disp_loop, daemon=True); dt.start()
            except ValueError:
                stop_disp.set(); dt.join(0.4)
                print(CLR + c("  Unknown command. Press ENTER to stamp, or type: angle / done / undo / list / skip", DIM))
                stop_disp.clear()
                dt = threading.Thread(target=disp_loop, daemon=True); dt.start()

    finally:
        stop_disp.set()
        dt.join(0.5)

    return pts

# ── Quality analysis ──────────────────────────────────────────────────────────
def quality_report(name, angles, adcs, sigmas):
    n = len(angles)
    if n < 2: return [], 99.9
    ang_span = abs(angles[-1]-angles[0]) or 1
    adc_span = abs(adcs[-1]-adcs[0]) or 1
    scale    = adc_span / ang_span   # ADC counts per degree

    # Residuals vs global 2-point linear baseline
    res = []
    for i,(ang,adc) in enumerate(zip(angles, adcs)):
        t = (ang-angles[0])/(angles[-1]-angles[0])
        exp = adcs[0] + t*(adcs[-1]-adcs[0])
        res.append((adc-exp)/scale)

    mono = (all(adcs[i]<adcs[i+1] for i in range(n-1)) or
            all(adcs[i]>adcs[i+1] for i in range(n-1)))
    maxR = max(abs(r) for r in res)
    rmsR = math.sqrt(sum(r**2 for r in res)/n)
    worst_noise = max(s/scale for s in sigmas)
    est_acc = maxR + worst_noise

    print(f'\n{c("  ── Quality Report: "+name, CYN, B)}')
    print(f'  Points: {n}   Span: {ang_span:.1f}°   ADC range: {min(adcs):.0f}→{max(adcs):.0f}')
    print(f'  Scale: {scale:.1f} counts/°   1°={1/scale:.4f} ADC   Monotonic: {c("✓",GRN) if mono else c("✗",RED)}')
    print(f'  Non-linearity: peak {c(f"{maxR:.2f}°",GRN if maxR<0.5 else YLW if maxR<1 else RED)}   RMS {rmsR:.2f}°')
    print()
    print(f'  {"#":>3}  {"Angle":>8}  {"ADC":>7}  {"σ":>6}  {"σ°":>7}  {"Resid":>8}')
    print(c('  '+'─'*50, DIM))
    for i,(ang,adc,sig) in enumerate(zip(angles,adcs,sigmas)):
        noise_deg = sig/scale
        flag = c('  ← outlier?', RED) if abs(res[i])>2 else ''
        print(f'  {i+1:>3}  {ang:>7.1f}°  {adc:>7.0f}  {sig:>5.1f}  {noise_deg:>6.3f}°  {res[i]:>7.2f}°{flag}')
    print()
    acc_col = GRN if est_acc<=1 else YLW if est_acc<=2 else RED
    print(f'  Worst-case accuracy: {c(f"±{est_acc:.2f}°", acc_col, B)}', end='  ')
    print(c('✓ TARGET MET', GRN, B) if est_acc<=1 else c('⚠  Add more points in curved regions', YLW))
    return res, est_acc

# ── Plot ──────────────────────────────────────────────────────────────────────
def plot_luts(results):
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print(c('\n  (pip install matplotlib for the plot)', DIM)); return

    n = len(results)
    fig, axes = plt.subplots(1, n, figsize=(7*n, 5))
    if n == 1: axes = [axes]
    bg = '#0d1117'
    fig.patch.set_facecolor(bg)

    for ax, (name, angles, adcs, sigmas) in zip(axes, results):
        aa = np.array(angles); vv = np.array(adcs); ss = np.array(sigmas)
        ad = np.linspace(aa[0], aa[-1], 600)
        vd = np.interp(ad, aa, vv)
        vi = np.interp(ad, [aa[0],aa[-1]], [vv[0],vv[-1]])

        ax.set_facecolor('#161b22')
        ax.plot(ad, vi, '--', color='#30363d', lw=1.5, label='Ideal linear')
        ax.plot(ad, vd, color='#388bfd', lw=2.5, label='LUT curve')
        ax.errorbar(aa, vv, yerr=ss*3, fmt='o', color='#f0883e',
                    ecolor='#da6820', elinewidth=1.5, capsize=4, ms=7, label='Points (±3σ)')
        ax.fill_between(ad, vd-3*np.interp(ad,aa,ss), vd+3*np.interp(ad,aa,ss),
                        alpha=0.12, color='#388bfd', label='±3σ band')

        for sp in ax.spines.values(): sp.set_color('#30363d')
        ax.tick_params(colors='#8b949e')
        ax.set_xlabel('Angle (°)', color='#8b949e')
        ax.set_ylabel('ADC', color='#8b949e')
        ax.set_title(name, color='#e6edf3', fontsize=13, pad=10)
        ax.legend(facecolor='#161b22', edgecolor='#30363d', labelcolor='#c9d1d9', fontsize=9)
        ax.grid(True, color='#21262d', linewidth=0.5)

    plt.suptitle('Pot Calibration LUT', color='#e6edf3', fontsize=14)
    plt.tight_layout()
    fname = 'pot_calibration_plot.png'
    plt.savefig(fname, dpi=150, bbox_inches='tight', facecolor=bg)
    print(c(f'\n  ✓ Plot saved: {fname}', GRN))
    plt.show()

# ── Code gen ──────────────────────────────────────────────────────────────────
def gen_c(var, angles, adcs, name):
    a = ', '.join(f'{x:.1f}f' for x in angles)
    v = ', '.join(str(int(round(x))) for x in adcs)
    return (f'// {name}  ({len(angles)} pts)\n'
            f'constexpr int   PTS_{var} = {len(angles)};\n'
            f'constexpr float angles_{var}[PTS_{var}] = {{ {a} }};\n'
            f'constexpr int   adcs_{var}[PTS_{var}]   = {{ {v} }};')

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print(c('\n╔══════════════════════════════════════════════════╗', CYN))
    print(c('║  POT CALIBRATION  —  Live LUT Builder            ║', CYN))
    print(c('╠══════════════════════════════════════════════════╣', CYN))
    print(c('║  HOW IT WORKS:                                   ║', CYN))
    print(c('║  1. Move joint → watch ADC settle → ● STABLE    ║', CYN))
    print(c('║  2. Press ENTER  (or type angle directly)        ║', CYN))
    print(c('║  3. Type the physical angle you read from level  ║', CYN))
    print(c('║  4. Repeat 10–15x across full range              ║', CYN))
    print(c('║  5. Type  done  → get quality report + C code   ║', CYN))
    print(c('╚══════════════════════════════════════════════════╝\n', CYN))

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.12)
        print(c(f'  ✓ Connected to {COM_PORT}', GRN))
    except Exception as e:
        print(c(f'  ✗ {e}', RED)); return

    lv.ser = ser
    threading.Thread(target=reader, daemon=True).start()

    print('  Warming up...', end='', flush=True)
    for _ in range(20):
        ser.write(pack_debug()); time.sleep(0.06)
    print(c(' done!\n', GRN))

    calibrated = []

    for ch, name, amin, amax in JOINTS:
        var = f'M{ch+1}'
        ans = input(f'  Calibrate {c(name, YLW)}? (y/n): ').strip().lower()
        if ans != 'y':
            print(c('  Skipped.\n', DIM)); continue

        pts = calibrate_joint(ch, name, amin, amax)
        if len(pts) < 2:
            print(c('  Not enough points.\n', RED)); continue

        pts.sort(key=lambda p: p[0])
        angles = [p[0] for p in pts]
        adcs   = [p[1] for p in pts]
        sigmas = [p[2] for p in pts]

        res, est = quality_report(name, angles, adcs, sigmas)

        print('\n  Remove outlier point numbers? (space-separated, Enter to skip): ', end='')
        resp = input().strip()
        if resp:
            try:
                to_rm = sorted({int(x)-1 for x in resp.split()}, reverse=True)
                for idx in to_rm:
                    if 0 <= idx < len(angles):
                        angles.pop(idx); adcs.pop(idx); sigmas.pop(idx)
                if len(angles) >= 2:
                    quality_report(name+' (cleaned)', angles, adcs, sigmas)
            except ValueError: pass

        calibrated.append((name, var, angles, adcs, sigmas))

    lv.running = False
    ser.close()

    if not calibrated:
        print(c('\n  Nothing calibrated.', RED)); return

    print(c('\n╔══════════════════════════════════════════════════╗', GRN))
    print(c('║  COPY THIS INTO  Core/Inc/RobotConfig.h          ║', GRN))
    print(c('╚══════════════════════════════════════════════════╝\n', GRN))

    blocks = []
    for (name, var, angles, adcs, sigmas) in calibrated:
        b = gen_c(var, angles, adcs, name)
        print(b + '\n')
        blocks.append(b)

    fname = 'pot_calibration_result.h'
    with open(fname, 'w') as f:
        f.write(f'// calibrate_pots.py — {datetime.datetime.now():%Y-%m-%d %H:%M}\n\n')
        for b in blocks: f.write(b+'\n\n')
    print(c(f'  ✓ Saved: {fname}', GRN))

    print(c('\n  After flashing: rebuild → test in test_usb_arm.py\n', DIM))

    plot_luts([(n, a, v, s) for (n,_,a,v,s) in calibrated])

if __name__ == '__main__':
    main()
