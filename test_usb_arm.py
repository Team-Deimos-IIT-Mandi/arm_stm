import serial
import struct
import time
import sys
import threading
import tkinter as tk

# Replace with your actual COM port (e.g., 'COM3' on Windows, '/dev/ttyACM0' on Mac/Linux)
COM_PORT = 'COM10' 
BAUD_RATE = 115200

# Global target variables state — initialized to 0 as safe fallback if hardware sync fails
targets = {
    'm1': 0.0,
    'm2': 0.0,
    'm3': 0.0,
    'pitch': 0.0,
    'roll': 0.0,
    'z': 0.0
}
feedback = [0.0]*6
running = True
is_calibration_mode = False
initial_sync_done = False
pid_update_pending = None
pitch_error = False  # Set when firmware reports pitch freeze
z_error = False      # Set when firmware reports Z-axis collision

# Protocol constants matching protocol.hpp
CMD_SIZE = 29   # 2s + 6i + B + B + c
FB_SIZE  = 52   # 2s + 6i + 6i + B + B

def compute_checksum(data: bytes, end: int) -> int:
    """XOR of bytes [0..end-1], matching firmware compute_checksum()."""
    x = 0
    for b in data[:end]:
        x ^= b
    return x

def pack_command(m1, m2, m3, pitch, roll, z, is_calib):
    # The STM32 code expects values multiplied by 1000
    header = b'DT' if is_calib else b'ST'
    flags = 0
    # Pack without checksum first, compute it, then repack
    partial = struct.pack('<2s 6i B', header, 
                       int(round(m1*1000)), int(round(m2*1000)), int(round(m3*1000)), 
                       int(round(pitch*1000)), int(round(roll*1000)), int(round(z*1000)), 
                       flags)
    checksum = compute_checksum(partial, len(partial))
    return struct.pack('<2s 6i B B c', header, 
                       int(round(m1*1000)), int(round(m2*1000)), int(round(m3*1000)), 
                       int(round(pitch*1000)), int(round(roll*1000)), int(round(z*1000)), 
                       flags, checksum, b'\n')

def serial_thread():
    global feedback, running, initial_sync_done, pid_update_pending, pitch_error, z_error
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        print(f"Successfully connected to {COM_PORT}!")
    except Exception as e:
        print(f"Error opening serial port: {e}\nPlease check Device Manager.")
        running = False
        return

    # Phase 1: Hardware Sync — actively send DT packets and listen for the first FB response
    # to capture the true physical resting state before the GUI creates sliders.
    start_time = time.time()
    while running and not initial_sync_done and (time.time() - start_time < 2.0):
        # Send a RT (Read Targets) packet to trigger physical angle response
        sync_cmd = struct.pack('<2s 6i B B c', b'RT', 0, 0, 0, 0, 0, 0, 0, 0, b'\n')
        try:
            ser.write(sync_cmd)
        except Exception:
            pass
        time.sleep(0.05)
        
        while ser.in_waiting >= FB_SIZE:
            raw_data = ser.read(FB_SIZE)
            if len(raw_data) == FB_SIZE and raw_data.startswith(b'FB'):
                unpacked = struct.unpack('<2s 6i 6i B B', raw_data)
                feedback = [val / 1000.0 for val in unpacked[1:7]]
                targets['m1'] = feedback[0]
                targets['m2'] = feedback[1]
                targets['m3'] = feedback[2]
                targets['pitch'] = feedback[3]
                targets['roll'] = feedback[4]
                targets['z'] = feedback[5]
                initial_sync_done = True
                print("Hardware synced! GUI is live.")
                break
        if initial_sync_done:
            break

    # Phase 2: RT warmup — send more RT packets so the MCU finishes its comms_ok
    # self-sync before we push the first real ST positional command.
    WARMUP_COUNT = 10  # ~500ms of RT packets at 20Hz
    for _ in range(WARMUP_COUNT):
        if not running:
            break
        warmup_cmd = pack_command(targets['m1'], targets['m2'],
                                 targets['m3'], targets['pitch'],
                                 targets['roll'], targets['z'], False)
        # Overwrite header to RT
        warmup_cmd = b'RT' + warmup_cmd[2:]
        try:
            ser.write(warmup_cmd)
            # Also read feedback during warmup to keep targets fresh
            while ser.in_waiting >= FB_SIZE:
                raw_data = ser.read(FB_SIZE)
                if len(raw_data) == FB_SIZE and raw_data.startswith(b'FB'):
                    unpacked = struct.unpack('<2s 6i 6i B B', raw_data)
                    feedback = [val / 1000.0 for val in unpacked[1:7]]
                    targets['m1'] = feedback[0]
                    targets['m2'] = feedback[1]
                    targets['m3'] = feedback[2]
                    targets['pitch'] = feedback[3]
                    targets['roll'] = feedback[4]
                    targets['z'] = feedback[5]
        except Exception:
            pass
        time.sleep(0.05)

    # Phase 3: Hardware sync complete. Holding current positions.
    print("Alignment complete! Arm will hold its current physical position.")

    # Phase 4: Main loop — now safe to send real ST commands
    while running:
        if pid_update_pending is not None:
            cmd_bytes = pid_update_pending
            pid_update_pending = None
        else:
            # Pass the calibration flag directly
            cmd_bytes = pack_command(targets['m1'], targets['m2'], targets['m3'], 
                                     targets['pitch'], targets['roll'], targets['z'],
                                     is_calibration_mode)
        
        try:
            ser.write(cmd_bytes)
            
            # Flush and Parse Feedback Data
            while ser.in_waiting >= FB_SIZE:
                raw_data = ser.read(FB_SIZE)
                if len(raw_data) == FB_SIZE and raw_data.startswith(b'FB'):
                    unpacked = struct.unpack('<2s 6i 6i B B', raw_data)
                    # unpacked: header, pos[0..5], vel[0..5], flags, checksum
                    fb_flags = unpacked[13]    # flags byte
                    fb_checksum = unpacked[14]  # checksum byte
                    
                    # Verify checksum
                    expected_cs = compute_checksum(raw_data, FB_SIZE - 1)  # XOR of [0..50]
                    if expected_cs != fb_checksum:
                        continue  # Drop corrupted frame
                    
                    if is_calibration_mode:
                        feedback = list(unpacked[1:7]) # Raw integers (0-4095)
                    else:
                        fb_vals = [val / 1000.0 for val in unpacked[1:7]]
                        # Use flags for error detection
                        pitch_error = bool(fb_flags & 0x01)
                        z_error     = bool(fb_flags & 0x02)
                        feedback = fb_vals
        except Exception as e:
            print("Serial disconnected or error:", e)
            running = False
            break
            
        # Send packets at roughly 20Hz
        time.sleep(0.05) 
        
    if ser.is_open:
        ser.close()

def on_closing(root):
    global running
    running = False
    root.destroy()

def create_gui():
    root = tk.Tk()
    root.title("STM32 Robot Arm Control")
    root.protocol("WM_DELETE_WINDOW", lambda: on_closing(root))
    root.geometry("520x450")
    
    sliders = {}
    
    def update_targets(val=None):
        try:
            if 'm1' in sliders: targets['m1'] = sliders['m1'].get()
            if 'm2' in sliders: targets['m2'] = sliders['m2'].get()
            if 'm3' in sliders: targets['m3'] = sliders['m3'].get()
            if 'pitch' in sliders: targets['pitch'] = sliders['pitch'].get()
            if 'roll' in sliders: targets['roll'] = sliders['roll'].get()
            if 'z' in sliders: targets['z'] = sliders['z'].get()
        except (tk.TclError, KeyError):
            pass # Ignore mid-typing blank or initialization errors
    
    def create_slider(name, label, min_val, max_val, init_val):
        frame = tk.Frame(root)
        frame.pack(fill='x', padx=10, pady=5)
        tk.Label(frame, text=label, width=15, anchor="w").pack(side='left')
        v = tk.DoubleVar(value=init_val)
        sliders[name] = v # Register variable BEFORE creating Scale to avoid KeyError in command
        s = tk.Scale(frame, variable=v, from_=min_val, to=max_val, resolution=0.1, 
                     orient='horizontal', length=250, command=update_targets)
        s.pack(side='left')
        
        e = tk.Entry(frame, textvariable=v, width=7, font=('Arial', 10), justify='center')
        e.pack(side='left', padx=15)
        e.bind('<Return>', lambda event: update_targets())
        e.bind('<FocusOut>', lambda event: update_targets())
        
    # Wait for serial thread to capture physical feedback (max 1s)
    start_wait = time.time()
    while not initial_sync_done and running and (time.time() - start_wait < 1.0):
        root.update()
        time.sleep(0.01)
    
    # Sliders default to current resting state via targets dict
    # Expanded boundaries to prevent Tkinter from silently clamping the physical synchronization!
    create_slider('m1', 'M1 (Link 1)', 47.4, 74.0, targets['m1'])
    create_slider('m2', 'M2 (Link 2)',  1.0, 61.2, targets['m2'])
    create_slider('m3', 'M3 (Link 3)', -180.0, 180.0, targets['m3'])
    create_slider('pitch', 'Pitch', 5.0, 163.0, targets['pitch'])
    create_slider('roll', 'Roll', 0.0, 360.0, targets['roll'])
    create_slider('z', 'Z-Axis', -180.0, 180.0, targets['z'])
    
    # Validation Mode Checkbox
    calib_var = tk.BooleanVar(value=False)
    def toggle_calib():
        global is_calibration_mode
        is_calibration_mode = calib_var.get()
    
    tk.Checkbutton(root, text="RAW Calibration Mode (Reads absolute 0-4096)", 
                   variable=calib_var, command=toggle_calib, font=('Arial', 9, 'bold'), fg="red").pack(pady=5)
                   
    # PID Tuning Section
    pid_frame = tk.LabelFrame(root, text="Live PID Tuning (Pushes KT Packet)", padx=5, pady=5)
    pid_frame.pack(fill='x', padx=10, pady=5)
    
    tk.Label(pid_frame, text="Joint:").grid(row=0, column=0)
    joint_var = tk.StringVar(value='M3 (Link 3)')
    joints = ['M1 (Pot)', 'M2 (Pot)', 'M3 (Link 3)', 'Pitch', 'Roll', 'Z-Axis']
    tk.OptionMenu(pid_frame, joint_var, *joints).grid(row=0, column=1)
    
    tk.Label(pid_frame, text="P:").grid(row=0, column=2)
    p_var = tk.StringVar(value="2.0")
    tk.Entry(pid_frame, textvariable=p_var, width=5).grid(row=0, column=3)
    
    tk.Label(pid_frame, text="I:").grid(row=0, column=4)
    i_var = tk.StringVar(value="0.0")
    tk.Entry(pid_frame, textvariable=i_var, width=5).grid(row=0, column=5)
    
    tk.Label(pid_frame, text="D:").grid(row=0, column=6)
    d_var = tk.StringVar(value="0.0")
    tk.Entry(pid_frame, textvariable=d_var, width=5).grid(row=0, column=7)
    
    def send_pid():
        global pid_update_pending
        idx = joints.index(joint_var.get())
        try:
            p_val = float(p_var.get())
            i_val = float(i_var.get())
            d_val = float(d_var.get())
            # The STM32 code expects values multiplied by 1000
            # Format: <2s 6i B B c> -> header, idx, p, i, d, 0, 0, flags=0, checksum, newline
            partial = struct.pack('<2s 6i B', b'KT', idx, int(p_val*1000), int(i_val*1000), int(d_val*1000), 0, 0, 0)
            cs = compute_checksum(partial, len(partial))
            pid_update_pending = struct.pack('<2s 6i B B c', b'KT', idx, int(p_val*1000), int(i_val*1000), int(d_val*1000), 0, 0, 0, cs, b'\n')
            print(f"Sent PID Update -> {joint_var.get()}: P={p_val}, I={i_val}, D={d_val}")
        except ValueError:
            print("Invalid Floating Point PID values")

    tk.Button(pid_frame, text="Push PID", command=send_pid, bg="lightblue").grid(row=0, column=8, padx=5)
    
    # Pitch Error Display
    pitch_err_label = tk.Label(root, text="", font=('Arial', 11, 'bold'), fg="red", bg="#FFE0E0")
    pitch_err_label.pack(pady=2)
    pitch_err_label.pack_forget()  # Hidden by default
    
    # Feedback display
    tk.Label(root, text="Live Hardware Feedback:", font=('Arial', 10, 'bold')).pack(pady=(5,0))
    fb_label = tk.Label(root, text="Waiting for feedback...", font=('Courier', 11), bg="black", fg="lime", width=42, height=3)
    fb_label.pack(pady=5)
    
    def update_feedback_ui():
        if not running: return
        
        # Pitch freeze error display
        msg = []
        if pitch_error:
            msg.append("⚠ PITCH LIMIT EXCEEDED")
        if z_error:
            msg.append("⚠ Z-AXIS COLLISION DETECTED")
            
        if msg:
            pitch_err_label.config(text=" | ".join(msg))
            pitch_err_label.pack(pady=2)
        else:
            pitch_err_label.pack_forget()
        
        if is_calibration_mode:
            t1 = f"M1: {int(feedback[0]):04d} | M2: {int(feedback[1]):04d} | M3: {int(feedback[2]):04d}"
            t2 = f"Pit:{int(feedback[3]):04d} | Rol:{int(feedback[4]):04d} | Z:  {int(feedback[5]):04d}"
        else:
            t1 = f"M1: {feedback[0]:6.1f} | M2: {feedback[1]:6.1f} | M3: {feedback[2]:6.1f}"
            t2 = f"Pit:{feedback[3]:6.1f} | Rol:{feedback[4]:6.1f} | Z:  {feedback[5]:6.1f}"
            
        fb_label.config(text=f"{t1}\n{t2}")
        root.after(50, update_feedback_ui)
        
    root.after(50, update_feedback_ui)
    root.mainloop()

if __name__ == '__main__':
    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()
    create_gui()
    running = False
