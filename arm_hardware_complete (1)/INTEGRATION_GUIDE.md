# V2CLAW Arm — ROS2 Hardware Interface Integration Guide
# From current firmware state to MoveIt2-ready
# ══════════════════════════════════════════════════════════════════════════════

## What you are doing and why

Your STM32 is a standalone closed-loop controller.
ROS2 + MoveIt2 needs to be the trajectory planner and the STM32 needs to be
a dumb position servo — it receives a target angle, holds it, and reports back.
This guide makes that happen without rewriting the STM32 firmware from scratch.

══════════════════════════════════════════════════════════════════════════════
## PART 1 — STM32 firmware changes  (stm32_patch/RobotCore_protocol_patch.h)
══════════════════════════════════════════════════════════════════════════════

### What changes and what does NOT change

DOES NOT CHANGE:
  - Your PID control logic
  - Trapezoidal profiler
  - Safety systems (stall, pitch freeze, Z collision)
  - ADC oversampling
  - FreeRTOS task structure

DOES CHANGE:
  - Packet struct sizes (29 bytes cmd, 52 bytes feedback)
  - Checksum added to both directions
  - Velocity added to feedback
  - flags byte replaces sentinel-only pattern

### Step 1.1 — Update packet structs in RobotCore.h

Open RobotCore.h and find the two struct definitions.
Replace them with the structs from stm32_patch/RobotCore_protocol_patch.h,
section "STEP 1".

The new CommandPacketUnified is 29 bytes (was 26).
The new FeedbackPacketUnified is 52 bytes (was 27).

Add the static_asserts — the linker will catch size mismatches at compile time.

### Step 1.2 — Add compute_checksum() helper

Add this function at the top of RobotCore.cpp, before parseCommandPacket():

    static uint8_t compute_checksum(const uint8_t* data, uint32_t len) {
        uint8_t x = 0;
        for (uint32_t i = 0; i < len; i++) x ^= data[i];
        return x;
    }

XOR checksum is simple and fast on Cortex-M4.
It catches single-byte corruption, which is the dominant failure mode on UART.

### Step 1.3 — Replace parseCommandPacket()

The new version (see STEP 2 in the patch file) adds one line:

    uint8_t expected_cs = compute_checksum(
        pkt, offsetof(CommandPacketUnified, checksum));

    if (cmd->header[1] == 'T' &&
        cmd->footer   == '\n' &&
        cmd->checksum == expected_cs)   // ← new
    {
        ...accept frame...
    }

This silently drops corrupted packets. The STM32 watchdog (2s) will trigger
if too many packets are dropped — which is the correct behaviour.

### Step 1.4 — Add velocity state variables

Add to the global state section of RobotCore.cpp (after the existing variables):

    float fb_vel_m3    = 0.0f,  fb_prev_m3    = 0.0f;
    float fb_vel_pitch = 0.0f,  fb_prev_pitch = 0.0f;
    float fb_vel_roll  = 0.0f,  fb_prev_roll  = 0.0f;
    float fb_vel_z     = 0.0f,  fb_prev_z     = 0.0f;

### Step 1.5 — Compute velocities in RobotCore_PIDTask()

Add the velocity computation block (see STEP 5 in patch file) BEFORE the
"5. Compute ESP-B PIDs" comment.

These compute deg/s at the output shaft using finite differences over the
fixed 20ms dt.  The STM32 has microsecond timing accuracy — much better
than the Pi's estimate of the control period — so this velocity is reliable.

Note the roll joint uses wrapError() for circular interpolation.
Note M3 and Z use their continuous positions (not raw encoder) for monotonic
velocity — otherwise a wrap at 360° would show a huge spike.

### Step 1.6 — Replace the feedback transmission block

Find "// 7. Transmit Feedback" in RobotCore_PIDTask() and replace the entire
block with the new version from STEP 3 in the patch file.

Key additions:
  - motor_vel[6] populated from fb_vel_* variables
  - flags byte: bit0=pitch_frozen, bit1=z_collision
  - checksum computed and written before transmission
  - NO footer byte (checksum is the last field)

### Step 1.7 — Update RobotCore_Init()

Old:
    current_fb.header[0] = 'F'; current_fb.header[1] = 'B';
    current_fb.footer = '\n';

New (no footer field):
    current_fb.header[0] = 'F';
    current_fb.header[1] = 'B';
    memset(current_fb.motor_pos, 0, sizeof(current_fb.motor_pos));
    memset(current_fb.motor_vel, 0, sizeof(current_fb.motor_vel));
    current_fb.flags    = 0;
    current_fb.checksum = 0;

### Step 1.8 — Verify baud rate and UART DMA buffer size

Your current baud rate is 115200.
At 115200 baud, 52 bytes takes 52 × 10 / 115200 = 4.5 ms to transmit.
At 50 Hz feedback, that's 4.5ms / 20ms = 22.5% of the tick just in UART time.
This is fine, but if you ever want to go to 100 Hz feedback, bump to 921600.

Your DMA RX buffer (rx_buffer, 128 bytes) is large enough — 29 bytes per cmd.

### Step 1.9 — Build and flash, then verify with a terminal

On your PC:
    screen /dev/ttyUSB0 115200

Power on the STM32. You should see binary data appearing (not ASCII).
The frame starts with 0x46 0x42 ('F','B') — confirm with:
    hexdump -C output

The first two bytes of every frame MUST be 46 42.
If you see garbage or missing data, check:
  - PC10/PC11 connected to your USB-UART adapter (TX/RX swapped as needed)
  - UART4 baud matches on both sides
  - Ground is shared between STM32 and USB-UART adapter

══════════════════════════════════════════════════════════════════════════════
## PART 2 — ROS2 hardware interface  (arm_hardware package)
══════════════════════════════════════════════════════════════════════════════

### Step 2.1 — Copy the package to your workspace

    cp -r arm_ws/src/arm_hardware ~/ros2_ws/src/
    cd ~/ros2_ws

### Step 2.2 — Install dependencies

    sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                     ros-humble-hardware-interface ros-humble-pluginlib

### Step 2.3 — Set UART permissions

The Pi's GPIO UART (/dev/ttyAMA0) requires group membership:

    sudo usermod -aG dialout $USER
    # Log out and back in, or:
    newgrp dialout

Verify:
    ls -la /dev/ttyAMA0
    # Should show: crw-rw---- 1 root dialout ...

If using USB-UART adapter (/dev/ttyUSB0), same command.

### Step 2.4 — Enable the Pi UART (if using /dev/ttyAMA0)

On Raspberry Pi, edit /boot/config.txt (Pi 4) or /boot/firmware/config.txt (Pi 5):

    enable_uart=1
    dtoverlay=disable-bt   # Frees ttyAMA0 from Bluetooth

Then:
    sudo systemctl disable hciuart
    sudo reboot

Confirm after reboot:
    ls -la /dev/ttyAMA0   # must exist
    cat /proc/device-tree/aliases/uart0  # should show serial0

### Step 2.5 — Build

    cd ~/ros2_ws
    colcon build --packages-select arm_hardware --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash

If build fails with "No such file protocol.hpp":
  Check the include path in CMakeLists.txt matches your directory layout.

### Step 2.6 — Verify plugin is discoverable

    ros2 run pluginlib_test test_pluginlib \
      hardware_interface::SystemInterface \
      arm_hardware/ArmHardwareInterface

Should print: "Plugin successfully loaded!"

══════════════════════════════════════════════════════════════════════════════
## PART 3 — URDF and controller config wiring
══════════════════════════════════════════════════════════════════════════════

### Step 3.1 — Add the ros2_control block to your URDF

Open your arm URDF (e.g. v2claw.urdf or v2claw.urdf.xacro).
Paste the contents of config/ros2_control.urdf.xacro inside the <robot> tag.

CRITICAL: Joint names in the <ros2_control> block MUST exactly match the
<joint name="..."> tags in the rest of your URDF.  Case-sensitive.

### Step 3.2 — Set real joint limits in the URDF

In the <joint> definitions (not the ros2_control block), set:

    <limit lower="0.827" upper="1.292" effort="10" velocity="1.0"/>

Use the same radian values as in JOINT_LIMITS in protocol.hpp.
MoveIt2's IK solver uses these limits. If they're wrong, IK will find
solutions the firmware will reject.

### Step 3.3 — Copy controllers.yaml to your MoveIt2 package

    cp arm_hardware/config/controllers.yaml \
       ~/ros2_ws/src/your_moveit_config/config/

### Step 3.4 — Launch controller_manager with your URDF

Minimal launch to test WITHOUT MoveIt2 first:

    ros2 launch ros2_control_demo_example_1 view_robot.launch.py \
      robot_description:="$(cat your_arm.urdf)"

Or write a launch file:

    # arm_control.launch.py (minimal)
    from launch import LaunchDescription
    from launch_ros.actions import Node
    import os
    from ament_index_python.packages import get_package_share_directory

    def generate_launch_description():
        urdf = open('path/to/your_arm.urdf').read()
        controllers_yaml = os.path.join(
            get_package_share_directory('arm_hardware'),
            'config', 'controllers.yaml')

        return LaunchDescription([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': urdf}]
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[
                    {'robot_description': urdf},
                    controllers_yaml
                ]
            ),
        ])

### Step 3.5 — Load and activate controllers

In a new terminal (after launching ros2_control_node):

    # Load joint state broadcaster (always first)
    ros2 control load_controller --set-state active joint_state_broadcaster

    # Load arm trajectory controller
    ros2 control load_controller --set-state active arm_controller

    # Verify both are active
    ros2 control list_controllers

Expected output:
    joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    arm_controller[joint_trajectory_controller/JointTrajectoryController] active

### Step 3.6 — Verify joint states are publishing

    ros2 topic echo /joint_states

You should see real joint positions from the STM32 updating at ~50 Hz.
If positions are stuck at 0.0, the feedback is not arriving — check UART.

══════════════════════════════════════════════════════════════════════════════
## PART 4 — Testing before MoveIt2
══════════════════════════════════════════════════════════════════════════════

### Step 4.1 — Send a single joint trajectory command

    ros2 action send_goal /arm_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory \
      "{
        trajectory: {
          joint_names: [m1_joint, m2_joint, m3_joint, pitch_joint, roll_joint, z_joint],
          points: [{
            positions: [1.05, 0.5, 0.0, 0.0, 0.0, 0.0],
            velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            time_from_start: {sec: 3, nanosec: 0}
          }]
        }
      }"

The arm should move slowly to that position over 3 seconds.

### Step 4.2 — Watch for faults

    ros2 topic echo /joint_states | grep -E "position|velocity"

If pitch or Z report 0.0 and frozen, the fault flags are active.
Deactivate and re-activate the hardware interface to reset:

    ros2 control set_controller_state arm_controller inactive
    ros2 control set_controller_state arm_controller active

### Step 4.3 — Check diagnostics in the log

    ros2 launch arm_hardware arm_control.launch.py --ros-args --log-level DEBUG

Look for:
  "Commands seeded from first feedback"  ← good, no cold-start jump
  "Checksum mismatch"                    ← cable noise, fix UART wiring
  "PITCH FROZEN"                         ← safety system triggered
  "Z-AXIS COLLISION"                     ← safety system triggered
  "UART write failed"                    ← serial port disconnected

══════════════════════════════════════════════════════════════════════════════
## PART 5 — MoveIt2 integration (after Part 4 works)
══════════════════════════════════════════════════════════════════════════════

### Step 5.1 — Run MoveIt2 Setup Assistant

    ros2 launch moveit_setup_assistant setup_assistant.launch.py

Load your URDF.
Define your planning group (e.g. "arm") including all 6 joints.
Set end effector if you have one.
Generate the moveit_config package.

### Step 5.2 — Add the controller config to MoveIt2

In your moveit_config package, edit moveit_controllers.yaml:

    moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

    controller_names:
      - arm_controller

    arm_controller:
      action_ns: follow_joint_trajectory
      type: FollowJointTrajectory
      default: true
      joints:
        - m1_joint
        - m2_joint
        - m3_joint
        - pitch_joint
        - roll_joint
        - z_joint

### Step 5.3 — Launch MoveIt2 with your hardware

    ros2 launch your_moveit_config move_group.launch.py

In RViz, add the MotionPlanning plugin.
Set start state to "current" (reads from /joint_states).
Set a goal state.
Click Plan, then Execute.

══════════════════════════════════════════════════════════════════════════════
## PART 6 — Common problems and fixes
══════════════════════════════════════════════════════════════════════════════

PROBLEM: /load_controller never responds (hangs indefinitely)
FIX:     rx_thread_ is running at FIFO priority, starving the CM service
         threads. The on_configure() in this code explicitly sets SCHED_OTHER.
         If you're still seeing this, verify with:
           ps -eLo pid,cls,rtprio,comm | grep arm_hardware

PROBLEM: Arm jumps to 0 on first command
FIX:     commands_seeded_ flag prevents this. If it still happens, check
         that on_activate() wait path is working (look for "Commands seeded"
         in the log). Never appears in this implementation if the STM32
         is sending feedback before activation.

PROBLEM: JTC immediately aborts with "goal tolerance violated"
FIX:     Loosen constraints in controllers.yaml (goal: 0.1 instead of 0.05).
         Then tighten gradually. Also check velocity feedback — if it's
         always 0, the JTC may see a "stopped" state incorrectly.

PROBLEM: STM32 ignores commands (2s timeout keeps triggering)
FIX:     New packet is 29 bytes. If your STM32 parseCommandPacket() still
         expects 26 bytes, it will never match. Recheck STEP 1.1 and 1.3.
         Confirm with: sizeof(CommandPacketUnified) == 29 in a static_assert.

PROBLEM: Checksum failures logged constantly
FIX:     UART noise. Check:
           - Cable length < 1m for bare UART, < 3m for RS-485
           - Ground shared between STM32 and Pi
           - Not running 3.3V UART into a 5V input
           - Pi GPIO UART baud matches STM32 baud exactly
         Temporary workaround: tolerate checksum errors (already implemented)
         but fix the root cause before URC.

PROBLEM: Velocity is always 0 in /joint_states
FIX:     STM32 is sending 0 in motor_vel[]. Check STEP 5 — the velocity
         variables must be computed before the feedback transmission block.
         The fallback numerical differentiation in read() will activate
         automatically, but it's less accurate than STM32-side computation.
