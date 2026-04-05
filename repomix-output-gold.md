This file is a merged representation of a subset of the codebase, containing specifically included files and files not matching ignore patterns, combined into a single document by Repomix.

<file_summary>
This section contains a summary of this file.

<purpose>
This file contains a packed representation of a subset of the repository's contents that is considered the most important context.
It is designed to be easily consumable by AI systems for analysis, code review,
or other automated processes.
</purpose>

<file_format>
The content is organized as follows:
1. This summary section
2. Repository information
3. Directory structure
4. Repository files (if enabled)
5. Multiple file entries, each consisting of:
  - File path as an attribute
  - Full contents of the file
</file_format>

<usage_guidelines>
- This file should be treated as read-only. Any changes should be made to the
  original repository files, not this packed version.
- When processing this file, use the file path to distinguish
  between different files in the repository.
- Be aware that this file may contain sensitive information. Handle it with
  the same level of security as you would the original repository.
</usage_guidelines>

<notes>
- Some files may have been excluded based on .gitignore rules and Repomix's configuration
- Binary files are not included in this packed representation. Please refer to the Repository Structure section for a complete list of file paths, including binary files
- Only files matching these patterns are included: Core/Inc/**/*, Core/Src/**/*, USB_DEVICE/App/**/*, arm_hardware_complete (1)/**/*, *.py
- Files matching these patterns are excluded: Drivers/**/*, Middlewares/**/*, .git/**/*, *.csv, *.sid
- Files matching patterns in .gitignore are excluded
- Files matching default ignore patterns are excluded
- Files are sorted by Git change count (files with more changes are at the bottom)
</notes>

</file_summary>

<directory_structure>
arm_hardware_complete (1)/INTEGRATION_GUIDE.md
arm_hardware_complete (1)/src/arm_hardware/arm_hardware_plugin.xml
arm_hardware_complete (1)/src/arm_hardware/CMakeLists.txt
arm_hardware_complete (1)/src/arm_hardware/config/controllers.yaml
arm_hardware_complete (1)/src/arm_hardware/config/ros2_control.urdf.xacro
arm_hardware_complete (1)/src/arm_hardware/include/arm_hardware/arm_hardware_interface.hpp
arm_hardware_complete (1)/src/arm_hardware/include/arm_hardware/protocol.hpp
arm_hardware_complete (1)/src/arm_hardware/include/arm_hardware/uart_driver.hpp
arm_hardware_complete (1)/src/arm_hardware/package.xml
arm_hardware_complete (1)/src/arm_hardware/src/arm_hardware_interface.cpp
arm_hardware_complete (1)/src/arm_hardware/src/uart_driver.cpp
arm_hardware_complete (1)/src/arm_hardware/stm32_patch/RobotCore_protocol_patch.h
calibrate_all.py
calibrate_pitch_dir.py
calibrate_pots.py
calibrate_wrist.py
check_z_encoder_dir.py
Core/Inc/adc.h
Core/Inc/dma.h
Core/Inc/FreeRTOSConfig.h
Core/Inc/gpio.h
Core/Inc/i2c.h
Core/Inc/main.h
Core/Inc/RobotConfig.h
Core/Inc/RobotCore.h
Core/Inc/spi.h
Core/Inc/stm32f4xx_hal_conf.h
Core/Inc/stm32f4xx_it.h
Core/Inc/tim.h
Core/Inc/usart.h
Core/Src/adc.c
Core/Src/dma.c
Core/Src/freertos.c
Core/Src/gpio.c
Core/Src/i2c.c
Core/Src/main.c
Core/Src/RobotCore.cpp
Core/Src/spi.c
Core/Src/stm32f4xx_hal_msp.c
Core/Src/stm32f4xx_hal_timebase_tim.c
Core/Src/stm32f4xx_it.c
Core/Src/syscalls.c
Core/Src/sysmem.c
Core/Src/system_stm32f4xx.c
Core/Src/tim.c
Core/Src/usart.c
debug_com10.py
pitch_diagnostic.py
read_encoders.py
read_limp.py
step_response_logger.py
test_usb_arm.py
USB_DEVICE/App/usb_device.c
USB_DEVICE/App/usb_device.h
USB_DEVICE/App/usbd_cdc_if.c
USB_DEVICE/App/usbd_cdc_if.h
USB_DEVICE/App/usbd_desc.c
USB_DEVICE/App/usbd_desc.h
z_step_logger.py
</directory_structure>

<files>
This section contains the contents of the repository's files.

<file path="arm_hardware_complete (1)/INTEGRATION_GUIDE.md">
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
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/arm_hardware_plugin.xml">
<library path="arm_hardware">
  <class name="arm_hardware/ArmHardwareInterface"
         type="arm_hardware::ArmHardwareInterface"
         base_class_type="hardware_interface::SystemInterface">
    <description>
      ros2_control hardware interface for the V2CLAW 6-DOF arm.
      Communicates with STM32F407 over UART (default /dev/ttyAMA0, 115200 baud).
    </description>
  </class>
</library>
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/CMakeLists.txt">
cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

# Require C++17 — needed for std::array initialiser, structured bindings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib          REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclcpp_lifecycle   REQUIRED)

# ── Library ───────────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/arm_hardware_interface.cpp
  src/uart_driver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Link pthread — needed for pthread_setschedparam in on_configure
target_link_libraries(${PROJECT_NAME} pthread)

# Export pluginlib plugin description
pluginlib_export_plugin_description_file(hardware_interface arm_hardware_plugin.xml)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES arm_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/config/controllers.yaml">
# =============================================================================
# controllers.yaml
# ros2_control controller configuration for the V2CLAW arm.
#
# Load order:
#   1. joint_state_broadcaster  — publishes /joint_states (needed by MoveIt2)
#   2. arm_controller           — JointTrajectoryController (MoveIt2 talks here)
#
# Tune goal_time_tolerance and goal_joint_tolerance to your arm's performance.
# Start loose (0.1 rad), tighten after real motion testing.
# =============================================================================

controller_manager:
  ros__parameters:
    update_rate: 100  # Hz — must match your realtime kernel capability

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_state_broadcaster:
  ros__parameters:
    joints:
      - m1_joint
      - m2_joint
      - m3_joint
      - pitch_joint
      - roll_joint
      - z_joint

arm_controller:
  ros__parameters:
    joints:
      - m1_joint
      - m2_joint
      - m3_joint
      - pitch_joint
      - roll_joint
      - z_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    # Allow JTC to start even if current position is within goal tolerance.
    allow_partial_joints_goal: false

    # Trajectory goal tolerances (tighten after tuning)
    goal_time_tolerance: 1.0       # seconds extra after trajectory end time
    constraints:
      stopped_velocity_tolerance: 0.05  # rad/s — considered "stopped"
      m1_joint:    { goal: 0.05 }  # radians
      m2_joint:    { goal: 0.05 }
      m3_joint:    { goal: 0.05 }
      pitch_joint: { goal: 0.05 }
      roll_joint:  { goal: 0.05 }
      z_joint:     { goal: 0.05 }

    # Interpolation: spline gives smooth velocity profiles
    # Options: none | linear | spline
    interpolation_method: spline

    # How long to wait for STM32 feedback before declaring hardware failure
    # Set to ~3× your UART frame period (20ms × 3 = 60ms)
    state_publish_rate: 50.0       # Hz (half control rate is fine)
    action_monitor_rate: 20.0      # Hz for goal monitoring
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/config/ros2_control.urdf.xacro">
<!--
  ros2_control snippet for v2claw.urdf.xacro
  
  This block declares the hardware plugin and all 6 joints to ros2_control.
  Paste this INSIDE your <robot> tag in the URDF (or include it as a xacro).
  
  Joint names MUST match:
    - The <joint> names in the rest of your URDF
    - The joint names in controllers.yaml
    - The order in JOINT_LIMITS in protocol.hpp (index 0..5)
  
  Joint limits MUST match protocol.hpp JOINT_LIMITS exactly.
  If they disagree, MoveIt2 will plan to a position the firmware will clamp.
-->

<ros2_control name="arm_hardware" type="system">

  <hardware>
    <plugin>arm_hardware/ArmHardwareInterface</plugin>
    <!-- Serial port connected to STM32 UART4 (PC10/PC11) via USB-UART or GPIO -->
    <param name="serial_port">/dev/ttyAMA0</param>
    <param name="baud_rate">115200</param>
  </hardware>

  <!-- ── Joint 0: M1 — shoulder rotation (potentiometer) ─────────────────── -->
  <joint name="m1_joint">
    <command_interface name="position">
      <param name="min">0.827</param>   <!-- 47.4 deg in radians -->
      <param name="max">1.292</param>   <!-- 74.0 deg in radians -->
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- ── Joint 1: M2 — shoulder lift (potentiometer) ──────────────────────── -->
  <joint name="m2_joint">
    <command_interface name="position">
      <param name="min">0.017</param>   <!-- 1.0 deg -->
      <param name="max">1.068</param>   <!-- 61.2 deg -->
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- ── Joint 2: M3 — elbow (AS5600, multi-turn) ──────────────────────────── -->
  <joint name="m3_joint">
    <command_interface name="position">
      <param name="min">-3.14159</param>  <!-- ±180 deg — adjust to your M3_OUTPUT_LIMIT -->
      <param name="max"> 3.14159</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- ── Joint 3: Pitch — wrist pitch (AS5600, differential output) ─────────── -->
  <joint name="pitch_joint">
    <command_interface name="position">
      <param name="min">-1.5708</param>  <!-- ±90 deg — adjust to your PITCH_RANGE_DEG -->
      <param name="max"> 1.5708</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- ── Joint 4: Roll — wrist roll (AS5600, differential output) ──────────── -->
  <joint name="roll_joint">
    <command_interface name="position">
      <param name="min">-3.14159</param>  <!-- ±180 deg — adjust to your ROLL_RANGE_DEG -->
      <param name="max"> 3.14159</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- ── Joint 5: Z — linear axis (AS5600, multi-turn) ─────────────────────── -->
  <joint name="z_joint">
    <command_interface name="position">
      <param name="min">-0.300</param>   <!-- meters — adjust to your ZA_OUTPUT_LIMIT -->
      <param name="max"> 0.300</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

</ros2_control>

<!--
  MoveIt2 SRDF note on the differential wrist
  ─────────────────────────────────────────────
  pitch_joint and roll_joint appear as independent joints to MoveIt2.
  The differential mixing (mA = pitch + roll, mB = -pitch + roll) happens
  inside the STM32 firmware.  This is transparent to MoveIt2 as long as:
    1. Your URDF kinematics model uses pitch/roll as independent joints
       (which is correct for the OUTPUT axes, not the motor axes).
    2. You do NOT add the two physical motors as URDF joints — only the
       output axes (pitch, roll) appear in the kinematic chain.
  
  If you have a different wrist geometry where the gear ratio between
  motor and output axis is not 1:1, apply that ratio inside the STM32
  (which you already do via PITCH_RANGE_DEG and ROLL_RANGE_DEG) and
  let MoveIt2 see only the output axis in degrees.
-->
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/include/arm_hardware/arm_hardware_interface.hpp">
#pragma once
// =============================================================================
// arm_hardware_interface.hpp
// ros2_control SystemInterface for the V2CLAW 6-DOF arm.
//
// Architecture
// ────────────
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │  controller_manager  (SCHED_FIFO 50, 100 Hz)                        │
//  │   ├── JointTrajectoryController  → write()  → pack_command()        │
//  │   └── JointStateBroadcaster     ← read()   ← rx_buffer_            │
//  │                           ▲                                          │
//  │                     mutex + atomic                                   │
//  │                           │                                          │
//  │  rx_thread_  (SCHED_OTHER, runs continuously)                        │
//  │   └── uart_.read_bytes() → frame sync → checksum → rx_buffer_       │
//  └──────────────────────────────────────────────────────────────────────┘
//
//  read() / write() are called on the CM's RT thread — they MUST NOT block.
//  rx_thread_ is NOT real-time — it does blocking UART I/O in the background.
//
// Lifecycle
// ─────────
//   on_init      → validate joints, read params
//   on_configure → open UART, start rx_thread_
//   on_activate  → seed positions, set commands = positions (no jump)
//   on_deactivate→ send 3× ESTOP, halt motors
//   on_cleanup   → stop rx_thread_, close UART
//   on_error     → ESTOP + clean shutdown
// =============================================================================

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <array>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arm_hardware/protocol.hpp"
#include "arm_hardware/uart_driver.hpp"

namespace arm_hardware {

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardwareInterface)

    // ── Lifecycle ──────────────────────────────────────────────────────────

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state) override;

    // ── ros2_control interface export ──────────────────────────────────────

    std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

    // ── RT callbacks (called at control loop rate, must not block) ─────────

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    // ── State & command buffers (owned by ros2_control) ────────────────────
    // Indexed 0..NUM_JOINTS-1, matching JOINT_LIMITS order in protocol.hpp.
    std::vector<double> hw_positions_;   // radians — filled by read()
    std::vector<double> hw_velocities_;  // rad/s   — filled by read()
    std::vector<double> hw_commands_;    // radians — set by JTC, read by write()

    // Previous positions for numerical differentiation (velocity estimation)
    std::array<double, NUM_JOINTS> prev_positions_{};
    double prev_read_time_{0.0};

    // ── UART ───────────────────────────────────────────────────────────────
    UartDriver  uart_;
    std::string serial_port_;
    int         baud_rate_{115200};

    // ── RX thread ──────────────────────────────────────────────────────────
    std::thread       rx_thread_;
    std::atomic<bool> rx_running_{false};

    // rx_buffer_ is written by rx_thread_ and read by read().
    // Protected by rx_mutex_.  rx_valid_ is set AFTER the lock is released
    // so read() can check it cheaply without taking the lock first.
    std::mutex        rx_mutex_;
    ParsedFeedback    rx_buffer_;
    std::atomic<bool> rx_valid_{false};

    // Consecutive checksum failure counter (logged, not fatal by itself)
    int rx_checksum_failures_{0};

    // ── Activation state ───────────────────────────────────────────────────
    // Set true in on_activate ONLY after hw_commands_ has been seeded from
    // real feedback.  write() skips transmission until this is true to
    // prevent a 0-rad command burst before the first feedback arrives.
    std::atomic<bool> commands_seeded_{false};

    // Joint fault state (latched — cleared on deactivate/reconnect)
    bool pitch_fault_{false};
    bool z_fault_{false};

    // ── Diagnostics ────────────────────────────────────────────────────────
    uint64_t tx_count_{0};
    uint64_t rx_count_{0};
    uint64_t rx_frame_errors_{0};

    // ── Private methods ────────────────────────────────────────────────────

    // RX thread body — blocking UART I/O + frame parsing
    void rx_thread_fn();

    // Send ESTOP frame (best-effort, no return value used for safety sends)
    bool send_estop();

    // Clean shutdown of rx_thread_ + UART (used by cleanup and error handlers)
    void shutdown_io();
};

} // namespace arm_hardware
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/include/arm_hardware/protocol.hpp">
#pragma once
// =============================================================================
// protocol.hpp
// Single source of truth for the STM32 <-> ROS2 wire protocol.
//
// PACKET LAYOUT
// ─────────────
// CommandPacketUnified  (ROS2 → STM32):  29 bytes
//   [0]      header[0]   'S' (position) | 'P' (estop/open-loop)
//   [1]      header[1]   'T'
//   [2..25]  motor_cmd[6] int32_t × 6   little-endian milli-degrees
//   [26]     flags        uint8_t        bit0=ESTOP, bit1=reserved
//   [27]     checksum     XOR of [0..26]
//   [28]     footer       '\n'
//
// FeedbackPacketUnified (STM32 → ROS2):  52 bytes
//   [0]      header[0]   'F'
//   [1]      header[1]   'B'
//   [2..25]  motor_pos[6] int32_t × 6   little-endian milli-degrees
//   [26..49] motor_vel[6] int32_t × 6   little-endian milli-deg/s
//   [50]     flags        uint8_t        bit0=pitch_frozen, bit1=z_collision
//   [51]     checksum     XOR of [0..50]  -- NOTE: no footer, checksum IS last byte
//
// JOINT INDEX MAP
//   0  M1    potentiometer  shoulder rotation
//   1  M2    potentiometer  shoulder lift
//   2  M3    AS5600         elbow (multi-turn)
//   3  Pitch AS5600         wrist pitch  (differential output)
//   4  Roll  AS5600         wrist roll   (differential output)
//   5  Z     AS5600         linear axis  (multi-turn)
//
// SENTINEL VALUE
//   999999 in motor_pos means that joint is in a fault state (frozen/collision).
//   The flags byte also carries this info.  ROS2 side MUST check flags byte;
//   sentinel in motor_pos is provided as a secondary human-readable indicator.
//
// CHECKSUM
//   XOR of all payload bytes preceding the checksum byte.
//   Both sender and receiver compute independently and compare.
// =============================================================================

#include <array>
#include <cstdint>
#include <cstring>

namespace arm_hardware {

// ── Constants ─────────────────────────────────────────────────────────────────

static constexpr size_t NUM_JOINTS    = 6;     // physical joints on the wire
static constexpr int32_t SENTINEL_VAL = 999999; // fault marker in motor_pos

// Command flags byte
static constexpr uint8_t FLAG_ESTOP = (1u << 0);

// Feedback flags byte
static constexpr uint8_t FLAG_PITCH_FROZEN  = (1u << 0);
static constexpr uint8_t FLAG_Z_COLLISION   = (1u << 1);

// ── Wire structs ──────────────────────────────────────────────────────────────
// #pragma pack ensures no padding bytes are inserted by the compiler.
// Both ARM Cortex-M4 (STM32) and ARM Cortex-A (Raspberry Pi) are
// little-endian, so int32_t byte order is identical on both sides.

#pragma pack(push, 1)

struct CommandPacketUnified {
    char    header[2];      // 'S','T' or 'P','T'
    int32_t motor_cmd[6];   // milli-degrees per joint
    uint8_t flags;          // FLAG_ESTOP etc.
    uint8_t checksum;       // XOR([0..26])
    char    footer;         // '\n'
};
static_assert(sizeof(CommandPacketUnified) == 29,
    "CommandPacketUnified size mismatch — check pack(push,1)");

struct FeedbackPacketUnified {
    char    header[2];      // 'F','B'
    int32_t motor_pos[6];   // milli-degrees (or SENTINEL_VAL on fault)
    int32_t motor_vel[6];   // milli-deg/s
    uint8_t flags;          // FLAG_PITCH_FROZEN | FLAG_Z_COLLISION
    uint8_t checksum;       // XOR([0..50])
};
static_assert(sizeof(FeedbackPacketUnified) == 52,
    "FeedbackPacketUnified size mismatch — check pack(push,1)");

#pragma pack(pop)

// ── Checksum helpers ──────────────────────────────────────────────────────────

inline uint8_t compute_checksum(const uint8_t* data, size_t len)
{
    uint8_t xor_val = 0;
    for (size_t i = 0; i < len; i++) xor_val ^= data[i];
    return xor_val;
}

inline bool verify_feedback_checksum(const FeedbackPacketUnified& f)
{
    // checksum covers everything before the checksum field itself
    const size_t cov = offsetof(FeedbackPacketUnified, checksum);
    return f.checksum == compute_checksum(
        reinterpret_cast<const uint8_t*>(&f), cov);
}

inline void sign_command_checksum(CommandPacketUnified& f)
{
    const size_t cov = offsetof(CommandPacketUnified, checksum);
    f.checksum = compute_checksum(
        reinterpret_cast<const uint8_t*>(&f), cov);
}

// ── Parsed representation ─────────────────────────────────────────────────────
// Used inside the ROS2 hardware interface after unpacking + validating a frame.

struct ParsedFeedback {
    std::array<double, NUM_JOINTS> positions{};   // radians
    std::array<double, NUM_JOINTS> velocities{};  // rad/s
    bool pitch_frozen{false};
    bool z_collision{false};
    bool checksum_ok{true};
};

// ── Joint software limits (radians) ───────────────────────────────────────────
// These MUST match your URDF <limit lower="" upper=""/> values.
// pack_cmd() enforces them as a defence-in-depth clamp before transmission.
// Adjust to your actual mechanical travel.

struct JointLimit { double lower; double upper; };

static constexpr std::array<JointLimit, NUM_JOINTS> JOINT_LIMITS = {{
    {  0.827,  1.292 },  // M1    47.4° .. 74.0°   (pot, degrees → radians)
    {  0.017,  1.068 },  // M2     1.0° .. 61.2°   (pot)
    { -3.142,  3.142 },  // M3    ±180°             (multi-turn elbow)
    { -1.571,  1.571 },  // Pitch ±90°              (wrist)
    { -3.142,  3.142 },  // Roll  ±180°             (wrist)
    { -0.300,  0.300 },  // Z     ±0.3 m linear     (adjust to your axis)
}};

// ── pack / unpack ─────────────────────────────────────────────────────────────

// Convert radians → milli-degrees, clamp to limits, fill + sign a command frame.
inline CommandPacketUnified pack_command(
    const std::array<double, NUM_JOINTS>& positions_rad,
    bool estop = false)
{
    CommandPacketUnified f{};
    f.header[0] = estop ? 'P' : 'S';
    f.header[1] = 'T';
    f.flags     = estop ? FLAG_ESTOP : 0u;

    for (size_t i = 0; i < NUM_JOINTS; i++) {
        double pos = positions_rad[i];
        // Clamp to software limits (defence-in-depth — URDF should already do this)
        if (pos < JOINT_LIMITS[i].lower) pos = JOINT_LIMITS[i].lower;
        if (pos > JOINT_LIMITS[i].upper) pos = JOINT_LIMITS[i].upper;
        double deg = pos * (180.0 / M_PI);
        f.motor_cmd[i] = static_cast<int32_t>(deg * 1000.0);
    }
    f.footer = '\n';
    sign_command_checksum(f);
    return f;
}

// Unpack a feedback frame into SI units (radians, rad/s).
// Returns false if header/footer/checksum invalid — caller should discard frame.
inline bool unpack_feedback(const FeedbackPacketUnified& f, ParsedFeedback& out)
{
    out = ParsedFeedback{};

    if (f.header[0] != 'F' || f.header[1] != 'B') return false;

    out.checksum_ok = verify_feedback_checksum(f);
    // Log but do NOT hard-reject on bad checksum — let caller decide.
    // We still populate the struct so the caller can see the bad data.

    out.pitch_frozen = (f.flags & FLAG_PITCH_FROZEN) != 0;
    out.z_collision  = (f.flags & FLAG_Z_COLLISION)  != 0;

    for (size_t i = 0; i < NUM_JOINTS; i++) {
        // Sentinel: joint in fault — position is unreliable, zero it.
        if (f.motor_pos[i] == SENTINEL_VAL) {
            out.positions[i]  = 0.0;
            out.velocities[i] = 0.0;
            continue;
        }
        out.positions[i]  = (f.motor_pos[i] / 1000.0) * (M_PI / 180.0);
        out.velocities[i] = (f.motor_vel[i] / 1000.0) * (M_PI / 180.0);
    }
    return true;
}

} // namespace arm_hardware
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/include/arm_hardware/uart_driver.hpp">
#pragma once
// =============================================================================
// uart_driver.hpp
// POSIX blocking UART with deadline-based read, non-blocking write with retry.
// Thread-safe: read and write may be called from different threads concurrently
// because POSIX guarantees that read/write on an fd are atomic per-call.
// close() is NOT thread-safe with concurrent read/write — caller must ensure
// threads are stopped before calling close().
// =============================================================================

#include <cstdint>
#include <string>

namespace arm_hardware {

class UartDriver {
public:
    UartDriver()  = default;
    ~UartDriver() { close(); }

    // Non-copyable, non-movable (owns an fd)
    UartDriver(const UartDriver&)            = delete;
    UartDriver& operator=(const UartDriver&) = delete;

    // Open and configure a real serial port (8N1, raw mode, no flow control).
    // Supported baud rates: 115200, 460800, 921600.
    bool open(const std::string& port, int baud_rate);

    // Inject a pre-opened fd (e.g. one end of socketpair() for unit tests).
    // Does NOT take ownership — caller is responsible for closing fd.
    bool open_fd(int fd);

    void close();
    bool is_open() const { return fd_ >= 0; }

    // Write exactly len bytes.  Retries on EAGAIN up to 1 second.
    // Returns false on hard error or timeout.
    bool write_bytes(const uint8_t* data, size_t len);

    // Read exactly len bytes within timeout_ms milliseconds.
    // Uses CLOCK_MONOTONIC deadline to handle partial reads correctly.
    // Returns false on timeout or hard error.
    bool read_bytes(uint8_t* buf, size_t len, int timeout_ms);

    // Flush the receive buffer (discard stale bytes after reconnect).
    void flush_rx();

private:
    int  fd_{-1};
    bool owns_fd_{false};

    static speed_t to_speed(int baud_rate);
};

} // namespace arm_hardware
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/package.xml">
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm_hardware</name>
  <version>1.0.0</version>
  <description>
    ros2_control hardware interface for the V2CLAW 6-DOF arm.
    Communicates with STM32F407 over UART at 115200 baud.
  </description>
  <maintainer email="your@email.com">Rohit</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/src/arm_hardware_interface.cpp">
// =============================================================================
// arm_hardware_interface.cpp
// =============================================================================
#include "arm_hardware/arm_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <chrono>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <thread>

PLUGINLIB_EXPORT_CLASS(
    arm_hardware::ArmHardwareInterface,
    hardware_interface::SystemInterface)

namespace arm_hardware {

static rclcpp::Logger LOGGER = rclcpp::get_logger("ArmHardwareInterface");

// =============================================================================
// on_init — validate URDF joints + read parameters
// =============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info.joints.size() != NUM_JOINTS) {
        RCLCPP_FATAL(LOGGER,
            "Expected %zu joints in URDF, got %zu. "
            "Check your ros2_control tag.",
            NUM_JOINTS, info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate that every joint has position command + position/velocity state
    for (const auto & joint : info.joints) {
        bool has_pos_cmd = false, has_pos_st = false, has_vel_st = false;
        for (const auto & ci : joint.command_interfaces)
            if (ci.name == hardware_interface::HW_IF_POSITION) has_pos_cmd = true;
        for (const auto & si : joint.state_interfaces) {
            if (si.name == hardware_interface::HW_IF_POSITION) has_pos_st = true;
            if (si.name == hardware_interface::HW_IF_VELOCITY)  has_vel_st = true;
        }
        if (!has_pos_cmd || !has_pos_st || !has_vel_st) {
            RCLCPP_FATAL(LOGGER,
                "Joint '%s' is missing required interfaces. "
                "Need: position command, position+velocity state.",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // Hardware parameters from URDF <hardware> block
    serial_port_ = info.hardware_parameters.count("serial_port")
        ? info.hardware_parameters.at("serial_port") : "/dev/ttyAMA0";
    baud_rate_   = info.hardware_parameters.count("baud_rate")
        ? std::stoi(info.hardware_parameters.at("baud_rate")) : 115200;

    // Allocate interface buffers
    hw_positions_.assign(NUM_JOINTS, 0.0);
    hw_velocities_.assign(NUM_JOINTS, 0.0);
    hw_commands_.assign(NUM_JOINTS, 0.0);
    prev_positions_.fill(0.0);

    RCLCPP_INFO(LOGGER,
        "on_init OK — port=%s baud=%d joints=%zu",
        serial_port_.c_str(), baud_rate_, NUM_JOINTS);

    return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// export_state_interfaces / export_command_interfaces
// =============================================================================
std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> v;
    for (size_t i = 0; i < NUM_JOINTS; i++) {
        v.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
        v.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,  &hw_velocities_[i]);
    }
    return v;
}

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> v;
    for (size_t i = 0; i < NUM_JOINTS; i++) {
        v.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
    }
    return v;
}

// =============================================================================
// on_configure — open UART, start rx_thread_ at SCHED_OTHER
// =============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State &)
{
    if (!uart_.open(serial_port_, baud_rate_)) {
        RCLCPP_FATAL(LOGGER,
            "Failed to open %s at %d baud. "
            "Check: ls -la /dev/ttyAMA0, sudo usermod -aG dialout $USER",
            serial_port_.c_str(), baud_rate_);
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(LOGGER, "UART opened: %s @ %d", serial_port_.c_str(), baud_rate_);

    // Flush any stale bytes from a previous session
    uart_.flush_rx();

    rx_running_.store(true, std::memory_order_relaxed);
    rx_valid_.store(false, std::memory_order_relaxed);
    rx_thread_ = std::thread(&ArmHardwareInterface::rx_thread_fn, this);

    // CRITICAL: rx_thread_ MUST run at SCHED_OTHER (normal priority).
    // controller_manager runs its update loop at SCHED_FIFO 50.
    // If rx_thread_ inherits or is elevated to FIFO, it can starve the CM's
    // service handler threads (which run at SCHED_OTHER), causing
    // /load_controller and /activate_controller services to never respond.
    struct sched_param param{};
    param.sched_priority = 0;
    if (pthread_setschedparam(rx_thread_.native_handle(), SCHED_OTHER, &param) != 0) {
        RCLCPP_WARN(LOGGER, "Could not set rx_thread to SCHED_OTHER (non-fatal)");
    }

    RCLCPP_INFO(LOGGER, "RX thread started");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// rx_thread_fn — runs continuously, parses FB frames from STM32
//
// Frame sync state machine:
//   HUNT_F → HUNT_B → READ_PAYLOAD
//
// Why a state machine instead of two sequential read_bytes(1)?
// If noise injects a byte mid-stream, the naive approach reads 'F' then the
// noise byte (not 'B'), discards it, and then reads the real 'B' as the first
// payload byte — corrupting the rest of the frame.  The state machine always
// re-hunts from the same position so the frame boundary is always found
// correctly even after partial corruption.
// =============================================================================
void ArmHardwareInterface::rx_thread_fn()
{
    enum class SyncState { HUNT_F, HUNT_B, READ_PAYLOAD };
    SyncState state = SyncState::HUNT_F;

    // Raw frame buffer — header bytes separately, payload as one block
    uint8_t hdr[2];
    uint8_t payload[sizeof(FeedbackPacketUnified) - 2]; // everything after header

    while (rx_running_.load(std::memory_order_relaxed)) {
        uint8_t b = 0;

        switch (state) {

        case SyncState::HUNT_F:
            // Short timeout so rx_running_ is checked frequently
            if (!uart_.read_bytes(&b, 1, 10)) break;
            if (b == 'F') {
                hdr[0] = 'F';
                state  = SyncState::HUNT_B;
            }
            // Any other byte: stay in HUNT_F
            break;

        case SyncState::HUNT_B:
            if (!uart_.read_bytes(&b, 1, 10)) {
                state = SyncState::HUNT_F;  // timeout — restart hunt
                break;
            }
            if (b == 'B') {
                hdr[1] = 'B';
                state  = SyncState::READ_PAYLOAD;
            } else if (b == 'F') {
                // Could be start of a new frame — stay in HUNT_B
                hdr[0] = 'F';
            } else {
                state = SyncState::HUNT_F;  // garbage — restart
            }
            break;

        case SyncState::READ_PAYLOAD: {
            // Read everything after the 2-byte header in one shot.
            // Timeout: one full STM32 control period (20ms) + margin = 50ms.
            if (!uart_.read_bytes(payload, sizeof(payload), 50)) {
                rx_frame_errors_++;
                state = SyncState::HUNT_F;
                break;
            }

            // Reconstruct full struct
            FeedbackPacketUnified frame{};
            frame.header[0] = 'F';
            frame.header[1] = 'B';
            std::memcpy(reinterpret_cast<uint8_t*>(&frame) + 2,
                        payload, sizeof(payload));

            // Unpack + validate
            ParsedFeedback parsed;
            if (!unpack_feedback(frame, parsed)) {
                rx_frame_errors_++;
                static rclcpp::Clock clk(RCL_STEADY_TIME);
                RCLCPP_WARN_THROTTLE(LOGGER, clk, 2000,
                    "Frame parse failed (header/checksum) — "
                    "total frame errors: %lu", rx_frame_errors_);
                state = SyncState::HUNT_F;
                break;
            }

            if (!parsed.checksum_ok) {
                rx_checksum_failures_++;
                // Log at 1 Hz so we notice UART noise without spamming
                static rclcpp::Clock csum_clk(RCL_STEADY_TIME);
                RCLCPP_WARN_THROTTLE(LOGGER, csum_clk, 1000,
                    "Checksum mismatch — cumulative: %d. "
                    "Possible UART noise or EMI on cable.",
                    rx_checksum_failures_);
                // Still accept the frame — bad checksum ≠ wrong positions,
                // but log it so you know if your cable is marginal.
            }

            // Latch fault flags
            if (parsed.pitch_frozen) {
                static rclcpp::Clock pf_clk(RCL_STEADY_TIME);
                RCLCPP_WARN_THROTTLE(LOGGER, pf_clk, 1000,
                    "STM32 reports PITCH FROZEN — joint is at hard limit. "
                    "Reconnect (deactivate/activate) to reset.");
            }
            if (parsed.z_collision) {
                static rclcpp::Clock zc_clk(RCL_STEADY_TIME);
                RCLCPP_WARN_THROTTLE(LOGGER, zc_clk, 1000,
                    "STM32 reports Z-AXIS COLLISION — motor stalled. "
                    "Command arm away from obstacle to unlatch.");
            }

            // Publish to read() — lock only while copying
            {
                std::lock_guard<std::mutex> lk(rx_mutex_);
                rx_buffer_ = parsed;
            }
            rx_valid_.store(true, std::memory_order_release);
            rx_count_++;

            state = SyncState::HUNT_F;
            break;
        }
        } // switch
    }

    RCLCPP_INFO(LOGGER, "RX thread exiting (rx_count=%lu frame_errors=%lu)",
        rx_count_, rx_frame_errors_);
}

// =============================================================================
// on_activate — seed positions from first real feedback, set commands = positions
// MUST be non-blocking — CM's lifecycle transition service will time out if
// this call blocks longer than ~100 ms.
// =============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &)
{
    commands_seeded_.store(false, std::memory_order_relaxed);
    pitch_fault_ = false;
    z_fault_     = false;

    if (rx_valid_.load(std::memory_order_acquire)) {
        // First FB already received — seed positions immediately
        {
            std::lock_guard<std::mutex> lk(rx_mutex_);
            for (size_t i = 0; i < NUM_JOINTS; i++) {
                hw_positions_[i]  = rx_buffer_.positions[i];
                hw_velocities_[i] = rx_buffer_.velocities[i];
                prev_positions_[i] = rx_buffer_.positions[i];
            }
        }
        hw_commands_ = hw_positions_;
        commands_seeded_.store(true, std::memory_order_release);
        RCLCPP_INFO(LOGGER, "Activated — positions seeded from STM32 feedback.");
    } else {
        // No FB yet.  commands_seeded_ stays false.
        // write() will NOT transmit until the first read() seeds commands.
        // This prevents a 0-rad burst before the STM32 is heard from.
        RCLCPP_WARN(LOGGER,
            "Activated — no STM32 feedback received yet. "
            "Commands will be held until first valid feedback packet arrives. "
            "If this persists >5s, check UART wiring and STM32 firmware.");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// read — called by CM at control loop rate (100 Hz).  Must not block.
// =============================================================================
hardware_interface::return_type ArmHardwareInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (!rx_valid_.load(std::memory_order_acquire)) {
        // No feedback yet — return OK, JTC will stay idle until state arrives
        return hardware_interface::return_type::OK;
    }

    ParsedFeedback local;
    {
        std::lock_guard<std::mutex> lk(rx_mutex_);
        local = rx_buffer_;
    }

    // Latch fault state (cleared only in on_activate)
    if (local.pitch_frozen) pitch_fault_ = true;
    if (local.z_collision)  z_fault_     = true;

    const double dt = period.seconds();

    for (size_t i = 0; i < NUM_JOINTS; i++) {
        // If joint is in fault, freeze position at last known good value
        if ((i == 3 && pitch_fault_) || (i == 5 && z_fault_)) {
            // hw_positions_[i] keeps previous value — do not update
            hw_velocities_[i] = 0.0;
            continue;
        }

        hw_positions_[i] = local.positions[i];

        // Velocity: prefer STM32-computed value if non-zero (STM32 has
        // higher-resolution timing than the CM's period estimate).
        // Fall back to numerical differentiation if STM32 sends zero.
        if (std::abs(local.velocities[i]) > 1e-6) {
            hw_velocities_[i] = local.velocities[i];
        } else if (dt > 1e-9) {
            hw_velocities_[i] =
                (hw_positions_[i] - prev_positions_[i]) / dt;
        }
        prev_positions_[i] = hw_positions_[i];
    }

    // First time we have real feedback — seed commands so JTC doesn't jump
    if (!commands_seeded_.load(std::memory_order_acquire)) {
        hw_commands_ = hw_positions_;
        commands_seeded_.store(true, std::memory_order_release);
        RCLCPP_INFO(LOGGER,
            "Commands seeded from first feedback in read() — "
            "write() will now transmit.");
    }

    return hardware_interface::return_type::OK;
}

// =============================================================================
// write — called by CM at control loop rate (100 Hz).  Must not block.
// =============================================================================
hardware_interface::return_type ArmHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    // Gate: do not transmit until STM32 is confirmed alive AND commands seeded
    if (!commands_seeded_.load(std::memory_order_acquire)) {
        return hardware_interface::return_type::OK;
    }

    std::array<double, NUM_JOINTS> positions{};
    for (size_t i = 0; i < NUM_JOINTS; i++) {
        // If a joint is faulted, hold it at current position — do not command
        // it toward whatever JTC computed (it might be past the fault point).
        if ((i == 3 && pitch_fault_) || (i == 5 && z_fault_)) {
            positions[i] = hw_positions_[i];
        } else {
            positions[i] = hw_commands_[i];
        }
    }

    CommandPacketUnified frame = pack_command(positions, false);

    if (!uart_.write_bytes(
            reinterpret_cast<const uint8_t*>(&frame), sizeof(frame))) {
        static rclcpp::Clock wclk(RCL_STEADY_TIME);
        RCLCPP_ERROR_THROTTLE(LOGGER, wclk, 1000,
            "UART write failed — tx_count=%lu. Check %s is still connected.",
            tx_count_, serial_port_.c_str());
        return hardware_interface::return_type::ERROR;
    }

    tx_count_++;
    return hardware_interface::return_type::OK;
}

// =============================================================================
// ESTOP helper
// =============================================================================
bool ArmHardwareInterface::send_estop()
{
    std::array<double, NUM_JOINTS> zeros{};
    CommandPacketUnified f = pack_command(zeros, true);
    return uart_.write_bytes(
        reinterpret_cast<const uint8_t*>(&f), sizeof(f));
}

// =============================================================================
// shutdown_io — signal + join rx_thread_, close UART.
// ORDER MATTERS: signal → join → close.
// Closing the fd before join would make uart_.read_bytes() block forever on a
// closed fd, and join() would never return.
// =============================================================================
void ArmHardwareInterface::shutdown_io()
{
    rx_running_.store(false, std::memory_order_relaxed);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
        RCLCPP_INFO(LOGGER, "RX thread joined.");
    }
    uart_.close();
    rx_valid_.store(false, std::memory_order_relaxed);
    commands_seeded_.store(false, std::memory_order_relaxed);
}

// =============================================================================
// on_deactivate — send ESTOP 3×, motors stop.  Do NOT shut down IO here —
// the hardware can be re-activated without going through configure again.
// =============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    int ok = 0;
    for (int i = 0; i < 3; i++) {
        if (send_estop()) ok++;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    commands_seeded_.store(false, std::memory_order_relaxed);
    pitch_fault_ = false;
    z_fault_     = false;

    RCLCPP_INFO(LOGGER,
        "Deactivated — ESTOP sent (%d/3 succeeded). "
        "tx_total=%lu rx_total=%lu frame_errors=%lu",
        ok, tx_count_, rx_count_, rx_frame_errors_);

    if (ok == 0) {
        RCLCPP_WARN(LOGGER,
            "All ESTOP sends failed — UART may be disconnected. "
            "STM32 2-second watchdog will stop motors automatically.");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// on_cleanup — full IO shutdown
// =============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    shutdown_io();
    RCLCPP_INFO(LOGGER, "Hardware cleaned up.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// =============================================================================
// on_error — best-effort ESTOP + full shutdown
// =============================================================================
hardware_interface::CallbackReturn ArmHardwareInterface::on_error(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_ERROR(LOGGER,
        "Hardware interface error — sending ESTOP and shutting down IO.");
    send_estop();
    shutdown_io();
    return hardware_interface::CallbackReturn::SUCCESS;
}

} // namespace arm_hardware
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/src/uart_driver.cpp">
// =============================================================================
// uart_driver.cpp
// =============================================================================
#include "arm_hardware/uart_driver.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>
#include <time.h>

namespace arm_hardware {

// ─────────────────────────────────────────────────────────────────────────────

speed_t UartDriver::to_speed(int baud_rate)
{
    switch (baud_rate) {
        case 115200:  return B115200;
        case 460800:  return B460800;
        case 921600:  return B921600;
        default:      return B0;  // invalid
    }
}

bool UartDriver::open(const std::string& port, int baud_rate)
{
    if (is_open()) return false;

    speed_t speed = to_speed(baud_rate);
    if (speed == B0) return false;

    // O_NOCTTY: don't become controlling terminal
    // O_NONBLOCK: open() won't block even if DCD is not asserted
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) { ::close(fd); return false; }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= CS8 | CREAD | CLOCAL;

    // Raw mode: no canonical processing, no echo, no signals
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

    // Disable all input processing
    tty.c_iflag &= ~(IXON | IXOFF | IXANY |
                     IGNBRK | BRKINT | PARMRK | ISTRIP |
                     INLCR | IGNCR | ICRNL);

    // Disable output processing
    tty.c_oflag &= ~(OPOST | ONLCR);

    // Non-blocking reads from the kernel side — we implement our own deadline
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return false; }

    // tcflush: discard any stale bytes already in the OS buffer
    tcflush(fd, TCIFLUSH);

    fd_       = fd;
    owns_fd_  = true;
    return true;
}

bool UartDriver::open_fd(int fd)
{
    if (is_open() || fd < 0) return false;
    fd_      = fd;
    owns_fd_ = false;
    return true;
}

void UartDriver::close()
{
    if (fd_ >= 0 && owns_fd_) ::close(fd_);
    fd_      = -1;
    owns_fd_ = false;
}

void UartDriver::flush_rx()
{
    if (is_open()) tcflush(fd_, TCIFLUSH);
}

// ─────────────────────────────────────────────────────────────────────────────

bool UartDriver::write_bytes(const uint8_t* data, size_t len)
{
    size_t written = 0;
    while (written < len) {
        ssize_t n = ::write(fd_, data + written, len - written);
        if (n > 0) {
            written += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // Wait up to 1 s for the fd to become writable
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(fd_, &wfds);
            struct timeval tv{1, 0};
            if (select(fd_ + 1, nullptr, &wfds, nullptr, &tv) <= 0)
                return false;  // timeout or error
            continue;
        }
        return false;  // hard error
    }
    return true;
}

bool UartDriver::read_bytes(uint8_t* buf, size_t len, int timeout_ms)
{
    // Compute absolute deadline on CLOCK_MONOTONIC
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec  += timeout_ms / 1000;
    deadline.tv_nsec += static_cast<long>(timeout_ms % 1000) * 1'000'000L;
    if (deadline.tv_nsec >= 1'000'000'000L) {
        deadline.tv_sec  += 1;
        deadline.tv_nsec -= 1'000'000'000L;
    }

    size_t received = 0;
    while (received < len) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        long rem_ns = (deadline.tv_sec  - now.tv_sec)  * 1'000'000'000L
                    + (deadline.tv_nsec - now.tv_nsec);
        if (rem_ns <= 0) return false;  // deadline expired

        struct timeval tv{
            rem_ns / 1'000'000'000L,
            (rem_ns % 1'000'000'000L) / 1000L
        };
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        int ret = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0)  return false;  // error
        if (ret == 0) return false;  // timeout

        ssize_t n = ::read(fd_, buf + received, len - received);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (n <= 0) return false;
        received += static_cast<size_t>(n);
    }
    return true;
}

} // namespace arm_hardware
</file>

<file path="arm_hardware_complete (1)/src/arm_hardware/stm32_patch/RobotCore_protocol_patch.h">
// =============================================================================
// RobotCore_protocol_patch.h
//
// DROP-IN REPLACEMENT for the packet structs and feedback transmission
// section of RobotCore.cpp / RobotCore.h
//
// WHAT CHANGED vs original:
//   1. CommandPacketUnified: +flags byte, +checksum byte  (29 bytes total)
//   2. FeedbackPacketUnified: +motor_vel[6], +flags byte, checksum replaces
//      footer  (52 bytes total, no '\n' footer — checksum is last byte)
//   3. parseCommandPacket(): validates checksum before accepting frame
//   4. Feedback transmission: populates velocities + flags + checksum
//
// HOW TO INTEGRATE:
//   Step 1. Replace the two struct definitions at the top of RobotCore.h
//           with the ones below.
//   Step 2. Replace the parseCommandPacket() body in RobotCore.cpp
//           with the one below.
//   Step 3. Replace the "7. Transmit Feedback" section in RobotCore_PIDTask()
//           with the block below.
//   Step 4. Add the velocity state variables listed in the "NEW STATE" section.
//   Step 5. Populate those velocity variables in RobotCore_PIDTask() as shown.
//   Step 6. In RobotCore_Init(): update current_fb init line (no footer field).
// =============================================================================

#pragma once
#include <stdint.h>

// =============================================================================
// STEP 1 — Replace in RobotCore.h
// =============================================================================

#pragma pack(push, 1)

// ROS2 → STM32 command  (29 bytes)
typedef struct {
    char    header[2];      // 'S','T' or 'P','T'
    int32_t motor_cmd[6];   // milli-degrees per joint
    uint8_t flags;          // bit0 = ESTOP
    uint8_t checksum;       // XOR of bytes [0..26]
    char    footer;         // '\n'
} CommandPacketUnified;
// Verify at compile time:
// static_assert(sizeof(CommandPacketUnified) == 29, "bad size");

// STM32 → ROS2 feedback  (52 bytes)
typedef struct {
    char    header[2];      // 'F','B'
    int32_t motor_pos[6];   // milli-degrees (or 999999 on fault)
    int32_t motor_vel[6];   // milli-deg/s
    uint8_t flags;          // bit0=pitch_frozen, bit1=z_collision
    uint8_t checksum;       // XOR of bytes [0..50]
    // NO footer — checksum is the last byte
} FeedbackPacketUnified;
// static_assert(sizeof(FeedbackPacketUnified) == 52, "bad size");

#pragma pack(pop)

// Flag bits (match protocol.hpp on ROS2 side)
#define CMD_FLAG_ESTOP          (1u << 0)
#define FB_FLAG_PITCH_FROZEN    (1u << 0)
#define FB_FLAG_Z_COLLISION     (1u << 1)
#define SENTINEL_VAL            999999

// =============================================================================
// STEP 2 — Replace parseCommandPacket() body in RobotCore.cpp
// =============================================================================
//
// static uint8_t compute_checksum(const uint8_t* data, uint32_t len) {
//     uint8_t x = 0;
//     for (uint32_t i = 0; i < len; i++) x ^= data[i];
//     return x;
// }
//
// bool parseCommandPacket(uint8_t* buf, uint32_t buf_size,
//                         uint32_t* tail, uint32_t head)
// {
//     bool found = false;
//     while (*tail != head) {
//         uint8_t b = buf[*tail];
//         if (b == 'S' || b == 'P' || b == 'D' || b == 'R' || b == 'K') {
//             uint32_t avail = (head >= *tail) ? (head - *tail)
//                                              : (buf_size - *tail + head);
//             if (avail >= sizeof(CommandPacketUnified)) {
//                 uint8_t pkt[sizeof(CommandPacketUnified)];
//                 for (size_t i = 0; i < sizeof(CommandPacketUnified); i++)
//                     pkt[i] = buf[(*tail + i) % buf_size];
//
//                 CommandPacketUnified* cmd = (CommandPacketUnified*)pkt;
//
//                 // Validate header[1], footer, AND checksum
//                 uint8_t expected_cs = compute_checksum(
//                     pkt, offsetof(CommandPacketUnified, checksum));
//
//                 if (cmd->header[1] == 'T' &&
//                     cmd->footer   == '\n' &&
//                     cmd->checksum == expected_cs)
//                 {
//                     memcpy(&current_cmd, cmd, sizeof(CommandPacketUnified));
//                     last_valid_cmd_time = osKernelGetTickCount();
//                     found = true;
//                     *tail = (*tail + sizeof(CommandPacketUnified)) % buf_size;
//                     continue;
//                 }
//                 // Bad checksum — reject and advance by 1 to resync
//             } else {
//                 break;  // wait for more bytes
//             }
//         }
//         *tail = (*tail + 1) % buf_size;
//     }
//     return found;
// }

// =============================================================================
// STEP 3 — Replace "7. Transmit Feedback" in RobotCore_PIDTask()
// =============================================================================
//
// The velocity variables (see STEP 4) must be computed BEFORE this block.
//
// // 7. Build and transmit feedback
// {
//     // --- Positions ---
//     if (current_cmd.header[0] == 'D' || current_cmd.header[0] == 'P') {
//         // Debug/open-loop: send raw sensor values
//         current_fb.motor_pos[0] = (int32_t)debug_raw_adcs[0];
//         current_fb.motor_pos[1] = (int32_t)debug_raw_adcs[1];
//         current_fb.motor_pos[2] = (int32_t)debug_raw_encoders[0];
//         current_fb.motor_pos[3] = (int32_t)debug_raw_encoders[1];
//         current_fb.motor_pos[4] = (int32_t)debug_raw_encoders[2];
//         current_fb.motor_pos[5] = (int32_t)debug_raw_encoders[3];
//     } else {
//         current_fb.motor_pos[0] = (int32_t)(pots[0]->currentAngle * 1000.0f);
//         current_fb.motor_pos[1] = (int32_t)(pots[1]->currentAngle * 1000.0f);
//         current_fb.motor_pos[2] = (int32_t)(((m3_continuous_pos / 4096.0f) * 360.0f / M3_GEAR_RATIO) * 1000.0f);
//
//         if (pitch_frozen) {
//             current_fb.motor_pos[3] = SENTINEL_VAL;
//         } else {
//             current_fb.motor_pos[3] = (int32_t)(
//                 ((float)PITCH_HOME_STEPS - current_pos[1])
//                 / (float)(PITCH_LIMIT_MAX - PITCH_LIMIT_MIN)
//                 * PITCH_RANGE_DEG * 1000.0f);
//         }
//
//         current_fb.motor_pos[4] = (int32_t)(((current_pos[2] / 4096.0f) * ROLL_RANGE_DEG) * 1000.0f);
//
//         if (z_collision_fault) {
//             current_fb.motor_pos[5] = SENTINEL_VAL;
//         } else {
//             current_fb.motor_pos[5] = (int32_t)(
//                 ((za_continuous_pos / 4096.0f) * 360.0f / ZA_GEAR_RATIO) * 1000.0f);
//         }
//     }
//
//     // --- Velocities (milli-deg/s) ---
//     // fb_vel_* are computed each PID tick — see STEP 5
//     current_fb.motor_vel[0] = (int32_t)(pots[0]->velocityCDS * 1000.0f);
//     current_fb.motor_vel[1] = (int32_t)(pots[1]->velocityCDS * 1000.0f);
//     current_fb.motor_vel[2] = (int32_t)(fb_vel_m3    * 1000.0f);
//     current_fb.motor_vel[3] = (int32_t)(fb_vel_pitch * 1000.0f);
//     current_fb.motor_vel[4] = (int32_t)(fb_vel_roll  * 1000.0f);
//     current_fb.motor_vel[5] = (int32_t)(fb_vel_z     * 1000.0f);
//
//     // --- Fault flags ---
//     current_fb.flags = 0;
//     if (pitch_frozen)     current_fb.flags |= FB_FLAG_PITCH_FROZEN;
//     if (z_collision_fault) current_fb.flags |= FB_FLAG_Z_COLLISION;
//
//     // --- Checksum (XOR all bytes before the checksum field) ---
//     current_fb.checksum = compute_checksum(
//         (uint8_t*)&current_fb,
//         offsetof(FeedbackPacketUnified, checksum));
//
//     // --- Transmit ---
//     HAL_UART_Transmit(&huart4,
//         (uint8_t*)&current_fb, sizeof(FeedbackPacketUnified), 10);
//     CDC_Transmit_FS(
//         (uint8_t*)&current_fb, sizeof(FeedbackPacketUnified));
// }

// =============================================================================
// STEP 4 — New global state variables (add to RobotCore.cpp top section)
// =============================================================================
//
// // Velocity feedback state (deg/s at the OUTPUT shaft)
// float fb_vel_m3    = 0.0f;
// float fb_vel_pitch = 0.0f;
// float fb_vel_roll  = 0.0f;
// float fb_vel_z     = 0.0f;
// float fb_prev_m3    = 0.0f;
// float fb_prev_pitch = 0.0f;
// float fb_prev_roll  = 0.0f;
// float fb_prev_z     = 0.0f;

// =============================================================================
// STEP 5 — Velocity computation (add BEFORE "5. Compute ESP-B PIDs" in PIDTask)
// =============================================================================
//
// const float DT = 0.02f; // 50 Hz tick
//
// // M3: continuous position is in raw encoder steps → convert to output deg/s
// fb_vel_m3 = ((m3_continuous_pos - fb_prev_m3) / 4096.0f * 360.0f / M3_GEAR_RATIO) / DT;
// fb_prev_m3 = m3_continuous_pos;
//
// // Pitch: corrected encoder steps → output deg/s (inverted axis)
// fb_vel_pitch = (-(current_pos[1] - fb_prev_pitch)
//                 / (float)(PITCH_LIMIT_MAX - PITCH_LIMIT_MIN)
//                 * PITCH_RANGE_DEG) / DT;
// fb_prev_pitch = current_pos[1];
//
// // Roll: 0..4096 steps → ROLL_RANGE_DEG
// fb_vel_roll = ((wrapError(current_pos[2] - fb_prev_roll) / 4096.0f) * ROLL_RANGE_DEG) / DT;
// fb_prev_roll = current_pos[2];
//
// // Z: continuous position → output deg/s
// fb_vel_z = ((za_continuous_pos - fb_prev_z) / 4096.0f * 360.0f / ZA_GEAR_RATIO) / DT;
// fb_prev_z = za_continuous_pos;

// =============================================================================
// STEP 6 — Update RobotCore_Init() feedback header init
// =============================================================================
//
// Old:
//   current_fb.header[0] = 'F'; current_fb.header[1] = 'B';
//   current_fb.footer = '\n';
//
// New (no footer field):
//   current_fb.header[0] = 'F'; current_fb.header[1] = 'B';
//   memset(current_fb.motor_pos, 0, sizeof(current_fb.motor_pos));
//   memset(current_fb.motor_vel, 0, sizeof(current_fb.motor_vel));
//   current_fb.flags = 0;
//   current_fb.checksum = 0;
</file>

<file path="calibrate_all.py">
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
</file>

<file path="calibrate_pitch_dir.py">
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
</file>

<file path="calibrate_pots.py">
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
</file>

<file path="calibrate_wrist.py">
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
</file>

<file path="check_z_encoder_dir.py">
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
</file>

<file path="Core/Inc/adc.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
</file>

<file path="Core/Inc/dma.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */
</file>

<file path="Core/Inc/FreeRTOSConfig.h">
/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.3.1
 * Portion Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Portion Copyright (C) 2019 StMicroelectronics, Inc.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
#endif
#ifndef CMSIS_device_header
#define CMSIS_device_header "stm32f4xx.h"
#endif /* CMSIS_device_header */

#define configENABLE_FPU                         0
#define configENABLE_MPU                         0

#define configUSE_PREEMPTION                     1
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     ( 56 )
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_TRACE_FACILITY                 1
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
/* USER CODE BEGIN MESSAGE_BUFFER_LENGTH_TYPE */
/* Defaults to size_t for backward compatibility, but can be changed
   if lengths will always be less than the number of bytes in a size_t. */
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t
/* USER CODE END MESSAGE_BUFFER_LENGTH_TYPE */

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( 2 )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             256

/* The following flag must be enabled only when using newlib */
#define configUSE_NEWLIB_REENTRANT          1

/* CMSIS-RTOS V2 flags */
#define configUSE_OS2_THREAD_SUSPEND_RESUME  1
#define configUSE_OS2_THREAD_ENUMERATE       1
#define configUSE_OS2_EVENTFLAGS_FROM_ISR    1
#define configUSE_OS2_THREAD_FLAGS           1
#define configUSE_OS2_TIMER                  1
#define configUSE_OS2_MUTEX                  1

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet             1
#define INCLUDE_uxTaskPriorityGet            1
#define INCLUDE_vTaskDelete                  1
#define INCLUDE_vTaskCleanUpResources        0
#define INCLUDE_vTaskSuspend                 1
#define INCLUDE_vTaskDelayUntil              1
#define INCLUDE_vTaskDelay                   1
#define INCLUDE_xTaskGetSchedulerState       1
#define INCLUDE_xTimerPendFunctionCall       1
#define INCLUDE_xQueueGetMutexHolder         1
#define INCLUDE_uxTaskGetStackHighWaterMark  1
#define INCLUDE_xTaskGetCurrentTaskHandle    1
#define INCLUDE_eTaskGetState                1

/*
 * The CMSIS-RTOS V2 FreeRTOS wrapper is dependent on the heap implementation used
 * by the application thus the correct define need to be enabled below
 */
#define USE_FreeRTOS_HEAP_4

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
 /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
/* USER CODE END 1 */

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: After 10.3.1 update, Systick_Handler comes from NVIC (if SYS timebase = systick), otherwise from cmsis_os2.c */

#define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 0

/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */
</file>

<file path="Core/Inc/gpio.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */
</file>

<file path="Core/Inc/i2c.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
</file>

<file path="Core/Inc/main.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
</file>

<file path="Core/Inc/RobotConfig.h">
#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

/* =============================================================================
 *  ROBOT ARM CONFIGURATION
 *  Pure C header (no constexpr). Use this file to tweak calibration,
 *  PID gains, and safety limits.
 * ============================================================================= */

/* =============================================================================
 *  ESP-A (Potentiometer Motors M1 & M2)
 * ============================================================================= */
/* M1 (Shoulder / Link-1): 47.4° to 74.0°  — 20 calibration points */
#define PTS_M1 20
static const float angles_M1[PTS_M1] = { 47.4f, 52.5f, 53.1f, 56.2f, 57.3f, 58.0f, 59.5f, 60.0f, 61.0f, 61.0f, 62.8f, 63.0f, 65.5f, 65.5f, 65.5f, 66.0f, 71.0f, 71.8f, 72.0f, 74.0f };
static const int   adcs_M1[PTS_M1]   = { 2689, 2629, 2616, 2581, 2567, 2553, 2532, 2528, 2503, 2510, 2491, 2492, 2439, 2440, 2454, 2438, 2363, 2347, 2346, 2328 };

/* M2 (Elbow / Link-2): 1.0° to 61.2°  — 26 calibration points */
#define PTS_M2 26
static const float angles_M2[PTS_M2] = { 1.0f, 3.4f, 4.5f, 6.5f, 8.3f, 10.0f, 11.7f, 13.0f, 15.0f, 17.0f, 18.8f, 20.5f, 22.7f, 24.8f, 27.4f, 29.0f, 34.2f, 38.0f, 41.5f, 43.0f, 45.5f, 47.8f, 50.4f, 54.0f, 57.4f, 61.2f };
static const int   adcs_M2[PTS_M2]   = { 2997, 2957, 2958, 2932, 2903, 2877, 2846, 2826, 2791, 2754, 2729, 2698, 2664, 2630, 2576, 2553, 2398, 2389, 2335, 2299, 2256, 2212, 2166, 2100, 2032, 1957 };

/* =============================================================================
 *  ESP-B (I2C Encoder Motors)
 *  [0] = M3 | [1] = Pitch | [2] = Roll | [3] = Z-Axis
 * ============================================================================= */

/* Enable/Disable individual joints */
static const int JOINT_ALLOWED[4] = {1, 1, 1, 1};

/* Encoder Home Positions (0 to 4095) */
static const int HOME_OFFSETS[4] = {3857, 2574, 733, 236};

/* Hard Encoder Limits (corrected encoder steps)
 * NOTE: For pitch, LIMIT_MIN is the max-extension end, LIMIT_MAX is home/min end
 *       because travel crosses zero → corrected values go 1123 → 2972 */
static const int POS_MIN[4] = {0,    1123, 0,    0};
static const int POS_MAX[4] = {4096, 2915, 4096, 4096};
#define PITCH_LIMIT_MIN  1123   /* Corrected value at raw 3697 (max extension) */
#define PITCH_LIMIT_MAX  2915   /* Corrected value ~5° before raw 1450 (home buffer) */

/* Raw encoder endpoint limits for pitch
 * Dead zone = raw 1451 to raw 3696 (arm cannot physically reach here) */
#define PITCH_RAW_MIN    1450   /* Home endpoint */
#define PITCH_RAW_MAX    3697   /* Max extension endpoint */

/* Degrees-to-encoder range mappings */
#define PITCH_RANGE_DEG  ((1849.0f / 4096.0f) * 360.0f)  /* = 162.5 degrees */
#define ROLL_RANGE_DEG   360.0f

/* Pitch home (0° on slider) in corrected encoder steps
 * 0° = home (corrected 2972), positive degrees = arm extends toward corrected 1123 */
#define PITCH_HOME_STEPS 2972

/* Pitch freeze: if current position exceeds limits by this many steps (~5°), freeze motor */
#define PITCH_FREEZE_TOLERANCE 57

/* Encoder direction: +1 = counts UP with joint movement, -1 = counts DOWN */
static const int ENCODER_DIRECTION[4] = {1, 1, 1, 1};

/* Gear Ratios and Shaft Output Limits */
#define M3_GEAR_RATIO    2.0f
#define M3_OUTPUT_LIMIT  180.0f    /* +/- 180 deg output shaft */

#define ZA_GEAR_RATIO       2.7272f
#define ZA_OUTPUT_LIMIT     120.0f  /* +/- 120 deg output shaft */
#define ZA_NORMAL_LIMIT_DEG  60.0f  /* +/- 60 deg normal speed zone */
#define ZA_DOCKING_SCALE      0.25f /* 25% speed outside normal zone */
#define ZA_PID_OUTPUT_MAX   127.0f  /* Cap Z-axis motor speed (0-255) */

/* Virtual Current Sensing / Collision Detection for Z-Axis */
#define ZA_COLLISION_ENABLED   1     /* Master switch — set 1 to enable */
#define ZA_COLLISION_INTEGRAL  80.0f /* Tune 1: Max integral before tripping (0-100) */
#define ZA_COLLISION_VELOCITY  0.3f  /* Tune 2: Speed below this is considered "stuck" */
#define ZA_COLLISION_TIMEOUT   20    /* Tune 3: 25 ticks @ 50Hz = 500ms delay */

/* I2C Multiplexer Wiring (TCA9548A Channels) */
static const int MUX_CHANNELS[4] = {5, 0, 1, 2};

/* =============================================================================
 *  PID TUNING (ESP-B Motors)
 * ============================================================================= */
static const float Kp_enc[4] = {1.0f,  2.0f,  2.0f,  1.0f};
static const float Ki_enc[4] = {0.05f, 0.02f, 0.02f, 0.05f};
static const float Kd_enc[4] = {0.05f, 0.01f, 0.01f, 0.1f};

/* PID Constraints / Physical Protection */
#define PID_OUTPUT_MAX   255.0f
#define INTEGRAL_MAX     100.0f
#define ERROR_DEADZONE   1.0f    /* Stop seeking when within 1.0 step */
#define MAX_SENSOR_FAILS 10
#define MAX_SLEW_RATE    40.0f   /* Control signal smoothing */

/* Direction Signs (+1 or -1) */
#define M3_PID_SIGN           (-1)
#define ZA_PID_SIGN           1
#define WRIST_MOTOR_A_SIGN    1
#define WRIST_MOTOR_B_SIGN    1
#define PITCH_PID_SIGN        1
#define ROLL_PID_SIGN         1

/* Multi-turn wrap tracking constraints (Steps) */
#define M3_CONT_LIMIT     4096.0f
#define M3_CONT_SOFT_ZONE  400.0f

#define ZA_CONT_LIMIT     4096.0f
#define ZA_CONT_SOFT_ZONE  400.0f

/* =============================================================================
 *  TRAPEZOIDAL MOTION PROFILE (Z-Axis and M3)
 *  Units: encoder steps per PID tick (50Hz → 1 tick = 20ms)
 * ============================================================================= */
#define ZA_PROF_MAX_VEL    110.0f  /* steps/tick  ≈ 30°/s output shaft */
#define ZA_PROF_MAX_ACCEL    6.0f  /* steps/tick² ≈ 0.5s to full speed */

#define M3_PROF_MAX_VEL    150.0f  /* steps/tick  ≈ 41°/s output shaft */
#define M3_PROF_MAX_ACCEL    8.0f  /* steps/tick² */

#endif /* ROBOTCONFIG_H */
</file>

<file path="Core/Inc/RobotCore.h">
#ifndef ROBOTCORE_H
#define ROBOTCORE_H

#ifdef __cplusplus
#include <cstdint>
#include <cmath>
#include "main.h"

// =============================================================================
//  ROS UART — UNIFIED PACKET STRUCTS
//  Wire layout MUST match protocol.hpp on the ROS2 side exactly.
// =============================================================================
struct __attribute__((packed)) CommandPacketUnified {
    char    header[2];       // 'S','T' (position) | 'P','T' (estop) | 'D','T' | 'K','T' | 'R','T'
    int32_t motor_cmd[6];    // [0]M1, [1]M2, [2]M3, [3]Pitch, [4]Roll, [5]Z  (milli-degrees)
    uint8_t flags;           // bit0=ESTOP
    uint8_t checksum;        // XOR of bytes [0..26]
    char    footer;          // '\n'
};
_Static_assert(sizeof(CommandPacketUnified) == 29, "CommandPacketUnified size mismatch");

struct __attribute__((packed)) FeedbackPacketUnified {
    char    header[2];       // 'F', 'B'
    int32_t motor_pos[6];    // milli-degrees (or 999999 on fault)
    int32_t motor_vel[6];    // milli-deg/s
    uint8_t flags;           // bit0=pitch_frozen, bit1=z_collision
    uint8_t checksum;        // XOR of bytes [0..50]
};
_Static_assert(sizeof(FeedbackPacketUnified) == 52, "FeedbackPacketUnified size mismatch");

// Flag bits (match protocol.hpp on ROS2 side)
#define CMD_FLAG_ESTOP          (1u << 0)
#define FB_FLAG_PITCH_FROZEN    (1u << 0)
#define FB_FLAG_Z_COLLISION     (1u << 1)
#define SENTINEL_VAL            999999

// =============================================================================
//  EMA FILTER (Exponential Moving Average)
//  alpha: 0.0 = maximum smoothing (infinite lag), 1.0 = no smoothing
//  At 50Hz, alpha=0.2 gives ~90ms settling time — fast enough for motor
//  control, slow enough to crush ADC noise by ~5x.
// =============================================================================
class EMAFilter {
private:
    float alpha;
    float estimate;
    bool  initialized;
public:
    EMAFilter(float a) : alpha(a), estimate(0), initialized(false) {}

    float updateEstimate(float mea) {
        if (!initialized) { estimate = mea; initialized = true; }
        estimate = alpha * mea + (1.0f - alpha) * estimate;
        return estimate;
    }

    void setEstimate(float est) {
        estimate = est;
        initialized = true;
    }

    float getEstimate() const { return estimate; }
};

// =============================================================================
//  POTENTIOMETER MOTOR CLASS
// =============================================================================
class PotMotor {
public:
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    GPIO_TypeDef* pinDirPort;
    uint16_t pinDirPin;
    
    const float* angleTable;
    const int* adcTable;
    int          numPoints;
    bool         reverseDir;
    int          motorIdx;

    EMAFilter ema;
    float currentAngle, lastAngle, integral;
    float lastValidAngle;   // Deadband: last angle that passed the threshold
    float angleDead;        // Deadband threshold in degrees (default 0.5°)
    int   currentPWM;
    uint32_t lastStallCheck;
    float         prevStallAngle;
    int           stallCounter;
    bool          stalled;
    bool          enabled;

    // Trajectory State
    float commandedAngle;
    float velocityCDS;
    
    float kp, ki, kd;
    int pid_sign;

    PotMotor(TIM_HandleTypeDef* t, uint32_t ch, GPIO_TypeDef* dirPort, uint16_t dirPin,
             const float* angTab, const int* adcTab, int nPoints,
             bool rev, int idx, float P, float I, float D, int sign)
      : htim(t), channel(ch), pinDirPort(dirPort), pinDirPin(dirPin),
        angleTable(angTab), adcTable(adcTab), numPoints(nPoints),
        reverseDir(rev), motorIdx(idx),
        ema(0.2f),
        currentAngle(0), lastAngle(0), integral(0),
        lastValidAngle(0), angleDead(0.5f),
        currentPWM(0),
        lastStallCheck(0), prevStallAngle(0), stallCounter(0), stalled(false),
        enabled(false), commandedAngle(0), velocityCDS(0),
        kp(P), ki(I), kd(D), pid_sign(sign) {}

    void init(int initialADC);
    float adcToAngle(int adc);
    void resetStall();
    void updateTrajectory(float target);
    void update(float target, float dt, int rawADC);
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
//  C-BRIDGES
// =============================================================================
void RobotCore_Init(void);
void RobotCore_ROSTask(void);
void RobotCore_PIDTask(void);
uint8_t compute_checksum(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // ROBOTCORE_H
</file>

<file path="Core/Inc/spi.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
</file>

<file path="Core/Inc/stm32f4xx_hal_conf.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_conf_template.h
  * @author  MCD Application Team
  * @brief   HAL configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to stm32f4xx_hal_conf.h.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver
  */
#define HAL_MODULE_ENABLED

  /* #define HAL_CRYP_MODULE_ENABLED */
#define HAL_ADC_MODULE_ENABLED
/* #define HAL_CAN_MODULE_ENABLED */
/* #define HAL_CRC_MODULE_ENABLED */
/* #define HAL_CAN_LEGACY_MODULE_ENABLED */
/* #define HAL_DAC_MODULE_ENABLED */
/* #define HAL_DCMI_MODULE_ENABLED */
/* #define HAL_DMA2D_MODULE_ENABLED */
/* #define HAL_ETH_MODULE_ENABLED */
/* #define HAL_ETH_LEGACY_MODULE_ENABLED */
/* #define HAL_NAND_MODULE_ENABLED */
/* #define HAL_NOR_MODULE_ENABLED */
/* #define HAL_PCCARD_MODULE_ENABLED */
/* #define HAL_SRAM_MODULE_ENABLED */
/* #define HAL_SDRAM_MODULE_ENABLED */
/* #define HAL_HASH_MODULE_ENABLED */
#define HAL_I2C_MODULE_ENABLED
/* #define HAL_I2S_MODULE_ENABLED */
/* #define HAL_IWDG_MODULE_ENABLED */
/* #define HAL_LTDC_MODULE_ENABLED */
/* #define HAL_RNG_MODULE_ENABLED */
/* #define HAL_RTC_MODULE_ENABLED */
/* #define HAL_SAI_MODULE_ENABLED */
/* #define HAL_SD_MODULE_ENABLED */
/* #define HAL_MMC_MODULE_ENABLED */
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
/* #define HAL_USART_MODULE_ENABLED */
/* #define HAL_IRDA_MODULE_ENABLED */
/* #define HAL_SMARTCARD_MODULE_ENABLED */
/* #define HAL_SMBUS_MODULE_ENABLED */
/* #define HAL_WWDG_MODULE_ENABLED */
#define HAL_PCD_MODULE_ENABLED
/* #define HAL_HCD_MODULE_ENABLED */
/* #define HAL_DSI_MODULE_ENABLED */
/* #define HAL_QSPI_MODULE_ENABLED */
/* #define HAL_QSPI_MODULE_ENABLED */
/* #define HAL_CEC_MODULE_ENABLED */
/* #define HAL_FMPI2C_MODULE_ENABLED */
/* #define HAL_FMPSMBUS_MODULE_ENABLED */
/* #define HAL_SPDIFRX_MODULE_ENABLED */
/* #define HAL_DFSDM_MODULE_ENABLED */
/* #define HAL_LPTIM_MODULE_ENABLED */
#define HAL_GPIO_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

/* ########################## HSE/HSI Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).
  */
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    8000000U /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    100U   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL).
  */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined  (LSI_VALUE)
 #define LSI_VALUE  32000U       /*!< LSI Typical Value in Hz*/
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.*/
/**
  * @brief External Low Speed oscillator (LSE) value.
  */
#if !defined  (LSE_VALUE)
 #define LSE_VALUE  32768U    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* LSE_VALUE */

#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    5000U   /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */

/**
  * @brief External clock source for I2S peripheral
  *        This value is used by the I2S HAL module to compute the I2S clock source
  *        frequency, this source is inserted directly through I2S_CKIN pad.
  */
#if !defined  (EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE    12288000U /*!< Value of the External audio frequency in Hz*/
#endif /* EXTERNAL_CLOCK_VALUE */

/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */

/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */
#define  VDD_VALUE		      3300U /*!< Value of VDD in mv */
#define  TICK_INT_PRIORITY            15U   /*!< tick interrupt priority */
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U

#define  USE_HAL_ADC_REGISTER_CALLBACKS         0U /* ADC register callback disabled       */
#define  USE_HAL_CAN_REGISTER_CALLBACKS         0U /* CAN register callback disabled       */
#define  USE_HAL_CEC_REGISTER_CALLBACKS         0U /* CEC register callback disabled       */
#define  USE_HAL_CRYP_REGISTER_CALLBACKS        0U /* CRYP register callback disabled      */
#define  USE_HAL_DAC_REGISTER_CALLBACKS         0U /* DAC register callback disabled       */
#define  USE_HAL_DCMI_REGISTER_CALLBACKS        0U /* DCMI register callback disabled      */
#define  USE_HAL_DFSDM_REGISTER_CALLBACKS       0U /* DFSDM register callback disabled     */
#define  USE_HAL_DMA2D_REGISTER_CALLBACKS       0U /* DMA2D register callback disabled     */
#define  USE_HAL_DSI_REGISTER_CALLBACKS         0U /* DSI register callback disabled       */
#define  USE_HAL_ETH_REGISTER_CALLBACKS         0U /* ETH register callback disabled       */
#define  USE_HAL_HASH_REGISTER_CALLBACKS        0U /* HASH register callback disabled      */
#define  USE_HAL_HCD_REGISTER_CALLBACKS         0U /* HCD register callback disabled       */
#define  USE_HAL_I2C_REGISTER_CALLBACKS         0U /* I2C register callback disabled       */
#define  USE_HAL_FMPI2C_REGISTER_CALLBACKS      0U /* FMPI2C register callback disabled    */
#define  USE_HAL_FMPSMBUS_REGISTER_CALLBACKS    0U /* FMPSMBUS register callback disabled  */
#define  USE_HAL_I2S_REGISTER_CALLBACKS         0U /* I2S register callback disabled       */
#define  USE_HAL_IRDA_REGISTER_CALLBACKS        0U /* IRDA register callback disabled      */
#define  USE_HAL_LPTIM_REGISTER_CALLBACKS       0U /* LPTIM register callback disabled     */
#define  USE_HAL_LTDC_REGISTER_CALLBACKS        0U /* LTDC register callback disabled      */
#define  USE_HAL_MMC_REGISTER_CALLBACKS         0U /* MMC register callback disabled       */
#define  USE_HAL_NAND_REGISTER_CALLBACKS        0U /* NAND register callback disabled      */
#define  USE_HAL_NOR_REGISTER_CALLBACKS         0U /* NOR register callback disabled       */
#define  USE_HAL_PCCARD_REGISTER_CALLBACKS      0U /* PCCARD register callback disabled    */
#define  USE_HAL_PCD_REGISTER_CALLBACKS         0U /* PCD register callback disabled       */
#define  USE_HAL_QSPI_REGISTER_CALLBACKS        0U /* QSPI register callback disabled      */
#define  USE_HAL_RNG_REGISTER_CALLBACKS         0U /* RNG register callback disabled       */
#define  USE_HAL_RTC_REGISTER_CALLBACKS         0U /* RTC register callback disabled       */
#define  USE_HAL_SAI_REGISTER_CALLBACKS         0U /* SAI register callback disabled       */
#define  USE_HAL_SD_REGISTER_CALLBACKS          0U /* SD register callback disabled        */
#define  USE_HAL_SMARTCARD_REGISTER_CALLBACKS   0U /* SMARTCARD register callback disabled */
#define  USE_HAL_SDRAM_REGISTER_CALLBACKS       0U /* SDRAM register callback disabled     */
#define  USE_HAL_SRAM_REGISTER_CALLBACKS        0U /* SRAM register callback disabled      */
#define  USE_HAL_SPDIFRX_REGISTER_CALLBACKS     0U /* SPDIFRX register callback disabled   */
#define  USE_HAL_SMBUS_REGISTER_CALLBACKS       0U /* SMBUS register callback disabled     */
#define  USE_HAL_SPI_REGISTER_CALLBACKS         0U /* SPI register callback disabled       */
#define  USE_HAL_TIM_REGISTER_CALLBACKS         0U /* TIM register callback disabled       */
#define  USE_HAL_UART_REGISTER_CALLBACKS        0U /* UART register callback disabled      */
#define  USE_HAL_USART_REGISTER_CALLBACKS       0U /* USART register callback disabled     */
#define  USE_HAL_WWDG_REGISTER_CALLBACKS        0U /* WWDG register callback disabled      */

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* ################## Ethernet peripheral configuration ##################### */

/* Section 1 : Ethernet peripheral configuration */

/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   2U
#define MAC_ADDR1   0U
#define MAC_ADDR2   0U
#define MAC_ADDR3   0U
#define MAC_ADDR4   0U
#define MAC_ADDR5   0U

/* Definition of the Ethernet driver buffers size and count */
#define ETH_RX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for receive               */
#define ETH_TX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for transmit              */
#define ETH_RXBUFNB                    4U       /* 4 Rx buffers of size ETH_RX_BUF_SIZE  */
#define ETH_TXBUFNB                    4U       /* 4 Tx buffers of size ETH_TX_BUF_SIZE  */

/* Section 2: PHY configuration section */

/* DP83848_PHY_ADDRESS Address*/
#define DP83848_PHY_ADDRESS
/* PHY Reset delay these values are based on a 1 ms Systick interrupt*/
#define PHY_RESET_DELAY                 0x000000FFU
/* PHY Configuration delay */
#define PHY_CONFIG_DELAY                0x00000FFFU

#define PHY_READ_TO                     0x0000FFFFU
#define PHY_WRITE_TO                    0x0000FFFFU

/* Section 3: Common PHY Registers */

#define PHY_BCR                         ((uint16_t)0x0000U)    /*!< Transceiver Basic Control Register   */
#define PHY_BSR                         ((uint16_t)0x0001U)    /*!< Transceiver Basic Status Register    */

#define PHY_RESET                       ((uint16_t)0x8000U)  /*!< PHY Reset */
#define PHY_LOOPBACK                    ((uint16_t)0x4000U)  /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100U)  /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000U)  /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100U)  /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000U)  /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION             ((uint16_t)0x1000U)  /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)0x0200U)  /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN                   ((uint16_t)0x0800U)  /*!< Select the power down mode           */
#define PHY_ISOLATE                     ((uint16_t)0x0400U)  /*!< Isolate PHY from MII                 */

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)0x0020U)  /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS               ((uint16_t)0x0004U)  /*!< Valid link established               */
#define PHY_JABBER_DETECTION            ((uint16_t)0x0002U)  /*!< Jabber condition detected            */

/* Section 4: Extended PHY Registers */
#define PHY_SR                          ((uint16_t))    /*!< PHY status register Offset                      */

#define PHY_SPEED_STATUS                ((uint16_t))  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS               ((uint16_t))  /*!< PHY Duplex mask                                 */

/* ################## SPI peripheral configuration ########################## */

/* CRC FEATURE: Use to activate CRC feature inside HAL SPI Driver
* Activated: CRC code is present inside driver
* Deactivated: CRC code cleaned from driver
*/

#define USE_SPI_CRC                     0U

/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file
  */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f4xx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f4xx_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32f4xx_hal_can.h"
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_CAN_LEGACY_MODULE_ENABLED
  #include "stm32f4xx_hal_can_legacy.h"
#endif /* HAL_CAN_LEGACY_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32f4xx_hal_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_CRYP_MODULE_ENABLED
  #include "stm32f4xx_hal_cryp.h"
#endif /* HAL_CRYP_MODULE_ENABLED */

#ifdef HAL_DMA2D_MODULE_ENABLED
  #include "stm32f4xx_hal_dma2d.h"
#endif /* HAL_DMA2D_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32f4xx_hal_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_DCMI_MODULE_ENABLED
  #include "stm32f4xx_hal_dcmi.h"
#endif /* HAL_DCMI_MODULE_ENABLED */

#ifdef HAL_ETH_MODULE_ENABLED
  #include "stm32f4xx_hal_eth.h"
#endif /* HAL_ETH_MODULE_ENABLED */

#ifdef HAL_ETH_LEGACY_MODULE_ENABLED
  #include "stm32f4xx_hal_eth_legacy.h"
#endif /* HAL_ETH_LEGACY_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sram.h"
#endif /* HAL_SRAM_MODULE_ENABLED */

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32f4xx_hal_nor.h"
#endif /* HAL_NOR_MODULE_ENABLED */

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32f4xx_hal_nand.h"
#endif /* HAL_NAND_MODULE_ENABLED */

#ifdef HAL_PCCARD_MODULE_ENABLED
  #include "stm32f4xx_hal_pccard.h"
#endif /* HAL_PCCARD_MODULE_ENABLED */

#ifdef HAL_SDRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sdram.h"
#endif /* HAL_SDRAM_MODULE_ENABLED */

#ifdef HAL_HASH_MODULE_ENABLED
 #include "stm32f4xx_hal_hash.h"
#endif /* HAL_HASH_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32f4xx_hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_SMBUS_MODULE_ENABLED
 #include "stm32f4xx_hal_smbus.h"
#endif /* HAL_SMBUS_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
 #include "stm32f4xx_hal_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_iwdg.h"
#endif /* HAL_IWDG_MODULE_ENABLED */

#ifdef HAL_LTDC_MODULE_ENABLED
 #include "stm32f4xx_hal_ltdc.h"
#endif /* HAL_LTDC_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32f4xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
 #include "stm32f4xx_hal_rng.h"
#endif /* HAL_RNG_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
 #include "stm32f4xx_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SAI_MODULE_ENABLED
 #include "stm32f4xx_hal_sai.h"
#endif /* HAL_SAI_MODULE_ENABLED */

#ifdef HAL_SD_MODULE_ENABLED
 #include "stm32f4xx_hal_sd.h"
#endif /* HAL_SD_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32f4xx_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f4xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f4xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
 #include "stm32f4xx_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_IRDA_MODULE_ENABLED
 #include "stm32f4xx_hal_irda.h"
#endif /* HAL_IRDA_MODULE_ENABLED */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
 #include "stm32f4xx_hal_smartcard.h"
#endif /* HAL_SMARTCARD_MODULE_ENABLED */

#ifdef HAL_WWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_wwdg.h"
#endif /* HAL_WWDG_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
 #include "stm32f4xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#ifdef HAL_HCD_MODULE_ENABLED
 #include "stm32f4xx_hal_hcd.h"
#endif /* HAL_HCD_MODULE_ENABLED */

#ifdef HAL_DSI_MODULE_ENABLED
 #include "stm32f4xx_hal_dsi.h"
#endif /* HAL_DSI_MODULE_ENABLED */

#ifdef HAL_QSPI_MODULE_ENABLED
 #include "stm32f4xx_hal_qspi.h"
#endif /* HAL_QSPI_MODULE_ENABLED */

#ifdef HAL_CEC_MODULE_ENABLED
 #include "stm32f4xx_hal_cec.h"
#endif /* HAL_CEC_MODULE_ENABLED */

#ifdef HAL_FMPI2C_MODULE_ENABLED
 #include "stm32f4xx_hal_fmpi2c.h"
#endif /* HAL_FMPI2C_MODULE_ENABLED */

#ifdef HAL_FMPSMBUS_MODULE_ENABLED
 #include "stm32f4xx_hal_fmpsmbus.h"
#endif /* HAL_FMPSMBUS_MODULE_ENABLED */

#ifdef HAL_SPDIFRX_MODULE_ENABLED
 #include "stm32f4xx_hal_spdifrx.h"
#endif /* HAL_SPDIFRX_MODULE_ENABLED */

#ifdef HAL_DFSDM_MODULE_ENABLED
 #include "stm32f4xx_hal_dfsdm.h"
#endif /* HAL_DFSDM_MODULE_ENABLED */

#ifdef HAL_LPTIM_MODULE_ENABLED
 #include "stm32f4xx_hal_lptim.h"
#endif /* HAL_LPTIM_MODULE_ENABLED */

#ifdef HAL_MMC_MODULE_ENABLED
 #include "stm32f4xx_hal_mmc.h"
#endif /* HAL_MMC_MODULE_ENABLED */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CONF_H */
</file>

<file path="Core/Inc/stm32f4xx_it.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void DMA1_Stream2_IRQHandler(void);
void UART4_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void OTG_FS_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
</file>

<file path="Core/Inc/tim.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
</file>

<file path="Core/Inc/usart.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART4_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
</file>

<file path="Core/Src/adc.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
</file>

<file path="Core/Src/dma.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
</file>

<file path="Core/Src/freertos.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RobotCore.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for rosTask */
osThreadId_t rosTaskHandle;
const osThreadAttr_t rosTask_attributes = {
  .name = "rosTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pidTask */
osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = {
  .name = "pidTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartRosTask(void *argument);
void StartPidTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of rosTask */
  rosTaskHandle = osThreadNew(StartRosTask, NULL, &rosTask_attributes);

  /* creation of pidTask */
  pidTaskHandle = osThreadNew(StartPidTask, NULL, &pidTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartRosTask */
/**
  * @brief  Function implementing the rosTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRosTask */
void StartRosTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartRosTask */
  RobotCore_Init();
  /* Infinite loop */
  for(;;)
  {
    RobotCore_ROSTask();
    osDelay(1);
  }
  /* USER CODE END StartRosTask */
}

/* USER CODE BEGIN Header_StartPidTask */
/**
* @brief Function implementing the pidTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPidTask */
void StartPidTask(void *argument)
{
  /* USER CODE BEGIN StartPidTask */
  /* Infinite loop */
  for(;;)
  {
    RobotCore_PIDTask();
    osDelay(20);  /* 50Hz PID loop (dt = 0.02s) */
  }
  /* USER CODE END StartPidTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
</file>

<file path="Core/Src/gpio.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_0|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE7 PE8 PE10
                           PE12 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           PD0 Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_0|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_MCK_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_MCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
</file>

<file path="Core/Src/i2c.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(Audio_SCL_GPIO_Port, Audio_SCL_Pin);

    HAL_GPIO_DeInit(Audio_SDA_GPIO_Port, Audio_SDA_Pin);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
</file>

<file path="Core/Src/main.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
</file>

<file path="Core/Src/RobotCore.cpp">
#include "RobotCore.h"
#include "cmsis_os2.h"
#include "main.h"
#include <cstring>
#include <cstdlib>

// ── XOR checksum (shared with protocol.hpp on the ROS2 side) ──────────────────
uint8_t compute_checksum(const uint8_t* data, size_t len) {
    uint8_t x = 0;
    for (size_t i = 0; i < len; i++) x ^= data[i];
    return x;
}

extern UART_HandleTypeDef huart4;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// =============================================================================
//  ADC DMA OVERSAMPLING BUFFER
//  DMA fills this circularly: [CH8, CH9, CH8, CH9, ...] interleaved.
//  ADC_OVS_SIZE pairs = ADC_OVS_SIZE samples per channel.
// =============================================================================
static const uint32_t ADC_OVS_SIZE = 1024;   // samples per channel
static volatile uint16_t adc_dma_buf[ADC_OVS_SIZE * 2];  // 2 channels, interleaved

// Read oversampled ADC for one channel (0=M1/CH8, 1=M2/CH9).
// Sums ADC_OVS_SIZE samples and divides: effectively a moving average
// that adds ~5 bits of virtual resolution via oversampling.
static inline uint16_t adc_read_oversampled(uint8_t ch) {
    uint32_t acc = 0;
    for (uint32_t i = ch; i < ADC_OVS_SIZE * 2; i += 2)
        acc += adc_dma_buf[i];
    return (uint16_t)(acc / ADC_OVS_SIZE);
}

// =============================================================================
//  UART DMA CACHE & PACKETS
// =============================================================================
static const int RX_BUF_SIZE = 128;
uint8_t rx_buffer[RX_BUF_SIZE];
static const int USB_RX_BUF_SIZE = 512;
uint8_t usb_rx_buffer[USB_RX_BUF_SIZE];
volatile uint32_t usb_rx_head = 0;
uint32_t usb_rx_tail = 0;

extern "C" uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

extern "C" void RobotCore_USB_Receive_Callback(uint8_t* buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        usb_rx_buffer[usb_rx_head] = buf[i];
        usb_rx_head = (usb_rx_head + 1) % USB_RX_BUF_SIZE;
    }
}
uint32_t rx_tail = 0;
uint32_t ms_ticks = 0;

float current_Kp_enc[4];
float current_Ki_enc[4];
float current_Kd_enc[4];

CommandPacketUnified  current_cmd;
FeedbackPacketUnified current_fb;

uint32_t last_valid_cmd_time = 0;
bool comms_ok = false;
const uint32_t COMMS_TIMEOUT_MS = 2000;

// =============================================================================
//  ESP-A: M1 / M2 CONFIG (POTENTIOMETERS)
// =============================================================================
#include "RobotConfig.h"
// PotMotors Instance
PotMotor motor1(&htim1, TIM_CHANNEL_1, GPIOE, GPIO_PIN_7, angles_M1, adcs_M1, PTS_M1, false, 0, 15.0f, 1.0f, 0.8f, 1);
PotMotor motor2(&htim1, TIM_CHANNEL_2, GPIOE, GPIO_PIN_8, angles_M2, adcs_M2, PTS_M2, false, 1, 15.0f, 1.0f, 0.8f, -1);
PotMotor* pots[2] = {&motor1, &motor2};

// =============================================================================
//  ESP-B: I2C ENCODERS CONFIG & STATE
// =============================================================================

struct I2CJoint {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    GPIO_TypeDef* dirPort;
    uint16_t dirPin;
};

// [0]=M3, [1]=Pitch, [2]=Roll, [3]=Z-Axis
I2CJoint enc_joints[4] = {
    {&htim1, TIM_CHANNEL_3, GPIOE, GPIO_PIN_10}, // M3
    {&htim1, TIM_CHANNEL_4, GPIOE, GPIO_PIN_12}, // Pitch
    {&htim2, TIM_CHANNEL_2, GPIOE, GPIO_PIN_15}, // Roll
    {&htim2, TIM_CHANNEL_3, GPIOD, GPIO_PIN_0}   // Z-ax
};



bool  enc_enabled[4]     = {false};
float target_pos[4]      = {0};
float commanded_pos[4]   = {0};
float current_pos[4]     = {0};
float prev_pos[4]        = {0};
float integral[4]        = {0};
float prev_error[4]      = {0};
float pid_output[4]      = {0};
int   sensor_fail_count[4] = {0};
bool  sensor_read_ok[4]  = {false};

// M3 Multi-turn
int   m3_wrap_count = 0;  int m3_prev_raw = 0;
double m3_continuous_pos = 0, m3_continuous_target = 0;
double m3_continuous_cmd = 0, m3_continuous_prev   = 0;

// Z-Axis Multi-turn
int   za_wrap_count = 0;  int za_prev_raw = 0;
double za_continuous_pos = 0, za_continuous_target = 0;
double za_continuous_cmd = 0, za_continuous_prev   = 0;
float za_profile_vel    = 0; // Trapezoidal profile: current velocity (steps/tick)
int   za_stall_counter = 0; // Tracks consecutive overload ticks
bool  z_collision_fault = false;

// ── Velocity feedback state (Step 4 from protocol patch) ─────────────────────
// Dedicated prev-state so PID D-term consumption doesn't zero the deltas.
float fb_vel_m3    = 0.0f, fb_prev_m3    = 0.0f;
float fb_vel_pitch = 0.0f, fb_prev_pitch = 0.0f;
float fb_vel_roll  = 0.0f, fb_prev_roll  = 0.0f;
float fb_vel_z     = 0.0f, fb_prev_z     = 0.0f;

// M3 profile velocity
float m3_profile_vel    = 0; // Trapezoidal profile: current velocity (steps/tick)

bool systemActive = false;
float pot_targets[2] = {0};

bool is_open_loop_mode = false;
int open_loop_pwm[6] = {0};

int debug_raw_encoders[4] = {0};
int debug_raw_adcs[2] = {0};
bool pitch_frozen = false;  // Set when pitch exceeds limits by >5°

// =============================================================================
//  HELPER MATH
// =============================================================================
float wrapValue(float val) {
    while (val < 0) val += 4096.0f;
    while (val >= 4096.0f) val -= 4096.0f;
    return val;
}
float wrapError(float error) {
    if (error > 2048.0f) error -= 4096.0f;
    if (error < -2048.0f) error += 4096.0f;
    return error;
}
float applyDeadzone(float error, float deadzone) {
    if (fabsf(error) < deadzone) return 0.0f;
    return (error > 0) ? (error - deadzone) : (error + deadzone);
}

// Conversions
float motorDegToSteps(float motor_deg) { return (motor_deg / 360.0f) * 4096.0f; }
float outputToMotorDeg(float output_deg)   { return output_deg * M3_GEAR_RATIO; }
float zaOutputToMotorDeg(float output_deg) { return output_deg * ZA_GEAR_RATIO; }
float pitchDegToSteps(float deg) {
    // Inverted: positive degrees DECREASE corrected value (travel crosses zero)
    float steps = (float)PITCH_HOME_STEPS - (deg / PITCH_RANGE_DEG) * (float)(PITCH_LIMIT_MAX - PITCH_LIMIT_MIN);
    if (steps < PITCH_LIMIT_MIN) steps = PITCH_LIMIT_MIN;
    if (steps > PITCH_LIMIT_MAX) steps = PITCH_LIMIT_MAX;
    return steps;
}
float rollDegToSteps(float deg)    { return (deg / ROLL_RANGE_DEG) * 4096.0f; }

float m3OutputDegToContSteps(float output_deg) {
    output_deg = (output_deg > M3_OUTPUT_LIMIT) ? M3_OUTPUT_LIMIT : (output_deg < -M3_OUTPUT_LIMIT) ? -M3_OUTPUT_LIMIT : output_deg;
    return motorDegToSteps(outputToMotorDeg(output_deg));
}
float zaOutputDegToContSteps(float output_deg) {
    output_deg = (output_deg > ZA_OUTPUT_LIMIT) ? ZA_OUTPUT_LIMIT : (output_deg < -ZA_OUTPUT_LIMIT) ? -ZA_OUTPUT_LIMIT : output_deg;
    return motorDegToSteps(zaOutputToMotorDeg(output_deg));
}

// =============================================================================
//  HARDWARE I/O WRAPPERS
// =============================================================================

void DriveMotor(TIM_HandleTypeDef* htim, uint32_t channel, GPIO_TypeDef* dirPort, uint16_t dirPin, int pwm_signed) {
    int speed = abs(pwm_signed);
    if (speed > 255) speed = 255;
    if (speed < 15) speed = 0; // Motor cutoff
    
    HAL_GPIO_WritePin(dirPort, dirPin, (pwm_signed >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(htim, channel, speed);
}

void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    uint8_t data = 1 << channel;
    HAL_I2C_Master_Transmit(&hi2c1, 0x70 << 1, &data, 1, 10);
}

int readEncoderRaw(int joint) {
    tcaSelect(MUX_CHANNELS[joint]);
    uint8_t buf[2] = {0};
    if (HAL_I2C_Mem_Read(&hi2c1, 0x36 << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, buf, 2, 10) == HAL_OK) {
        return (buf[0] << 8) | buf[1];
    }
    return -1;
}

int readEncoder(int joint) {
    int raw = readEncoderRaw(joint);
    debug_raw_encoders[joint] = raw;
    if (raw == -1) return -1;
    // ENCODER_DIRECTION: +1 = encoder counts UP with movement (normal)
    //                    -1 = encoder counts DOWN with movement (invert, e.g. Pitch)
    int corrected;
    if (ENCODER_DIRECTION[joint] == -1) {
        corrected = HOME_OFFSETS[joint] - raw;  // inverted: home=0, extends to positive
    } else {
        corrected = raw - HOME_OFFSETS[joint];  // normal
    }
    while (corrected < 0)    corrected += 4096;
    while (corrected >= 4096) corrected -= 4096;
    return corrected;
}

void setLed(bool connected) {
    // Green = LD4 (PD12), Red = LD5 (PD14)
    if (connected) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    }
}

// =============================================================================
//  POT MOTOR LOGIC (ESP-A)
// =============================================================================
void PotMotor::init(int initialADC) {
    currentAngle   = adcToAngle(initialADC);
    lastAngle      = currentAngle;
    commandedAngle = currentAngle;
    prevStallAngle = currentAngle;
    lastValidAngle = currentAngle;
    ema.setEstimate((float)initialADC);
}

float PotMotor::adcToAngle(int adc) {
    bool increasing = adcTable[numPoints - 1] > adcTable[0];
    if (increasing) {
        if (adc <= adcTable[0])             return angleTable[0];
        if (adc >= adcTable[numPoints - 1]) return angleTable[numPoints - 1];
    } else {
        if (adc >= adcTable[0])             return angleTable[0];
        if (adc <= adcTable[numPoints - 1]) return angleTable[numPoints - 1];
    }
    for (int i = 0; i < numPoints - 1; i++) {
        int a0 = adcTable[i], a1 = adcTable[i + 1];
        if ((adc >= a0 && adc <= a1) || (adc <= a0 && adc >= a1)) {
            if (a1 == a0) return angleTable[i]; // Prevent division by zero
            float t = (float)(adc - a0) / (float)(a1 - a0);
            return angleTable[i] + t * (angleTable[i + 1] - angleTable[i]);
        }
    }
    return angleTable[numPoints - 1];
}

void PotMotor::resetStall() {
    stalled        = false;
    stallCounter   = 0;
    prevStallAngle = currentAngle;
}

void PotMotor::updateTrajectory(float target) {
    const float MAX_SLEW_DEG = 2.0f;
    float err = target - commandedAngle;
    if (fabsf(err) > MAX_SLEW_DEG)
        commandedAngle += (err > 0.0f) ? MAX_SLEW_DEG : -MAX_SLEW_DEG;
    else
        commandedAngle = target;
}

void PotMotor::update(float target, float dt, int rawADC) {
    if (rawADC < 10 || rawADC > 4085) { DriveMotor(htim, channel, pinDirPort, pinDirPin, 0); return; }

    float filteredADC = ema.updateEstimate((float)rawADC);
    float prevAngle   = currentAngle;
    float newAngle    = adcToAngle((int)filteredADC);

    // Angle deadband: only update currentAngle if the joint actually moved
    // more than angleDead. Prevents integrator wind-up against ADC jitter.
    if (fabsf(newAngle - lastValidAngle) >= angleDead) {
        lastValidAngle = newAngle;
    }
    currentAngle  = lastValidAngle;
    velocityCDS   = ((currentAngle - prevAngle) / dt) * 100.0f;

    if (!enabled) { DriveMotor(htim, channel, pinDirPort, pinDirPin, 0); return; }

    updateTrajectory(target);
    float error = commandedAngle - currentAngle;
    const float DEADZONE = 1.0f;

    if (stalled) {
        resetStall(); // Fallback reset in case it was somehow set
    }

    // Removed stall detection completely as requested
    if (fabsf(error) > DEADZONE) {
        float P         = kp * error;
        integral        = integral + (error * ki) * dt;
        if(integral > 100.0f) integral = 100.0f;
        if(integral < -100.0f) integral = -100.0f;
        
        float D         = -kd * ((currentAngle - lastAngle) / dt);
        float rawOutput = (P + integral + D) * pid_sign;

        // HARD LIMIT: prevent driving past calibration bounds
        float minAngle = angleTable[0];
        float maxAngle = angleTable[numPoints - 1];
        if (minAngle > maxAngle) { float t = minAngle; minAngle = maxAngle; maxAngle = t; }
        // If at/past min angle and trying to drive lower (negative error -> negative output for pot)
        if (currentAngle <= minAngle && rawOutput < 0) { rawOutput = 0; integral = 0; }
        // If at/past max angle and trying to drive higher
        if (currentAngle >= maxAngle && rawOutput > 0) { rawOutput = 0; integral = 0; }
        
        if(rawOutput > 255.0f) rawOutput = 255.0f;
        if(rawOutput < -255.0f) rawOutput = -255.0f;

        // Map minimum power
        const int MIN_POWER = 120;
        int pwm = 0;
        if (fabsf(rawOutput) > 0) {
            pwm = MIN_POWER + (int)(fabsf(rawOutput) * (255 - MIN_POWER) / 255.0f);
        }

        bool dir = (rawOutput > 0);
        if (reverseDir) dir = !dir;
        
        int signed_pwm = dir ? pwm : -pwm;
        DriveMotor(htim, channel, pinDirPort, pinDirPin, signed_pwm);

        currentPWM = pwm;
        lastAngle  = currentAngle;
    } else {
        DriveMotor(htim, channel, pinDirPort, pinDirPin, 0);
        integral   = 0;
        currentPWM = 0;
        resetStall();
    }
}

// =============================================================================
//  SYSTEM INITIALIZATION
// =============================================================================
extern "C" {

void RobotCore_Init(void) {
    // Start PWM Timers
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // Start ADC DMA — runs continuously in background, circular
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, ADC_OVS_SIZE * 2);

    // Read initial oversampled ADC values for motor init
    // Wait one full buffer fill (~4ms at 84-cycle sample time, 168MHz APB2/2)
    HAL_Delay(10);
    uint16_t adc_init[2];
    adc_init[0] = adc_read_oversampled(0);
    adc_init[1] = adc_read_oversampled(1);

    pots[0]->init(adc_init[0]);
    pots[1]->init(adc_init[1]);
    pot_targets[0] = pots[0]->currentAngle;
    pot_targets[1] = pots[1]->currentAngle;

    // Initialize dynamic PID constants for encoders
    for (int i = 0; i < 4; i++) {
        current_Kp_enc[i] = Kp_enc[i];
        current_Ki_enc[i] = Ki_enc[i];
        current_Kd_enc[i] = Kd_enc[i];
    }

    // Start UART DMA
    HAL_UART_Receive_DMA(&huart4, rx_buffer, RX_BUF_SIZE);
    
    current_fb.header[0] = 'F'; current_fb.header[1] = 'B';
    memset(current_fb.motor_vel, 0, sizeof(current_fb.motor_vel));
    current_fb.flags = 0;
    current_fb.checksum = 0;
}

void emergencyStopAll() {
    systemActive   = false;
    comms_ok       = false;
    for (int i = 0; i < 2; i++) {
        pots[i]->enabled = false;
        DriveMotor(pots[i]->htim, pots[i]->channel, pots[i]->pinDirPort, pots[i]->pinDirPin, 0);
    }
    for (int i = 0; i < 4; i++) {
        enc_enabled[i] = false;
        DriveMotor(enc_joints[i].htim, enc_joints[i].channel, enc_joints[i].dirPort, enc_joints[i].dirPin, 0);
        integral[i] = 0;
    }
}

// =============================================================================
// =============================================================================
bool parseCommandPacket(uint8_t* buf, uint32_t buf_size, uint32_t* tail, uint32_t head) {
    bool found = false;
    while (*tail != head) {
        if (buf[*tail] == 'S' || buf[*tail] == 'D' || buf[*tail] == 'K' || buf[*tail] == 'R') {
            uint32_t bytes_available = (head >= *tail) ? (head - *tail) : (buf_size - *tail + head);
            if (bytes_available >= sizeof(CommandPacketUnified)) {
                uint8_t pkt[sizeof(CommandPacketUnified)];
                for (size_t i = 0; i < sizeof(CommandPacketUnified); i++) {
                    pkt[i] = buf[(*tail + i) % buf_size];
                }
                CommandPacketUnified* cmd = (CommandPacketUnified*)pkt;
                if (cmd->header[1] == 'T' && cmd->footer == '\n') {
                    // Validate XOR checksum: covers bytes [0..26]
                    uint8_t expected = compute_checksum(pkt, offsetof(CommandPacketUnified, checksum));
                    if (expected != cmd->checksum) {
                        // Bad checksum — skip this byte and keep scanning
                        *tail = (*tail + 1) % buf_size;
                        continue;
                    }
                    memcpy(&current_cmd, cmd, sizeof(CommandPacketUnified));
                    last_valid_cmd_time = osKernelGetTickCount();
                    found = true;
                    *tail = (*tail + sizeof(CommandPacketUnified)) % buf_size;
                    continue;
                }
            } else {
                break;
            }
        }
        *tail = (*tail + 1) % buf_size;
    }
    return found;
}

void RobotCore_ROSTask(void) {
    uint32_t uart_head = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);
    bool uart_pkt = parseCommandPacket(rx_buffer, RX_BUF_SIZE, &rx_tail, uart_head);
    
    uint32_t cur_usb_head = usb_rx_head; // Safe snapshot
    bool usb_pkt = parseCommandPacket(usb_rx_buffer, USB_RX_BUF_SIZE, &usb_rx_tail, cur_usb_head);

    if (uart_pkt || usb_pkt) {
        if (!comms_ok) {
            comms_ok = true;
            setLed(true);
            
            // Clear latching safety faults on reconnect
            pitch_frozen = false;
            z_collision_fault = false;
            za_stall_counter = 0;
            za_profile_vel = 0;
            m3_profile_vel = 0;
            
            // Re-enable ESP-A (DMA Oversampling)
            uint16_t adcs[2];
            adcs[0] = adc_read_oversampled(0);
            adcs[1] = adc_read_oversampled(1);
            
            for (int i=0; i<2; i++) {
                pots[i]->currentAngle = pots[i]->adcToAngle(adcs[i]);
                pots[i]->commandedAngle = pots[i]->currentAngle;
                
                // Ensure targets can't exceed calibration bounds,
                // otherwise a corrupt re-enable target can cause runaway.
                if (pot_targets[0] < 47.4f) pot_targets[0] = 47.4f;
                if (pot_targets[0] > 74.0f) pot_targets[0] = 74.0f;
                if (pot_targets[1] < 1.0f)  pot_targets[1] = 1.0f;
                if (pot_targets[1] > 61.2f) pot_targets[1] = 61.2f;

                pots[i]->integral = 0;
                pots[i]->enabled = true;
                pots[i]->resetStall();
            }
            
            // Re-enable ESP-B
            for (int i=0; i<4; i++) {
                if (!JOINT_ALLOWED[i]) continue;  // Skip disabled joints
                int val = readEncoder(i);
                if (val != -1) {
                    enc_enabled[i] = true; current_pos[i] = val;
                    commanded_pos[i] = val; prev_pos[i] = val; target_pos[i] = val; integral[i] = 0;
                    if (i == 0) {
                        int val = readEncoder(0);
                        if(val != -1) {
                            m3_prev_raw = val;
                            // Home sector: if corrected val>2048 the arm is in the negative half.
                            // Set wrap_count=-1 so the running formula (wrap_count*4096)+raw gives
                            // the correct signed value automatically every tick.
                            if (val > 2048) {
                                m3_wrap_count = -1;
                            } else {
                                m3_wrap_count = 0;
                            }
                            m3_continuous_pos = (double)m3_wrap_count * 4096.0 + (double)val;
                        }
                        m3_continuous_cmd = m3_continuous_pos; m3_continuous_prev = m3_continuous_pos; m3_continuous_target = m3_continuous_pos;
                    }
                    if (i == 3) {
                        int val = readEncoder(3);
                        if(val != -1) {
                            za_prev_raw = val;
                            // Home sector: if corrected val>2048 the arm is in the negative half.
                            // Set wrap_count=-1 so the running formula (wrap_count*4096)+raw gives
                            // the correct signed value automatically every tick.
                            if (val > 2048) {
                                za_wrap_count = -1;
                            } else {
                                za_wrap_count = 0;
                            }
                            za_continuous_pos = (double)za_wrap_count * 4096.0 + (double)val;
                        }
                        za_continuous_cmd = za_continuous_pos; za_continuous_prev = za_continuous_pos; za_continuous_target = za_continuous_pos;
                    }
                }
            }
        }
        
        // Target Update
        if (current_cmd.header[0] == 'S') {
            is_open_loop_mode = false;
            pot_targets[0] = (float)current_cmd.motor_cmd[0] / 1000.0f;
            pot_targets[1] = (float)current_cmd.motor_cmd[1] / 1000.0f;
            if (pot_targets[0] < 47.4f) pot_targets[0] = 47.4f; else if (pot_targets[0] > 74.0f)  pot_targets[0] = 74.0f;
            if (pot_targets[1] <  1.0f) pot_targets[1] =  1.0f; else if (pot_targets[1] > 61.2f)  pot_targets[1] = 61.2f;
            
            m3_continuous_target = (double)m3OutputDegToContSteps((float)current_cmd.motor_cmd[2]/1000.0f);
            target_pos[1] = pitchDegToSteps((float)current_cmd.motor_cmd[3]/1000.0f);
            if (target_pos[1] < PITCH_LIMIT_MIN) target_pos[1] = PITCH_LIMIT_MIN;
            if (target_pos[1] > PITCH_LIMIT_MAX) target_pos[1] = PITCH_LIMIT_MAX;
            target_pos[2] = wrapValue(rollDegToSteps((float)current_cmd.motor_cmd[4]/1000.0f));
            za_continuous_target = (double)zaOutputDegToContSteps((float)current_cmd.motor_cmd[5]/1000.0f);
        } else if (current_cmd.header[0] == 'D') {
            // Passive debugging mode. Do not change physical setpoints, just loop.
            is_open_loop_mode = false;
        } else if (current_cmd.header[0] == 'R') {
            // Passive Request mode. Just reply with angles, do not move.
            is_open_loop_mode = false;
        } else if (current_cmd.header[0] == 'K') {
            int target_var = current_cmd.motor_cmd[0];
            float p = (float)current_cmd.motor_cmd[1] / 1000.0f;
            float i = (float)current_cmd.motor_cmd[2] / 1000.0f;
            float d = (float)current_cmd.motor_cmd[3] / 1000.0f;
            if (target_var == 0)      { pots[0]->kp = p; pots[0]->ki = i; pots[0]->kd = d; }
            else if (target_var == 1) { pots[1]->kp = p; pots[1]->ki = i; pots[1]->kd = d; }
            else if (target_var == 2) { current_Kp_enc[0] = p; current_Ki_enc[0] = i; current_Kd_enc[0] = d; }
            else if (target_var == 3) { current_Kp_enc[1] = p; current_Ki_enc[1] = i; current_Kd_enc[1] = d; }
            else if (target_var == 4) { current_Kp_enc[2] = p; current_Ki_enc[2] = i; current_Kd_enc[2] = d; }
            else if (target_var == 5) { current_Kp_enc[3] = p; current_Ki_enc[3] = i; current_Kd_enc[3] = d; }
            is_open_loop_mode = false;
        }
    }
    
    // Watchdog
    if (comms_ok && (osKernelGetTickCount() - last_valid_cmd_time > COMMS_TIMEOUT_MS)) {
        emergencyStopAll();
        setLed(false);
    }
}

// =============================================================================
//  50Hz PID CONTROL LOOP (HIGH PRIORITY TASK)
// =============================================================================
void RobotCore_PIDTask(void) {
    const float dt = 0.02f; // 50Hz
    
    // 1. Read ADCs for ESP-A (DMA Oversampling — non-blocking)
    uint16_t adcs[2];
    adcs[0] = adc_read_oversampled(0);
    adcs[1] = adc_read_oversampled(1);
    
    debug_raw_adcs[0] = adcs[0];
    debug_raw_adcs[1] = adcs[1];
    
    // 2. Read Encoders for ESP-B
    if (enc_enabled[0]) {
        int raw = readEncoder(0);
        if (raw != -1) {
            int delta = raw - m3_prev_raw;
            if (delta > 2048) m3_wrap_count--; else if (delta < -2048) m3_wrap_count++;
            m3_prev_raw = raw;
            m3_continuous_pos = (double)m3_wrap_count * 4096.0 + (double)raw;
            current_pos[0] = (float)raw;
            sensor_read_ok[0] = true;
            sensor_fail_count[0] = 0;
        } else {
            if (++sensor_fail_count[0] > MAX_SENSOR_FAILS) emergencyStopAll();
        }
    }
    for (int i=1; i<3; i++) {
        if (!enc_enabled[i]) continue;
        int raw = readEncoder(i);
        if (raw != -1) { current_pos[i] = raw; sensor_fail_count[i] = 0; sensor_read_ok[i] = true; }
        else if (++sensor_fail_count[i] > MAX_SENSOR_FAILS) emergencyStopAll();
    }
    if (enc_enabled[3]) {
        int raw = readEncoder(3);
        if (raw != -1) {
            int delta = raw - za_prev_raw;
            if (delta > 2048) za_wrap_count--; else if (delta < -2048) za_wrap_count++;
            za_prev_raw = raw;
            za_continuous_pos = (double)za_wrap_count * 4096.0 + (double)raw;
            current_pos[3] = (float)raw;
            sensor_read_ok[3] = true;
            sensor_fail_count[3] = 0;
        } else {
            if (++sensor_fail_count[3] > MAX_SENSOR_FAILS) emergencyStopAll();
        }
    }
    
    // 3. Update ESP-A Motors
    pots[0]->update(pot_targets[0], dt, adcs[0]);
    pots[1]->update(pot_targets[1], dt, adcs[1]);
    
    // 4. Update ESP-B Trajectories — Trapezoidal Motion Profiles
    // ── M3 ────────────────────────────────────────────────────────────────────
    if (enc_enabled[0]) {
        float err = (float)(m3_continuous_target - m3_continuous_cmd);
        // Acceleration step toward desired velocity
        float desired_vel = (err > 0) ? M3_PROF_MAX_VEL : (err < 0) ? -M3_PROF_MAX_VEL : 0.0f;
        // Deceleration: if we're close, reduce desired vel proportionally
        float brake_dist = (m3_profile_vel * m3_profile_vel) / (2.0f * M3_PROF_MAX_ACCEL);
        if (fabsf(err) < brake_dist + 1.0f) {
            desired_vel = (err > 0) ? M3_PROF_MAX_VEL * (fabsf(err) / (brake_dist + 1.0f))
                                    : -M3_PROF_MAX_VEL * (fabsf(err) / (brake_dist + 1.0f));
        }
        // Clamp acceleration
        float dv = desired_vel - m3_profile_vel;
        if (fabsf(dv) > M3_PROF_MAX_ACCEL) dv = (dv > 0) ? M3_PROF_MAX_ACCEL : -M3_PROF_MAX_ACCEL;
        m3_profile_vel += dv;
        // At target — stop
        if (fabsf(err) < 0.5f) { m3_profile_vel = 0; m3_continuous_cmd = m3_continuous_target; }
        else { m3_continuous_cmd += (double)m3_profile_vel; }
        if (m3_continuous_cmd > (double)M3_CONT_LIMIT)  { m3_continuous_cmd = (double)M3_CONT_LIMIT;  m3_profile_vel = 0; }
        if (m3_continuous_cmd < -(double)M3_CONT_LIMIT) { m3_continuous_cmd = -(double)M3_CONT_LIMIT; m3_profile_vel = 0; }
    } else { m3_continuous_cmd = m3_continuous_pos; m3_profile_vel = 0; }

    // Pitch trajectory (LINEAR — no wrapping, corrected values are continuous)
    if (enc_enabled[1]) {
        float e = target_pos[1] - commanded_pos[1];
        if (fabsf(e) > MAX_SLEW_RATE) {
            commanded_pos[1] += (e > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
        } else commanded_pos[1] = target_pos[1];
        if (commanded_pos[1] < PITCH_LIMIT_MIN) commanded_pos[1] = PITCH_LIMIT_MIN;
        if (commanded_pos[1] > PITCH_LIMIT_MAX) commanded_pos[1] = PITCH_LIMIT_MAX;
    } else { commanded_pos[1] = current_pos[1]; }

    // Roll trajectory (CIRCULAR — uses wrap)
    if (enc_enabled[2]) {
        float e = wrapError(target_pos[2] - commanded_pos[2]);
        if (fabsf(e) > MAX_SLEW_RATE) {
            commanded_pos[2] += (e > 0) ? MAX_SLEW_RATE : -MAX_SLEW_RATE;
            commanded_pos[2] = wrapValue(commanded_pos[2]);
        } else commanded_pos[2] = target_pos[2];
    } else { commanded_pos[2] = current_pos[2]; }

    // ── Z-Axis ────────────────────────────────────────────────────────────────
    if (enc_enabled[3]) {
        float err = (float)(za_continuous_target - za_continuous_cmd);
        // Desired velocity based on direction
        float desired_vel = (err > 0) ? ZA_PROF_MAX_VEL : (err < 0) ? -ZA_PROF_MAX_VEL : 0.0f;
        // Deceleration: scale velocity to zero as we approach target
        float brake_dist = (za_profile_vel * za_profile_vel) / (2.0f * ZA_PROF_MAX_ACCEL);
        if (fabsf(err) < brake_dist + 1.0f) {
            desired_vel = (err > 0) ? ZA_PROF_MAX_VEL * (fabsf(err) / (brake_dist + 1.0f))
                                    : -ZA_PROF_MAX_VEL * (fabsf(err) / (brake_dist + 1.0f));
        }
        // Clamp acceleration
        float dv = desired_vel - za_profile_vel;
        if (fabsf(dv) > ZA_PROF_MAX_ACCEL) dv = (dv > 0) ? ZA_PROF_MAX_ACCEL : -ZA_PROF_MAX_ACCEL;
        za_profile_vel += dv;
        // At target — stop
        if (fabsf(err) < 0.5f) { za_profile_vel = 0; za_continuous_cmd = za_continuous_target; }
        else { za_continuous_cmd += (double)za_profile_vel; }
        if (za_continuous_cmd > (double)ZA_CONT_LIMIT)  { za_continuous_cmd = (double)ZA_CONT_LIMIT;  za_profile_vel = 0; }
        if (za_continuous_cmd < -(double)ZA_CONT_LIMIT) { za_continuous_cmd = -(double)ZA_CONT_LIMIT; za_profile_vel = 0; }
    } else { za_continuous_cmd = za_continuous_pos; za_profile_vel = 0; }
    
    // 5. Compute ESP-B PIDs
    // M3
    if (enc_enabled[0]) {
        float error_dz = applyDeadzone((float)(m3_continuous_cmd - m3_continuous_pos), ERROR_DEADZONE);
        float p_term = current_Kp_enc[0] * error_dz;
        if (fabsf(pid_output[0]) < PID_OUTPUT_MAX) {
            integral[0] += error_dz * dt;
            if(integral[0] > INTEGRAL_MAX) integral[0] = INTEGRAL_MAX;
            if(integral[0] < -INTEGRAL_MAX) integral[0] = -INTEGRAL_MAX;
        }
        if (fabsf(error_dz) < 0.1f) integral[0] = 0;
        float d_term = 0.0f;
        if (sensor_read_ok[0]) {
            d_term = current_Kd_enc[0] * (float)((m3_continuous_pos - m3_continuous_prev) / (double)dt);
            m3_continuous_prev = m3_continuous_pos;
        }
        pid_output[0] = p_term + current_Ki_enc[0] * integral[0] - d_term;
        if(pid_output[0] > PID_OUTPUT_MAX) pid_output[0] = PID_OUTPUT_MAX;
        if(pid_output[0] < -PID_OUTPUT_MAX) pid_output[0] = -PID_OUTPUT_MAX;

        // --- M3 Physical Hard Stop & Soft limits ---
        float m3_pos = (float)m3_continuous_pos;
        if (m3_pos >= M3_CONT_LIMIT && pid_output[0] > 0) { pid_output[0] = 0; integral[0] = 0; }
        if (m3_pos <= -M3_CONT_LIMIT && pid_output[0] < 0) { pid_output[0] = 0; integral[0] = 0; }
        
        if (m3_pos > (M3_CONT_LIMIT - M3_CONT_SOFT_ZONE) && m3_pos < M3_CONT_LIMIT && pid_output[0] > 0)
            pid_output[0] *= (M3_CONT_LIMIT - m3_pos) / M3_CONT_SOFT_ZONE;
        if (m3_pos < (-M3_CONT_LIMIT + M3_CONT_SOFT_ZONE) && m3_pos > -M3_CONT_LIMIT && pid_output[0] < 0)
            pid_output[0] *= (m3_pos + M3_CONT_LIMIT) / M3_CONT_SOFT_ZONE;
    } else { pid_output[0] = 0; integral[0] = 0; }
    
    // Pitch PID (LINEAR — no wrapError, corrected values are continuous)
    if (enc_enabled[1]) {
        float error_dz = applyDeadzone(commanded_pos[1] - current_pos[1], ERROR_DEADZONE);
        float p_term = current_Kp_enc[1] * error_dz;
        if (fabsf(pid_output[1]) < PID_OUTPUT_MAX) {
            integral[1] += error_dz * dt;
            if(integral[1] > INTEGRAL_MAX) integral[1] = INTEGRAL_MAX;
            if(integral[1] < -INTEGRAL_MAX) integral[1] = -INTEGRAL_MAX;
        }
        if (fabsf(error_dz) < 0.1f) integral[1] = 0;
        float d_term = 0.0f;
        if (sensor_read_ok[1]) {
            d_term = current_Kd_enc[1] * ((current_pos[1] - prev_pos[1]) / dt);
            prev_pos[1] = current_pos[1];
        }
        pid_output[1] = p_term + (current_Ki_enc[1] * integral[1]) - d_term;
        if(pid_output[1] > PID_OUTPUT_MAX) pid_output[1] = PID_OUTPUT_MAX;
        if(pid_output[1] < -PID_OUTPUT_MAX) pid_output[1] = -PID_OUTPUT_MAX;
    } else { pid_output[1] = 0; integral[1] = 0; }

    // Roll PID (CIRCULAR — uses wrapError)
    if (enc_enabled[2]) {
        float error_dz = applyDeadzone(wrapError(commanded_pos[2] - current_pos[2]), ERROR_DEADZONE);
        float p_term = current_Kp_enc[2] * error_dz;
        if (fabsf(pid_output[2]) < PID_OUTPUT_MAX) {
            integral[2] += error_dz * dt;
            if(integral[2] > INTEGRAL_MAX) integral[2] = INTEGRAL_MAX;
            if(integral[2] < -INTEGRAL_MAX) integral[2] = -INTEGRAL_MAX;
        }
        if (fabsf(error_dz) < 0.1f) integral[2] = 0;
        float d_term = 0.0f;
        if (sensor_read_ok[2]) {
            d_term = current_Kd_enc[2] * (wrapError(current_pos[2] - prev_pos[2]) / dt);
            prev_pos[2] = current_pos[2];
        }
        pid_output[2] = p_term + (current_Ki_enc[2] * integral[2]) - d_term;
        if(pid_output[2] > PID_OUTPUT_MAX) pid_output[2] = PID_OUTPUT_MAX;
        if(pid_output[2] < -PID_OUTPUT_MAX) pid_output[2] = -PID_OUTPUT_MAX;
    } else { pid_output[2] = 0; integral[2] = 0; }
    
    // Z-Axis
    if (enc_enabled[3]) {
        float error_dz = applyDeadzone((float)(za_continuous_cmd - za_continuous_pos), ERROR_DEADZONE);
        float p_term = current_Kp_enc[3] * error_dz;
        if (fabsf(pid_output[3]) < ZA_PID_OUTPUT_MAX) {
            integral[3] += error_dz * dt;
            if(integral[3] > INTEGRAL_MAX) integral[3] = INTEGRAL_MAX;
            if(integral[3] < -INTEGRAL_MAX) integral[3] = -INTEGRAL_MAX;
        }
        if (fabsf(error_dz) < 0.1f) integral[3] = 0;
        float d_term = 0.0f;
        float actual_velocity = 0.0f;
        if (sensor_read_ok[3]) {
            actual_velocity = (float)((za_continuous_pos - za_continuous_prev) / (double)dt);
            d_term = current_Kd_enc[3] * actual_velocity;
            za_continuous_prev = za_continuous_pos;
        }
        pid_output[3] = p_term + (current_Ki_enc[3] * integral[3]) - d_term;
        if(pid_output[3] > ZA_PID_OUTPUT_MAX) pid_output[3] = ZA_PID_OUTPUT_MAX;
        if(pid_output[3] < -ZA_PID_OUTPUT_MAX) pid_output[3] = -ZA_PID_OUTPUT_MAX;

        // --- Z-axis Physical Hard Stop & Soft limits ---
        float za_pos = za_continuous_pos;
        if (za_pos >= ZA_CONT_LIMIT && pid_output[3] > 0) { pid_output[3] = 0; integral[3] = 0; }
        if (za_pos <= -ZA_CONT_LIMIT && pid_output[3] < 0) { pid_output[3] = 0; integral[3] = 0; }
        
        // Continuous Proportional Soft Stop (Smooth braking at physical extremes)
        if (za_pos > (ZA_CONT_LIMIT - ZA_CONT_SOFT_ZONE) && za_pos < ZA_CONT_LIMIT && pid_output[3] > 0)
            pid_output[3] *= (ZA_CONT_LIMIT - za_pos) / ZA_CONT_SOFT_ZONE;
        if (za_pos < (-ZA_CONT_LIMIT + ZA_CONT_SOFT_ZONE) && za_pos > -ZA_CONT_LIMIT && pid_output[3] < 0)
            pid_output[3] *= (za_pos + ZA_CONT_LIMIT) / ZA_CONT_SOFT_ZONE;
        
        // --- User Docking Zone Limits ---
        // Converts the chosen physical normal limit to continuous raw encoder steps
        float normal_limit_steps = (ZA_NORMAL_LIMIT_DEG * ZA_GEAR_RATIO / 360.0f) * 4096.0f;
        if (fabsf(za_pos) > normal_limit_steps) {
            pid_output[3] *= ZA_DOCKING_SCALE;
        }

        // --- Virtual Current Sensing (Collision Detection) ---
        if (ZA_COLLISION_ENABLED) {
            // If the integral windup is high BUT the arm is barely moving, we've hit an obstacle.
            if (fabsf(integral[3]) > ZA_COLLISION_INTEGRAL && fabsf(actual_velocity) < ZA_COLLISION_VELOCITY) {
                za_stall_counter++;
                if (za_stall_counter > ZA_COLLISION_TIMEOUT) {
                    z_collision_fault = true;
                    za_continuous_cmd = za_continuous_pos; // Zero the effort immediately
                    za_continuous_target = za_continuous_pos;
                    integral[3] = 0;
                }
            } else {
                za_stall_counter = 0; // Reset immediately if we start moving or pressure releases
            }
            
            // If user pulls the slider back in the opposite direction of the error, unlatch fault.
            if (z_collision_fault && fabsf(za_continuous_target - za_continuous_pos) > 10.0f) {
                z_collision_fault = false;
            }
        } else {
            za_stall_counter = 0;
            z_collision_fault = false;
        }

    } else { pid_output[3] = 0; integral[3] = 0; za_stall_counter = 0; }
    
    // 6. Apply ESP-B PWM Outputs
    if (is_open_loop_mode) {
        DriveMotor(pots[0]->htim, pots[0]->channel, pots[0]->pinDirPort, pots[0]->pinDirPin, open_loop_pwm[0]);
        DriveMotor(pots[1]->htim, pots[1]->channel, pots[1]->pinDirPort, pots[1]->pinDirPin, open_loop_pwm[1]);
        DriveMotor(enc_joints[0].htim, enc_joints[0].channel, enc_joints[0].dirPort, enc_joints[0].dirPin, open_loop_pwm[2]);
        DriveMotor(enc_joints[1].htim, enc_joints[1].channel, enc_joints[1].dirPort, enc_joints[1].dirPin, open_loop_pwm[3]);
        DriveMotor(enc_joints[2].htim, enc_joints[2].channel, enc_joints[2].dirPort, enc_joints[2].dirPin, open_loop_pwm[4]);
        DriveMotor(enc_joints[3].htim, enc_joints[3].channel, enc_joints[3].dirPort, enc_joints[3].dirPin, open_loop_pwm[5]);
    } else {
        if (enc_enabled[0]) {
            DriveMotor(enc_joints[0].htim, enc_joints[0].channel, enc_joints[0].dirPort, enc_joints[0].dirPin, pid_output[0] * M3_PID_SIGN);
        } else {
            DriveMotor(enc_joints[0].htim, enc_joints[0].channel, enc_joints[0].dirPort, enc_joints[0].dirPin, 0);
        }
        
        bool pitch_active = enc_enabled[1] && sensor_read_ok[1];
        bool roll_active  = enc_enabled[2] && sensor_read_ok[2];
        if (pitch_active || roll_active) {
            float pitch_cmd = pitch_active ? (pid_output[1] * PITCH_PID_SIGN) : 0.0f;
            
            // RAW ENCODER DEAD-ZONE SAFETY for pitch:
            // If the raw encoder is in the dead zone (between endpoints), kill pitch.
            if (pitch_active) {
                int pitch_raw = debug_raw_encoders[1];
                if (pitch_raw > PITCH_RAW_MIN && pitch_raw < PITCH_RAW_MAX) {
                    pitch_cmd = 0;  // In dead zone = beyond physical limits
                }
            }
            
            // PITCH FREEZE: if corrected position exceeds limits by >5°, freeze completely
            if (pitch_active && !pitch_frozen) {
                if (current_pos[1] < (PITCH_LIMIT_MIN - PITCH_FREEZE_TOLERANCE) ||
                    current_pos[1] > (PITCH_LIMIT_MAX + PITCH_FREEZE_TOLERANCE)) {
                    pitch_frozen = true;
                    pitch_cmd = 0;
                }
            }
            if (pitch_frozen) pitch_cmd = 0;
            float roll_cmd  = roll_active  ? (pid_output[2] * ROLL_PID_SIGN) : 0.0f;
            float mA = pitch_cmd + roll_cmd;
            float mB = -pitch_cmd + roll_cmd;
            float max_val = fmaxf(fabsf(mA), fabsf(mB));
            if (max_val > 255.0f) { float s = 255.0f / max_val; mA *= s; mB *= s; }
            DriveMotor(enc_joints[1].htim, enc_joints[1].channel, enc_joints[1].dirPort, enc_joints[1].dirPin, mA * WRIST_MOTOR_A_SIGN);
            DriveMotor(enc_joints[2].htim, enc_joints[2].channel, enc_joints[2].dirPort, enc_joints[2].dirPin, mB * WRIST_MOTOR_B_SIGN);
        } else {
            DriveMotor(enc_joints[1].htim, enc_joints[1].channel, enc_joints[1].dirPort, enc_joints[1].dirPin, 0);
            DriveMotor(enc_joints[2].htim, enc_joints[2].channel, enc_joints[2].dirPort, enc_joints[2].dirPin, 0);
        }
        
        if (enc_enabled[3]) {
            DriveMotor(enc_joints[3].htim, enc_joints[3].channel, enc_joints[3].dirPort, enc_joints[3].dirPin, pid_output[3] * ZA_PID_SIGN);
        } else {
            DriveMotor(enc_joints[3].htim, enc_joints[3].channel, enc_joints[3].dirPort, enc_joints[3].dirPin, 0);
        }
    }

    // 7. Transmit Feedback
    if (current_cmd.header[0] == 'D' || current_cmd.header[0] == 'P') {
        current_fb.motor_pos[0] = (int32_t)debug_raw_adcs[0];
        current_fb.motor_pos[1] = (int32_t)debug_raw_adcs[1];
        current_fb.motor_pos[2] = (int32_t)debug_raw_encoders[0];
        current_fb.motor_pos[3] = (int32_t)debug_raw_encoders[1];
        current_fb.motor_pos[4] = (int32_t)debug_raw_encoders[2];
        current_fb.motor_pos[5] = (int32_t)debug_raw_encoders[3];
        // In debug mode, velocities are zero
        memset(current_fb.motor_vel, 0, sizeof(current_fb.motor_vel));
    } else {
        current_fb.motor_pos[0] = (int32_t)(pots[0]->currentAngle * 1000.0f);
        current_fb.motor_pos[1] = (int32_t)(pots[1]->currentAngle * 1000.0f);
        
        // Convert continuous hardware steps back to Output Degrees (* 1000)
        current_fb.motor_pos[2] = (int32_t)(((m3_continuous_pos / 4096.0f) * 360.0f / M3_GEAR_RATIO) * 1000.0f); 
        // Pitch feedback: inverted formula (positive degrees = decreasing corrected)
        if (pitch_frozen) {
            current_fb.motor_pos[3] = SENTINEL_VAL;
        } else {
            current_fb.motor_pos[3] = (int32_t)(((float)PITCH_HOME_STEPS - current_pos[1]) / (float)(PITCH_LIMIT_MAX - PITCH_LIMIT_MIN) * PITCH_RANGE_DEG * 1000.0f);
        }
        current_fb.motor_pos[4] = (int32_t)(((current_pos[2] / 4096.0f) * ROLL_RANGE_DEG) * 1000.0f);
        if (z_collision_fault) {
            current_fb.motor_pos[5] = SENTINEL_VAL;
        } else {
            current_fb.motor_pos[5] = (int32_t)(((za_continuous_pos / 4096.0f) * 360.0f / ZA_GEAR_RATIO) * 1000.0f);
        }

        // ── Velocities (milli-deg/s) — uses dedicated fb_prev_* state ─────────
        const float DT = 0.02f;  // 50 Hz tick

        // Pots: velocityCDS already in deg/s from PotMotor::update()
        current_fb.motor_vel[0] = (int32_t)(pots[0]->velocityCDS * 1000.0f);
        current_fb.motor_vel[1] = (int32_t)(pots[1]->velocityCDS * 1000.0f);

        // M3: continuous position → output deg/s
        fb_vel_m3 = ((m3_continuous_pos - fb_prev_m3) / 4096.0f * 360.0f / M3_GEAR_RATIO) / DT;
        fb_prev_m3 = m3_continuous_pos;
        current_fb.motor_vel[2] = (int32_t)(fb_vel_m3 * 1000.0f);

        // Pitch: inverted axis, range-scaled (matches position formula)
        fb_vel_pitch = (-(current_pos[1] - fb_prev_pitch)
                        / (float)(PITCH_LIMIT_MAX - PITCH_LIMIT_MIN)
                        * PITCH_RANGE_DEG) / DT;
        fb_prev_pitch = current_pos[1];
        current_fb.motor_vel[3] = (int32_t)(fb_vel_pitch * 1000.0f);

        // Roll: wrap-aware delta → ROLL_RANGE_DEG
        fb_vel_roll = ((wrapError(current_pos[2] - fb_prev_roll) / 4096.0f) * ROLL_RANGE_DEG) / DT;
        fb_prev_roll = current_pos[2];
        current_fb.motor_vel[4] = (int32_t)(fb_vel_roll * 1000.0f);

        // Z: continuous position → output deg/s
        fb_vel_z = ((za_continuous_pos - fb_prev_z) / 4096.0f * 360.0f / ZA_GEAR_RATIO) / DT;
        fb_prev_z = za_continuous_pos;
        current_fb.motor_vel[5] = (int32_t)(fb_vel_z * 1000.0f);
    }

    // ── Flags byte ────────────────────────────────────────────────────────────
    current_fb.flags = 0;
    if (pitch_frozen)      current_fb.flags |= FB_FLAG_PITCH_FROZEN;
    if (z_collision_fault) current_fb.flags |= FB_FLAG_Z_COLLISION;

    // ── Checksum: XOR of bytes [0..50] ────────────────────────────────────────
    current_fb.checksum = compute_checksum(
        (const uint8_t*)&current_fb,
        offsetof(FeedbackPacketUnified, checksum));

    HAL_UART_Transmit(&huart4, (uint8_t*)&current_fb, sizeof(FeedbackPacketUnified), 10);
    CDC_Transmit_FS((uint8_t*)&current_fb, sizeof(FeedbackPacketUnified));
}

} // extern "C"
</file>

<file path="Core/Src/spi.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
</file>

<file path="Core/Src/stm32f4xx_hal_msp.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
</file>

<file path="Core/Src/stm32f4xx_hal_timebase_tim.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_tim.c
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim6;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM6 as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock, uwAPB1Prescaler = 0U;

  uint32_t              uwPrescalerValue = 0U;
  uint32_t              pFLatency;

  HAL_StatusTypeDef     status;

  /* Enable TIM6 clock */
  __HAL_RCC_TIM6_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Get APB1 prescaler */
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;
  /* Compute TIM6 clock */
  if (uwAPB1Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq();
  }

  /* Compute the prescaler value to have TIM6 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM6 */
  htim6.Instance = TIM6;

  /* Initialize TIMx peripheral as follow:
   * Period = [(TIM6CLK/1000) - 1]. to have a (1/1000) s time base.
   * Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
   * ClockDivision = 0
   * Counter direction = Up
   */
  htim6.Init.Period = (1000000U / 1000U) - 1U;
  htim6.Init.Prescaler = uwPrescalerValue;
  htim6.Init.ClockDivision = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  status = HAL_TIM_Base_Init(&htim6);
  if (status == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    status = HAL_TIM_Base_Start_IT(&htim6);
    if (status == HAL_OK)
    {
    /* Enable the TIM6 global Interrupt */
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
      /* Configure the SysTick IRQ priority */
      if (TickPriority < (1UL << __NVIC_PRIO_BITS))
      {
        /* Configure the TIM IRQ priority */
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0U);
        uwTickPrio = TickPriority;
      }
      else
      {
        status = HAL_ERROR;
      }
    }
  }

 /* Return function status */
  return status;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM6 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM6 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM6 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM6 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}
</file>

<file path="Core/Src/stm32f4xx_it.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
</file>

<file path="Core/Src/syscalls.c">
/**
 ******************************************************************************
 * @file      syscalls.c
 * @author    Auto-generated by STM32CubeIDE
 * @brief     STM32CubeIDE Minimal System calls file
 *
 *            For more information about which c-functions
 *            need which of these lowlevel functions
 *            please consult the Newlib libc-manual
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2020-2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>


/* Variables */
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));


char *__env[1] = { 0 };
char **environ = __env;


/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
  return 1;
}

int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}

void _exit (int status)
{
  _kill(status, -1);
  while (1) {}    /* Make sure we hang here */
}

__attribute__((weak)) int _read(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    *ptr++ = __io_getchar();
  }

  return len;
}

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
}

int _close(int file)
{
  (void)file;
  return -1;
}


int _fstat(int file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  (void)file;
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}

int _open(char *path, int flags, ...)
{
  (void)path;
  (void)flags;
  /* Pretend like we always fail */
  return -1;
}

int _wait(int *status)
{
  (void)status;
  errno = ECHILD;
  return -1;
}

int _unlink(char *name)
{
  (void)name;
  errno = ENOENT;
  return -1;
}

int _times(struct tms *buf)
{
  (void)buf;
  return -1;
}

int _stat(char *file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _link(char *old, char *new)
{
  (void)old;
  (void)new;
  errno = EMLINK;
  return -1;
}

int _fork(void)
{
  errno = EAGAIN;
  return -1;
}

int _execve(char *name, char **argv, char **env)
{
  (void)name;
  (void)argv;
  (void)env;
  errno = ENOMEM;
  return -1;
}
</file>

<file path="Core/Src/sysmem.c">
/**
 ******************************************************************************
 * @file      sysmem.c
 * @author    Generated by STM32CubeIDE
 * @brief     STM32CubeIDE System Memory calls file
 *
 *            For more information about which C functions
 *            need which of these lowlevel functions
 *            please consult the newlib libc manual
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes */
#include <errno.h>
#include <stdint.h>

/**
 * Pointer to the current high watermark of the heap usage
 */
static uint8_t *__sbrk_heap_end = NULL;

/**
 * @brief _sbrk() allocates memory to the newlib heap and is used by malloc
 *        and others from the C library
 *
 * @verbatim
 * ############################################################################
 * #  .data  #  .bss  #       newlib heap       #          MSP stack          #
 * #         #        #                         # Reserved by _Min_Stack_Size #
 * ############################################################################
 * ^-- RAM start      ^-- _end                             _estack, RAM end --^
 * @endverbatim
 *
 * This implementation starts allocating at the '_end' linker symbol
 * The '_Min_Stack_Size' linker symbol reserves a memory for the MSP stack
 * The implementation considers '_estack' linker symbol to be RAM end
 * NOTE: If the MSP stack, at any point during execution, grows larger than the
 * reserved size, please increase the '_Min_Stack_Size'.
 *
 * @param incr Memory size
 * @return Pointer to allocated memory
 */
void *_sbrk(ptrdiff_t incr)
{
  extern uint8_t _end; /* Symbol defined in the linker script */
  extern uint8_t _estack; /* Symbol defined in the linker script */
  extern uint32_t _Min_Stack_Size; /* Symbol defined in the linker script */
  const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
  const uint8_t *max_heap = (uint8_t *)stack_limit;
  uint8_t *prev_heap_end;

  /* Initialize heap end at first call */
  if (NULL == __sbrk_heap_end)
  {
    __sbrk_heap_end = &_end;
  }

  /* Protect heap from growing into the reserved MSP stack */
  if (__sbrk_heap_end + incr > max_heap)
  {
    errno = ENOMEM;
    return (void *)-1;
  }

  prev_heap_end = __sbrk_heap_end;
  __sbrk_heap_end += incr;

  return (void *)prev_heap_end;
}
</file>

<file path="Core/Src/system_stm32f4xx.c">
/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from 
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *                                     
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f4xx_system
  * @{
  */  
  
/** @addtogroup STM32F4xx_System_Private_Includes
  * @{
  */


#include "stm32f4xx.h"

#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to use external SRAM or SDRAM as data memory  */
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)
/* #define DATA_IN_ExtSRAM */
#endif /* STM32F40xxx || STM32F41xxx || STM32F42xxx || STM32F43xxx || STM32F469xx || STM32F479xx ||\
          STM32F412Zx || STM32F412Vx */
 
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
/* #define DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx ||\
          STM32F479xx */

/* Note: Following vector table addresses must be defined in line with linker
         configuration. */
/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in Flash or Sram, else the vector table is kept at the automatic
     remap of boot address selected */
/* #define USER_VECT_TAB_ADDRESS */

#if defined(USER_VECT_TAB_ADDRESS)
/*!< Uncomment the following line if you need to relocate your vector Table
     in Sram else user remap will be done in Flash. */
/* #define VECT_TAB_SRAM */
#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM_BASE       /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BASE      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_SRAM */
#if !defined(VECT_TAB_OFFSET)
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table offset field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_OFFSET */
#endif /* USER_VECT_TAB_ADDRESS */
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
  * @{
  */

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  static void SystemInit_ExtMemCtl(void); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory 
  *         configuration.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  SystemInit_ExtMemCtl(); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

  /* Configure the Vector Table location -------------------------------------*/
#if defined(USER_VECT_TAB_ADDRESS)
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#endif /* USER_VECT_TAB_ADDRESS */
}

/**
   * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *           
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.         
  *     
  * @note   - The system frequency computed by this function is not the real 
  *           frequency in the chip. It is calculated based on the predefined 
  *           constant and the selected clock source:
  *             
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *                                              
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *                          
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *         
  *         (*) HSI_VALUE is a constant defined in stm32f4xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f4xx_hal_conf.h file (its value
  *              depends on the application requirements), user has to ensure that HSE_VALUE
  *              is same as the real frequency of the crystal used. Otherwise, this function
  *              may have wrong result.
  *                
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *     
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t tmp, pllvco, pllp, pllsource, pllm;
  
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock source */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock source */

      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */    
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      
      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      SystemCoreClock = pllvco/pllp;
      break;
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK frequency --------------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK frequency */
  SystemCoreClock >>= tmp;
}

#if defined (DATA_IN_ExtSRAM) && defined (DATA_IN_ExtSDRAM)
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx)
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f4xx.s before jump to main.
  *         This function configures the external memories (SRAM/SDRAM)
  *         This SRAM/SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
  __IO uint32_t tmp = 0x00;

  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register __IO uint32_t index;

  /* Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface clock */
  RCC->AHB1ENR |= 0x000001F8;

  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
  
  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x00CCC0CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A8A;
  /* Configure PDx pins speed to 100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;
  
  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA800AAA;
  /* Configure PFx pins speed to 50 MHz */ 
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0xAAAAAAAA;
  /* Configure PGx pins speed to 50 MHz */ 
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
  /* Connect PHx pins to FMC Alternate function */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* Configure PHx pins in Alternate function mode */ 
  GPIOH->MODER   = 0xAAAA08A0;
  /* Configure PHx pins speed to 50 MHz */ 
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* Configure PHx pins Output type to push-pull */  
  GPIOH->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PHx pins */ 
  GPIOH->PUPDR   = 0x00000000;
  
  /* Connect PIx pins to FMC Alternate function */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* Configure PIx pins in Alternate function mode */ 
  GPIOI->MODER   = 0x0028AAAA;
  /* Configure PIx pins speed to 50 MHz */ 
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* Configure PIx pins Output type to push-pull */  
  GPIOI->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PIx pins */ 
  GPIOI->PUPDR   = 0x00000000;
  
/*-- FMC Configuration -------------------------------------------------------*/
  /* Enable the FMC interface clock */
  RCC->AHB3ENR |= 0x00000001;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

  FMC_Bank5_6->SDCR[0] = 0x000019E4;
  FMC_Bank5_6->SDTR[0] = 0x01115351;      
  
  /* SDRAM initialization sequence */
  /* Clock enable command */
  FMC_Bank5_6->SDCMR = 0x00000011; 
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }

  /* Delay */
  for (index = 0; index<1000; index++);
  
  /* PALL command */
  FMC_Bank5_6->SDCMR = 0x00000012;           
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
  
  /* Auto refresh command */
  FMC_Bank5_6->SDCMR = 0x00000073;
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
 
  /* MRD register program */
  FMC_Bank5_6->SDCMR = 0x00046014;
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  } 
  
  /* Set refresh count */
  tmpreg = FMC_Bank5_6->SDRTR;
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));
  
  /* Disable write protection */
  tmpreg = FMC_Bank5_6->SDCR[0]; 
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */ 
#if defined(STM32F469xx) || defined(STM32F479xx)
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001091;
  FMC_Bank1->BTCR[3]  = 0x00110212;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F469xx || STM32F479xx */

  (void)(tmp); 
}
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
#elif defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f4xx.s before jump to main.
  *         This function configures the external memories (SRAM/SDRAM)
  *         This SRAM/SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
  __IO uint32_t tmp = 0x00;
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
#if defined (DATA_IN_ExtSDRAM)
  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register __IO uint32_t index;

#if defined(STM32F446xx)
  /* Enable GPIOA, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG interface
      clock */
  RCC->AHB1ENR |= 0x0000007D;
#else
  /* Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface 
      clock */
  RCC->AHB1ENR |= 0x000001F8;
#endif /* STM32F446xx */  
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
  
#if defined(STM32F446xx)
  /* Connect PAx pins to FMC Alternate function */
  GPIOA->AFR[0]  |= 0xC0000000;
  GPIOA->AFR[1]  |= 0x00000000;
  /* Configure PDx pins in Alternate function mode */
  GPIOA->MODER   |= 0x00008000;
  /* Configure PDx pins speed to 50 MHz */
  GPIOA->OSPEEDR |= 0x00008000;
  /* Configure PDx pins Output type to push-pull */
  GPIOA->OTYPER  |= 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOA->PUPDR   |= 0x00000000;

  /* Connect PCx pins to FMC Alternate function */
  GPIOC->AFR[0]  |= 0x00CC0000;
  GPIOC->AFR[1]  |= 0x00000000;
  /* Configure PDx pins in Alternate function mode */
  GPIOC->MODER   |= 0x00000A00;
  /* Configure PDx pins speed to 50 MHz */
  GPIOC->OSPEEDR |= 0x00000A00;
  /* Configure PDx pins Output type to push-pull */
  GPIOC->OTYPER  |= 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOC->PUPDR   |= 0x00000000;
#endif /* STM32F446xx */

  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x000000CC;
  GPIOD->AFR[1]  = 0xCC000CCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xA02A000A;
  /* Configure PDx pins speed to 50 MHz */  
  GPIOD->OSPEEDR = 0xA02A000A;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00000CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA800A;
  /* Configure PEx pins speed to 50 MHz */ 
  GPIOE->OSPEEDR = 0xAAAA800A;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA800AAA;
  /* Configure PFx pins speed to 50 MHz */ 
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0xAAAAAAAA;
  /* Configure PGx pins speed to 50 MHz */ 
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx)  
  /* Connect PHx pins to FMC Alternate function */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* Configure PHx pins in Alternate function mode */ 
  GPIOH->MODER   = 0xAAAA08A0;
  /* Configure PHx pins speed to 50 MHz */ 
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* Configure PHx pins Output type to push-pull */  
  GPIOH->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PHx pins */ 
  GPIOH->PUPDR   = 0x00000000;
  
  /* Connect PIx pins to FMC Alternate function */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* Configure PIx pins in Alternate function mode */ 
  GPIOI->MODER   = 0x0028AAAA;
  /* Configure PIx pins speed to 50 MHz */ 
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* Configure PIx pins Output type to push-pull */  
  GPIOI->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PIx pins */ 
  GPIOI->PUPDR   = 0x00000000;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
  
/*-- FMC Configuration -------------------------------------------------------*/
  /* Enable the FMC interface clock */
  RCC->AHB3ENR |= 0x00000001;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

  /* Configure and enable SDRAM bank1 */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCR[0] = 0x00001954;
#else  
  FMC_Bank5_6->SDCR[0] = 0x000019E4;
#endif /* STM32F446xx */
  FMC_Bank5_6->SDTR[0] = 0x01115351;      
  
  /* SDRAM initialization sequence */
  /* Clock enable command */
  FMC_Bank5_6->SDCMR = 0x00000011; 
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }

  /* Delay */
  for (index = 0; index<1000; index++);
  
  /* PALL command */
  FMC_Bank5_6->SDCMR = 0x00000012;           
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
  
  /* Auto refresh command */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCMR = 0x000000F3;
#else  
  FMC_Bank5_6->SDCMR = 0x00000073;
#endif /* STM32F446xx */
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
 
  /* MRD register program */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCMR = 0x00044014;
#else  
  FMC_Bank5_6->SDCMR = 0x00046014;
#endif /* STM32F446xx */
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  } 
  
  /* Set refresh count */
  tmpreg = FMC_Bank5_6->SDRTR;
#if defined(STM32F446xx)
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000050C<<1));
#else    
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));
#endif /* STM32F446xx */
  
  /* Disable write protection */
  tmpreg = FMC_Bank5_6->SDCR[0]; 
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);
#endif /* DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx || STM32F479xx */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)

#if defined(DATA_IN_ExtSRAM)
/*-- GPIOs Configuration -----------------------------------------------------*/
   /* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
  RCC->AHB1ENR   |= 0x00000078;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
  
  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x00CCC0CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A8A;
  /* Configure PDx pins speed to 100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0x00CCCCCC;
  GPIOF->AFR[1]  = 0xCCCC0000;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA000AAA;
  /* Configure PFx pins speed to 100 MHz */ 
  GPIOF->OSPEEDR = 0xFF000FFF;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0x00CCCCCC;
  GPIOG->AFR[1]  = 0x000000C0;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0x00085AAA;
  /* Configure PGx pins speed to 100 MHz */ 
  GPIOG->OSPEEDR = 0x000CAFFF;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
/*-- FMC/FSMC Configuration --------------------------------------------------*/
  /* Enable the FMC/FSMC interface clock */
  RCC->AHB3ENR         |= 0x00000001;

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */ 
#if defined(STM32F469xx) || defined(STM32F479xx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001091;
  FMC_Bank1->BTCR[3]  = 0x00110212;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F469xx || STM32F479xx */
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx)|| defined(STM32F417xx)\
   || defined(STM32F412Zx) || defined(STM32F412Vx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FSMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FSMC_Bank1->BTCR[2]  = 0x00001011;
  FSMC_Bank1->BTCR[3]  = 0x00000201;
  FSMC_Bank1E->BWTR[2] = 0x0FFFFFFF;
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F412Zx || STM32F412Vx */

#endif /* DATA_IN_ExtSRAM */
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx ||\
          STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx  */ 
  (void)(tmp); 
}
#endif /* DATA_IN_ExtSRAM && DATA_IN_ExtSDRAM */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
</file>

<file path="Core/Src/tim.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 19;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    PE13     ------> TIM1_CH3
    PE14     ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    PA2     ------> TIM2_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
</file>

<file path="Core/Src/usart.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
</file>

<file path="debug_com10.py">
import serial
import time
try:
    print("Opening COM10...")
    ser = serial.Serial('COM10', 115200, timeout=0.1)
    print("Opened!")
    
    # Enable DTR/RTS to signal terminal readiness to some USB drivers
    ser.dtr = True
    ser.rts = True
    time.sleep(0.5)
    
    print(f"Initial bytes waiting: {ser.in_waiting}")
    # Send a dummy packet
    cmd = b"ST" + b"\x00"*24 + b"\n"
    print(f"Sending {len(cmd)} bytes...")
    ser.write(cmd)
    
    for i in range(10):
        time.sleep(0.2)
        print(f"[{i}] Bytes waiting: {ser.in_waiting}")
        if ser.in_waiting > 0:
            print("DATA:", ser.read(ser.in_waiting))
    ser.close()
    print("Closed.")
except Exception as e:
    print(f"Error: {e}")
</file>

<file path="pitch_diagnostic.py">
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
</file>

<file path="read_encoders.py">
import serial
import struct
import time
import sys

COM_PORT = 'COM10'
BAUD_RATE = 115200
PACKET_SIZE = 27  # 2s + 6i + c = 2 + 24 + 1 = 27

def pack_debug_command():
    # DT mode: firmware responds with raw ADC + raw encoder values
    # (no HOME_OFFSET or direction correction applied)
    return struct.pack('<2s 6i c', b'DT', 0, 0, 0, 0, 0, 0, b'\n')

def find_and_parse_packet(ser):
    """Read bytes and scan for a valid FB packet using header sync."""
    # Read all available bytes
    if ser.in_waiting < PACKET_SIZE:
        return None
    
    data = ser.read(ser.in_waiting)
    
    # Scan for 'FB' header and try to parse a full packet from there
    for i in range(len(data) - PACKET_SIZE + 1):
        if data[i:i+2] == b'FB':
            candidate = data[i:i+PACKET_SIZE]
            # Verify footer is newline
            if candidate[-1:] == b'\n':
                unpacked = struct.unpack('<2s 6i c', candidate)
                return list(unpacked[1:7])
    return None

def main():
    print("=== RAW SENSOR FEEDBACK TOOL ===")
    print("Reads absolute raw encoder/ADC values (no HOME_OFFSET correction)")
    print("Use this to find HOME_OFFSETS for RobotConfig.h\n")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {COM_PORT}!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Flush any stale data
    ser.reset_input_buffer()
    
    # Warmup: send several DT packets so firmware sets comms_ok = true
    # and starts reading encoders
    print("Warming up comms...", end="", flush=True)
    for i in range(20):
        ser.write(pack_debug_command())
        time.sleep(0.05)
    print(" done!\n")

    try:
        while True:
            ser.write(pack_debug_command())
            time.sleep(0.05)
            
            fb = find_and_parse_packet(ser)
            if fb is not None:
                sys.stdout.write(
                    f"\rRAW_ADC[M1]: {fb[0]:04d} | RAW_ADC[M2]: {fb[1]:04d} || "
                    f"RAW_ENC[M3]: {fb[2]:04d} | RAW_ENC[Pitch]: {fb[3]:04d} | "
                    f"RAW_ENC[Roll]: {fb[4]:04d} | RAW_ENC[Z]: {fb[5]:04d}   "
                )
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nExiting...")
        if ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
</file>

<file path="read_limp.py">
import serial
import struct
import time
import sys

COM_PORT = 'COM10'
BAUD_RATE = 115200
PACKET_SIZE = 27  # 2s + 6i + c = 2 + 24 + 1 = 27

def pack_limp_command():
    # PT mode (Open Loop): Firmware applies direct PWM values.
    # By sending all 0's, we force all motors to 0 PWM (freewheeling/limp).
    # The firmware still reads sensors and responds with RAW values!
    return struct.pack('<2s 6i c', b'PT', 0, 0, 0, 0, 0, 0, b'\n')

def find_and_parse_packet(ser):
    """Read bytes and scan for a valid FB packet using header sync."""
    if ser.in_waiting < PACKET_SIZE:
        return None
    
    data = ser.read(ser.in_waiting)
    
    # Scan for 'FB' header and try to parse a full packet from there
    for i in range(len(data) - PACKET_SIZE + 1):
        if data[i:i+2] == b'FB':
            candidate = data[i:i+PACKET_SIZE]
            # Verify footer is newline
            if candidate[-1:] == b'\n':
                unpacked = struct.unpack('<2s 6i c', candidate)
                return list(unpacked[1:7])
    return None

def main():
    print("=" * 60)
    print("  LIMP SENSOR READER — Move the arm by hand freely!")
    print("=" * 60)
    print("This script continuously pushes PWM=0 commands to force the PID")
    print("loops off. The arm will give zero resistance, while still")
    print("transmitting true RAW encoder and ADC values to the terminal.\n")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {COM_PORT}!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Flush any stale data
    ser.reset_input_buffer()
    
    # Warmup string
    print("Warming up comms...", end="", flush=True)
    for i in range(10):
        ser.write(pack_limp_command())
        time.sleep(0.05)
    print(" done!\n")

    try:
        while True:
            # Continuously overwrite the open-loop command array with 0s
            # to guarantee the motors never wake up and fight you.
            ser.write(pack_limp_command())
            time.sleep(0.05)
            
            fb = find_and_parse_packet(ser)
            if fb is not None:
                # FB[0]=M1_ADC, FB[1]=M2_ADC
                # FB[2]=M3_ENC, FB[3]=Pitch_ENC, FB[4]=Roll_ENC, FB[5]=Z_ENC
                sys.stdout.write(
                    f"\rADC [M1:{fb[0]:04d} M2:{fb[1]:04d}]  ||  "
                    f"I2C [M3:{fb[2]:04d} Pit:{fb[3]:04d} Rol:{fb[4]:04d} Z:{fb[5]:04d}]    "
                )
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n\nExiting... Watchdog will safely stop the arm in ~2.0s.")
        if ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
</file>

<file path="step_response_logger.py">
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
</file>

<file path="test_usb_arm.py">
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
</file>

<file path="USB_DEVICE/App/usb_device.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */
</file>

<file path="USB_DEVICE/App/usb_device.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.h
  * @version        : v1.0_Cube
  * @brief          : Header for usb_device.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_def.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup USBD_OTG_DRIVER
  * @{
  */

/** @defgroup USBD_DEVICE USBD_DEVICE
  * @brief Device file for Usb otg low level driver.
  * @{
  */

/** @defgroup USBD_DEVICE_Exported_Variables USBD_DEVICE_Exported_Variables
  * @brief Public variables.
  * @{
  */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN VARIABLES */

/* USER CODE END VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_DEVICE_Exported_FunctionsPrototype USBD_DEVICE_Exported_FunctionsPrototype
  * @brief Declaration of public functions for Usb device.
  * @{
  */

/** USB Device initialization function. */
void MX_USB_DEVICE_Init(void);

/*
 * -- Insert functions declaration here --
 */
/* USER CODE BEGIN FD */

/* USER CODE END FD */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEVICE__H__ */
</file>

<file path="USB_DEVICE/App/usbd_cdc_if.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
volatile uint8_t usb_client_connected = 0;
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:
      if (pbuf != NULL) {
          usb_client_connected = (pbuf[0] & 0x01); // Read DTR flag
      }
    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  extern void RobotCore_USB_Receive_Callback(uint8_t* buf, uint32_t len);
  RobotCore_USB_Receive_Callback(Buf, *Len);

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
</file>

<file path="USB_DEVICE/App/usbd_cdc_if.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
/* USER CODE BEGIN EXPORTED_DEFINES */

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
</file>

<file path="USB_DEVICE/App/usbd_desc.c">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : App/usbd_desc.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB device descriptors.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @addtogroup USBD_DESC
  * @{
  */

/** @defgroup USBD_DESC_Private_TypesDefinitions USBD_DESC_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_Defines USBD_DESC_Private_Defines
  * @brief Private defines.
  * @{
  */

#define USBD_VID     1155
#define USBD_LANGID_STRING     1033
#define USBD_MANUFACTURER_STRING     "STMicroelectronics"
#define USBD_PID_FS     22336
#define USBD_PRODUCT_STRING_FS     "STM32 Virtual ComPort"
#define USBD_CONFIGURATION_STRING_FS     "CDC Config"
#define USBD_INTERFACE_STRING_FS     "CDC Interface"

#define USB_SIZ_BOS_DESC            0x0C

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/** @defgroup USBD_DESC_Private_Macros USBD_DESC_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_FunctionPrototypes USBD_DESC_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static void Get_SerialNum(void);
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len);

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_FunctionPrototypes USBD_DESC_Private_FunctionPrototypes
  * @brief Private functions declaration for FS.
  * @{
  */

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#if (USBD_LPM_ENABLED == 1)
uint8_t * USBD_FS_USR_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#endif /* (USBD_LPM_ENABLED == 1) */

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_Variables USBD_DESC_Private_Variables
  * @brief Private variables.
  * @{
  */

USBD_DescriptorsTypeDef FS_Desc =
{
  USBD_FS_DeviceDescriptor
, USBD_FS_LangIDStrDescriptor
, USBD_FS_ManufacturerStrDescriptor
, USBD_FS_ProductStrDescriptor
, USBD_FS_SerialStrDescriptor
, USBD_FS_ConfigStrDescriptor
, USBD_FS_InterfaceStrDescriptor
#if (USBD_LPM_ENABLED == 1)
, USBD_FS_USR_BOSDescriptor
#endif /* (USBD_LPM_ENABLED == 1) */
};

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
/** USB standard device descriptor. */
__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
#if (USBD_LPM_ENABLED == 1)
  0x01,                       /*bcdUSB */ /* changed to USB version 2.01
                                             in order to support LPM L1 suspend
                                             resume test of USBCV3.0*/
#else
  0x00,                       /*bcdUSB */
#endif /* (USBD_LPM_ENABLED == 1) */
  0x02,
  0x02,                       /*bDeviceClass*/
  0x02,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_FS),        /*idProduct*/
  HIBYTE(USBD_PID_FS),        /*idProduct*/
  0x00,                       /*bcdDevice rel. 2.00*/
  0x02,
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

/* USB_DeviceDescriptor */
/** BOS descriptor. */
#if (USBD_LPM_ENABLED == 1)
#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
__ALIGN_BEGIN uint8_t USBD_FS_BOSDesc[USB_SIZ_BOS_DESC] __ALIGN_END =
{
  0x5,
  USB_DESC_TYPE_BOS,
  0xC,
  0x0,
  0x1,  /* 1 device capability*/
        /* device capability*/
  0x7,
  USB_DEVICE_CAPABITY_TYPE,
  0x2,
  0x2,  /* LPM capability bit set*/
  0x0,
  0x0,
  0x0
};
#endif /* (USBD_LPM_ENABLED == 1) */

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_Variables USBD_DESC_Private_Variables
  * @brief Private variables.
  * @{
  */

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */

/** USB lang identifier descriptor. */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
/* Internal string descriptor. */
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};

/**
  * @}
  */

/** @defgroup USBD_DESC_Private_Functions USBD_DESC_Private_Functions
  * @brief Private functions.
  * @{
  */

/**
  * @brief  Return the device descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

/**
  * @brief  Return the LangID string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

/**
  * @brief  Return the product string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the manufacturer string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
  * @brief  Return the serial number string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_SERIAL;

  /* Update the serial number string descriptor with the data from the unique
   * ID */
  Get_SerialNum();
  /* USER CODE BEGIN USBD_FS_SerialStrDescriptor */

  /* USER CODE END USBD_FS_SerialStrDescriptor */
  return (uint8_t *) USBD_StringSerial;
}

/**
  * @brief  Return the configuration string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the interface string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

#if (USBD_LPM_ENABLED == 1)
/**
  * @brief  Return the BOS descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_FS_USR_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_FS_BOSDesc);
  return (uint8_t*)USBD_FS_BOSDesc;
}
#endif /* (USBD_LPM_ENABLED == 1) */

/**
  * @brief  Create the serial number string descriptor
  * @param  None
  * @retval None
  */
static void Get_SerialNum(void)
{
  uint32_t deviceserial0;
  uint32_t deviceserial1;
  uint32_t deviceserial2;

  deviceserial0 = *(uint32_t *) DEVICE_ID1;
  deviceserial1 = *(uint32_t *) DEVICE_ID2;
  deviceserial2 = *(uint32_t *) DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0)
  {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}

/**
  * @brief  Convert Hex 32Bits value into char
  * @param  value: value to convert
  * @param  pbuf: pointer to the buffer
  * @param  len: buffer length
  * @retval None
  */
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len)
{
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++)
  {
    if (((value >> 28)) < 0xA)
    {
      pbuf[2 * idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[2 * idx + 1] = 0;
  }
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
</file>

<file path="USB_DEVICE/App/usbd_desc.h">
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_desc.c
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_conf.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_DESC__C__
#define __USBD_DESC__C__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_DESC USBD_DESC
  * @brief Usb device descriptors module.
  * @{
  */

/*
 * User to provide a unique ID to define the USB device serial number
 * The use of UID_BASE register can be considered as an example
 */
#define         DEVICE_ID1          (UID_BASE)
#define         DEVICE_ID2          (UID_BASE + 0x4)
#define         DEVICE_ID3          (UID_BASE + 0x8)

#define  USB_SIZ_STRING_SERIAL       0x1A

/* USER CODE BEGIN EXPORTED_CONSTANTS */

/* USER CODE END EXPORTED_CONSTANTS */

/**
  * @}
  */

/** @defgroup USBD_DESC_Exported_Defines USBD_DESC_Exported_Defines
  * @brief Defines.
  * @{
  */

/* USER CODE BEGIN EXPORTED_DEFINES */

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_DESC_Exported_TypesDefinitions USBD_DESC_Exported_TypesDefinitions
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */

/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_DESC_Exported_Macros USBD_DESC_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_DESC_Exported_Variables USBD_DESC_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** Descriptor for the Usb device. */
extern USBD_DescriptorsTypeDef FS_Desc;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_DESC_Exported_FunctionsPrototype USBD_DESC_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_DESC__C__ */
</file>

<file path="z_step_logger.py">
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
</file>

</files>
