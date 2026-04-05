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
