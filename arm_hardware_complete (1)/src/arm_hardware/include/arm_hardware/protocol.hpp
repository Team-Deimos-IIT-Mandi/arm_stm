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
