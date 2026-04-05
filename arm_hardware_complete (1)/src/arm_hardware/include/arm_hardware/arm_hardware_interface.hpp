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
