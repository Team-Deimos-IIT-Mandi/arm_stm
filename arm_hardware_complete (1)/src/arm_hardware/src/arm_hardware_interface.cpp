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
