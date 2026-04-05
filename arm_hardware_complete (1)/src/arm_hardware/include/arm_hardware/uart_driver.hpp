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
