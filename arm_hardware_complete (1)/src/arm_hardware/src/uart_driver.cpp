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
