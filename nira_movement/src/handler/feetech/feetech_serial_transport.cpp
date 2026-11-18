#include "feetech_serial_transport.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <utility>

namespace {
std::optional<speed_t> baudCode(int baud) {
    switch (baud) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 500000:
            return B500000;
        case 1000000:
            return B1000000;
        default:
            return std::nullopt;
    }
}
}  // namespace

FeetechSerialTransport::~FeetechSerialTransport() {
    end();
}

bool FeetechSerialTransport::begin(int baud_rate, const char* kSerialPort) {
    end();
    error_code_ = 0;
    const auto kBaud = baudCode(baud_rate);
    if (!kBaud || !kSerialPort) {
        error_code_ = ERR_IO;
        return false;
    }
    fd_ = open(kSerialPort, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    termios settings{};
    if (fd_ < 0 || tcgetattr(fd_, &settings) != 0) {
        end();
        error_code_ = ERR_IO;
        return false;
    }
    cfmakeraw(&settings);
    settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    settings.c_cflag |= CS8 | CREAD | CLOCAL;
    if (cfsetispeed(&settings, *kBaud) != 0 || cfsetospeed(&settings, *kBaud) != 0 ||
        tcsetattr(fd_, TCSANOW, &settings) != 0) {
        end();
        error_code_ = ERR_IO;
        return false;
    }
    return true;
}

int FeetechSerialTransport::setBaudRate(int baud_rate) {
    const auto kBaud = baudCode(baud_rate);
    termios settings{};
    if (fd_ < 0 || !kBaud || tcgetattr(fd_, &settings) != 0 || cfsetispeed(&settings, *kBaud) != 0 ||
        cfsetospeed(&settings, *kBaud) != 0 || tcsetattr(fd_, TCSANOW, &settings) != 0) {
        error_code_ = ERR_IO;
        return -1;
    }
    error_code_ = 0;
    return 1;
}

void FeetechSerialTransport::end() {
    // Invalidate ownership before close so repeated cleanup cannot close a new
    // descriptor that happens to reuse this number. Do not retry close on EINTR.
    const int kOld = std::exchange(fd_, -1);
    if (kOld >= 0) close(kOld);
    tx_size_ = 0;
    deadline_.reset();
}

bool FeetechSerialTransport::waitReady(short events, Clock::time_point deadline) {
    // poll avoids select's descriptor-size limit. Recompute the remaining time
    // after interruptions instead of granting a new timeout on each retry.
    while (true) {
        if (fd_ < 0) {
            error_code_ = ERR_IO;
            return false;
        }
        const auto kRemaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now());
        if (kRemaining.count() <= 0) {
            error_code_ = ERR_NO_REPLY;
            return false;
        }
        pollfd descriptor{fd_, events, 0};
        const int kResult =
            poll(&descriptor, 1, static_cast<int>(std::min<int64_t>(kRemaining.count(), INT_MAX)));
        if (kResult < 0 && errno == EINTR) continue;
        if (kResult < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            error_code_ = ERR_IO;
            return false;
        }
        if (kResult == 0) {
            error_code_ = ERR_NO_REPLY;
            return false;
        }
        if (descriptor.revents & events) return true;
    }
}

int FeetechSerialTransport::readSCS(unsigned char* data, int size) {
    if (size < 0 || (size > 0 && !data) || fd_ < 0) {
        error_code_ = ERR_IO;
        return -1;
    }
    const auto kDeadline = deadline_.value_or(Clock::now() + std::chrono::milliseconds(io_timeout_ms_));
    int received = 0;
    while (received < size) {
        if (!waitReady(POLLIN, kDeadline)) return -1;
        const auto kCount = read(fd_, data + received, size - received);
        if (kCount < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (kCount <= 0) {
            error_code_ = ERR_IO;
            return -1;
        }
        received += static_cast<int>(kCount);
    }
    return received;
}

int FeetechSerialTransport::writeSCS(unsigned char* data, int size) {
    if (error_code_) return -1;
    if (size < 0 || (size > 0 && !data) || static_cast<size_t>(size) > tx_buffer_.size() - tx_size_) {
        error_code_ = ERR_BUFF_LEN;
        return -1;
    }
    if (size > 0) std::copy_n(data, size, tx_buffer_.begin() + tx_size_);
    tx_size_ += size;
    return size;
}

int FeetechSerialTransport::writeSCS(unsigned char value) {
    return writeSCS(&value, 1);
}

void FeetechSerialTransport::rFlushSCS() {
    error_code_ = 0;
    servo_status_ = 0;
    // Discard bytes left by an earlier transaction, including its partial
    // transmit buffer. This starts a new packet, never a replay of the old one.
    tx_size_ = 0;
    deadline_ = Clock::now() + std::chrono::milliseconds(io_timeout_ms_);
    if (fd_ < 0 || tcflush(fd_, TCIFLUSH) != 0) error_code_ = ERR_IO;
}

ssize_t FeetechSerialTransport::writePort(const unsigned char* kData, size_t size) {
    return write(fd_, kData, size);
}

bool FeetechSerialTransport::wFlushSCS() {
    // Consume the queued frame once. If transmission fails after a prefix was
    // sent, the device outcome is uncertain; leave reporting to the caller and
    // never resend that prefix. Short successful writes continue at the suffix.
    const auto kSize = std::exchange(tx_size_, 0);
    if (error_code_) return false;
    const auto kDeadline = deadline_.value_or(Clock::now() + std::chrono::milliseconds(io_timeout_ms_));
    size_t sent = 0;
    while (sent < kSize) {
        if (!waitReady(POLLOUT, kDeadline)) return false;
        const auto kCount = writePort(tx_buffer_.data() + sent, kSize - sent);
        if (kCount < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (kCount <= 0) {
            error_code_ = ERR_IO;
            return false;
        }
        sent += static_cast<size_t>(kCount);
    }
    return true;
}
