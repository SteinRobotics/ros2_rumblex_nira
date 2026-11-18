#pragma once

#include <sys/types.h>

#include <array>
#include <chrono>
#include <optional>

#include "feetech_packet_protocol.hpp"

// POSIX transport for the Feetech packet layer. Owns one nonblocking descriptor
// and closes it on destruction. Copying is forbidden to prevent shared ownership
// of the descriptor. Callers must serialize complete transactions on this port.
// rFlushSCS starts one monotonic deadline shared by transmission and all response
// reads; a fragmented packet must not reset the timeout at each fragment.
class FeetechSerialTransport : public FeetechPacketProtocol {
   public:
    FeetechSerialTransport() = default;
    explicit FeetechSerialTransport(u8 byte_order) : FeetechPacketProtocol(byte_order) {
    }
    FeetechSerialTransport(u8 byte_order, u8 response_level)
        : FeetechPacketProtocol(byte_order, response_level) {
    }
    ~FeetechSerialTransport() override;
    FeetechSerialTransport(const FeetechSerialTransport&) = delete;
    FeetechSerialTransport& operator=(const FeetechSerialTransport&) = delete;

    unsigned long io_timeout_ms_ = 100;  // Transaction timeout in milliseconds.
    int setBaudRate(int baud_rate);
    // Replaces any existing connection. Failure leaves the transport closed.
    bool begin(int baud_rate, const char* kSerialPort);
    void end();

   protected:
    int writeSCS(unsigned char* data, int size) override;
    int writeSCS(unsigned char value) override;
    int readSCS(unsigned char* data, int size) override;
    void rFlushSCS() override;
    bool wFlushSCS() override;
    // System-call seam for deterministic short-write/EINTR/EAGAIN tests.
    virtual ssize_t writePort(const unsigned char* kData, size_t size);
    int fd_ = -1;

   private:
    using Clock = std::chrono::steady_clock;
    bool waitReady(short events, Clock::time_point deadline);
    // The one-byte packet length covers up to 255 bytes following ID/length;
    // two header bytes plus ID and length bring the maximum frame to 259 bytes.
    std::array<unsigned char, 259> tx_buffer_{};
    size_t tx_size_ = 0;
    std::optional<Clock::time_point> deadline_;
};
