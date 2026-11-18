#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <thread>
#include <type_traits>
#include <vector>

#include "feetech_serial_transport.hpp"

using namespace std::chrono_literals;

namespace {
class SerialProbe : public FeetechSerialTransport {
   public:
    int descriptor() const {
        return fd_;
    }
    using FeetechSerialTransport::readSCS;
    using FeetechSerialTransport::rFlushSCS;
    using FeetechSerialTransport::wFlushSCS;
    using FeetechSerialTransport::writeSCS;
};
static_assert(!std::is_copy_constructible_v<FeetechSerialTransport>);
static_assert(!std::is_copy_assignable_v<FeetechSerialTransport>);

class SerialTest : public testing::Test {
   protected:
    void SetUp() override {
        ASSERT_EQ(openpty(&master, &slave, name.data(), nullptr, nullptr), 0);
        ASSERT_TRUE(serial.begin(115200, name.data()));
        serial.io_timeout_ms_ = 100;
    }
    void TearDown() override {
        serial.end();
        if (master >= 0) close(master);
        if (slave >= 0) close(slave);
    }
    ssize_t readMaster(unsigned char* data, size_t size) {
        pollfd descriptor{master, POLLIN, 0};
        if (poll(&descriptor, 1, 200) != 1) return -1;
        return read(master, data, size);
    }
    int master = -1, slave = -1;
    std::array<char, 256> name{};
    SerialProbe serial;
};

TEST_F(SerialTest, AppliesBaudRateAndRejectsUnsupportedValues) {
    EXPECT_EQ(serial.setBaudRate(230400), 1);
    termios settings{};
    ASSERT_EQ(tcgetattr(slave, &settings), 0);
    EXPECT_EQ(cfgetispeed(&settings), B230400);
    EXPECT_EQ(cfgetospeed(&settings), B230400);
    EXPECT_EQ(serial.setBaudRate(12345), -1);
    ASSERT_EQ(tcgetattr(slave, &settings), 0);
    EXPECT_EQ(cfgetospeed(&settings), B230400);
    EXPECT_EQ(serial.setBaudRate(1000000), 1);
    ASSERT_EQ(tcgetattr(slave, &settings), 0);
    EXPECT_EQ(cfgetospeed(&settings), B1000000);
}

TEST_F(SerialTest, ClosesOnEndAndDestruction) {
    int owned = serial.descriptor();
    serial.end();
    EXPECT_EQ(fcntl(owned, F_GETFD), -1);
    EXPECT_EQ(errno, EBADF);
    // Repeated close must not close another descriptor reusing the old number.
    int replacement = open("/dev/null", O_RDONLY);
    ASSERT_GE(replacement, 0);
    serial.end();
    EXPECT_NE(fcntl(replacement, F_GETFD), -1);
    close(replacement);
    {
        SerialProbe scoped;
        ASSERT_TRUE(scoped.begin(115200, name.data()));
        owned = scoped.descriptor();
    }
    EXPECT_EQ(fcntl(owned, F_GETFD), -1);
    EXPECT_EQ(errno, EBADF);
}

TEST_F(SerialTest, FailedInitializationClosesAndCanRecover) {
    EXPECT_FALSE(serial.begin(12345, name.data()));
    EXPECT_EQ(serial.descriptor(), -1);
    EXPECT_FALSE(serial.begin(115200, "/dev/null"));
    EXPECT_EQ(serial.descriptor(), -1);
    EXPECT_FALSE(serial.begin(115200, nullptr));
    EXPECT_TRUE(serial.begin(115200, name.data()));
    EXPECT_EQ(serial.getLastError(), 0);
}

TEST_F(SerialTest, ReadsFragmentedData) {
    serial.rFlushSCS();
    std::jthread sender([&] {
        const unsigned char kFirst[] = {1, 2}, kLast[] = {3, 4};
        EXPECT_EQ(write(master, kFirst, 2), 2);
        std::this_thread::sleep_for(10ms);
        EXPECT_EQ(write(master, kLast, 2), 2);
    });
    std::array<unsigned char, 4> data{};
    EXPECT_EQ(serial.readSCS(data.data(), data.size()), 4);
    EXPECT_EQ(data, (std::array<unsigned char, 4>{1, 2, 3, 4}));
}

TEST_F(SerialTest, TimeoutPreservesBufferBoundsAndIsSharedByReads) {
    serial.io_timeout_ms_ = 30;
    serial.rFlushSCS();
    const unsigned char kValue = 7;
    ASSERT_EQ(write(master, &kValue, 1), 1);
    std::array<unsigned char, 4> data{99, 99, 99, 99};
    EXPECT_EQ(serial.readSCS(data.data() + 1, 2), -1);
    EXPECT_EQ(serial.getLastError(), ERR_NO_REPLY);
    EXPECT_EQ(data, (std::array<unsigned char, 4>{99, 7, 99, 99}));
    // An expired transaction must not get a fresh timeout for the next read.
    ASSERT_EQ(write(master, &kValue, 1), 1);
    EXPECT_EQ(serial.readSCS(data.data() + 1, 1), -1);
    EXPECT_EQ(serial.getLastError(), ERR_NO_REPLY);
}

TEST_F(SerialTest, DisconnectAndClosedDescriptorFailWithoutTouchingBuffer) {
    serial.rFlushSCS();
    close(master);
    master = -1;
    unsigned char value = 99;
    EXPECT_EQ(serial.readSCS(&value, 1), -1);
    EXPECT_EQ(serial.getLastError(), ERR_IO);
    EXPECT_EQ(value, 99);
    serial.end();
    EXPECT_EQ(serial.readSCS(&value, 1), -1);
    EXPECT_EQ(serial.setBaudRate(115200), -1);
}

TEST_F(SerialTest, OverflowDoesNotTransmitTruncatedPacket) {
    serial.rFlushSCS();
    std::array<unsigned char, 259> data{};
    EXPECT_EQ(serial.writeSCS(data.data(), data.size()), 259);
    EXPECT_EQ(serial.writeSCS(1), -1);
    EXPECT_FALSE(serial.wFlushSCS());
    EXPECT_EQ(serial.getLastError(), ERR_BUFF_LEN);
    ASSERT_NE(fcntl(master, F_SETFL, O_NONBLOCK), -1);
    unsigned char received;
    EXPECT_EQ(read(master, &received, 1), -1);
    EXPECT_EQ(errno, EAGAIN);
    serial.rFlushSCS();
    EXPECT_EQ(serial.writeSCS(42), 1);
    EXPECT_TRUE(serial.wFlushSCS());
    ASSERT_EQ(read(master, &received, 1), 1);
    EXPECT_EQ(received, 42);
}

class ShortWriter : public SerialProbe {
   public:
    int calls = 0;
    bool fail = false;
    bool always_again = false;

   protected:
    ssize_t writePort(const unsigned char* kData, size_t size) override {
        ++calls;
        if (fail) {
            errno = EIO;
            return -1;
        }
        if (always_again || calls == 1 || calls == 3) {
            errno = calls == 1 ? EINTR : EAGAIN;
            return -1;
        }
        return FeetechSerialTransport::writePort(kData, std::min(size, size_t{2}));
    }
};

TEST_F(SerialTest, CompletesShortWritesAndTransientErrorsWithoutDuplicatingBytes) {
    ShortWriter writer;
    ASSERT_TRUE(writer.begin(115200, name.data()));
    writer.rFlushSCS();
    unsigned char payload[] = {1, 2, 3, 4, 5};
    ASSERT_EQ(writer.writeSCS(payload, 5), 5);
    EXPECT_TRUE(writer.wFlushSCS());
    std::array<unsigned char, 5> actual{};
    ASSERT_EQ(readMaster(actual.data(), actual.size()), 5);
    EXPECT_EQ(actual, (std::array<unsigned char, 5>{1, 2, 3, 4, 5}));
    EXPECT_EQ(writer.calls, 5);
}

TEST_F(SerialTest, WriteFailureReachesBroadcastCallerAndNextPacketCanRecover) {
    ShortWriter writer;
    ASSERT_TRUE(writer.begin(115200, name.data()));
    writer.fail = true;
    EXPECT_EQ(writer.RegWriteAction(), 0);
    EXPECT_EQ(writer.getLastError(), ERR_IO);
    writer.fail = false;
    EXPECT_EQ(writer.RegWriteAction(), 1);
    EXPECT_EQ(writer.getLastError(), 0);
}

TEST_F(SerialTest, RepeatedWouldBlockHasBoundedDeadline) {
    ShortWriter writer;
    ASSERT_TRUE(writer.begin(115200, name.data()));
    writer.io_timeout_ms_ = 20;
    writer.always_again = true;
    EXPECT_EQ(writer.RegWriteAction(), 0);
    EXPECT_EQ(writer.getLastError(), ERR_NO_REPLY);
}
}  // namespace
