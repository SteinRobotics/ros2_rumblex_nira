#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <tuple>
#include <vector>

#include "handler/feetech_protocol.hpp"

namespace nira_movement {
namespace {
using Request = nira_interfaces::srv::ServoDashboard::Request;
using Response = nira_interfaces::srv::ServoDashboard::Response;

class RegisterProtocol : public CFeetechProtocol {
   public:
    RegisterProtocol() : CFeetechProtocol(nullptr, "") {
    }
    bool isConnected() override {
        return connected;
    }
    bool getServoID(uint8_t, uint8_t&) override {
        return occupied;
    }
    int dashboardRead(uint8_t id, uint8_t address, bool word) override {
        reads.emplace_back(id, address, word);
        return address == failed_read ? -1 : registers[address];
    }
    bool dashboardWrite(uint8_t id, uint8_t address, uint16_t value, bool word) override {
        writes.emplace_back(id, address, value, word);
        if (address == failed_write) return false;
        if (address != ignored_write) registers[address] = value;
        return true;
    }
    Response run(const Request& kRequest) {
        Response response;
        dashboard(id, kRequest, response);
        return response;
    }
    uint8_t id = 1;
    bool connected = true, occupied = false;
    int failed_read = -1, failed_write = -1, ignored_write = -1;
    std::map<int, int> registers{{9, 100}, {11, 3900}, {56, 2048}, {58, 0x8001}, {62, 120}};
    std::vector<std::tuple<uint8_t, uint8_t, bool>> reads;
    std::vector<std::tuple<uint8_t, uint8_t, uint16_t, bool>> writes;
};

TEST(FeetechDashboard, ReadsNativeUnitsAndSignedVelocity) {
    RegisterProtocol protocol;
    Request request;
    request.operation = Request::STATE;
    const auto kResult = protocol.run(request);
    EXPECT_EQ(kResult.outcome, Response::VERIFIED);
    EXPECT_EQ(kResult.state.position, 2048);
    EXPECT_EQ(kResult.state.velocity, -1);
    EXPECT_FLOAT_EQ(kResult.state.voltage, 12.0F);
    EXPECT_TRUE(protocol.writes.empty());
}

TEST(FeetechDashboard, RejectsInvalidCommandsBeforeWriting) {
    for (const int kScenario : {0, 1, 2, 3, 4, 5, 6}) {
        RegisterProtocol protocol;
        Request request;
        request.operation = Request::MOVE;
        request.goal = 2048;
        switch (kScenario) {
            case 0:
                request.goal = 99;
                break;
            case 1:
                request.goal = 3901;
                break;
            case 2:
                request.speed = 3401;
                break;
            case 3:
                request.acc = 151;
                break;
            case 4:
                request.operation = Request::LIMITS;
                request.minimum = request.maximum = 100;
                break;
            case 5:
                request.operation = Request::ID;
                request.new_id = 254;
                break;
            case 6:
                request.operation = 255;
                break;
        }
        SCOPED_TRACE(kScenario);
        EXPECT_EQ(protocol.run(request).outcome, Response::REJECTED);
        EXPECT_TRUE(protocol.writes.empty());
    }
}

TEST(FeetechDashboard, SeedsAndVerifiesGoalBeforeEnablingTorque) {
    RegisterProtocol protocol;
    Request request;
    request.operation = Request::ENABLE_TORQUE;
    EXPECT_EQ(protocol.run(request).outcome, Response::VERIFIED);
    ASSERT_EQ(protocol.writes.size(), 2U);
    EXPECT_EQ(protocol.writes[0], std::make_tuple(1, 42, 2048, true));
    EXPECT_EQ(protocol.writes[1], std::make_tuple(1, 40, 1, false));
    protocol.writes.clear();
    protocol.registers[42] = 0;
    protocol.ignored_write = 42;
    EXPECT_EQ(protocol.run(request).outcome, Response::UNVERIFIED);
    ASSERT_EQ(protocol.writes.size(), 1U);
    EXPECT_EQ(std::get<1>(protocol.writes[0]), 42);
}

TEST(FeetechDashboard, StopsAfterFailedWriteAndDoesNotRetry) {
    RegisterProtocol protocol;
    Request request;
    request.operation = Request::MOVE;
    request.goal = 2500;
    protocol.failed_write = 41;
    EXPECT_EQ(protocol.run(request).outcome, Response::UNVERIFIED);
    EXPECT_EQ(protocol.writes.size(), 1U);
}

TEST(FeetechDashboard, DoesNotPublishPartialTelemetry) {
    RegisterProtocol protocol;
    Request request;
    request.operation = Request::STATE;
    protocol.failed_read = 6;  // Last register in the sample.
    auto result = protocol.run(request);
    EXPECT_EQ(result.outcome, Response::REJECTED);
    EXPECT_EQ(result.state.position, 0);
    request.operation = Request::HOLD;
    result = protocol.run(request);
    EXPECT_EQ(result.outcome, Response::UNVERIFIED);
    EXPECT_EQ(result.state.position, 0);
}

TEST(FeetechDashboard, UpdatesAddressOnlyAfterReadbackEvenWhenTelemetryFails) {
    RegisterProtocol protocol;
    Request request;
    request.operation = Request::ID;
    request.new_id = 7;
    protocol.occupied = true;
    EXPECT_EQ(protocol.run(request).outcome, Response::REJECTED);
    EXPECT_TRUE(protocol.writes.empty());
    protocol.occupied = false;
    protocol.ignored_write = 5;
    EXPECT_EQ(protocol.run(request).outcome, Response::UNVERIFIED);
    EXPECT_EQ(protocol.id, 1);
    protocol.ignored_write = -1;
    protocol.failed_read = 56;
    EXPECT_EQ(protocol.run(request).outcome, Response::UNVERIFIED);
    EXPECT_EQ(protocol.id, 7);
    EXPECT_EQ(protocol.reads.back(), std::make_tuple(7, 56, true));
}

TEST(FeetechDashboard, RejectsDisconnectedAndBroadcastAccess) {
    RegisterProtocol protocol;
    Request request;
    request.operation = Request::HOLD;
    protocol.connected = false;
    EXPECT_EQ(protocol.run(request).outcome, Response::REJECTED);
    protocol.connected = true;
    protocol.id = 254;
    EXPECT_EQ(protocol.run(request).outcome, Response::REJECTED);
    EXPECT_TRUE(protocol.reads.empty());
    EXPECT_TRUE(protocol.writes.empty());
}

TEST(FeetechOffset, RelocksAfterUnlockReadbackOrOffsetWriteFailure) {
    for (int address : {55, 31}) {
        RegisterProtocol protocol;
        protocol.failed_read = address == 55 ? 55 : -1;
        protocol.failed_write = address == 31 ? 31 : -1;
        EXPECT_FALSE(protocol.writeHardwareOffset(1, -4095));
        ASSERT_FALSE(protocol.writes.empty());
        EXPECT_EQ(protocol.writes.back(), std::make_tuple(1, 55, 1, false));
        EXPECT_EQ(protocol.registers[40], 0);
    }
}

TEST(FeetechOffset, RejectsInvalidMagnitudeAndLeavesOutputUntouched) {
    RegisterProtocol protocol;
    protocol.registers[31] = 0x9000;
    int16_t offset = 123;
    EXPECT_FALSE(protocol.readHardwareOffset(1, offset));
    EXPECT_EQ(offset, 123);
    EXPECT_FALSE(protocol.writeHardwareOffset(1, -4096));
    EXPECT_TRUE(protocol.writes.empty());
}

// Exercise the real HLS packet builder and parser without serial hardware.
class PacketServo : public FeetechHlsServo {
   public:
    std::vector<u8> packet, reply;
    std::vector<u8> payload;
    bool acknowledge = true;
    bool corrupt_checksum = false, wrong_id = false, truncate = false, transport_failure = false;
    u8 status = 0;

   protected:
    int writeSCS(u8* data, int size) override {
        packet.insert(packet.end(), data, data + size);
        return size;
    }
    int writeSCS(u8 value) override {
        packet.push_back(value);
        return 1;
    }
    int readSCS(u8* data, int size) override {
        int count = std::min(size, static_cast<int>(reply.size()));
        std::copy_n(reply.begin(), count, data);
        reply.erase(reply.begin(), reply.begin() + count);
        return count;
    }
    void rFlushSCS() override {
        packet.clear();
        reply.clear();
    }
    bool wFlushSCS() override {
        if (transport_failure) return false;
        if (!acknowledge) return true;
        reply = {0xff, 0xff, static_cast<u8>(packet[2] + (wrong_id ? 1 : 0)),
                 static_cast<u8>(payload.size() + 2), status};
        reply.insert(reply.end(), payload.begin(), payload.end());
        unsigned sum = 0;
        for (size_t i = 2; i < reply.size(); ++i) sum += reply[i];
        reply.push_back(static_cast<u8>(~sum));
        if (corrupt_checksum) reply.back() ^= 1;
        if (truncate) reply.pop_back();
        return true;
    }
};

TEST(HlsPackets, ImmediateAndDeferredMotionSharePayload) {
    PacketServo servo;
    ASSERT_EQ(servo.WritePosEx(1, -123, 100, 50, 500), 1);
    const auto kImmediate = servo.packet;
    ASSERT_EQ(servo.RegWritePosEx(1, -123, 100, 50, 500), 1);
    const auto kDeferred = servo.packet;
    ASSERT_EQ(kImmediate.size(), 14U);
    EXPECT_EQ(kImmediate[4], 3);
    EXPECT_EQ(kDeferred[4], 4);
    EXPECT_EQ(std::vector<u8>(kImmediate.begin() + 5, kImmediate.end() - 1),
              (std::vector<u8>{41, 50, 123, 128, 244, 1, 100, 0}));
    EXPECT_TRUE(std::equal(kImmediate.begin() + 5, kImmediate.end() - 1, kDeferred.begin() + 5));
}

TEST(HlsPackets, MissingAcknowledgementFailsAndReadFailureIsNotDecoded) {
    PacketServo servo;
    servo.acknowledge = false;
    EXPECT_EQ(servo.WritePosEx(1, 2048, 100, 50, 500), 0);
    EXPECT_EQ(servo.ReadPos(1), -1);
    EXPECT_EQ(servo.ReadSpeed(1), -1);
    servo.acknowledge = true;
    servo.payload = {7, 128};
    EXPECT_EQ(servo.ReadSpeed(1), -7);
}

TEST(HlsPackets, ReportsPacketAndServoErrorsAndRecovers) {
    PacketServo servo;
    servo.status = 4;
    EXPECT_EQ(servo.EnableTorque(1, 1), 0);
    EXPECT_EQ(servo.getLastError(), ERR_SERVO);
    EXPECT_EQ(servo.getState(), 4);
    servo.status = 0;
    servo.corrupt_checksum = true;
    EXPECT_EQ(servo.Ping(1), -1);
    EXPECT_EQ(servo.getLastError(), ERR_CRC_CMP);
    EXPECT_EQ(servo.getState(), 0);
    servo.corrupt_checksum = false;
    servo.wrong_id = true;
    EXPECT_EQ(servo.Ping(1), -1);
    EXPECT_EQ(servo.getLastError(), ERR_SLAVE_ID);
    servo.wrong_id = false;
    servo.payload = {1};
    EXPECT_EQ(servo.Ping(1), -1);
    EXPECT_EQ(servo.getLastError(), ERR_BUFF_LEN);
    servo.payload.clear();
    servo.truncate = true;
    EXPECT_EQ(servo.Ping(1), -1);
    EXPECT_EQ(servo.getLastError(), ERR_NO_REPLY);
    servo.truncate = false;
    servo.transport_failure = true;
    EXPECT_EQ(servo.RegWriteAction(), 0);
    EXPECT_EQ(servo.getLastError(), ERR_IO);
    servo.transport_failure = false;
    EXPECT_EQ(servo.Ping(1), 1);
    EXPECT_EQ(servo.getLastError(), 0);
}

TEST(HlsPackets, FailedResponseDoesNotModifyOutput) {
    PacketServo servo;
    servo.payload = {0x34, 0x12};
    servo.corrupt_checksum = true;
    u8 data[] = {99, 99};
    EXPECT_EQ(servo.Read(1, 56, data, 2), 0);
    EXPECT_EQ(data[0], 99);
    EXPECT_EQ(data[1], 99);
    servo.corrupt_checksum = false;
    servo.status = 4;
    EXPECT_EQ(servo.Read(1, 56, data, 2), 0);
    EXPECT_EQ(servo.getLastError(), ERR_SERVO);
    EXPECT_EQ(data[0], 99);
    servo.status = 0;
    EXPECT_EQ(servo.readWord(1, 56), 0x1234);
    servo.byte_order_ = 1;
    EXPECT_EQ(servo.readWord(1, 56), 0x3412);
}

TEST(HlsPackets, RejectsOversizedPayloadAndBroadcastRead) {
    PacketServo servo;
    std::vector<u8> data(253);
    EXPECT_EQ(servo.genWrite(1, 0, data.data(), 253), 0);
    EXPECT_EQ(servo.getLastError(), ERR_BUFF_LEN);
    EXPECT_TRUE(servo.packet.empty());
    EXPECT_EQ(servo.readWord(254, 56), -1);
    EXPECT_TRUE(servo.packet.empty());
    EXPECT_EQ(servo.genWrite(1, 0, data.data(), 252), 1);
    EXPECT_EQ(servo.packet.size(), 259U);
    EXPECT_EQ(servo.packet[3], 255);
}

TEST(HlsPackets, ActionAndCalibrationUseCorrectInstructionsAndChecksum) {
    PacketServo servo;
    EXPECT_EQ(servo.RegWriteAction(), 1);
    EXPECT_EQ(servo.packet, (std::vector<u8>{255, 255, 254, 2, 5, 250}));
    EXPECT_EQ(servo.Recal(1), 1);
    EXPECT_EQ(servo.packet, (std::vector<u8>{255, 255, 1, 2, 11, 241}));
}

TEST(FeetechDashboard, InvalidTravelLimitsDoNotFallBackToFullRange) {
    for (auto [low, high] : {std::pair{0, 0}, std::pair{3000, 1000}, std::pair{0, 4096}}) {
        RegisterProtocol protocol;
        protocol.registers[9] = low;
        protocol.registers[11] = high;
        Request request;
        request.operation = Request::MOVE;
        request.goal = 2048;
        EXPECT_EQ(protocol.run(request).outcome, Response::REJECTED);
        EXPECT_TRUE(protocol.writes.empty());
    }
}

TEST(FeetechMotion, RejectsMissingOrOutOfRangePositionAndLeavesOutputUntouched) {
    for (int position : {-1, 4096, 0x8001}) {
        RegisterProtocol protocol;
        protocol.registers[56] = position;
        int16_t output = 123;
        EXPECT_FALSE(protocol.getPosition(1, output));
        EXPECT_EQ(output, 123);
        EXPECT_FALSE(protocol.setPosition(1, 500, 100));
        EXPECT_FALSE(protocol.setRegPos(1, 500, 100));
        EXPECT_TRUE(protocol.writes.empty());
    }
}

TEST(HlsEncoding, SignedMagnitudeRoundTrips) {
    for (int16_t value : {-4095, -1, 0, 1, 4095}) {
        EXPECT_EQ(hls::decodeSigned(hls::encodeSigned(value)), value);
    }
    EXPECT_EQ(hls::encodeSigned(-1), 0x8001);
}
}  // namespace
}  // namespace nira_movement
