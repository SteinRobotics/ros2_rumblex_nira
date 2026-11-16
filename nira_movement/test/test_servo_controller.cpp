#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <limits>
#include <tuple>

#include "handler/feetech_protocol.hpp"
#include "handler/leg_servo_conversion.hpp"
#include "handler/servo_controller.hpp"
#include "nira_utils/body_model.hpp"

namespace nira_movement {
class ServoControllerTestAccess {
   public:
    static void protocol(CServoController& controller, std::shared_ptr<CServoProtocol> protocol) {
        controller.protocol_ = std::move(protocol);
    }
    static auto dashboard(CServoController& controller,
                          const nira_interfaces::srv::ServoDashboard::Request& request) {
        nira_interfaces::srv::ServoDashboard::Response response;
        controller.dashboard(request, response);
        return response;
    }
    static bool hasTimer(const CServoController& controller) {
        return bool(controller.timer_);
    }
    static void status(CServoController& controller) {
        controller.onTimerStatus();
    }
    static bool hasPublishers(const CServoController& controller) {
        return bool(controller.pub_status_) && bool(controller.pub_angles_);
    }
    static int ticks(CServoController& controller, double angle, int index) {
        return controller.angle_to_ticks(angle, index);
    }
    static double angle(CServoController& controller, int ticks, int index) {
        return controller.ticks_to_angle(ticks, index);
    }
};

class RecordingProtocol : public CServoProtocol {
   public:
    explicit RecordingProtocol(std::shared_ptr<rclcpp::Node> node) : CServoProtocol(node) {
    }
    bool readHardwareOffset(uint8_t, int16_t& ticks) override {
        ticks = offset;
        return !read_failure;
    }
    bool writeHardwareOffset(uint8_t, int16_t ticks) override {
        offset = ticks;
        return !fail;
    }
    bool calibrateHardwareOffset(uint8_t) override {
        offset = 123;
        return !fail;
    }
    int16_t offset = 0;
    int dashboardRead(uint8_t, uint8_t address, bool) override {
        return read_failure ? -1 : registers[address];
    }
    bool dashboardWrite(uint8_t, uint8_t address, uint16_t value, bool) override {
        writes.push_back(address);
        registers[address] = value;
        return !fail;
    }
    std::map<int, int> registers{{9, 0}, {11, 4095}, {56, 2000}, {58, 0x8007}};
    std::vector<int> writes;
    bool read_failure = false;
    bool setRegPos(uint8_t id, uint16_t ticks, uint16_t duration_ms) override {
        positions.emplace_back(id, ticks, duration_ms);
        return !fail;
    }
    bool actionStart(uint8_t) override {
        ++actions;
        return true;
    }
    std::vector<std::tuple<uint8_t, uint16_t, uint16_t>> positions;
    int actions = 0;
    bool fail = false;
};

class ServoControllerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides(
            {rclcpp::Parameter("servo.offline", true),
             rclcpp::Parameter("servo.names", std::vector<std::string>{"HEAD_YAW"}),
             rclcpp::Parameter("servo.adaptation_deg", std::vector<double>{0.0}),
             rclcpp::Parameter("servo.offset_deg", std::vector<double>{0.0}),
             rclcpp::Parameter("servo.orientation_clockwise", std::vector<bool>{true}),
             rclcpp::Parameter("servo.serial_ids", std::vector<int64_t>{1})});
        node = std::make_shared<rclcpp::Node>("servo_resolution_test", options);
        controller = std::make_unique<CServoController>(node);
        protocol = std::make_shared<RecordingProtocol>(node);
        ServoControllerTestAccess::protocol(*controller, protocol);
    }
    void TearDown() override {
        controller.reset();
        rclcpp::shutdown();
    }
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<CServoController> controller;
    std::shared_ptr<RecordingProtocol> protocol;
};

TEST_F(ServoControllerTest, MissingSerialDeviceDoesNotInitializeOrCrash) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("servo.serial_port", "/dev/nira-test-nonexistent/servo"),
                                 rclcpp::Parameter("servo.names", std::vector<std::string>{"HEAD_YAW"}),
                                 rclcpp::Parameter("servo.adaptation_deg", std::vector<double>{0.0}),
                                 rclcpp::Parameter("servo.offset_deg", std::vector<double>{0.0}),
                                 rclcpp::Parameter("servo.orientation_clockwise", std::vector<bool>{true}),
                                 rclcpp::Parameter("servo.serial_ids", std::vector<int64_t>{10})});
    auto missing_node = std::make_shared<rclcpp::Node>("missing_serial_test", options);
    CServoController missing(missing_node);
    bool initialized = false;
    missing.setInitialAnglesCallback([&](const auto&, const auto&) { initialized = true; });
    EXPECT_FALSE(initialized);
    EXPECT_FALSE(ServoControllerTestAccess::hasTimer(missing));
    EXPECT_TRUE(ServoControllerTestAccess::hasPublishers(missing));
    EXPECT_NO_THROW(ServoControllerTestAccess::status(missing));
    nira_interfaces::srv::ServoDashboard::Request q;
    auto response = ServoControllerTestAccess::dashboard(missing, q);
    EXPECT_EQ(response.outcome, response.VERIFIED);
    EXPECT_EQ(response.names.size(), 1U);
    q.name = "HEAD_YAW";
    q.operation = q.STATE;
    response = ServoControllerTestAccess::dashboard(missing, q);
    EXPECT_EQ(response.outcome, response.REJECTED);
    EXPECT_NE(response.detail.find("Serial bus unavailable"), std::string::npos);
    nira_interfaces::msg::ServoAngle angle;
    angle.name = "HEAD_YAW";
    EXPECT_NO_THROW(missing.onSingleServoRequestReceived(angle));
    nira_interfaces::msg::ServoDirectRequest direct;
    direct.name = "HEAD_YAW";
    EXPECT_NO_THROW(missing.onServoDirectRequestReceived(direct));
}

class OffsetProtocol : public CFeetechProtocol {
   public:
    explicit OffsetProtocol(std::shared_ptr<rclcpp::Node> node) : CFeetechProtocol(node, "") {
    }
    bool isConnected() override {
        return true;
    }
    int dashboardRead(uint8_t, uint8_t address, bool) override {
        return registers[address];
    }
    bool dashboardWrite(uint8_t, uint8_t address, uint16_t value, bool) override {
        writes.push_back(address);
        if (fail_offset && address == 31) return false;
        registers[address] = value;
        return true;
    }
    std::map<int, int> registers;
    std::vector<int> writes;
    bool fail_offset = false;
};

TEST_F(ServoControllerTest, HardwareOffsetPersistsSignedTicksAndRelocksOnFailure) {
    OffsetProtocol hardware(node);
    EXPECT_TRUE(hardware.writeHardwareOffset(1, -1234));
    EXPECT_EQ(hardware.registers[31], 0x8000 | 1234);
    EXPECT_EQ(hardware.registers[40], 0);
    EXPECT_EQ(hardware.registers[55], 1);
    int16_t ticks = 0;
    EXPECT_TRUE(hardware.readHardwareOffset(1, ticks));
    EXPECT_EQ(ticks, -1234);
    hardware.fail_offset = true;
    EXPECT_FALSE(hardware.writeHardwareOffset(1, 10));
    EXPECT_EQ(hardware.writes.back(), 55);
    EXPECT_EQ(hardware.registers[55], 1);
    hardware.writes.clear();
    EXPECT_FALSE(hardware.writeHardwareOffset(1, 4096));
    EXPECT_TRUE(hardware.writes.empty());
}

TEST_F(ServoControllerTest, DashboardHardwareOffsetCommands) {
    nira_interfaces::srv::ServoDashboard::Request q;
    q.name = "HEAD_YAW";
    q.operation = q.SAVE_OFFSET;
    q.offset_ticks = -321;
    auto response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.outcome, response.VERIFIED);
    EXPECT_EQ(response.offset_ticks, -321);
    q.operation = q.CALIBRATE_OFFSET;
    response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.offset_ticks, 123);
    protocol->fail = true;
    response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.outcome, response.UNVERIFIED);
    q.operation = q.SAVE_OFFSET;
    q.offset_ticks = 4096;
    response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.outcome, response.REJECTED);
}

TEST_F(ServoControllerTest, DashboardRawMoveAndProfile) {
    nira_interfaces::srv::ServoDashboard::Request q;
    q.name = "HEAD_YAW";
    q.operation = q.MOVE;
    q.goal = 3000;
    q.speed = 100;
    q.acc = 10;
    auto response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.outcome, response.VERIFIED);
    EXPECT_EQ(protocol->registers[42], 3000);
    EXPECT_EQ(response.state.velocity, -7);
    protocol->writes.clear();
    q.operation = q.PROFILE;
    q.speed = 200;
    response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.outcome, response.VERIFIED);
    EXPECT_EQ(protocol->writes, (std::vector<int>{41, 46}));
}

TEST_F(ServoControllerTest, DashboardRejectsAndReportsUnverifiedWrites) {
    nira_interfaces::srv::ServoDashboard::Request q;
    q.name = "HEAD_YAW";
    q.operation = q.MOVE;
    q.goal = 4096;
    auto response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.outcome, response.REJECTED);
    EXPECT_TRUE(protocol->writes.empty());
    q.goal = 2048;
    protocol->fail = true;
    response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.outcome, response.UNVERIFIED);
    EXPECT_EQ(protocol->writes.size(), 1U);
}

TEST_F(ServoControllerTest, DashboardHoldLimitsAndDiscovery) {
    nira_interfaces::srv::ServoDashboard::Request q;
    auto response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.names, (std::vector<std::string>{"HEAD_YAW"}));
    q.name = "HEAD_YAW";
    q.operation = q.HOLD;
    response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.outcome, response.VERIFIED);
    EXPECT_EQ(protocol->registers[42], 2000);
    q.operation = q.LIMITS;
    q.minimum = 100;
    q.maximum = 3900;
    response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.state.minimum, 100);
    EXPECT_EQ(response.state.maximum, 3900);
    q.operation = q.ID;
    q.new_id = 1;
    response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.outcome, response.REJECTED);
    protocol->read_failure = true;
    q.operation = q.STATE;
    response = ServoControllerTestAccess::dashboard(*controller, q);
    EXPECT_EQ(response.outcome, response.REJECTED);
}

TEST_F(ServoControllerTest, SmallMovesUseOneTickRatherThanHalfDegreeSteps) {
    controller->requestAngles({{0, 0.1}}, 0.02);
    EXPECT_TRUE(protocol->positions.empty());
    EXPECT_EQ(protocol->actions, 0);
    controller->requestAngles({{0, 0.13}}, 0.02);
    ASSERT_EQ(protocol->positions.size(), 1u);
    EXPECT_EQ(std::get<1>(protocol->positions.back()), 501);
    EXPECT_EQ(std::get<2>(protocol->positions.back()), 20);
    controller->requestAngles({{0, 0.26}}, 0.02);
    EXPECT_EQ(protocol->positions.size(), 1u);
    controller->requestAngles({{0, 0.37}}, 0.02);
    ASSERT_EQ(protocol->positions.size(), 2u);
    EXPECT_EQ(std::get<1>(protocol->positions.back()), 502);
    controller->requestAngles({{0, -0.13}}, 0.02);
    ASSERT_EQ(protocol->positions.size(), 3u);
    EXPECT_EQ(std::get<1>(protocol->positions.back()), 499);
    EXPECT_EQ(protocol->actions, 3);
}

TEST_F(ServoControllerTest, FailedWritesCanRetryAndInvalidCommandsDoNotStartActions) {
    protocol->fail = true;
    controller->requestAngles({{0, 0.24}}, 0.02);
    EXPECT_EQ(protocol->actions, 0);
    protocol->fail = false;
    controller->requestAngles({{0, 0.24}}, 0.02);
    EXPECT_EQ(protocol->positions.size(), 2u);
    EXPECT_EQ(protocol->actions, 1);
    controller->requestAngles({{0, std::numeric_limits<double>::quiet_NaN()}}, 0.02);
    controller->requestAngles({{0, 10.0}}, -0.02);
    EXPECT_EQ(protocol->positions.size(), 2u);
    EXPECT_EQ(protocol->actions, 1);
}

TEST_F(ServoControllerTest, SmallFrameDurationsKeepServoSpeedLimit) {
    controller->requestAngles({{0, 10.08}}, 0.02);
    ASSERT_EQ(protocol->positions.size(), 1u);
    EXPECT_EQ(std::get<2>(protocol->positions.back()), 31);
}

}  // namespace nira_movement
