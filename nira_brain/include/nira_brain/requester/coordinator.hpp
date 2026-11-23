/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
//
#include "movement_request.hpp"
#include "nira_interfaces/msg/joystick_request.hpp"
#include "nira_interfaces/msg/servo_status.hpp"
//
#include <array>
#include <chrono>
#include <nira_utils/simpletimer.hpp>
#include <optional>

#include "action/action_planner.hpp"
#include "irequester.hpp"
#include "parser/sequence_parser.hpp"
#include "requester/error_management.hpp"
#include "requester/text_interpreter.hpp"
#include "requester/utility.hpp"

namespace brain {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::rad;

class CCoordinator {
   public:
    CCoordinator(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CActionPlanner> jobHandler);
    virtual ~CCoordinator() = default;

    void update();

    void joystickRequestReceived(const nira_interfaces::msg::JoystickRequest& msg);
    void sequenceRequested(const std::string& name);
    void cmdVelReceived(const geometry_msgs::msg::Twist& msg);
    enum class VoiceDispatch { Ignored, Submitted, Unavailable, Unsupported };
    VoiceDispatch speechRecognized(std::string_view text);
    void supplyVoltageReceived(float voltage);
    void servoStatusReceived(const nira_interfaces::msg::ServoStatus& msg);
    void movementTypeActualReceived(const brain::MovementRequest& msg);

   private:
    friend class CoordinatorTestAccess;

    void loadSequences();
    void executeSequence(const Sequence& sequence, Prio prio = Prio::High);
    void submitRequest(std::shared_ptr<RequestBase> request, Prio prio);

    struct MovementCommand {
        uint32_t type = MovementRequest::NO_REQUEST;
        units::Duration duration = 0.0 * s;
        std::string comment{};
        Prio priority = Prio::Normal;
        nira_interfaces::msg::BodyPose body_pose{};
        std::optional<geometry_msgs::msg::Twist> velocity{};
        std::optional<uint8_t> direction{};
    };

    struct ErrorReaction {
        bool turn_off_servo_relay = false;
        bool shutdown = false;
        Prio priority = Prio::Normal;
    };

    double applyDeadzone(double value) const;
    MovementCommand poseFromJoystick(const nira_interfaces::msg::JoystickRequest& msg) const;
    MovementCommand movementFromJoystick(const nira_interfaces::msg::JoystickRequest& msg) const;
    void submitRequestMove(const MovementCommand& command);
    void requestShutdown(Prio prio);
    void requestReactionOnError(std::string text, const ErrorReaction& reaction);

    void requestTellSupplyVoltage(Prio prio = Prio::Normal);
    void requestTellServoVoltage(Prio prio = Prio::Normal);
    void requestTellServoTemperature(Prio prio = Prio::Normal);
    void requestMusikOn(Prio prio = Prio::Normal);
    void requestMusikOff(Prio prio = Prio::Normal);
    void cycleGaitMode();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CActionPlanner> actionPlanner_;
    std::shared_ptr<CErrorManagement> errorManagement_;
    std::optional<CTextInterpreter> textInterpreter_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cameraCapture_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr lidarStart_, lidarStop_;
    std::shared_ptr<CSequenceParser> sequenceParser_;
    std::shared_ptr<utils::CSimpleTimer> timerErrorRequest_;
    std::shared_ptr<utils::CSimpleTimer> timerNoRequest_;
    struct MovementDeadline {
        std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<double>> time;
        uint32_t movement_type;
    };
    std::optional<MovementDeadline> movement_deadline_;

    uint32_t actualMovementType_ = brain::MovementRequest::NO_REQUEST;
    bool isStanding_ = false;
    bool isServoRelayOn_ = true;

    // Gait cycling: button_start iterates through these modes
    const std::vector<uint32_t> gaitModes_ = {
        brain::MovementRequest::MOVE_COMBINED, brain::MovementRequest::CONTINUOUS_POSE,
        brain::MovementRequest::MOVE_TRIPOD,   brain::MovementRequest::MOVE_RIPPLE,
        brain::MovementRequest::MOVE_WAVE,     brain::MovementRequest::MOVE_CRAWL,
    };
    std::array<float, 4> last_sticks_{};
    size_t activeGaitIndex_ = 0;

    units::LinearVelocity kMaxVelocityLinear_ = 0.0 * m / s;
    units::AngularVelocity kMaxVelocityRotation_ = 0.0 * rad / s;
    units::Length kBodyFactorHeight_ = 0.0 * m;
    units::Length kMinBodyHeight_ = 0.0 * m;
    units::Length kMaxBodyHeight_ = 0.0 * m;
    double kJoystickDeadzone_ = 0.0;
    bool kActivateMovementWaiting_ = false;
};

}  // namespace brain
