/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
//
#include "nira_utils/body_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
//

#include "handler/i_servo_controller.hpp"
#include "nira_utils/body_model.hpp"

namespace nira_movement {

class CRequester {
   public:
    CRequester(std::shared_ptr<rclcpp::Node> node,
               std::shared_ptr<IServoController> servo_controller = nullptr);
    virtual ~CRequester() = default;

    void update(std::chrono::milliseconds timeslice);

    void onBodyPose(const nira_interfaces::msg::BodyPose& msg);

   private:
    rclcpp::Subscription<nira_interfaces::msg::BodyPose>::SharedPtr sub_body_pose_;
    rclcpp::Publisher<nira_interfaces::msg::BodyPose>::SharedPtr pub_body_pose_;
    std::optional<std::chrono::steady_clock::time_point> last_pose_received_;
    std::optional<double> pending_duration_s_;
    bool pending_pose_ = false;

    void sendServoRequest(const double duration_s);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<nira_geometry::CBodyModel> kinematics_;
    std::shared_ptr<IServoController> servo_controller_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    void publishJointStates(const std::map<nira_geometry::ELegIndex, nira_geometry::CLegAngles>& legs,
                            const nira_geometry::COrientation& head);
};
}  // namespace nira_movement
