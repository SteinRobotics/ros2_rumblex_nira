#pragma once

#include <chrono>
#include <optional>

#include "nira_interfaces/msg/face_target.hpp"
#include "nira_utils/body_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace brain {
struct FaceCorrection {
    nira_geometry::COrientation torso;
    nira_geometry::COrientation head;
};

class FaceFollower {
   public:
    explicit FaceFollower(const std::shared_ptr<rclcpp::Node>& node);
    void setEnabled(bool enabled);
    std::optional<FaceCorrection> update(const FaceCorrection& current, nira_geometry::units::Duration dt);
    FaceCorrection release(FaceCorrection current, nira_geometry::units::Duration dt) const;

   private:
    void resetVelocities();
    void status(const std::string& value);
    std::shared_ptr<rclcpp::Node> node_;
    bool desktop_ = false, enabled_ = false;
    double max_yaw_ = 0, max_pitch_ = 0, max_speed_ = 0, acceleration_ = 0, gain_ = 0, deadband_ = 0;
    double torso_yaw_ = 0, torso_pitch_ = 0, torso_roll_ = 0;
    nira_geometry::units::AngularVelocity yaw_velocity_{}, pitch_velocity_{}, roll_velocity_{};
    nira_geometry::units::AngularVelocity torso_yaw_velocity_{}, torso_pitch_velocity_{};
    std::string state_;
    nira_interfaces::msg::FaceTarget::ConstSharedPtr target_;
    std::chrono::steady_clock::time_point received_{};
    int64_t last_stamp_ = 0;
    rclcpp::Subscription<nira_interfaces::msg::FaceTarget>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};
}  // namespace brain
