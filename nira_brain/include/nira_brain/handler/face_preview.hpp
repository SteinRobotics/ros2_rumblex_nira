#pragma once

#include <chrono>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nira_interfaces/msg/face_target.hpp"
#include "nira_utils/body_types.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace brain {
// Desktop camera directions are fixed relative to the virtual robot's neutral
// head. They are absolute pointing targets, not persistent image-error velocities.
struct FacePreviewPose {
    nira_geometry::COrientation torso;
    nira_geometry::COrientation head;
};

class FacePreview {
   public:
    explicit FacePreview(const std::shared_ptr<rclcpp::Node>& node);
    bool active() const {
        return preview_ && enabled_;
    }
    bool previewMode() const {
        return preview_;
    }
    void suspend();
    std::optional<FacePreviewPose> update(const FacePreviewPose& current, nira_geometry::units::Duration dt,
                                          bool idle);

   private:
    void resetVelocities();
    void status(const std::string& value);
    std::shared_ptr<rclcpp::Node> node_;
    bool preview_ = false, enabled_ = false;
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
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};
}  // namespace brain
