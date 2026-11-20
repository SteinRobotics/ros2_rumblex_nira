#pragma once

#include "gait/level_torso_control.hpp"
#include "handler/face_follower.hpp"
#include "movement_request.hpp"

namespace brain {
class PoseOverlays {
   public:
    explicit PoseOverlays(const std::shared_ptr<rclcpp::Node>& node);
    void toggle(bool follow_face, bool level_torso);
    void apply(nira_interfaces::msg::BodyPose& pose, MovementRequest::_type_type gait,
               nira_geometry::units::Duration dt);

   private:
    std::shared_ptr<rclcpp::Node> node_;
    FaceFollower follower_;
    FaceCorrection face_;
    LevelTorsoLaw leveling_;
    std::optional<ImuFeedback> imu_;
    nira_geometry::COrientation torso_limits_, head_limits_;
    bool follow_face_ = false, level_torso_ = false;
};
}  // namespace brain
