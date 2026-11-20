#include "handler/pose_overlays.hpp"

namespace brain {
using namespace mp_units;
using mp_units::angular::unit_symbols::deg;
using mp_units::angular::unit_symbols::rad;
using namespace nira_geometry::units;

PoseOverlays::PoseOverlays(const std::shared_ptr<rclcpp::Node>& node)
    : node_(node),
      follower_(node),
      leveling_(loadLevelTorsoSettings(node)),
      torso_limits_(node->get_parameter("gait.generic.torso_max_roll_deg").as_double(),
                    node->get_parameter("gait.generic.torso_max_pitch_deg").as_double(),
                    node->get_parameter("gait.watch.torso_max_yaw_deg").as_double()),
      head_limits_(0.0, node->get_parameter("gait.generic.head_max_pitch_deg").as_double(),
                   node->get_parameter("gait.generic.head_max_yaw_deg").as_double()) {
}

void PoseOverlays::toggle(bool follow_face, bool level_torso) {
    if (follow_face) follow_face_ = !follow_face_;
    if (level_torso) level_torso_ = !level_torso_;
}

void PoseOverlays::apply(nira_interfaces::msg::BodyPose& pose, MovementRequest::_type_type gait,
                         Duration dt) {
    const bool walking = isLocomotion(gait);
    const bool eligible = walking || gait == MovementRequest::SEQUENCE_STAND_UP;
    follower_.setEnabled(eligible && follow_face_);
    if (eligible && level_torso_) {
        if (!imu_) imu_.emplace(node_);
    } else {
        imu_.reset();
    }
    if (!eligible) {
        face_ = {};
        leveling_.reset({});
        return;
    }
    if (walking && follow_face_) pose.head_orientation.yaw = 0.0;
    if (!isfinite(dt) || dt <= 0.0 * s) return;
    dt = std::min(dt, 0.1 * s);
    if (const auto next = follower_.update(face_, dt))
        face_ = *next;
    else
        face_ = follower_.release(face_, dt);

    const auto measured = imu_ ? imu_->tilt() : std::nullopt;
    const double seconds = dt.numerical_value_in(s);
    const auto tilt = measured ? leveling_.update(*measured, seconds) : leveling_.release(seconds);

    // Restrict the correction to the remaining travel, preserving the gait's own target.
    const auto add = [](double& base, Angle offset, Angle limit) {
        const double bound = limit.numerical_value_in(deg);
        base += std::clamp(offset.numerical_value_in(deg), std::min(0.0, -bound - base),
                           std::max(0.0, bound - base));
    };
    auto& torso = pose.torso_pose.orientation;
    add(torso.roll, face_.torso.roll + tilt.roll * rad, torso_limits_.roll);
    add(torso.pitch, face_.torso.pitch + tilt.pitch * rad, torso_limits_.pitch);
    add(torso.yaw, face_.torso.yaw, torso_limits_.yaw);
    add(pose.head_orientation.pitch, face_.head.pitch, head_limits_.pitch);
    add(pose.head_orientation.yaw, face_.head.yaw, head_limits_.yaw);
}
}  // namespace brain
