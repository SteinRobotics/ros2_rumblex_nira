#include "offline_visualization.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("offline_visualization");
    nira_geometry::CBodyModel model(node);
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("visualization_joint_states", 10);
    auto subscription = node->create_subscription<nira_interfaces::msg::BodyPose>(
        "body_pose_actual", rclcpp::QoS(1).transient_local(),
        [&](const nira_interfaces::msg::BodyPose& pose) {
            if (!nira_geometry::completeBodyPose(pose) || !nira_geometry::validBodyPose(pose)) {
                RCLCPP_WARN(node->get_logger(), "Ignoring non-finite offline body pose");
                return;
            }
            auto joints = nira_movement::visualizationJoints(model, pose);
            if (!std::all_of(joints.position.begin(), joints.position.end(),
                             [](double angle) { return std::isfinite(angle); }))
                return;
            joints.header.stamp = node->now();
            publisher->publish(joints);
        });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
