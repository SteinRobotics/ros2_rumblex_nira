#include <gtest/gtest.h>

#include <thread>

#include "handler/face_preview.hpp"

namespace {
using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;
using namespace nira_geometry;
using namespace std::chrono_literals;

class FacePreviewTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        rclcpp::NodeOptions options;
        options.parameter_overrides({{"face_tracking.desktop_preview", true},
                                     {"face_tracking.max_yaw_deg", 30.0},
                                     {"face_tracking.max_pitch_deg", 20.0},
                                     {"face_tracking.max_speed_deg_s", 25.0},
                                     {"face_tracking.acceleration_deg_s2", 60.0},
                                     {"face_tracking.gain", 3.0},
                                     {"face_tracking.deadband_deg", 0.5}});
        node = std::make_shared<rclcpp::Node>("face_preview_test", options);
        node->declare_parameter("gait.generic.head_max_yaw_deg", 30.0);
        node->declare_parameter("gait.generic.head_max_pitch_deg", 20.0);
        node->declare_parameter("gait.generic.torso_max_roll_deg", 12.0);
        node->declare_parameter("gait.generic.torso_max_pitch_deg", 12.0);
        node->declare_parameter("gait.watch.torso_max_yaw_deg", 10.0);
        preview = std::make_unique<brain::FacePreview>(node);
        publisher = node->create_publisher<nira_interfaces::msg::FaceTarget>("perception/face_target", 1);
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
    }
    void TearDown() override {
        executor->remove_node(node);
        executor.reset();
        preview.reset();
        node.reset();
        rclcpp::shutdown();
    }
    void observe(double x, double y, double age = 0,
                 const std::string& frame = "desktop_camera_optical_frame", std::optional<double> roll = {}) {
        nira_interfaces::msg::FaceTarget target;
        target.valid = true;
        target.confidence = 0.95;
        target.header.frame_id = frame;
        target.header.stamp = node->now() - rclcpp::Duration::from_seconds(age);
        target.x_error = x;
        target.y_error = y;
        target.roll_valid = roll.has_value();
        target.roll_rad = roll.value_or(0.0);
        publisher->publish(target);
        std::this_thread::sleep_for(5ms);
        executor->spin_some();
    }
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<brain::FacePreview> preview;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    rclcpp::Publisher<nira_interfaces::msg::FaceTarget>::SharedPtr publisher;
};

TEST_F(FacePreviewTest, FixedCameraFaceConvergesWithoutAccumulatingToLimit) {
    brain::FacePreviewPose pose;
    for (int i = 0; i < 160; ++i) {
        observe(0.2, -0.1);
        const auto next = preview->update(pose, 0.02 * s, true);
        ASSERT_TRUE(next);
        const auto yaw_change = next->head.yaw + next->torso.yaw - pose.head.yaw - pose.torso.yaw;
        const auto pitch_change = next->head.pitch + next->torso.pitch - pose.head.pitch - pose.torso.pitch;
        EXPECT_LE(abs(yaw_change).numerical_value_in(deg), 0.5 + 1e-9);
        EXPECT_LE(abs(pitch_change).numerical_value_in(deg), 0.5 + 1e-9);
        if (abs(next->torso.yaw) < 10.0 * deg) {
            EXPECT_EQ(next->head.yaw, 0.0 * deg);
        }
        pose = *next;
    }
    EXPECT_EQ(pose.torso.yaw, -10.0 * deg);
    EXPECT_EQ(pose.head.pitch, 0.0 * deg);
    EXPECT_NEAR(pose.head.yaw.numerical_value_in(deg), -1.31, 0.6);
    EXPECT_NEAR(pose.torso.pitch.numerical_value_in(deg), -5.60, 0.6);
}
TEST_F(FacePreviewTest, MissingStaleWrongFrameAndManualOverrideInhibitTracking) {
    EXPECT_FALSE(preview->update({}, 0.02 * s, true));
    observe(0.2, 0.1, 1.0);
    EXPECT_FALSE(preview->update({}, 0.02 * s, true));
    observe(0.2, 0.1, 0, "camera_optical_frame");
    EXPECT_FALSE(preview->update({}, 0.02 * s, true));
    observe(0.2, 0.1);
    EXPECT_FALSE(preview->update({}, 0.02 * s, false));
    EXPECT_TRUE(preview->update({}, 0.02 * s, true));
    preview->suspend();
    observe(0.2, 0.1);
    EXPECT_FALSE(preview->active());
    EXPECT_FALSE(preview->update({}, 0.02 * s, true));
}
TEST_F(FacePreviewTest, SaturatesWithinConfiguredLimitsAndStopsAfterTimeout) {
    brain::FacePreviewPose pose;
    for (int i = 0; i < 200; ++i) {
        observe(10, 100);
        const auto next = preview->update(pose, 0.02 * s, true);
        ASSERT_TRUE(next);
        pose = *next;
        EXPECT_LE(abs(pose.head.yaw).numerical_value_in(deg), 30.0);
        EXPECT_LE(abs(pose.head.pitch).numerical_value_in(deg), 20.0);
    }
    EXPECT_NEAR(pose.head.yaw.numerical_value_in(deg), -30.0, 0.6);
    EXPECT_NEAR(pose.head.pitch.numerical_value_in(deg), 20.0, 0.6);
    std::this_thread::sleep_for(320ms);
    EXPECT_FALSE(preview->update(pose, 0.02 * s, true));
}
TEST_F(FacePreviewTest, AcquiresFromAnOffsetHeadWithoutJumpingOrIncreasingTheOffset) {
    brain::FacePreviewPose pose;
    pose.head.pitch = -20.0 * deg;
    observe(0, 0.1);
    for (int i = 0; i < 200; ++i) {
        const auto next = preview->update(pose, 0.02 * s, true);
        ASSERT_TRUE(next);
        EXPECT_GE(next->head.pitch, pose.head.pitch);
        EXPECT_LE(next->head.pitch - pose.head.pitch, 0.500001 * deg);
        EXPECT_LE(abs(next->torso.pitch - pose.torso.pitch), 0.500001 * deg);
        pose = *next;
    }
    EXPECT_EQ(pose.head.pitch, 0.0 * deg);
    EXPECT_NEAR(pose.torso.pitch.numerical_value_in(deg), 5.71, 0.51);
}

TEST_F(FacePreviewTest, ReversesAcrossThresholdAndMirrorsRoll) {
    brain::FacePreviewPose pose;
    for (double angle : {16.0, -7.0, 10.0, -16.0, 0.0}) {
        const double radians = angle * std::acos(-1.0) / 180.0;
        const double x = -std::tan(radians);
        observe(x, std::tan(radians) * std::hypot(1.0, x), 0, "desktop_camera_optical_frame", radians);
        for (int i = 0; i < 200; ++i) {
            const auto next = preview->update(pose, 0.02 * s, true);
            ASSERT_TRUE(next);
            pose = *next;
            if (abs(pose.torso.pitch) < 10.0 * deg) {
                EXPECT_EQ(pose.head.pitch, 0.0 * deg);
            }
            if (abs(pose.torso.yaw) < 10.0 * deg) {
                EXPECT_EQ(pose.head.yaw, 0.0 * deg);
            }
            EXPECT_EQ(pose.head.roll, 0.0 * deg);
        }
        const double torso = std::clamp(angle, -10.0, 10.0);
        EXPECT_NEAR(pose.torso.roll.numerical_value_in(deg), torso, 0.51);
        EXPECT_NEAR(pose.torso.pitch.numerical_value_in(deg), torso, 0.51);
        EXPECT_NEAR(pose.head.yaw.numerical_value_in(deg), angle - torso, 0.51);
        observe(-x, 0);
        const auto next = preview->update(pose, 0.02 * s, true);
        ASSERT_TRUE(next);
        EXPECT_EQ(next->torso.roll, pose.torso.roll);
    }
}
}  // namespace
