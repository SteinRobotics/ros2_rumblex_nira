#include <gtest/gtest.h>

#include <limits>

#include "movement_request.hpp"

TEST(MovementTypes, LocomotionContainsOnlyTheWalkingModes) {
    for (uint32_t type = 0; type <= 255; ++type) {
        const bool expected =
            type == brain::MovementRequest::MOVE_COMBINED || type == brain::MovementRequest::MOVE_TRIPOD ||
            type == brain::MovementRequest::MOVE_RIPPLE || type == brain::MovementRequest::MOVE_WAVE ||
            type == brain::MovementRequest::MOVE_CRAWL;
        EXPECT_EQ(brain::isLocomotion(type), expected) << type;
    }
    EXPECT_FALSE(brain::isLocomotion(256));
}

namespace {
using namespace brain;
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

TEST(MovementValidation, AbsoluteAndRelativeDurationsKeepDistinctRules) {
    MovementRequest request;
    request.type = MovementRequest::SINGLE_POSE;
    request.body_pose.has_head_orientation = true;
    request.duration_s = 0.0 * s;
    EXPECT_TRUE(request.hasFiniteDuration());
    EXPECT_TRUE(request.validPoseOptions());
    EXPECT_TRUE(request.hasPoseTarget());
    request.body_pose.has_head_orientation = false;
    request.pose_delta.emplace();
    EXPECT_FALSE(request.validRelativePose());
    request.duration_s = 0.02 * s;
    EXPECT_TRUE(request.validRelativePose());
    request.body_pose.has_head_orientation = true;
    EXPECT_FALSE(request.validRelativePose());
    request.body_pose.has_head_orientation = false;
    request.pose_delta->head_rotation.yaw = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(request.validRelativePose());
}

TEST(MovementValidation, SensorRulesDoNotApplyToOrdinaryPoseDurations) {
    MovementRequest request;
    request.duration_s = -1.0 * s;
    EXPECT_TRUE(request.validSensorPose());
    request.type = MovementRequest::HOLD_DISTANCE;
    EXPECT_FALSE(request.validSensorPose());
    EXPECT_FALSE(request.validDistance());
    request.duration_s = kDefaultPoseDuration;
    request.distance_m = 0.5 * m;
    EXPECT_TRUE(request.validSensorPose());
    EXPECT_TRUE(request.validDistance());
    request.body_pose.has_head_orientation = true;
    EXPECT_FALSE(request.validSensorPose());
    request.type = MovementRequest::MOVE_TRIPOD;
    EXPECT_FALSE(request.validDistance());
    request.distance_m.reset();
    EXPECT_TRUE(request.validDistance());
}
}  // namespace
