#include "movement_handler_test_fixture.hpp"

namespace {
TEST_F(MovementHandlerTest, LevelingOnStandRecoversAfterDataLossAndToggleOff) {
    stand();
    const auto initial = poses.back();
    feed_imu = true;
    imu_roll = 0.1;
    handler->toggleOverlays(false, true);
    pump(700ms);
    EXPECT_LT(poses.back().torso_pose.orientation.roll, -0.1);
    EXPECT_EQ(poses.back().toe_positions, initial.toe_positions);
    EXPECT_EQ(poses.back().head_orientation, initial.head_orientation);
    feed_imu = false;
    pump(2s);
    EXPECT_EQ(poses.back(), initial);
    feed_imu = true;
    pump(400ms);
    EXPECT_LT(poses.back().torso_pose.orientation.roll, -0.1);
    handler->toggleOverlays(false, true);
    pump(1s);
    EXPECT_EQ(poses.back(), initial);
    EXPECT_FALSE(sequence_ended);
    EXPECT_TRUE(handler->requestFinished());
}

TEST_F(MovementHandlerTest, LevelingUsesOnlyRemainingTravelAroundAGaitPose) {
    auto seed = initialPose();
    seed.torso_pose.orientation.roll = 11.8;
    feedback->publish(seed);
    pump(150ms);
    request(MovementRequest::MOVE_TRIPOD);
    feed_imu = true;
    imu_roll = -0.2;
    handler->toggleOverlays(false, true);
    pump(600ms);
    ASSERT_FALSE(poses.empty());
    EXPECT_DOUBLE_EQ(poses.back().torso_pose.orientation.roll, 12.0);
    EXPECT_EQ(poses.back().toe_positions, seed.toe_positions);
    handler->toggleOverlays(false, true);
    pump(1s);
    EXPECT_DOUBLE_EQ(poses.back().torso_pose.orientation.roll, 11.8);
}

TEST_F(MovementHandlerTest, LevelingResumesAcrossLaydownAndAllMoveGaits) {
    feed_imu = true;
    imu_roll = 0.2;
    handler->toggleOverlays(false, true);
    stand();
    ASSERT_LT(poses.back().torso_pose.orientation.roll, -0.1);
    request(MovementRequest::SEQUENCE_LAYDOWN);
    pump(1300ms);
    EXPECT_DOUBLE_EQ(poses.back().torso_pose.orientation.roll, 0.0);
    request(MovementRequest::SEQUENCE_STAND_UP);
    pump(1300ms);
    EXPECT_LT(poses.back().torso_pose.orientation.roll, -0.1);
    for (const auto gait :
         {MovementRequest::MOVE_CRAWL, MovementRequest::MOVE_TRIPOD, MovementRequest::MOVE_RIPPLE,
          MovementRequest::MOVE_WAVE, MovementRequest::MOVE_COMBINED}) {
        SCOPED_TRACE(gait);
        request(gait);
        pump(300ms);
        EXPECT_LT(poses.back().torso_pose.orientation.roll, -0.1);
        EXPECT_EQ(velocities.back(), geometry_msgs::msg::Twist());
    }
    EXPECT_FALSE(sequence_ended);
}
}  // namespace
