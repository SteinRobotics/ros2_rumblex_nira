#include <gtest/gtest.h>

#include <thread>

#include "gait/gait_controller.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "test_helpers.hpp"

namespace {
using namespace brain;
using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using mp_units::angular::unit_symbols::deg;

class SensorGaitsTest : public ::testing::Test {
   protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        auto parameters = test_helpers::defaultRobotParameters();
        const auto range_params = test_helpers::holdDistanceParameters();
        parameters.insert(parameters.end(), range_params.begin(), range_params.end());
        rclcpp::NodeOptions options;
        options.parameter_overrides(parameters);
        node = std::make_shared<rclcpp::Node>("sensor_gaits_test", options);
        model = std::make_shared<CPoseModel>(node);
        controller = std::make_unique<CGaitController>(node, model);
        model->moveTorso(model->getStandingToePositions(), CPose());
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
        ranges = node->create_publisher<sensor_msgs::msg::Range>("test_range", rclcpp::SensorDataQoS());
        controller->on_gait_changed = [&](const MovementRequest& request) { changes.push_back(request); };
    }
    void TearDown() override {
        controller.reset();
        executor.reset();
        model.reset();
        node.reset();
        rclcpp::shutdown();
    }
    void select(MovementRequest::Type type, double duration, double distance = 0.6) {
        MovementRequest request;
        request.type = type;
        request.duration_s = duration * s;
        if (type == MovementRequest::HOLD_DISTANCE) request.distance_m = distance * m;
        ASSERT_TRUE(controller->setGait(request));
        if (controller->takeSequenceAbort()) ++sequence_ends;
    }
    void tick(int count = 1) {
        for (int i = 0; i < count; ++i) {
            if (feed_range) {
                sensor_msgs::msg::Range range;
                range.header.frame_id = "base_link";
                range.header.stamp = node->now();
                range.min_range = 0.05;
                range.max_range = 10;
                range.range = range_m;
                ranges->publish(range);
            }
            executor->spin_some();
            controller->updateSelectedGait(external, CPose(), COrientation(), 0.02 * s);
            if (controller->takeSequenceAbort()) ++sequence_ends;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<CPoseModel> model;
    std::unique_ptr<CGaitController> controller;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ranges;
    bool feed_range = false;
    float range_m = 0.6;
    Velocity external;
    int sequence_ends = 0;
    std::vector<MovementRequest> changes;
};
TEST_F(SensorGaitsTest, HoldDistanceIsFiniteAndOwnsVelocityAndHeadMotion) {
    model->setHeadOrientation(COrientation(0.0 * deg, -5.0 * deg, 15.0 * deg));
    const auto initial = bodyPose(*model);
    external.linear.x = 0.1 * m / s;
    feed_range = true;
    select(MovementRequest::HOLD_DISTANCE, 0.3);
    EXPECT_EQ(controller->currentGait(), MovementRequest::HOLD_DISTANCE);
    EXPECT_TRUE(controller->finishesAutomatically());
    EXPECT_EQ(bodyPose(*model), initial);  // start captures; the first update owns motion.
    EXPECT_EQ(controller->commandedVelocity(external).toMsg(), geometry_msgs::msg::Twist());
    tick(100);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_EQ(sequence_ends, 0);
    EXPECT_EQ(model->getHeadOrientation(), COrientation());
    EXPECT_EQ(model->getTorsoPose(), CPose(initial.torso_pose));
    EXPECT_EQ(controller->commandedVelocity(external).toMsg(), geometry_msgs::msg::Twist());
    EXPECT_EQ(node->count_subscribers("test_imu"), 0u);
}
TEST_F(SensorGaitsTest, SameHoldGaitRequestReplacesDistanceAfterStopping) {
    feed_range = true;
    range_m = 1;
    select(MovementRequest::HOLD_DISTANCE, 10, 1.0);
    tick(70);
    EXPECT_EQ(controller->commandedVelocity(external).linear.x, 0.0 * m / s);
    select(MovementRequest::HOLD_DISTANCE, 1, 0.6);
    EXPECT_TRUE(controller->hasPendingGait());
    for (int i = 0; i < 100 && controller->commandedVelocity(external).linear.x == 0.0 * m / s; ++i) tick();
    EXPECT_GT(controller->commandedVelocity(external).linear.x, 0.0 * m / s);
    ASSERT_EQ(changes.size(), 2u);
    ASSERT_TRUE(changes.back().distance_m);
    EXPECT_EQ(*changes.back().distance_m, 0.6 * m);
    controller->requestStopSelectedGait();
    tick(400);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_EQ(sequence_ends, 0);
}
TEST_F(SensorGaitsTest, StoppingDoesNotRequestSequenceEndAgain) {
    feed_range = true;
    range_m = 1.0;
    select(MovementRequest::HOLD_DISTANCE, 10);
    tick(80);
    controller->updateSelectedGait(external, CPose(), COrientation(), 1.0 * s);
    EXPECT_TRUE(controller->takeSequenceAbort());
    controller->updateSelectedGait(external, CPose(), COrientation(), 1.0 * s);
    EXPECT_FALSE(controller->takeSequenceAbort());
    tick(400);
    EXPECT_TRUE(controller->motionStopped());
    EXPECT_EQ(sequence_ends, 0);
}
TEST_F(SensorGaitsTest, InvalidDistanceIsRejectedBeforeTheCurrentGaitChanges) {
    const auto initial_gait = controller->currentGait();
    MovementRequest request;
    request.type = MovementRequest::HOLD_DISTANCE;
    EXPECT_FALSE(controller->setGait(request));
    EXPECT_EQ(controller->currentGait(), initial_gait);
    request.distance_m = -0.6 * m;
    EXPECT_FALSE(controller->setGait(request));
    request.type = MovementRequest::MOVE_TRIPOD;
    EXPECT_FALSE(controller->setGait(request));
    EXPECT_EQ(sequence_ends, 0);
}
}  // namespace
