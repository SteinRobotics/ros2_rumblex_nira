#include <gtest/gtest.h>

#include <limits>
#include <opencv2/objdetect/face.hpp>

#include "nira_perception_face/tracking.hpp"

using namespace nira_perception_face;
TEST(TargetSelector, AcquiresPersistentFaceAndKeepsItWhenLargerFaceArrives) {
    TargetSelector selector(3);
    const Detection a{{20, 20, 40, 40}, 0.9F, {28, 32}, {42, 36}};
    const Detection b{{200, 20, 80, 80}, 0.95F, {210, 40}, {240, 30}};
    EXPECT_FALSE(selector.update({a}));
    EXPECT_FALSE(selector.update({a}));
    ASSERT_TRUE(selector.update({a}));
    const auto id = selector.trackId();
    ASSERT_TRUE(selector.update({a, b}));
    const auto tracked = selector.update({a, b});
    ASSERT_TRUE(tracked);
    EXPECT_EQ(tracked->box, a.box);
    EXPECT_EQ(tracked->right_eye, a.right_eye);
    EXPECT_EQ(tracked->left_eye, a.left_eye);
    EXPECT_EQ(selector.trackId(), id);
}
TEST(TargetSelector, LossAndAmbiguityRequireFreshAcquisition) {
    TargetSelector selector(2);
    const Detection a{{20, 20, 40, 40}, 0.9F};
    const Detection b{{22, 22, 40, 40}, 0.95F};
    EXPECT_FALSE(selector.update({a}));
    ASSERT_TRUE(selector.update({a}));
    const auto id = selector.trackId();
    EXPECT_FALSE(selector.update({a, b}));
    EXPECT_FALSE(selector.update({a}));
    ASSERT_TRUE(selector.update({a}));
    EXPECT_NE(selector.trackId(), id);
    EXPECT_FALSE(selector.update({}));
    EXPECT_FALSE(selector.update({a}));
}
TEST(TargetSelector, InvalidDetectionsNeverAcquire) {
    TargetSelector selector(1);
    EXPECT_FALSE(selector.update({{{0, 0, -1, 20}, 0.9F}}));
    EXPECT_FALSE(selector.update({{{0, 0, 20, 20}, std::numeric_limits<float>::quiet_NaN()}}));
}
TEST(EyeLineRoll, ReportsSignedTiltAndRejectsMissingEyes) {
    EXPECT_EQ(eyeLineRoll({2, 3}, {4, 3}), 0.0);
    EXPECT_NEAR(*eyeLineRoll({2, 3}, {4, 5}), std::atan(1.0), 1e-6);
    EXPECT_NEAR(*eyeLineRoll({2, 3}, {4, 1}), -std::atan(1.0), 1e-6);
    EXPECT_FALSE(eyeLineRoll({}, {}));
    EXPECT_FALSE(eyeLineRoll({std::numeric_limits<float>::quiet_NaN(), 3}, {4, 5}));
}
TEST(YuNet, BundledModelRunsOnBlankImageWithoutDetection) {
    cv::setNumThreads(1);
    const auto detector = cv::FaceDetectorYN::create(FACE_MODEL_PATH, "", {320, 240});
    cv::Mat faces;
    detector->detect(cv::Mat::zeros(240, 320, CV_8UC3), faces);
    EXPECT_EQ(faces.rows, 0);
}
