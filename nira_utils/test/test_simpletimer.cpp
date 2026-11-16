#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <nira_utils/simpletimer.hpp>
#include <thread>

TEST(SimpleTimerTest, GetSecondsElapsedReportsCorrectTime) {
    utils::CSimpleTimer timer;
    timer.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    double elapsed = timer.getSecondsElapsed();
    EXPECT_GE(elapsed, 0.25);
    EXPECT_LT(elapsed, 0.3);

    timer.stop();
    EXPECT_EQ(timer.getSecondsElapsed(), 0.0);
}

TEST(SimpleTimerTest, HaveSecondsElapsedWorksCorrectly) {
    utils::CSimpleTimer timer;
    timer.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    EXPECT_TRUE(timer.haveSecondsElapsed(0.2));
    EXPECT_FALSE(timer.haveSecondsElapsed(0.4));

    timer.stop();
    EXPECT_FALSE(timer.haveSecondsElapsed(0.1));
}