#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "requester/text_interpreter.hpp"

using namespace brain;

class TextInterpreterTest : public ::testing::Test {
   protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = std::make_shared<rclcpp::Node>("test_text_interpreter_node");
        interpreter_ = std::make_unique<CTextInterpreter>(node_);
    }

    void TearDown() override {
        // reset interpreter before shutting down rclcpp
        interpreter_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<CTextInterpreter> interpreter_;
};

TEST_F(TextInterpreterTest, CommandLookLeft) {
    std::string text = "sieh nach links";
    auto words = interpreter_->parseText(text);
    EXPECT_EQ(words.size(), 3u);

    std::string command = interpreter_->searchInterpretation(words);
    EXPECT_EQ(command, "commandLookLeft");
}

TEST_F(TextInterpreterTest, CommandWatch) {
    std::string text = "sieh dich um";
    auto words = interpreter_->parseText(text);
    EXPECT_EQ(words.size(), 3u);

    std::string command = interpreter_->searchInterpretation(words);
    EXPECT_EQ(command, "commandWatch");
}

TEST_F(TextInterpreterTest, CommandStandup) {
    std::string text = "steh jetzt auf";
    auto words = interpreter_->parseText(text);
    // "steh auf" is two tokens
    EXPECT_EQ(words.size(), 2u);

    std::string command = interpreter_->searchInterpretation(words);
    EXPECT_EQ(command, "commandStandup");
}

TEST_F(TextInterpreterTest, CommandLaydown) {
    std::string text = "leg dich hin";
    auto words = interpreter_->parseText(text);
    EXPECT_EQ(words.size(), 3u);

    std::string command = interpreter_->searchInterpretation(words);
    EXPECT_EQ(command, "commandLaydown");
}

TEST_F(TextInterpreterTest, RecognizesMovementAndMusicCommands) {
    for (const auto& [phrase, command] :
         std::vector<std::pair<std::string, std::string>>{{"sieh nach links", "commandLookLeft"},
                                                          {"steh auf", "commandStandup"},
                                                          {"leg dich hin", "commandLaydown"},
                                                          {"spiele musik", "musicOn"},
                                                          {"teste gang", "commandWalkDemo"}}) {
        auto text = phrase;
        EXPECT_EQ(interpreter_->searchInterpretation(interpreter_->parseText(text)), command) << phrase;
    }
}

TEST_F(TextInterpreterTest, RecognizesNiraSensorCommands) {
    for (const auto& [phrase, command] :
         std::vector<std::pair<std::string, std::string>>{{"aktiviere lidar", "commandLidarOn"},
                                                          {"deaktiviere laserscanner", "commandLidarOff"},
                                                          {"aktivier kamera", "commandCameraOn"},
                                                          {"deaktivier camera", "commandCameraOff"}}) {
        auto text = phrase;
        EXPECT_EQ(interpreter_->searchInterpretation(interpreter_->parseText(text)), command) << phrase;
    }
}

TEST_F(TextInterpreterTest, CanConstructAnotherInterpreter) {
    CTextInterpreter second_interpreter(node_);
    std::string text = "aktiviere lidar";
    EXPECT_EQ(second_interpreter.searchInterpretation(second_interpreter.parseText(text)), "commandLidarOn");
}
