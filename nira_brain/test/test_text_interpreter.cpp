#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <tuple>

#include "requester/text_interpreter.hpp"

using namespace brain;

namespace {
std::ifstream rulesFile() {
    return std::ifstream(ament_index_cpp::get_package_share_path("nira_brain") /
                         "config/nira/interpretation.json");
}

class TextInterpreterTest : public ::testing::Test {
   protected:
    std::ifstream file_ = rulesFile();
    CTextInterpreter interpreter_{file_};
};

TEST_F(TextInterpreterTest, RecognizesEveryConfiguredAliasCombination) {
    const auto rules = nlohmann::json::parse(rulesFile());
    for (const auto& [command, languages] : rules.items()) {
        for (const auto language : {"de", "en"}) {
            ASSERT_TRUE(languages.contains(language)) << command;
            const auto& groups = languages.at(language);
            std::vector<std::string> phrases{""};
            for (const auto& [group, aliases] : groups.items()) {
                if (group == "exact") {
                    for (const std::string alias : aliases)
                        EXPECT_EQ(interpreter_.interpret(alias).command, command) << alias;
                    continue;
                }
                std::vector<std::string> expanded;
                for (const auto& prefix : phrases)
                    for (const std::string alias : aliases) expanded.push_back(prefix + " " + alias);
                phrases = std::move(expanded);
            }
            for (const auto& phrase : phrases) {
                if (phrase.empty()) continue;
                EXPECT_EQ(interpreter_.interpret(phrase).command, command) << phrase;
            }
        }
    }
}

TEST_F(TextInterpreterTest, AcceptsFillerPunctuationAndGermanCapitalization) {
    for (auto phrase : {"steh auf", "Steh auf!", "„Steh jetzt auf!“", "STEH AUF"})
        EXPECT_EQ(interpreter_.interpret(phrase).command, "commandStandup") << phrase;
    EXPECT_EQ(interpreter_.interpret("BEIẞ!").command, "commandBite");
    EXPECT_EQ(interpreter_.interpret("sieh dich um").command, "commandWatch");
    EXPECT_EQ(interpreter_.interpret("leg dich hin").command, "commandLaydown");
}

TEST_F(TextInterpreterTest, DeclinesNegatedAndAmbiguousCommands) {
    for (auto phrase :
         {"steh nicht auf", "spiel KEINE Musik!", "renne auf keinen Fall nach vorne", "niemals kalibrieren"})
        EXPECT_EQ(interpreter_.interpret(phrase).status, InterpretationStatus::Negated) << phrase;
    for (auto phrase : {"schau nach links und rechts", "geh nach links oder rechts", "renn vorne hinten",
                        "flitz links rechts", "stoppe musik und geh nach links", "geh nach vorne oder links",
                        "dreh nach vorne links"})
        EXPECT_EQ(interpreter_.interpret(phrase).status, InterpretationStatus::Ambiguous) << phrase;
    const auto diagonal = interpreter_.interpret("geh nach vorne links");
    EXPECT_EQ(diagonal.command, "commandMove");
    EXPECT_EQ(diagonal.forward, 1);
    EXPECT_EQ(diagonal.left, 1);
}

TEST_F(TextInterpreterTest, RequiresContiguousTitleAliasesAndExactStandaloneStop) {
    for (auto phrase : {"tanz auf der Straße", "tanze hot", "tanz dancing jetzt street", "party",
                        "stopp die kamera", "dreh dich nach vorne", "", "!!!"})
        EXPECT_EQ(interpreter_.interpret(phrase).status, InterpretationStatus::NoMatch) << phrase;
    EXPECT_EQ(interpreter_.interpret("tanz bitte Dancing Street").command, "commandDanceDancingStreet");
    EXPECT_EQ(interpreter_.interpret("SToPP!").command, "commandStopMove");
    EXPECT_EQ(interpreter_.interpret("stopp musik").command, "musicOff");
}

TEST_F(TextInterpreterTest, AlternatesNaturalGermanAndEnglishPhrases) {
    for (const auto& [german, english, command] :
         std::vector<std::tuple<std::string, std::string, std::string>>{
             {"stampf", "stomp", "commandStomp"},
             {"steh auf", "stand up", "commandStandup"},
             {"dreh nach links", "turn left", "commandTurn"},
             {"beiß", "bite", "commandBite"},
             {"schau nach rechts", "look to the right", "commandLookRight"},
             {"leg dich hin", "lie down", "commandLaydown"},
             {"gib mir five", "give me a high five", "commandHighFive"},
             {"lauf nach vorne links", "walk forward and left", "commandMove"},
             {"renn nach hinten", "run backwards", "commandRun"},
             {"geh nach vorne", "walk forward right now", "commandMove"},
             {"sag servo spannung", "what is the servo's voltage?", "tellMeServoVoltage"},
             {"ist servo warm", "how hot are the servos?", "tellMeServoTemperature"},
             {"stopp", "stop", "commandStopMove"},
             {"stop musik", "stop the music", "musicOff"},
             {"tanz hot dogs", "dance to Hot Dogs for Breakfast", "commandDanceHotDogsForBreakfast"},
             {"deaktivier kamera", "please turn the camera off", "commandCameraOff"}}) {
        const auto original = interpreter_.interpret(german);
        ASSERT_EQ(original.command, command) << german;
        EXPECT_EQ(interpreter_.interpret(english), original) << english;
    }
}

TEST_F(TextInterpreterTest, RejectsEnglishNegationAndConflictingInstructions) {
    for (const auto phrase : {"don't stand up", "don’t stomp", "do not move right", "never run forward",
                              "no music", "please dont turn left"}) {
        EXPECT_EQ(interpreter_.interpret(phrase).status, InterpretationStatus::Negated) << phrase;
    }
    for (const auto phrase :
         {"turn left or right", "run backwards and forwards", "turn left and look left", "stop and walk left",
          "walk left and stop", "stampf und stand up", "walk left rechts", "play music or dance"}) {
        EXPECT_EQ(interpreter_.interpret(phrase).status, InterpretationStatus::Ambiguous) << phrase;
    }
    EXPECT_EQ(interpreter_.interpret("schau nach right").status, InterpretationStatus::NoMatch);
    EXPECT_EQ(interpreter_.interpret("walk right away").status, InterpretationStatus::NoMatch);
}

TEST(TextInterpreterRules, EquivalentLanguagePatternsMatchOnceAndConflictingArgumentsAreAmbiguous) {
    std::istringstream file(R"({
        "same":{"de":{"doing":["test"],"direction":["links"]},"en":{"doing":["test"],"direction":["left"]}},
        "different":{"de":{"doing":["demo"],"direction":["links"]},"en":{"exact":["demo links"]}}
    })");
    CTextInterpreter interpreter(file);
    const auto result = interpreter.interpret("test links left");
    EXPECT_EQ(result.command, "same");
    EXPECT_EQ(result.left, 1);
    EXPECT_EQ(interpreter.interpret("demo links").status, InterpretationStatus::Ambiguous);
}

TEST(TextInterpreterRules, NormalizesAliasesAndAcceptsExactOnlyRules) {
    std::istringstream file(R"({"greeting":{"exact":["GRÜẞ GOTT!"]}})");
    CTextInterpreter interpreter(file);
    EXPECT_EQ(interpreter.interpret("gru\u0308ss Gott").command, "greeting");
    EXPECT_EQ(interpreter.interpret("grüß Gott bitte").status, InterpretationStatus::NoMatch);
}

TEST(TextInterpreterRules, RejectsMalformedConfigurationWithContext) {
    for (auto invalid : {"[]", "{}", R"("stop")", R"({"stop":{}})", R"({"stop":{"en":{}}})",
                         R"({"stop":{"de":{"exact":["stopp"]},"bad":[]}})"}) {
        std::istringstream file(invalid);
        EXPECT_THROW((void)CTextInterpreter(file), std::invalid_argument);
    }
    for (auto invalid : {R"({"bad":{"noun":[]}})", R"({"bad":{"noun":[2]}})", R"({"bad":{"noun":["?!"]}})",
                         R"({"bad":{"noun":"wrong shape"}})"}) {
        std::istringstream file(invalid);
        try {
            (void)CTextInterpreter(file);
            FAIL() << invalid;
        } catch (const std::invalid_argument& error) {
            EXPECT_NE(std::string_view(error.what()).find("bad.noun"), std::string_view::npos);
        }
    }
}
}  // namespace
