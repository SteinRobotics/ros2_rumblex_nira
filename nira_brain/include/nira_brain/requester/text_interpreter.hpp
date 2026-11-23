/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#pragma once

#include <istream>
#include <string>
#include <string_view>
#include <vector>

namespace brain {

enum class InterpretationStatus { NoMatch, Matched, Negated, Ambiguous };

struct VoiceInterpretation {
    InterpretationStatus status = InterpretationStatus::NoMatch;
    std::string command{};
    int forward = 0;  // +1 forward, -1 backward
    int left = 0;     // +1 left, -1 right
    bool operator==(const VoiceInterpretation&) const = default;
};

class CTextInterpreter {
   public:
    explicit CTextInterpreter(std::istream& configuration);
    VoiceInterpretation interpret(std::string_view text) const;

   private:
    using Tokens = std::vector<std::string>;
    using Aliases = std::vector<Tokens>;
    struct Rule {
        std::string command;
        std::vector<Aliases> required;
        Aliases exact;
        bool directional = false;
    };
    std::vector<Rule> rules_;
};

}  // namespace brain
