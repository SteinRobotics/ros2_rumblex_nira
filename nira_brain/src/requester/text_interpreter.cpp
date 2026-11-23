/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "requester/text_interpreter.hpp"

#include <unicode/normalizer2.h>
#include <unicode/uchar.h>
#include <unicode/unistr.h>

#include <algorithm>
#include <iterator>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace brain {
namespace {

std::vector<std::string> tokenize(std::string_view text) {
    UErrorCode error = U_ZERO_ERROR;
    const auto* normalizer = icu::Normalizer2::getNFKCCasefoldInstance(error);
    if (U_FAILURE(error)) throw std::runtime_error(u_errorName(error));
    auto normalized = normalizer->normalize(icu::UnicodeString::fromUTF8(text), error);
    if (U_FAILURE(error)) throw std::runtime_error(u_errorName(error));

    // Make the negative suffix a token before punctuation splits English contractions.
    normalized.findAndReplace(u"n’t", u" not");
    normalized.findAndReplace(u"n't", u" not");

    std::vector<std::string> tokens;
    int32_t begin = 0;
    for (int32_t i = 0; i < normalized.length();) {
        const UChar32 c = normalized.char32At(i);
        const int32_t next = i + U16_LENGTH(c);
        if (!u_isalnum(c) && !(U_GET_GC_MASK(c) & U_GC_M_MASK)) {
            if (begin < i) normalized.tempSubStringBetween(begin, i).toUTF8String(tokens.emplace_back());
            begin = next;
        }
        i = next;
    }
    if (begin < normalized.length()) normalized.tempSubString(begin).toUTF8String(tokens.emplace_back());
    return tokens;
}

struct Direction {
    int forward = 0;
    int left = 0;
};

Direction directionOf(std::string_view word) {
    if (word == "vorn" || word == "vorne" || word == "forward" || word == "forwards") return {.forward = 1};
    if (word == "hinten" || word == "back" || word == "backward" || word == "backwards")
        return {.forward = -1};
    if (word == "links" || word == "left") return {.left = 1};
    if (word == "rechts" || word == "right") return {.left = -1};
    return {};
}

}  // namespace

CTextInterpreter::CTextInterpreter(std::istream& configuration) {
    const auto data = nlohmann::json::parse(configuration);
    if (!data.is_object() || data.empty()) throw std::invalid_argument("expected a nonempty command object");
    const auto addPattern = [this](const std::string& command, const std::string& path,
                                   const nlohmann::json& groups) {
        if (!groups.is_object() || groups.empty())
            throw std::invalid_argument(path + " requires a nonempty object of word groups");
        Rule rule{.command = command, .required = {}, .exact = {}};
        for (const auto& [name, entries] : groups.items()) {
            const auto context = path + "." + name;
            if (name.empty() || !entries.is_array() || entries.empty())
                throw std::invalid_argument(context + " must be a nonempty array of aliases");
            auto& aliases = name == "exact" ? rule.exact : rule.required.emplace_back();
            for (const auto& entry : entries) {
                if (!entry.is_string()) throw std::invalid_argument(context + " requires string aliases");
                auto tokens = tokenize(entry.get_ref<const std::string&>());
                if (tokens.empty()) throw std::invalid_argument(context + " contains an empty alias");
                if (name == "direction") {
                    const auto direction = directionOf(tokens.front());
                    if (tokens.size() != 1 || (!direction.forward && !direction.left))
                        throw std::invalid_argument(context + " has unknown direction: " + entry.dump());
                    rule.directional = true;
                }
                aliases.push_back(std::move(tokens));
            }
        }
        rules_.push_back(std::move(rule));
    };

    for (const auto& [command, patterns] : data.items()) {
        if (command.empty()) throw std::invalid_argument("expected a named command");
        if (patterns.contains("de") || patterns.contains("en")) {
            for (const auto& [language, groups] : patterns.items()) {
                if (language != "de" && language != "en")
                    throw std::invalid_argument(command + ": unexpected language " + language);
                addPattern(command, command + "." + language, groups);
            }
        } else {
            // Existing catalogs put the groups directly under the command ID.
            addPattern(command, command, patterns);
        }
    }
}

VoiceInterpretation CTextInterpreter::interpret(std::string_view text) const {
    auto words = tokenize(text);
    // In "walk forward right now", "right" modifies the time, not the direction.
    for (auto word = words.begin(); word != words.end(); ++word) {
        if (*word == "right" && std::next(word) != words.end() &&
            (*std::next(word) == "now" || *std::next(word) == "away"))
            word = words.erase(word);
    }
    static constexpr std::string_view negations[]{
        "nicht",  "nie",   "niemals",   "kein", "keine", "keinen", "keinem", "keiner",
        "keines", "keins", "keinerlei", "no",   "not",   "never",  "dont",   "cannot"};
    if (std::ranges::any_of(words, [](const auto& word) { return std::ranges::contains(negations, word); }))
        return {.status = InterpretationStatus::Negated};

    const auto groupMatches = [&words](const Aliases& aliases) {
        return std::ranges::any_of(
            aliases, [&words](const Tokens& phrase) { return !std::ranges::search(words, phrase).empty(); });
    };
    const bool hasAlternative = std::ranges::contains(words, "oder") || std::ranges::contains(words, "or");
    VoiceInterpretation match;
    for (const auto& rule : rules_) {
        if (!std::ranges::contains(rule.exact, words) &&
            (rule.required.empty() || !std::ranges::all_of(rule.required, groupMatches)))
            continue;
        if (hasAlternative) return {.status = InterpretationStatus::Ambiguous};
        VoiceInterpretation candidate{.status = InterpretationStatus::Matched, .command = rule.command};
        if (rule.directional) {
            for (const auto& word : words) {
                const auto direction = directionOf(word);
                if ((candidate.forward && direction.forward == -candidate.forward) ||
                    (candidate.left && direction.left == -candidate.left))
                    return {.status = InterpretationStatus::Ambiguous};
                if (direction.forward) candidate.forward = direction.forward;
                if (direction.left) candidate.left = direction.left;
            }
            if (rule.command == "commandTurn" && candidate.forward)
                return {.status = InterpretationStatus::Ambiguous};
        }
        if (match.status == InterpretationStatus::Matched && match != candidate)
            return {.status = InterpretationStatus::Ambiguous};
        match = std::move(candidate);
    }
    // A standalone alias can also be a competing instruction in a compound utterance
    // ("stop and walk left"). Only the full utterance can dispatch an exact alias.
    const auto conjunction = [](const std::string& word) { return word == "and" || word == "und"; };
    if (match.status == InterpretationStatus::Matched && std::ranges::any_of(words, conjunction)) {
        for (auto begin = words.begin(); begin != words.end();) {
            const auto end = std::find_if(begin, words.end(), conjunction);
            const auto clause = std::ranges::subrange(begin, end);
            for (const auto& rule : rules_) {
                if (rule.command != match.command &&
                    std::ranges::any_of(
                        rule.exact, [&](const Tokens& phrase) { return std::ranges::equal(clause, phrase); }))
                    return {.status = InterpretationStatus::Ambiguous};
            }
            if (end == words.end()) break;
            begin = std::next(end);
        }
    }
    return match;
}

}  // namespace brain
