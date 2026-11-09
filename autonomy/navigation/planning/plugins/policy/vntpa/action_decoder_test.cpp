/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>
#include <vector>

#include "navigation/planning/plugins/policy/vntpa/action_decoder.hpp"

int main() {
    using navigation::planning::vntpa::ActionDecoder;
    using navigation::planning::vntpa::ActionDecoderOptions;

    ActionDecoderOptions options;
    options.forward_step = 0.25;
    options.turn_step_rad = 0.5235987756;
    options.dt_s = 0.5;
    ActionDecoder decoder(options);

    // Classes are STOP, FORWARD, TURN_LEFT, TURN_RIGHT.
    const std::vector<float> logits = {0.1F, 0.9F, 0.1F, 0.1F,
                                       0.1F, 0.1F, 0.9F, 0.1F};
    std::vector<float> actions;
    std::vector<std::uint32_t> kinds;
    if (!decoder.Decode(logits, 2, &actions, &kinds) || kinds.size() != 2 ||
        kinds[0] != 1 || kinds[1] != 2 || actions.size() != 6) {
        std::cerr << "logit decode failed\n";
        return 1;
    }
    if (std::abs(actions[0] - 0.5F) > 1e-5F ||
        std::abs(actions[5] - static_cast<float>(options.turn_step_rad /
                                                 options.dt_s)) > 1e-5F) {
        std::cerr << "action mapping failed\n";
        return 1;
    }

    const std::vector<float> legacy = {0.1F, 0.0F, 0.2F, 0.3F, 0.0F, 0.1F};
    if (!decoder.Decode(legacy, 2, &actions, &kinds) || !kinds.empty() ||
        std::abs(actions[0] - 0.1F) > 1e-6F) {
        std::cerr << "legacy decode failed\n";
        return 1;
    }
    return 0;
}
