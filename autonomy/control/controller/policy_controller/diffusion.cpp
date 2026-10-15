/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/diffusion.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

float Clamp01(float value) {
    return std::max(0.0f, std::min(1.0f, value));
}

// Cosine alpha_bar schedule in [0, 1], matching common diffusion training.
float AlphaBar(float timestep_normalized) {
    const float t = Clamp01(timestep_normalized);
    const float value =
        std::cos((t + 0.008f) / 1.008f * static_cast<float>(M_PI) * 0.5f);
    return std::max(1e-5f, value * value);
}

void SampleGaussian(std::vector<float>* data, std::mt19937* rng) {
    std::normal_distribution<float> normal(0.0f, 1.0f);
    for (float& value : *data) {
        value = normal(*rng);
    }
}

}  // namespace

proto::Diffusion MakeEffectiveDiffusion(const proto::PolicyConfig& proto) {
    proto::Diffusion options = proto.diffusion();
    if (options.steps() <= 0) {
        options.set_steps(1);
    }
    if (options.horizon() <= 0) {
        options.set_horizon(1);
    }
    if (options.dim() <= 0) {
        int slots = proto.action().layout_size();
        if (slots <= 0 && proto.outputs_size() > 0) {
            const int index = std::max(0, proto.action().output_index());
            if (index < proto.outputs_size()) {
                slots = proto.outputs(index).layout_size();
            }
        }
        options.set_dim(slots > 0 ? slots : 2);
    }
    if (options.sample().empty()) {
        options.set_sample("sample");
    }
    if (options.timestep().empty()) {
        options.set_timestep("timestep");
    }
    if (options.prediction_type() == proto::DIFFUSION_PRED_UNSPECIFIED) {
        options.set_prediction_type(proto::DIFFUSION_PRED_SAMPLE);
    }
    if (options.timestep_scale() <= 0.0f) {
        options.set_timestep_scale(1.0f);
    }
    if (options.execute_index() < 0) {
        options.set_execute_index(0);
    }
    return options;
}

namespace {

std::string PredictionTensorName(const PolicyConfig& config,
                                 const proto::Diffusion& diffusion) {
    if (!diffusion.prediction().empty()) {
        return diffusion.prediction();
    }
    return config.action_schema().tensor_name;
}

void DdimUpdate(const std::vector<float>& prediction,
                const std::vector<float>& current, float alpha_bar_t,
                float alpha_bar_prev, float eta,
                proto::DiffusionPrediction prediction_type,
                std::vector<float>* next, std::mt19937* rng) {
    const std::size_t n = current.size();
    next->resize(n);
    const float sqrt_alpha_bar_t = std::sqrt(alpha_bar_t);
    const float sqrt_one_minus_alpha_bar_t = std::sqrt(1.0f - alpha_bar_t);
    const float sqrt_alpha_bar_prev = std::sqrt(alpha_bar_prev);

    std::vector<float> x0(n);
    std::vector<float> epsilon(n);
    if (prediction_type == proto::DIFFUSION_PRED_EPSILON) {
        for (std::size_t i = 0; i < n; ++i) {
            epsilon[i] = prediction[i];
            x0[i] = (current[i] - sqrt_one_minus_alpha_bar_t * epsilon[i]) /
                    std::max(1e-6f, sqrt_alpha_bar_t);
        }
    } else {
        for (std::size_t i = 0; i < n; ++i) {
            x0[i] = prediction[i];
            epsilon[i] = (current[i] - sqrt_alpha_bar_t * x0[i]) /
                         std::max(1e-6f, sqrt_one_minus_alpha_bar_t);
        }
    }

    const float sigma =
        eta * std::sqrt(std::max(
                  0.0f, (1.0f - alpha_bar_prev) /
                                std::max(1e-6f, 1.0f - alpha_bar_t) *
                            (1.0f - alpha_bar_t / std::max(1e-6f, alpha_bar_prev))));
    const float direction =
        std::sqrt(std::max(0.0f, 1.0f - alpha_bar_prev - sigma * sigma));

    std::normal_distribution<float> normal(0.0f, 1.0f);
    for (std::size_t i = 0; i < n; ++i) {
        float noise = 0.0f;
        if (sigma > 0.0f && rng != nullptr) {
            noise = normal(*rng);
        }
        (*next)[i] =
            sqrt_alpha_bar_prev * x0[i] + direction * epsilon[i] + sigma * noise;
    }
}

}  // namespace

bool RunDiffusionInference(
    const PolicyModel& model,
    const common::network::FloatTensorMap& condition_inputs,
    std::vector<float>* action_sequence, std::string* error,
    InferBudget* budget) {
    if (action_sequence == nullptr) {
        return false;
    }

    const auto& config = model.config();
    const auto& diffusion = config.effective_diffusion();
    const int horizon = diffusion.horizon();
    const int action_dimension = diffusion.dim();
    int steps = diffusion.steps();
    if (budget != nullptr && budget->max_diffusion_steps > 0) {
        steps = std::min(steps, budget->max_diffusion_steps);
    }
    const std::size_t sequence_size =
        static_cast<std::size_t>(horizon) *
        static_cast<std::size_t>(action_dimension);
    if (sequence_size == 0) {
        if (error != nullptr) {
            *error = "Diffusion action sequence size is zero";
        }
        return false;
    }

    const std::string prediction_name = PredictionTensorName(config, diffusion);
    if (prediction_name.empty()) {
        if (error != nullptr) {
            *error = "Diffusion prediction tensor name is empty";
        }
        return false;
    }

    thread_local std::mt19937 rng{std::random_device{}()};
    std::vector<float> sample(sequence_size, 0.0f);
    SampleGaussian(&sample, &rng);

    // Build working map once; only refresh sample/timestep each step.
    common::network::FloatTensorMap inputs = condition_inputs;
    auto& noisy_slot = inputs[diffusion.sample()];
    auto& timestep_slot = inputs[diffusion.timestep()];
    noisy_slot.resize(sequence_size);
    timestep_slot.resize(1);
    common::network::FloatTensorMap outputs;
    std::vector<float> prediction;
    prediction.resize(sequence_size);
    std::vector<float> next;

    const int iterations = std::max(1, steps);
    for (int step_index = 0; step_index < iterations; ++step_index) {
        if (budget != nullptr && budget->Exceeded()) {
            if (error != nullptr) {
                *error = "Inference timeout during diffusion";
            }
            return false;
        }
        const int reverse_index = iterations - 1 - step_index;
        const float t_norm =
            iterations == 1
                ? 0.0f
                : static_cast<float>(reverse_index) /
                      static_cast<float>(iterations - 1);
        const float timestep_value = t_norm * diffusion.timestep_scale();

        noisy_slot.assign(sample.begin(), sample.end());
        timestep_slot[0] = timestep_value;

        outputs.clear();
        if (!model.RunInference(inputs, &outputs, error)) {
            return false;
        }
        const auto it = outputs.find(prediction_name);
        if (it == outputs.end()) {
            if (error != nullptr) {
                *error =
                    "Missing diffusion prediction tensor: " + prediction_name;
            }
            return false;
        }
        if (it->second.size() < sequence_size) {
            if (error != nullptr) {
                *error = "Diffusion prediction shorter than action sequence";
            }
            return false;
        }
        prediction.assign(it->second.begin(),
                          it->second.begin() +
                              static_cast<std::ptrdiff_t>(sequence_size));

        if (iterations == 1 || reverse_index == 0) {
            if (diffusion.prediction_type() == proto::DIFFUSION_PRED_EPSILON) {
                const float alpha_bar_t = AlphaBar(t_norm);
                const float sqrt_alpha = std::sqrt(alpha_bar_t);
                const float sqrt_one_minus = std::sqrt(1.0f - alpha_bar_t);
                for (std::size_t i = 0; i < sequence_size; ++i) {
                    sample[i] = (sample[i] - sqrt_one_minus * prediction[i]) /
                                std::max(1e-6f, sqrt_alpha);
                }
            } else {
                sample.swap(prediction);
            }
            break;
        }

        const float t_prev_norm = static_cast<float>(reverse_index - 1) /
                                  static_cast<float>(iterations - 1);
        if (diffusion.ddim() ||
            diffusion.prediction_type() == proto::DIFFUSION_PRED_EPSILON) {
            DdimUpdate(prediction, sample, AlphaBar(t_norm),
                       AlphaBar(t_prev_norm), diffusion.ddim_eta(),
                       diffusion.prediction_type(), &next, &rng);
            sample.swap(next);
        } else {
            sample.swap(prediction);
        }
    }

    *action_sequence = std::move(sample);
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
