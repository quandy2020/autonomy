/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>
#include <memory>

#include "navigation/perception/defm_encoder.hpp"

namespace
{

class FakeModel final : public navigation::perception::DefmModel
{
public:
    bool Load(const navigation::config::ModelManifest&, std::string*) override
    {
        return true;
    }

    bool Run(const std::vector<float>& input,
             navigation::perception::DefmOutput* output) override
    {
        if (input.empty() || output == nullptr)
        {
            return false;
        }
        output->token_count = 4;
        output->token_dim = 2;
        output->tokens = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F};
        output->pooled = {4.0F, 5.0F};
        return true;
    }

    std::string_view Name() const override
    {
        return "fake";
    }
};

}  // namespace

int main()
{
    using namespace navigation;  // NOLINT

    auto encoder = std::make_unique<perception::DepthFoundationModelEncoder>(
        std::make_unique<perception::DepthPreprocessor>(),
        std::make_unique<FakeModel>(), std::make_unique<perception::AttentionPooler>(),
        std::make_unique<perception::PerceptualUncertaintyEstimator>(),
        perception::DepthFoundationModelEncoder::Options{0.15});

    observation::Observation observation;
    observation.mutable_header()->mutable_stamp()->set_sec(1);
    observation.set_sequence(7);
    auto* depth = observation.mutable_depth();
    depth->set_width(2);
    depth->set_height(2);
    depth->set_encoding("32FC1");
    depth->set_step(2 * sizeof(float));
    const float values[4] = {1.0F, 2.0F, 3.0F, 4.0F};
    depth->set_data(reinterpret_cast<const char*>(values), sizeof(values));

    observation::DepthFeature feature;
    if (!encoder->Encode(observation, &feature).Ok())
    {
        std::cerr << "encode failed\n";
        return 1;
    }
    if (feature.token_count() != 4 || feature.token_dim() != 2 ||
        feature.tokens_size() != 8 || feature.pooled_size() != 2)
    {
        std::cerr << "feature shape failed\n";
        return 1;
    }
    if (feature.stamp_ns() != 1000000000LL || feature.sequence() != 7)
    {
        std::cerr << "feature metadata failed\n";
        return 1;
    }
    if (feature.uncertainty() < 0.0F || feature.uncertainty() > 1.0F)
    {
        std::cerr << "uncertainty range failed\n";
        return 1;
    }

    observation.mutable_depth()->clear_data();
    if (encoder->Encode(observation, &feature).Ok())
    {
        std::cerr << "empty depth accepted\n";
        return 1;
    }
    return 0;
}
