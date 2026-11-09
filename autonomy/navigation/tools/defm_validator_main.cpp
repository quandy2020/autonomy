/* Copyright 2026 The Navigation Authors (navigation) */
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

#include "navigation/common/configuration_loader.hpp"
#include "navigation/perception/defm_engine_model.hpp"

std::vector<float> Read(const std::string& path, std::size_t count) {
    std::ifstream stream(path, std::ios::binary);
    std::vector<float> values(count);
    stream.read(reinterpret_cast<char*>(values.data()), count * sizeof(float));
    if (!stream || stream.peek() != std::char_traits<char>::eof())
        return {};
    return values;
}

bool Compare(const std::vector<float>& actual,
             const std::vector<float>& expected, const std::string& name) {
    if (actual.size() != expected.size() || actual.empty())
        return false;
    double maximum = 0.0;
    for (std::size_t i = 0; i < actual.size(); ++i) {
        if (!std::isfinite(actual[i]) || !std::isfinite(expected[i]))
            return false;
        maximum = std::max(
            maximum, std::abs(static_cast<double>(actual[i]) - expected[i]));
    }
    std::cout << name << " max_abs_error=" << maximum << "\n";
    return maximum < 1e-4;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr
            << "usage: navigation_defm_validator <manifest> <reference-dir>\n";
        return 2;
    }
    navigation::config::ModelManifest manifest;
    if (!navigation::common::LoadTextProto(argv[1], &manifest).Ok())
        return 1;
    navigation::perception::EngineDefmModel model;
    std::string error;
    if (!model.Load(manifest, &error)) {
        std::cerr << error << "\n";
        return 1;
    }
    for (const std::string name : {"zeros", "random", "ramp"}) {
        const auto prefix = std::string(argv[2]) + "/" + name;
        const auto input = Read(prefix + ".depth.f32", 3 * 224 * 224);
        navigation::perception::DefmOutput output;
        if (!model.Run(input, &output) || output.token_count != 256 ||
            output.token_dim != 384 ||
            !Compare(output.tokens, Read(prefix + ".tokens.f32", 256 * 384),
                     name + "/tokens") ||
            !Compare(output.pooled, Read(prefix + ".pooled.f32", 384),
                     name + "/pooled"))
            return 1;
    }
    navigation::perception::DefmOutput output;
    if (model.Run(std::vector<float>(1), &output) ||
        model.Run(std::vector<float>(3 * 224 * 224,
                                     std::numeric_limits<float>::quiet_NaN()),
                  &output))
        return 1;
    auto invalid = manifest;
    invalid.set_sha256(std::string(64, '0'));
    if (model.Load(invalid, &error) ||
        error.find("sha256 mismatch") == std::string::npos ||
        model.Run(std::vector<float>(3 * 224 * 224), &output))
        return 1;
    invalid = manifest;
    invalid.mutable_outputs(0)->set_name("wrong_tokens");
    if (model.Load(invalid, &error) ||
        error.find("metadata") == std::string::npos)
        return 1;
    invalid = manifest;
    invalid.mutable_inputs(0)->set_dtype("float16");
    if (model.Load(invalid, &error) ||
        error.find("metadata") == std::string::npos)
        return 1;
    invalid = manifest;
    invalid.mutable_inputs(0)->set_shape(0, 2);
    if (model.Load(invalid, &error) ||
        error.find("input must") == std::string::npos)
        return 1;
    invalid = manifest;
    invalid.mutable_outputs(0)->set_shape(0, 2);
    if (model.Load(invalid, &error) ||
        error.find("shapes") == std::string::npos)
        return 1;
    std::cout << "DeFM hash, tensor metadata and C++/ORT validation passed\n";
    return 0;
}
