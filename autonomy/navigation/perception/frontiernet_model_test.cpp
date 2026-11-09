/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "navigation/common/configuration_loader.hpp"
#include "navigation/perception/depth_preprocessor.hpp"
#include "navigation/perception/frontiernet_model.hpp"

namespace
{

bool LoadRawDepth(const std::string& path, std::size_t count, std::vector<float>* out)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        return false;
    }
    out->resize(count);
    stream.read(reinterpret_cast<char*>(out->data()),
                static_cast<std::streamsize>(count * sizeof(float)));
    return stream.gcount() == static_cast<std::streamsize>(count * sizeof(float));
}

}  // namespace

/**
 * @brief End-to-end check: manifest -> preprocessor -> ONNX -> frontier pixels.
 *
 * The fixture is a raw ``float32`` 640x480 depth frame supplied through
 * ``FRONTIERNET_TEST_DEPTH``; without it the test reports "skipped" so a clean
 * checkout still builds and passes. ``FRONTIERNET_TEST_MANIFEST`` overrides the
 * manifest path (default ``conf/models/frontiernet_defm.pb.txt``).
 */
int main()
{
    const char* depth_path = std::getenv("FRONTIERNET_TEST_DEPTH");
    if (depth_path == nullptr)
    {
        std::cout << "skipped: FRONTIERNET_TEST_DEPTH not set\n";
        return 0;
    }
    const char* manifest_env = std::getenv("FRONTIERNET_TEST_MANIFEST");
    const std::string manifest_path =
        manifest_env != nullptr ? manifest_env : "conf/models/frontiernet_defm.pb.txt";

    navigation::config::ModelManifest manifest;
    if (!navigation::common::LoadTextProto(manifest_path, &manifest).Ok())
    {
        std::cerr << "cannot load manifest " << manifest_path << "\n";
        return 1;
    }
    std::string error;
    auto model = navigation::perception::CreateFrontierNetModel(manifest, &error);
    if (model == nullptr || !model->Ready())
    {
        std::cerr << "model load failed: " << error << "\n";
        return 1;
    }

    constexpr std::uint32_t kWidth = 640;
    constexpr std::uint32_t kHeight = 480;
    std::vector<float> depth;
    if (!LoadRawDepth(depth_path, kWidth * kHeight, &depth))
    {
        std::cerr << "cannot read depth fixture " << depth_path << "\n";
        return 1;
    }

    automsgs::msgs::sensor_msgs::Image image;
    image.set_width(kWidth);
    image.set_height(kHeight);
    image.set_encoding("32FC1");
    image.set_step(kWidth * sizeof(float));
    image.set_data(reinterpret_cast<const char*>(depth.data()),
                   static_cast<int>(depth.size() * sizeof(float)));

    navigation::perception::DepthPreprocessor preprocessor(
        navigation::perception::DepthPreprocessorOptions{});
    std::vector<float> input;
    navigation::perception::DepthStats stats;
    if (!preprocessor.Process(image, &input, &stats))
    {
        std::cerr << "preprocess failed\n";
        return 1;
    }

    navigation::perception::FrontierNetOutput output;
    if (!model->Run(input, &output))
    {
        std::cerr << "model run failed\n";
        return 1;
    }
    if (output.size != model->InputSize() || output.classes != model->Classes())
    {
        std::cerr << "output shape mismatch\n";
        return 1;
    }

    double df_max = -1e9;
    std::size_t frontier_pixels = 0;
    for (const float value : output.df)
    {
        df_max = std::max(df_max, static_cast<double>(value));
        const double metres = std::exp(-static_cast<double>(value)) * 10.0;
        if (metres < 0.5)
        {
            ++frontier_pixels;
        }
    }
    std::cout << "valid_ratio=" << stats.valid_ratio << " df_max=" << df_max
              << " frontier_pixels=" << frontier_pixels << "\n";
    // The trained DeFM heads must produce a frontier region on a real frame;
    // an untrained adapter saturates below log(20) and yields zero pixels.
    if (df_max <= std::log(20.0) || frontier_pixels == 0)
    {
        std::cerr << "no frontier detected: df_max=" << df_max << "\n";
        return 1;
    }
    return 0;
}
