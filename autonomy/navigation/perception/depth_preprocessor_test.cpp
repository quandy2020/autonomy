/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/perception/depth_preprocessor.hpp"

int main()
{
    automsgs::msgs::sensor_msgs::Image image;
    image.set_width(2);
    image.set_height(2);
    image.set_encoding("32FC1");
    image.set_step(2 * sizeof(float));
    const float values[4] = {1.0F, 1.0F, 1.0F, 1.0F};
    image.set_data(reinterpret_cast<const char*>(values), sizeof(values));

    navigation::perception::DepthPreprocessor preprocessor(
        navigation::perception::DepthPreprocessorOptions{});
    std::vector<float> chw;
    navigation::perception::DepthStats stats;
    if (!preprocessor.Process(image, &chw, &stats))
    {
        std::cerr << "preprocess failed\n";
        return 1;
    }
    if (chw.size() != 3U * 224U * 224U)
    {
        std::cerr << "tensor size failed\n";
        return 1;
    }
    if (std::abs(stats.valid_ratio - 1.0) > 1e-9)
    {
        std::cerr << "valid ratio failed\n";
        return 1;
    }
    const float expected =
        static_cast<float>(((1.0 - 0.1) / 7.9 - 0.5) / 0.5);
    if (std::abs(chw[0] - expected) > 1e-5)
    {
        std::cerr << "normalization failed: " << chw[0] << " vs " << expected << "\n";
        return 1;
    }

    const float invalid[4] = {1.0F, 0.0F, 1.0F, 1.0F};
    image.set_data(reinterpret_cast<const char*>(invalid), sizeof(invalid));
    if (!preprocessor.Process(image, &chw, &stats) ||
        std::abs(stats.valid_ratio - 0.75) > 1e-9)
    {
        std::cerr << "invalid ratio failed\n";
        return 1;
    }

    image.set_encoding("rgb8");
    if (preprocessor.Process(image, &chw, &stats))
    {
        std::cerr << "unsupported encoding accepted\n";
        return 1;
    }
    return 0;
}
