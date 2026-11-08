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
    const double log_loss = std::log(2.0);
    const double c1 = log_loss / std::log(101.0);
    const double c2 = std::min(1.0, log_loss / std::log(10.0));
    const double c3 = 0.0;
    const double mean[3] = {0.248880, 0.495620, 0.492858};
    const double stddev[3] = {0.139357, 0.271314, 0.297177};
    const double expected[3] = {(c1 - mean[0]) / stddev[0],
                                (c2 - mean[1]) / stddev[1],
                                (c3 - mean[2]) / stddev[2]};
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        if (std::abs(chw[channel * 224U * 224U] - expected[channel]) > 1e-5)
        {
            std::cerr << "normalization failed: " << chw[channel * 224U * 224U]
                      << " vs " << expected[channel] << "\n";
            return 1;
        }
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
