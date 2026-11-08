/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/perception/attention_pooler.hpp"

int main()
{
    const std::vector<float> tokens = {1.0F, 2.0F, 3.0F, 4.0F};
    std::vector<float> pooled;
    double norm = 0.0;
    navigation::perception::AttentionPooler pooler;
    if (!pooler.Pool(tokens, 2, 2, &pooled, &norm) || pooled.size() != 2)
    {
        std::cerr << "pool failed\n";
        return 1;
    }
    if (std::abs(pooled[0] - 2.0F) > 1e-6 || std::abs(pooled[1] - 3.0F) > 1e-6)
    {
        std::cerr << "pool values failed\n";
        return 1;
    }
    if (std::abs(norm - std::sqrt(13.0)) > 1e-6)
    {
        std::cerr << "norm failed\n";
        return 1;
    }
    return 0;
}
