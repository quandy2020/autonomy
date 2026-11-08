/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/memory/voxel_grid.hpp"

int main()
{
    navigation::memory::VoxelGrid grid(0.1);
    grid.Insert(0.0, 0.0, 0.0, 1);
    grid.Insert(0.05, 0.0, 0.0, 2);
    grid.Insert(1.0, 0.0, 0.0, 3);
    if (grid.Size() != 3)
    {
        std::cerr << "size failed\n";
        return 1;
    }
    int found = 0;
    grid.QueryRadius(0.0, 0.0, 0.0, 0.2, [&found](double, double, double, std::uint32_t) {
        ++found;
    });
    if (found != 2)
    {
        std::cerr << "radius query failed: " << found << "\n";
        return 1;
    }
    grid.Clear();
    if (grid.Size() != 0)
    {
        std::cerr << "clear failed\n";
        return 1;
    }
    return 0;
}
