/* Copyright 2026 The Openbot Authors (duyongquan)
 * Licensed under the Apache License, Version 2.0.
 */
/** @file tools/memory_inspector_main.cpp
 * @brief Navigation contract, implementation or regression fixture.
 */
#include <iostream>
#include "navigation/memory/context_provider.hpp"
int main(int argc, char** argv) {
    using namespace autonomy::navigation;
    if (argc != 2) {
        std::cerr << "usage: memory_inspector snapshot\n";
        return 2;
    }
    MemoryConfig config;
    config.max_entries = 100000;
    auto memory = CreatePoseIndexedMemory(config);
    if (!memory->Load(argv[1])) {
        std::cerr << "invalid or unreadable snapshot\n";
        return 1;
    }
    std::cout << "id,stamp,x,y,z,correction_version\n";
    for (const auto& entry : memory->View().Entries())
        std::cout << entry.id << ',' << entry.pose.stamp << ','
                  << entry.pose.pose.translation().transpose().format(
                         Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
                  << ',' << entry.correction_version << '\n';
    return 0;
}
