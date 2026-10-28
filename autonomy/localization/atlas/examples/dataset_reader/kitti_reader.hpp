/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0.
 */
#pragma once

#include <string>
#include <vector>

namespace autonomy::localization::atlas::examples {

struct KittiImageSample {
    std::size_t frame_index = 0;
    double timestamp = 0.0;
    std::string left_path;
    std::string right_path;
};

class KittiReader
{
public:
    static bool LoadStereo(const std::string& sequence_root,
                           std::vector<KittiImageSample>* samples);
};

}  // namespace autonomy::localization::atlas::examples
