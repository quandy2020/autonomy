/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "autonomy/localization/atlas/io/g2p5/g2p5.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <memory>
#include <vector>

namespace autonomy::localization::atlas {
namespace map {

/**
 * Thin facade over G2P5 for LocalizationServer / one-shot projection.
 * Prefer owning a G2P5 instance; Projector keeps Project() for callers that
 * only need a flat occupancy buffer.
 */
class G2P5Projector {
public:
    struct Options {
        double resolution = 0.05;
        double min_z = -0.5;
        double max_z = 1.5;
        int width = 400;
        int height = 400;
        //! When wrapping a live G2P5, these map into G2P5::Options.
        G2P5::Options g2p5;
    };

    G2P5Projector();
    explicit G2P5Projector(Options options);
    explicit G2P5Projector(std::shared_ptr<G2P5> g2p5);

    //! Project world points into a row-major occupancy buffer (0 free, 100 occ).
    std::vector<int8_t> Project(const std::vector<Vec3_t>& points_world) const;

    //! One-shot project via a temporary G2P5Map (uses Options.g2p5).
    static std::vector<int8_t> ProjectOnce(
        const std::vector<Vec3_t>& points_world, const Options& options);

    G2P5* g2p5() { return g2p5_.get(); }
    const G2P5* g2p5() const { return g2p5_.get(); }
    std::shared_ptr<G2P5> shared_g2p5() const { return g2p5_; }

    const Options& options() const { return options_; }

private:
    Options options_;
    std::shared_ptr<G2P5> g2p5_;
};

}  // namespace map
}  // namespace autonomy::localization::atlas
