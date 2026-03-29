/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"
#include "urdf_model/model.h"

namespace autonomy {
namespace transform {

// PIMPL Forward Declaration
class ModelImplementation;

/// \brief Populates itself based on a robot descripton
///
/// This class uses `urdf_parser_plugin` to parse the given robot description.
/// The chosen plugin is the one that reports the most confident score.
/// There is no way to override this choice except by uninstalling undesirable
/// parser plugins.
class Model : public ::urdf::ModelInterface
{
public:
    Model();

    ~Model();

    /// \brief Load Model given a filename
    bool initFile(const std::string& filename);

    /// \brief Load Model from a XML-string
    bool initString(const std::string& xmlstring);

private:
    std::unique_ptr<ModelImplementation> impl_;
};

}  // namespace transform
}  // namespace autonomy
