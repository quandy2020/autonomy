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

#include "autonomy/prediction/proto/prediction_options.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/prediction/common/prediction_interface.hpp"

namespace autonomy {
namespace prediction {

proto::PredictionOptions CreatePredictionOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);
    
class PredictionServer : common::PredictionInterface
{
public:
    /**
     * Define PredictionServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(PredictionServer)

    /**
     * @brief A constructor for autonomy::prediction::PredictionServer
     * @param options Additional options to control creation of the node.
     */
    explicit PredictionServer(const proto::PredictionOptions& options);

    /**
     * @brief A Destructor for autonomy::prediction::PredictionServer
     */
    ~PredictionServer();

protected:
    // Options prediction
    proto::PredictionOptions options_;
};


}  // namespace prediction
}  // namespace autonomy