/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/tools/aviz/common/transformation/transformation_manager.hpp"

namespace aviz {
namespace common {
namespace transformation {

TransformationManager::TransformationManager() {}

void TransformationManager::load(const Config& config) {
    QString class_id;
    if (config.mapGetString("Transformer", &class_id)) {
        PluginInfo info;
        info.class_id = class_id;
        // For now, use class_id as name and description
        info.name = class_id;
        info.description = class_id;
        setTransformer(info);
    }
}

void TransformationManager::save(Config config) const {
    if (current_transformer_) {
        config.mapSetValue("Transformer", current_transformer_info_.class_id);
    }
}

std::vector<PluginInfo> TransformationManager::getAvailableTransformers()
    const {
    // For now, return empty list
    // Can be extended later to discover available transformers
    return std::vector<PluginInfo>();
}

std::shared_ptr<FrameTransformer> TransformationManager::getCurrentTransformer()
    const {
    return current_transformer_;
}

PluginInfo TransformationManager::getCurrentTransformerInfo() const {
    return current_transformer_info_;
}

void TransformationManager::setTransformer(const PluginInfo& plugin_info) {
    current_transformer_info_ = plugin_info;
    // For now, just store the info
    // Actual transformer creation can be implemented later
    Q_EMIT configChanged();
}

void TransformationManager::setTransformer(
    std::shared_ptr<FrameTransformer> transformer) {
    if (current_transformer_ != transformer) {
        current_transformer_ = transformer;
        if (transformer) {
            current_transformer_info_.class_id = transformer->getClassId();
            current_transformer_info_.name = transformer->getClassId();
            current_transformer_info_.description =
                transformer->getDescription();
        }
        Q_EMIT transformerChanged(transformer);
        Q_EMIT configChanged();
    }
}

}  // namespace transformation
}  // namespace common
}  // namespace aviz
