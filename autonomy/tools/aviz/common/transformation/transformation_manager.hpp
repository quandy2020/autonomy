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

#ifndef AVIZ_COMMON__TRANSFORMATION__TRANSFORMATION_MANAGER_HPP_
#define AVIZ_COMMON__TRANSFORMATION__TRANSFORMATION_MANAGER_HPP_

#include <QObject>  // NOLINT
#include <QString>  // NOLINT
#include <memory>
#include <string>
#include <vector>

#include "autonomy/tools/aviz/common/config.hpp"
#include "autonomy/tools/aviz/common/transformation/frame_transformer.hpp"

namespace aviz {
namespace common {
namespace transformation {

struct PluginInfo {
    QString class_id;
    QString name;
    QString description;
};

/// Manages frame transformers
/**
 * TransformationManager manages the current frame transformer and provides
 * access to available transformers.
 */
class TransformationManager : public QObject
{
    Q_OBJECT

public:
    explicit TransformationManager();

    /// Load configuration from a Config object.
    void load(const Config& config);

    /// Save configuration to a Config object.
    void save(Config config) const;

    /// Get list of available transformers
    std::vector<PluginInfo> getAvailableTransformers() const;

    /// Get the current transformer
    std::shared_ptr<FrameTransformer> getCurrentTransformer() const;

    /// Get info about the current transformer
    PluginInfo getCurrentTransformerInfo() const;

    /// Set the transformer by plugin info
    void setTransformer(const PluginInfo& plugin_info);

    /// Set the transformer directly
    void setTransformer(std::shared_ptr<FrameTransformer> transformer);

Q_SIGNALS:
    /// Emitted when the current transformer changes.
    void configChanged();

    /// Emitted when the current transformer changes.
    void transformerChanged(
        std::shared_ptr<aviz::common::transformation::FrameTransformer>
            new_transformer);

private:
    std::shared_ptr<FrameTransformer> current_transformer_;
    PluginInfo current_transformer_info_;
};

}  // namespace transformation
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__TRANSFORMATION__TRANSFORMATION_MANAGER_HPP_
