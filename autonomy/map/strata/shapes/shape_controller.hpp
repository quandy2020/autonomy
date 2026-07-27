/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace shapes {

template <typename FeatureT>
class FeatureCollection
{
public:
    using SharedPtr = std::shared_ptr<FeatureCollection>;
    static SharedPtr make_shared(render::SceneState::SharedPtr scene,
                                 std::vector<FeatureT> render::SceneState::*member,
                                 const std::string& layerId) {
        return std::make_shared<FeatureCollection>(std::move(scene), member, layerId);
    }

    explicit FeatureCollection(render::SceneState::SharedPtr scene,
                               std::vector<FeatureT> render::SceneState::*member,
                               const std::string& layerId)
        : scene_(std::move(scene)), member_(member), layerId_(layerId) {}

    FeatureT* Add(const FeatureT& feature) {
        auto& collection = (*scene_).*member_;
        collection.push_back(feature);
        scene_->Touch();
        return &collection.back();
    }

    bool Update(const std::string& id, const FeatureT& feature) {
        auto& collection = (*scene_).*member_;
        for (auto& item : collection) {
            if (item.id == id) {
                item = feature;
                item.id = id;
                scene_->Touch();
                return true;
            }
        }
        return false;
    }

    bool Remove(const std::string& id) {
        auto& collection = (*scene_).*member_;
        const auto it = std::remove_if(collection.begin(), collection.end(),
                                       [&](const FeatureT& f) { return f.id == id; });
        if (it == collection.end()) {
            return false;
        }
        collection.erase(it, collection.end());
        scene_->Touch();
        return true;
    }

    void Clear() {
        ((*scene_).*member_).clear();
        scene_->Touch();
    }

    std::vector<FeatureT> GetAll() const { return (*scene_).*member_; }

    std::optional<FeatureT> GetById(const std::string& id) const {
        for (const auto& item : (*scene_).*member_) {
            if (item.id == id) {
                return item;
            }
        }
        return std::nullopt;
    }

    bool Highlight(const std::string& id, bool highlighted) {
        auto& collection = (*scene_).*member_;
        for (auto& item : collection) {
            if (item.id == id) {
                item.highlighted = highlighted;
                scene_->Touch();
                return true;
            }
        }
        return false;
    }

    void ClearHighlights() {
        for (auto& item : (*scene_).*member_) {
            item.highlighted = false;
        }
        scene_->Touch();
    }

    void SetVisible(bool visible) {
        for (auto& item : (*scene_).*member_) {
            item.visible = visible;
        }
        scene_->Touch();
    }

    void SetOnClick(FeatureClickCallback callback) { on_click_ = std::move(callback); }

    bool HandleClick(const std::string& id) {
        if (!on_click_) {
            return false;
        }
        if (!GetById(id).has_value()) {
            return false;
        }
        on_click_(id);
        return true;
    }

    void RemoveFromScene() {
        Clear();
    }

private:
    render::SceneState::SharedPtr scene_;
    std::vector<FeatureT> render::SceneState::*member_;
    std::string layerId_;
    FeatureClickCallback on_click_;
};

using RectangleCollection = FeatureCollection<RectangleFeature>;
using PolygonCollection = FeatureCollection<PolygonFeature>;
using CircleCollection = FeatureCollection<CircleFeature>;
using PolylineCollection = FeatureCollection<PolylineFeature>;
using WideLineCollection = FeatureCollection<WideLineFeature>;

std::vector<LngLat> PathToWidePolygon(const std::vector<LngLat>& path, double widthM);

}  // namespace shapes
}  // namespace strata
}  // namespace map
}  // namespace autonomy
