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

#ifndef AVIZ_COMMON__INTERACTION__SELECTION_MANAGER_IFACE_HPP_
#define AVIZ_COMMON__INTERACTION__SELECTION_MANAGER_IFACE_HPP_

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autonomy/tools/aviz/common/interaction/forwards.hpp"
#include "autonomy/tools/aviz/common/interaction/selection_handler.hpp"

class SceneViewer;

namespace aviz {
namespace common {
namespace properties {
class PropertyTreeModel;
}

class DisplayContext;

namespace interaction {

class SelectionManagerIface : public QObject
{
    Q_OBJECT

public:
    enum SelectType { Add, Remove, Replace };

    virtual void initialize() = 0;

    /// Control the highlight box being displayed while selecting.
    virtual void highlight(SceneViewer* viewer, int x1, int y1, int x2,
                           int y2) = 0;

    virtual void removeHighlight() = 0;

    /// Select all objects in bounding box.
    virtual void select(SceneViewer* viewer, int x1, int y1, int x2, int y2,
                        SelectType type) = 0;

    /// Get all objects in a bounding box.
    /**
     * \return handles of all objects in the given bounding box
     */
    virtual void pick(SceneViewer* viewer, int x1, int y1, int x2, int y2,
                      M_Picked& results) = 0;

    virtual void update() = 0;

    virtual const M_Picked& getSelection() const = 0;

    /// Tell the view controller to look at the selection.
    virtual void focusOnSelection() = 0;

    /// Change the size of the off-screen selection buffer texture.
    virtual void setTextureSize(unsigned size) = 0;

    virtual aviz::common::properties::PropertyTreeModel* getPropertyModel() = 0;
};

}  // namespace interaction
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__INTERACTION__SELECTION_MANAGER_IFACE_HPP_
