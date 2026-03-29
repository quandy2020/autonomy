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

#ifndef AVIZ_COMMON__INTERACTION__VIEW_PICKER_IFACE_HPP_
#define AVIZ_COMMON__INTERACTION__VIEW_PICKER_IFACE_HPP_

#include <QVector3D>  // NOLINT: cpplint is unable to handle the include order here
#include <vector>

class SceneViewer;

namespace aviz {
namespace common {
namespace interaction {

class ViewPickerIface
{
public:
    virtual ~ViewPickerIface() = default;

    virtual void initialize() = 0;

    /// Return true if the point at x, y in the viewport is showing an object,
    /// false otherwise.
    /**
     * If it is showing an object, result will be changed to contain the 3D
     * point corresponding to it.
     */
    virtual bool get3DPoint(SceneViewer* viewer, int x, int y,
                            QVector3D& result_point) = 0;

    /// Return true if the point at x,y in the viewport is showing an object,
    /// false otherwise.
    /**
     * Gets the 3D points in a box around a point in a view port.
     */
    virtual bool get3DPatch(SceneViewer* viewer, int x, int y, unsigned width,
                            unsigned height, bool skip_missing,
                            std::vector<QVector3D>& result_points) = 0;
};

}  // namespace interaction
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__INTERACTION__VIEW_PICKER_IFACE_HPP_
