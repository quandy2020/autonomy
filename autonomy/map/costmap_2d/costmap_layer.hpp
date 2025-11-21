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

#include "autonomy/map/costmap_2d/layer.hpp"
#include "autonomy/map/costmap_2d/layered_costmap.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class CostmapLayer
 * @brief A costmap layer base class for costmap plugin layers.
 * Rather than just a layer, this object also contains an internal
 * costmap object to populate and maintain state.
 */
class CostmapLayer : public Layer, public Costmap2D
{
public:
    /**
     * @brief CostmapLayer constructor
     */
    CostmapLayer()
        : has_extra_bounds_(false),
        extra_min_x_(1e6), extra_max_x_(-1e6),
        extra_min_y_(1e6), extra_max_y_(-1e6) {}

    /**
     * @brief If layer is discrete
     */
    bool isDiscretized()
    {
        return true;
    }

    /**
     * @brief Match the size of the master costmap
     */
    virtual void matchSize();

    /**
     * @brief Clear an are in the costmap with the given dimension
     * if invert, then clear everything except these dimensions
     */
    virtual void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert);

    /**
     * If an external source changes values in the costmap,
     * it should call this method with the area that it changed
     * to ensure that the costmap includes this region as well.
     * @param mx0 Minimum x value of the bounding box
     * @param my0 Minimum y value of the bounding box
     * @param mx1 Maximum x value of the bounding box
     * @param my1 Maximum y value of the bounding box
     */
    void addExtraBounds(double mx0, double my0, double mx1, double my1);

protected:
    /*
     * Updates the master_grid within the specified
     * bounding box using this layer's values.
     *
     * TrueOverwrite means every value from this layer
     * is written into the master grid.
     */
    void updateWithTrueOverwrite(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    /** 
     * Updates the master_grid within the specified
     * bounding box using this layer's values.
     *
     * Overwrite means every valid value from this layer
     * is written into the master grid (does not copy NO_INFORMATION)
     */
    void updateWithOverwrite(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    /*
    * Updates the master_grid within the specified
    * bounding box using this layer's values.
    *
    * Sets the new value to the maximum of the master_grid's value
    * and this layer's value. If the master value is NO_INFORMATION,
    * it is overwritten. If the layer's value is NO_INFORMATION,
    * the master value does not change.
    */
    void updateWithMax(Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);

    /*
    * Updates the master_grid within the specified
    * bounding box using this layer's values.
    *
    * Sets the new value to the maximum of the master_grid's value
    * and this layer's value. If the master value is NO_INFORMATION,
    * it is NOT overwritten. If the layer's value is NO_INFORMATION,
    * the master value does not change.
    */
    void updateWithMaxWithoutUnknownOverwrite(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    /*
    * Updates the master_grid within the specified
    * bounding box using this layer's values.
    *
    * Sets the new value to the sum of the master grid's value
    * and this layer's value. If the master value is NO_INFORMATION,
    * it is overwritten with the layer's value. If the layer's value
    * is NO_INFORMATION, then the master value does not change.
    *
    * If the sum value is larger than INSCRIBED_INFLATED_OBSTACLE,
    * the master value is set to (INSCRIBED_INFLATED_OBSTACLE - 1).
    */
    void updateWithAddition(Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);

    /**
     * Updates the bounding box specified in the parameters to include
     * the location (x,y)
     *
     * @param x x-coordinate to include
     * @param y y-coordinate to include
     * @param min_x bounding box
     * @param min_y bounding box
     * @param max_x bounding box
     * @param max_y bounding box
     */
    void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

    /*
    * Updates the bounding box specified in the parameters
    * to include the bounding box from the addExtraBounds
    * call. If addExtraBounds was not called, the method will do nothing.
    *
    * Should be called at the beginning of the updateBounds method
    *
    * @param min_x bounding box (input and output)
    * @param min_y bounding box (input and output)
    * @param max_x bounding box (input and output)
    * @param max_y bounding box (input and output)
    */
    void useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y);
    bool has_extra_bounds_;

    // /**
    //  * @brief Converts an integer to a CombinationMethod enum and logs on failure
    //  * @param value The integer to convert
    //  * @param function_name The name of the function calling this conversion (for logging)
    //  */
    // CombinationMethod combination_method_from_int(const int value);

private:
    double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy