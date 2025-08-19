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

#ifndef CHESSBOARDQUAD_H
#define CHESSBOARDQUAD_H

#include <boost/shared_ptr.hpp>

#include "autonomy/localization/vins/camera_models/chessboard/chessboard_corner.hpp"

namespace camodocal
{

class ChessboardQuad;
typedef boost::shared_ptr<ChessboardQuad> ChessboardQuadPtr;

class ChessboardQuad
{
public:
    ChessboardQuad() : count(0), group_idx(-1), edge_len(FLT_MAX), labeled(false) {}

    int count;                         // Number of quad neighbors
    int group_idx;                     // Quad group ID
    float edge_len;                    // Smallest side length^2
    ChessboardCornerPtr corners[4];    // Coordinates of quad corners
    ChessboardQuadPtr neighbors[4];    // Pointers of quad neighbors
    bool labeled;                      // Has this corner been labeled?
};

}

#endif
