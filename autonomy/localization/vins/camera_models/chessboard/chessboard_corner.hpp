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

#ifndef CHESSBOARDCORNER_H
#define CHESSBOARDCORNER_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

namespace camodocal
{

class ChessboardCorner;
typedef boost::shared_ptr<ChessboardCorner> ChessboardCornerPtr;

class ChessboardCorner
{
public:
    ChessboardCorner() : row(0), column(0), needsNeighbor(true), count(0) {}

    float meanDist(int &n) const
    {
        float sum = 0;
        n = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (neighbors[i].get())
            {
                float dx = neighbors[i]->pt.x - pt.x;
                float dy = neighbors[i]->pt.y - pt.y;
                sum += sqrt(dx*dx + dy*dy);
                n++;
            }
        }
        return sum / std::max(n, 1);
    }

    cv::Point2f pt;                     // X and y coordinates
    int row;                            // Row and column of the corner
    int column;                         // in the found pattern
    bool needsNeighbor;                 // Does the corner require a neighbor?
    int count;                          // number of corner neighbors
    ChessboardCornerPtr neighbors[4];   // pointer to all corner neighbors
};

}

#endif
