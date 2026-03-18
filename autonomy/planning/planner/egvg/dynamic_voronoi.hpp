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

#include <Eigen/Core>
#include <climits>
#include <queue>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/planning/planner/egvg/bucketedqueue.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace egvg {

// Use autonomy::commsgs::geometry_msgs::Point as the grid point type.
using Point = autonomy::commsgs::geometry_msgs::Point;

//! A DynamicVoronoi object computes and updates a distance map and Voronoi
//! diagram.
class DynamicVoronoi {
 public:
  DynamicVoronoi();
  ~DynamicVoronoi();

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap = true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

  //! add an obstacle at the specified cell coordinate
  /**
   * @brief add an obstacle at the specified cell coordinate
   * @param x x coordinate
   * @param y y coordinate
   */
  void occupyCell(int x, int y);

  /**
   * @brief remove an obstacle at the specified cell coordinate
   * @param x x coordinate
   * @param y y coordinate
   */
  void clearCell(int x, int y);
  //! remove old dynamic obstacles and add the new ones
  void exchangeObstacles(std::vector<Point>& newObstacles);

  //! update distance map and Voronoi diagram to reflect the changes
  void update(bool updateRealDist = true);
  //! prune the Voronoi diagram
  void prune();
  //! prune the Voronoi diagram by globally revisiting all Voronoi nodes.
  //! Takes more time but gives a more sparsely pruned Voronoi graph. You need
  //! to call this after every call to udpate()
  void updateAlternativePrunedDiagram();
  //! retrieve the alternatively pruned diagram. see
  //! updateAlternativePrunedDiagram()
  int** alternativePrunedDiagram() const { return alternativeDiagram; };
  //! retrieve the number of neighbors that are Voronoi nodes (4-connected)
  int getNumVoronoiNeighborsAlternative(int x, int y) const;
  //! returns whether the specified cell is part of the alternatively pruned
  //! diagram. See updateAlternativePrunedDiagram.
  bool isVoronoiAlternative(int x, int y) const;

  //! returns the obstacle distance at the specified location
  float getDistance(int x, int y) const;
  float getDistanceNoBoundCheck(int x, int y) const;
  float getDistanceSq(int x, int y) const;
  int getObstacleX(int x, int y) const;
  int getObstacleY(int x, int y) const;
  Eigen::Vector2i getObstacle(int x, int y) const;
  std::vector<Point> getAllPruneQueuePoints();
  std::vector<Point> getAllSortedPruneQueuePoints();

  //! returns whether the specified cell is part of the (pruned) Voronoi graph
  bool isVoronoi(int x, int y) const;
  bool isVoronoiWithDisThr(int x, int y, float disThreSq) const;
  bool isVoronoiWithDisThr(int x, int y, float disThreSqLow, float disThreSqHight) const;
  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y) const;
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename = "result.ppm");

  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() const { return sizeX; }
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() const { return sizeY; }

 private:
  struct dataCell {
    float dist;
    char voronoi;
    char queueing;
    int obstX;
    int obstY;
    bool needsRaise;
    int sqdist;
  };

  typedef enum {
    // voronoiVertex and voronoiKeep are both Voronoi vertices (junctions),
    // voronoiVertex is further away from obstacles.
    voronoiVertex = -5,
    // Used to stop erosion at voronoiVertex when pruning from the end side.
    voronoiKeep = -4,
    freeQueued = -3,
    voronoiRetry = -2,
    voronoiPrune = -1,
    free = 0,
    occupied = 1
  } State;

  typedef enum { fwNotQueued = 1, fwQueued = 2, fwProcessed = 3, bwQueued = 4, bwProcessed = 1 } QueueingState;

  typedef enum { invalidObstData = SHRT_MAX / 2 } ObstDataState;

  typedef enum { pruned, keep, retry } markerMatchResult;

  // methods
  void setObstacle(int x, int y);
  void removeObstacle(int x, int y);
  inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
  void recheckVoro();
  void commitAndColorize(bool updateRealDist = true);
  inline void reviveVoroNeighbors(int& x, int& y);

  inline bool isOccupied(int& x, int& y, dataCell& c);
  inline markerMatchResult markerMatch(int x, int y);
  inline bool markerMatchAlternative(int x, int y);
  inline int getVoronoiPruneValence(int x, int y);

  // queues
  BucketPrioQueue<Point> open;
  std::queue<Point> pruneQueue;
  BucketPrioQueue<Point> sortedPruneQueue;
  BucketPrioQueue<Point> sortedPruneQueueBackup;

  std::vector<Point> removeList;
  std::vector<Point> addList;
  std::vector<Point> lastObstacles;

  // maps
  int sizeY;
  int sizeX;
  dataCell** data;
  bool** gridMap;
  bool allocatedGridMap;

  // parameters
  int padding;
  double doubleThreshold;

  double sqrt2;

  //  dataCell** getData(){ return data; }
  int** alternativeDiagram;
};

}  // namespace egvg
}  // namespace planner
}  // namespace planning
}  // namespace autonomy