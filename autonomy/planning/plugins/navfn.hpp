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

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <functional>

namespace autonomy {
namespace planning {
namespace plugins {

#define COST_UNKNOWN_ROS 255  // 255 is unknown cost
#define COST_OBS 254  // 254 for forbidden regions
#define COST_OBS_ROS 253  // ROS values of 253 are obstacles

// navfn cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
// Incoming costmap cost values are in the range 0 to 252.
// With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
// ensure the input values are spread evenly over the output range, 50
// to 253.  If COST_FACTOR is higher, cost values will have a plateau
// around obstacles and the planner will then treat (for example) the
// whole width of a narrow hallway as equally undesirable and thus
// will not plan paths down the center.

#define COST_NEUTRAL 50  // Set this to "open space" value
#define COST_FACTOR 0.8  // Used for translating costs in NavFn::setCostmap()

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
#ifndef COSTTYPE
#define COSTTYPE unsigned char  // Whatever is used...
#endif

// potential defs
#define POT_HIGH 1.0e10  // unassigned cell potential

// priority buffers
#define PRIORITYBUFSIZE 10000

/**
 * @brief Navigation function call.
 * 
 * @param costmap Cost map array, of type COSTTYPE; origin is upper left
    NOTE: will be modified to have a border of obstacle costs
 * @param nx Width of map in cells
 * @param ny Height of map in cells
 * @param goal X,Y position of goal cell
 * @param start X,Y position of start cell
 * @return  Returns length of plan if found, and fills an array with x,y interpolated 
 *          positions at about 1/2 cell resolution; else returns 0.
 */
int create_nav_plan_astar(
  const COSTTYPE * costmap, int nx, int ny,
  int * goal, int * start,
  float * plan, int nplan);

/**
 * @class NavFn
 * @brief Navigation function class. Holds buffers for costmap, navfn map. Maps are pixel-based.
 *  Origin is upper left, x is right, y is down.
 */
class NavFn
{
public:
  /**
   * @brief  Constructs the planner
   * @param nx The x size of the map
   * @param ny The y size of the map
   */
  NavFn(int nx, int ny);

  ~NavFn();

  /**
   * @brief  Sets or resets the size of the map
   * @param nx The x size of the map
   * @param ny The y size of the map
   */
  void setNavArr(int nx, int ny);
  int nx, ny, ns;  /**< size of grid, in pixels */

  /**
   * @brief  Set up the cost array for the planner, usually from ROS
   * @param cmap The costmap
   * @param isROS Whether or not the costmap is coming in in ROS format
   * @param allow_unknown Whether or not the planner should be allowed to plan through
   *   unknown space
   */
  void setCostmap(const COSTTYPE * cmap, bool isROS = true, bool allow_unknown = true);

  /**
   * @brief  Calculates a plan using the A* heuristic, returns true if one is found
   * @param cancelChecker Function to check if the task has been canceled
   * @return True if a plan is found, false otherwise
   */
  bool calcNavFnAstar(std::function<bool()> cancelChecker);

  /**
   * @brief Calculates the full navigation function using Dijkstra
   * @param cancelChecker Function to check if the task has been canceled
   */
  bool calcNavFnDijkstra(std::function<bool()> cancelChecker, bool atStart = false);

  /**
   * @brief  Accessor for the x-coordinates of a path
   * @return The x-coordinates of a path
   */
  float * getPathX();

  /**
   * @brief  Accessor for the y-coordinates of a path
   * @return The y-coordinates of a path
   */
  float * getPathY();

  /**
   * @brief  Accessor for the length of a path
   * @return The length of a path, 0 if not found
   */
  int getPathLen();

  /**
   * @brief  Gets the cost of the path found the last time a navigation function was computed
   * @return The cost of the last path found
   */
  float getLastPathCost();

  /** cell arrays */
  COSTTYPE * costarr;  /**< cost array in 2D configuration space */
  float * potarr;  /**< potential array, navigation function potential */
  bool * pending;  /**< pending cells during propagation */
  int nobs;  /**< number of obstacle cells */

  /** block priority buffers */
  int * pb1, * pb2, * pb3;  /**< storage buffers for priority blocks */
  int * curP, * nextP, * overP;  /**< priority buffer block ptrs */
  int curPe, nextPe, overPe;  /**< end points of arrays */

  /** block priority thresholds */
  float curT;  /**< current threshold */
  float priInc;  /**< priority threshold increment */

  /**< number of cycles between checks for cancellation */
  static constexpr int terminal_checking_interval = 5000;

  /** goal and start positions */
  /**
   * @brief  Sets the goal position for the planner.
   * Note: the navigation cost field computed gives the cost to get to a given point
   * from the goal, not from the start.
   * @param goal the goal position
   */
  void setGoal(int * goal);

  /**
   * @brief  Sets the start position for the planner.
   * Note: the navigation cost field computed gives the cost to get to a given point
   * from the goal, not from the start.
   * @param start the start position
   */
  void setStart(int * start);

  int goal[2];
  int start[2];
  /**
   * @brief  Initialize cell k with cost v for propagation
   * @param k the cell to initialize
   * @param v the cost to give to the cell
   */
  void initCost(int k, float v);

  /** propagation */

  /**
   * @brief  Updates the cell at index n
   * @param n The index to update
   */
  void updateCell(int n);

  /**
   * @brief  Updates the cell at index n using the A* heuristic
   * @param n The index to update
   */
  void updateCellAstar(int n);

  /**
   * @brief  Set up navigation potential arrays for new propagation
   * @param keepit whether or not use COST_NEUTRAL
   */
  void setupNavFn(bool keepit = false);

  /**
   * @brief  Run propagation for <cycles> iterations, or until start is reached using
   * breadth-first Dijkstra method
   * @param cycles The maximum number of iterations to run for
   * @param cancelChecker Function to check if the task has been canceled
   * @param atStart Whether or not to stop when the start point is reached
   * @return true if the start point is reached
   */
  bool propNavFnDijkstra(int cycles, std::function<bool()> cancelChecker, bool atStart = false);

  /**
   * @brief  Run propagation for <cycles> iterations, or until start is reached using
   * the best-first A* method with Euclidean distance heuristic
   * @param cycles The maximum number of iterations to run for
   * @param cancelChecker Function to check if the task has been canceled
   * @return true if the start point is reached
   */
  bool propNavFnAstar(int cycles, std::function<bool()> cancelChecker);

  /** gradient and paths */
  float * gradx, * grady;  /**< gradient arrays, size of potential array */
  float * pathx, * pathy;  /**< path points, as subpixel cell coordinates */
  int npath;  /**< number of path points */
  int npathbuf;  /**< size of pathx, pathy buffers */

  float last_path_cost_;  /**< Holds the cost of the path found the last time A* was called */

  /**
   * @brief  Calculates the path for at mose <n> cycles
   * @param n The maximum number of cycles to run for
   * @return The lenght of the path found, 0 if none
   */
  int calcPath(int n, int * st = NULL);

  /**
   * @brief  Calculate gradient at a cell
   * @param n Cell number <n>
   * @return float norm
   */
  float gradCell(int n);  /**< calculates gradient at cell <n>, returns norm */

  float pathStep;  /**< step size for following gradient */

  /** display callback */
  /**< <n> is the number of cycles between updates  */
  // void display(void fn(NavFn * nav), int n = 100);
  // int displayInt;  /**< save second argument of display() above */
  // void (* displayFn)(NavFn * nav);  /**< display function itself */

  /** save costmap */
  /**< write out costmap and start/goal states as fname.pgm and fname.txt */
  // void savemap(const char * fname);
};

}  // namespace plugins
}  // namespace planning
}  // namespace autonomy