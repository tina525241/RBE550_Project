#pragma once

#include <math.h>
#include <random>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>

#include "math_funcs.hpp"

#define TWO_M_PI 2 * M_PI
#define M_PI_10 M_PI / 10.

/**
 *  @brief Calculates Euclidean distance between two 2D points.
 *
 *  @details
 *   Calculates Euclidean distance between two 2D points.
 *
 *  @param distance L2 Norm.
 *  @param point1 First point.
 *  @param point2 Second point.
 *  @return L2 Norm.
 *
 */
double getDistance(const geometry_msgs::Point point1, const geometry_msgs::Point point2)
{
  double distance = sqrt(pow(point2.y - point1.y, 2) + pow(point2.x - point1.x, 2));
  return distance;
}

/**
 *  @brief Checks whether a point is free on global 2D costmap.
 *
 *  @details
 *   Checks point in global 2D costmap to confirm that the robot
 *   can be placed in any orientation and will only occupy free space.
 *
 *  @param point Two points representing a line.
 *  @param costmap_ros  Pointer to ROS wrapper for global 2D costmap.
 *  @param robot_radius_max Circumscribed and padded robot radius.
 *  @return                 Whether point is in free space.
 *
 */
bool inFreeSpace(const geometry_msgs::Point point, const costmap_2d::Costmap2DROS* costmap_ros,
                 const double robot_radius_max)
{
  bool result{ 1 };
  double theta{ 0 };
  double robot_radius_ii{ robot_radius_max };
  double robot_radius_step(0.05);  // Assume 5cm step.
  costmap_2d::Costmap2D* costmap_;
  geometry_msgs::Point point_to_check;
  unsigned int mx, my;
  std::vector<costmap_2d::MapLocation> map_polygon, polygon_cells;

  costmap_ = costmap_ros->getCostmap();

  // Build inflated/circumscribed robot footprint
  while (theta <= TWO_M_PI)
  {
    costmap_2d::MapLocation map_loc;

    // Try to convert footprint to map coordinates
    if (!costmap_->worldToMap(point.x + robot_radius_max * cos(theta), point.y + robot_radius_max * sin(theta),
                              map_loc.x, map_loc.y))
    {
      // ROS_INFO("Footprint point is outside of map bounds.");
      return false;
    }

    map_polygon.push_back(map_loc);

    theta += M_PI_10;
  }

  // Get the all map cells within inflated/circumscribed robot footprint
  costmap_->convexFillCells(map_polygon, polygon_cells);

  // For each cell in polygon_cells, check the cost against the threshold.
  for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  {
    if (costmap_->getCost(polygon_cells[i].x, polygon_cells[i].y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      result = 0;
      break;
    }
  }

  return result;
}

/**
 *  @brief Checks whether an edge is free on global 2D costmap.
 *
 *  @details
 *   Discretizes edge, then checks several points along edge on global 2D costmap
 *   to confirm that the robot can be placed in any orientation and
 *   will only occupy free space.
 *
 *  @param edge Two points representing a line.
 *  @param costmap_ros  Pointer to ROS wrapper for global 2D costmap.
 *  @param robot_radius Circumscribed and padded robot radius.
 *  @return             Whether edge is in free space.
 *
 */
bool edgeInFreeSpace(const std::vector<geometry_msgs::Point> edge, const costmap_2d::Costmap2DROS* costmap_ros,
                     const double robot_radius)
{
  bool result{ 1 };

  // Discretize edge into an array of points
  double dist = getDistance(edge[0], edge[1]);
  // Get num of points. radius acts as resolution.
  double num_points = dist / robot_radius;
  geometry_msgs::Point edge_pt_ii{};
  for (double ii = 0.; ii <= num_points; ii++)
  {
    edge_pt_ii.x = edge[0].x + ii * (edge[1].x - edge[0].x) / num_points;
    edge_pt_ii.y = edge[0].y + ii * (edge[1].y - edge[0].y) / num_points;

    if (!inFreeSpace(edge_pt_ii, costmap_ros, robot_radius))
    {
      result = 0;
      break;
    }
  }

  return result;
}

/**
 *  @brief Picks a random interger on a uniform distribution.
 *
 *  @details
 *   Picks a random interger to be compared to the bias. If the
 *   random number chosen is less than or equal to the bias,
 *   the goal state is chosen as our "random" state.

 *  @see   https://stackoverflow.com/questions/5008804/generating-random-integer-from-a-range
 *
 *  @param bias The percentage chance of picking the goal state.
 *  @return             If the chosen point should be bias (the Goal State).
 *
 */
bool isBias(const int bias)
{

  std::random_device rd;     // only used once to initialise (seed) engine
  std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
  std::uniform_int_distribution<int> uni(1,100); // guaranteed unbiased

  int random_integer = uni(rng);

  //ROS_INFO("The random bias is %i.",random_integer); 

  if (random_integer <= bias)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 *  @brief Picks a random state on global 2D costmap.
 *
 *  @details
 *   Picks a random point on global 2D costmap at which the robot can be placed
 *   in any orientation and will only occupy free space.
 *
 *  @param costmap_ros  Pointer to ROS wrapper for global 2D costmap.
 *  @param robot_radius Circumscribed and padded robot radius.
 *  @param bias The percentage chance of picking the goal state.
 *  @param goalState the goal state of the search.
 *  @return             Random point on global costmap where robot is in free space.
 *
 */
geometry_msgs::Point getRandomState(costmap_2d::Costmap2DROS* costmap_ros, const double robot_radius, const int bias, const geometry_msgs::Point goalState)
{
  geometry_msgs::Point randomState{};
  randomState.z = 0.;  // Assume z=0 for now.
  costmap_2d::Costmap2D* costmap_;
  costmap_ = costmap_ros->getCostmap();
  //ROS_INFO("got the map!");

  // Keep picking points until you find one in free space
  bool pointIsFree{ 0 };

  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();


  while (!pointIsFree)
  {
    //ROS_INFO("Check the the bias!");
    // Bias Check. Should we pick the goal position?
    if (isBias(bias))
    {
      randomState.x = goalState.x;
      randomState.y = goalState.y;
    }
    else
    {
      randomState.x = randomDouble(origin_x, origin_x + costmap_->getSizeInMetersX());
      randomState.y = randomDouble(origin_y, origin_y + costmap_->getSizeInMetersY());
    }
    //ROS_INFO("Check the point!"); 
    pointIsFree = inFreeSpace(randomState, costmap_ros, robot_radius);
  }
  //ROS_INFO("The random state is x: %f and y: %f.", randomState.x, randomState.y);
  return randomState;
}