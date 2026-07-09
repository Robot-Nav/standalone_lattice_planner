// Lattice Planner - Obstacle Heuristic
// Ported from nav2_smac_planner/obstacle_heuristic.hpp with no ROS dependencies.

#ifndef LATTICE_PLANNER__HEURISTIC__OBSTACLE_HEURISTIC_HPP_
#define LATTICE_PLANNER__HEURISTIC__OBSTACLE_HEURISTIC_HPP_

#include <utility>
#include <vector>

#include "lattice_planner/core/constants.hpp"
#include "lattice_planner/core/config.hpp"
#include "lattice_planner/core/types.hpp"
#include "lattice_planner/costmap/costmap_2d.hpp"

namespace lattice_planner {

typedef std::pair<float, uint64_t> ObstacleHeuristicElement;

struct ObstacleHeuristicComparator {
  bool operator()(const ObstacleHeuristicElement & a, const ObstacleHeuristicElement & b) const
  {
    return a.first > b.first;
  }
};

typedef std::vector<ObstacleHeuristicElement> ObstacleHeuristicQueue;

/**
 * @class ObstacleHeuristic
 * @brief 2D obstacle wavefront heuristic for State Lattice A* search.
 *
 * Computes a Dijkstra-style BFS from the goal over the 2D costmap to estimate
 * the obstacle-aware distance to goal. Supports 2x downsampling for speed.
 */
class ObstacleHeuristic {
public:
  ObstacleHeuristic() {}
  ~ObstacleHeuristic() {}

  /**
   * @brief Compute (or reset) the wavefront heuristic from the goal.
   * @param costmap Raw costmap pointer (replaces shared_ptr<Costmap2DROS>).
   * @param start_x Start X in costmap cells (used for distance heuristic priority).
   * @param start_y Start Y in costmap cells.
   * @param goal_x Goal X in costmap cells.
   * @param goal_y Goal Y in costmap cells.
   * @param downsample_obstacle_heuristic If true, downsample costmap 2x.
   */
  void resetObstacleHeuristic(
    Costmap2D * costmap,
    const unsigned int & start_x, const unsigned int & start_y,
    const unsigned int & goal_x, const unsigned int & goal_y,
    const bool downsample_obstacle_heuristic);

  /**
   * @brief Get the obstacle heuristic value at node_coords.
   * @param node_coords Coordinates of the node to query.
   * @param cost_penalty Cost penalty factor.
   * @param use_quadratic_cost_penalty If true, use quadratic cost penalty.
   * @param downsample_obstacle_heuristic If true, values are from downsampled map.
   * @return Heuristic value.
   */
  float getObstacleHeuristic(
    const Coordinates & node_coords,
    const float & cost_penalty,
    const bool use_quadratic_cost_penalty,
    const bool downsample_obstacle_heuristic);

  /**
   * @brief 2D Euclidean distance from idx to a target point.
   */
  inline float distanceHeuristic2D(
    const uint64_t idx, const unsigned int size_x,
    const unsigned int target_x, const unsigned int target_y)
  {
    int dx = static_cast<int>(idx % size_x) - static_cast<int>(target_x);
    int dy = static_cast<int>(idx / size_x) - static_cast<int>(target_y);
    return std::sqrt(dx * dx + dy * dy);
  }

protected:
  LookupTable obstacle_heuristic_lookup_table_;
  ObstacleHeuristicQueue obstacle_heuristic_queue_;
  Costmap2D * costmap_{nullptr};
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__HEURISTIC__OBSTACLE_HEURISTIC_HPP_
