// Lattice Planner - Distance Heuristic
// Ported from nav2_smac_planner/distance_heuristic.hpp with no ROS dependencies.
// NodeHybrid specialization removed; only NodeLattice is supported.

#ifndef LATTICE_PLANNER__HEURISTIC__DISTANCE_HEURISTIC_HPP_
#define LATTICE_PLANNER__HEURISTIC__DISTANCE_HEURISTIC_HPP_

#include "lattice_planner/core/constants.hpp"
#include "lattice_planner/core/config.hpp"
#include "lattice_planner/core/types.hpp"

namespace lattice_planner {

class NodeLattice;
struct LatticeMotionTable;

/**
 * @class DistanceHeuristic
 * @brief SE2 distance heuristic using Dubins/Reeds-Shepp lookup table.
 *
 * Precomputes a windowed lookup table of Dubins/Reeds-Shepp distances around
 * the goal and uses symmetry to halve storage. Falls back to live OMPL
 * state space distance when outside the lookup window.
 */
template<typename NodeT>
class DistanceHeuristic {
public:
  DistanceHeuristic() {}

  /**
   * @brief Precompute the SE2 distance heuristic lookup table.
   * @param lookup_table_dim Size (in costmap cells) of each lookup dimension.
   * @param motion_model Motion model (unused for NodeLattice; determined by lattice metadata).
   * @param dim_3_size Number of heading quantization bins.
   * @param search_info Search info containing minimum turning radius and lattice filepath.
   * @param motion_table Motion table to populate with metadata and state space.
   */
  template<typename MotionTableT>
  void precomputeDistanceHeuristic(
    const float & lookup_table_dim,
    const MotionModel & motion_model,
    const unsigned int & dim_3_size,
    const SearchInfo & search_info,
    MotionTableT & motion_table);

  /**
   * @brief Get the distance heuristic value at node_coords relative to goal_coords.
   * @param node_coords Node coordinates.
   * @param goal_coords Goal coordinates.
   * @param obstacle_heuristic Obstacle heuristic value (used to decide fallback).
   * @param motion_table Motion table for angle/bin conversions.
   * @return Heuristic value.
   */
  template<typename MotionTableT>
  float getDistanceHeuristic(
    const Coordinates & node_coords,
    const Coordinates & goal_coords,
    const float & obstacle_heuristic,
    MotionTableT & motion_table);

protected:
  // Dubin / Reeds-Shepp lookup and size for dereferencing
  LookupTable dist_heuristic_lookup_table_;
  float size_lookup_{0.0f};
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__HEURISTIC__DISTANCE_HEURISTIC_HPP_
