// Lattice Planner - Distance Heuristic Implementation
// Ported from nav2_smac_planner/src/distance_heuristic.cpp with no ROS dependencies.
// NodeHybrid specialization removed; only NodeLattice is supported.

#include <cmath>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"

#include "lattice_planner/heuristic/distance_heuristic.hpp"
#include "lattice_planner/state_space/node_lattice.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lattice_planner {

template<>
template<typename MotionTableT>
void DistanceHeuristic<NodeLattice>::precomputeDistanceHeuristic(
  const float & lookup_table_dim,
  const MotionModel & /*motion_model*/,
  const unsigned int & dim_3_size,
  const SearchInfo & search_info,
  MotionTableT & motion_table)
{
  motion_table.lattice_metadata =
    LatticeMotionTable::getLatticeMetadata(search_info.lattice_filepath);

  // Select state space based on motion model from lattice file
  if (motion_table.lattice_metadata.motion_model == "omni") {
    // Holonomic robots: Euclidean distance heuristic
    motion_table.state_space = std::make_shared<ompl::base::SE2StateSpace>();
    motion_table.motion_model = MotionModel::OMNI;
  } else if (!search_info.allow_reverse_expansion) {
    motion_table.state_space = std::make_shared<ompl::base::DubinsStateSpace>(
      search_info.minimum_turning_radius);
    motion_table.motion_model = MotionModel::DUBIN;
  } else {
    motion_table.state_space = std::make_shared<ompl::base::ReedsSheppStateSpace>(
      search_info.minimum_turning_radius);
    motion_table.motion_model = MotionModel::REEDS_SHEPP;
  }

  ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
  to[0] = 0.0;
  to[1] = 0.0;
  to[2] = 0.0;
  size_lookup_ = lookup_table_dim;
  float motion_heuristic = 0.0;
  unsigned int index = 0;
  int dim_3_size_int = static_cast<int>(dim_3_size);

  // Create a lookup table of Dubin/Reeds-Shepp distances in a window around the goal
  // to help drive the search towards admissible approaches. Due to symmetries in the
  // Heuristic space, we need to only store 2 of the 4 quadrants and simply mirror
  // around the X axis any relative node lookup. This reduces memory overhead and increases
  // the size of a window a platform can store in memory.
  dist_heuristic_lookup_table_.resize(size_lookup_ * ceil(size_lookup_ / 2.0) * dim_3_size_int);
  for (float x = ceil(-size_lookup_ / 2.0); x <= floor(size_lookup_ / 2.0); x += 1.0) {
    for (float y = 0.0; y <= floor(size_lookup_ / 2.0); y += 1.0) {
      for (int heading = 0; heading != dim_3_size_int; heading++) {
        from[0] = x;
        from[1] = y;
        from[2] = motion_table.getAngleFromBin(heading);
        motion_heuristic = motion_table.state_space->distance(from(), to());
        dist_heuristic_lookup_table_[index] = motion_heuristic;
        index++;
      }
    }
  }
}

template<typename NodeT>
template<typename MotionTableT>
float DistanceHeuristic<NodeT>::getDistanceHeuristic(
  const Coordinates & node_coords,
  const Coordinates & goal_coords,
  const float & obstacle_heuristic,
  MotionTableT & motion_table)
{
  // rotate and translate node_coords such that goal_coords relative is (0,0,0)
  // Due to the rounding involved in exact cell increments for caching,
  // this is not an exact replica of a live heuristic, but has bounded error.
  // (Usually less than 1 cell length)

  // This angle is negative since we are de-rotating the current node
  // by the goal angle; cos(-th) = cos(th) & sin(-th) = -sin(th)
  const TrigValues & trig_vals = motion_table.trig_values[goal_coords.theta];
  const float cos_th = trig_vals.first;
  const float sin_th = -trig_vals.second;
  const float dx = node_coords.x - goal_coords.x;
  const float dy = node_coords.y - goal_coords.y;

  double dtheta_bin = node_coords.theta - goal_coords.theta;
  if (dtheta_bin < 0) {
    dtheta_bin += motion_table.num_angle_quantization;
  }
  if (dtheta_bin > motion_table.num_angle_quantization) {
    dtheta_bin -= motion_table.num_angle_quantization;
  }

  Coordinates node_coords_relative(
    round(dx * cos_th - dy * sin_th),
    round(dx * sin_th + dy * cos_th),
    round(dtheta_bin));

  // Check if the relative node coordinate is within the localized window around the goal
  // to apply the distance heuristic. Since the lookup table contains only the positive
  // X axis, we mirror the Y and theta values across the X axis to find the heuristic values.
  float motion_heuristic = 0.0;
  const int floored_size = floor(size_lookup_ / 2.0);
  const int ceiling_size = ceil(size_lookup_ / 2.0);
  const float mirrored_relative_y = abs(node_coords_relative.y);
  if (abs(node_coords_relative.x) < floored_size && mirrored_relative_y < floored_size) {
    // Need to mirror angle if Y coordinate was mirrored
    int theta_pos;
    if (node_coords_relative.y < 0.0) {
      theta_pos = motion_table.num_angle_quantization - node_coords_relative.theta;
    } else {
      theta_pos = node_coords_relative.theta;
    }
    const int x_pos = node_coords_relative.x + floored_size;
    const int y_pos = static_cast<int>(mirrored_relative_y);
    const int index =
      x_pos * ceiling_size * motion_table.num_angle_quantization +
      y_pos * motion_table.num_angle_quantization +
      theta_pos;
    motion_heuristic = dist_heuristic_lookup_table_[index];
  } else if (obstacle_heuristic <= 0.0) {
    ompl::base::ScopedState<> from(motion_table.state_space), to(motion_table.state_space);
    to[0] = goal_coords.x;
    to[1] = goal_coords.y;
    from[0] = node_coords.x;
    from[1] = node_coords.y;
    // NodeLattice path: use getAngleFromBin for theta
    to[2] = motion_table.getAngleFromBin(goal_coords.theta);
    from[2] = motion_table.getAngleFromBin(node_coords.theta);
    motion_heuristic = motion_table.state_space->distance(from(), to());
  }

  return motion_heuristic;
}

// Explicit instantiation for NodeLattice + LatticeMotionTable
template void DistanceHeuristic<NodeLattice>::precomputeDistanceHeuristic<LatticeMotionTable>(
  const float &, const MotionModel &, const unsigned int &, const SearchInfo &,
  LatticeMotionTable &);
template float DistanceHeuristic<NodeLattice>::getDistanceHeuristic<LatticeMotionTable>(
  const Coordinates &, const Coordinates &, const float &, LatticeMotionTable &);

}  // namespace lattice_planner
