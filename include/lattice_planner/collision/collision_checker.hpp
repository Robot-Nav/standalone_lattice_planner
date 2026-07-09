// Lattice Planner - Grid Collision Checker
// Replaces nav2_smac_planner::GridCollisionChecker with no ROS dependencies.

#ifndef LATTICE_PLANNER__COLLISION__COLLISION_CHECKER_HPP_
#define LATTICE_PLANNER__COLLISION__COLLISION_CHECKER_HPP_

#include <vector>
#include "lattice_planner/collision/footprint_collision_checker.hpp"
#include "lattice_planner/costmap/costmap_2d.hpp"
#include "lattice_planner/core/constants.hpp"

namespace lattice_planner {

/**
 * @class GridCollisionChecker
 * @brief A costmap grid collision checker with precomputed oriented footprints.
 *        Replaces nav2_smac_planner::GridCollisionChecker.
 */
class GridCollisionChecker : public FootprintCollisionChecker<Costmap2D *>
{
public:
  /**
   * @brief Constructor with regular angle quantization.
   * @param costmap Pointer to the costmap to check against
   * @param num_quantizations Number of angle bins to precompute footprints for
   */
  GridCollisionChecker(Costmap2D * costmap, unsigned int num_quantizations);

  /**
   * @brief Constructor with irregular angle bins (e.g. from lattice metadata heading_angles).
   * @param costmap Pointer to the costmap to check against
   * @param angles Vector of angle values (radians) for footprint precomputation
   */
  GridCollisionChecker(Costmap2D * costmap, const std::vector<float> & angles);

  /**
   * @brief Set the robot footprint.
   * @param footprint Vector of points forming the footprint polygon (robot frame)
   * @param radius If true, footprint is treated as a circle (use center cost only)
   * @param possible_collision_cost Cost threshold below which no full footprint check is needed
   */
  void setFootprint(
    const Footprint & footprint,
    const bool & radius,
    const double & possible_collision_cost);

  /**
   * @brief Check if a pose is in collision.
   * @param x X cell coordinate (continuous)
   * @param y Y cell coordinate (continuous)
   * @param theta_bin Angle bin index (NOT radians)
   * @param traverse_unknown Whether to allow traversal of unknown cells
   * @return true if in collision, false otherwise.
   */
  bool inCollision(
    const float & x,
    const float & y,
    const float & theta_bin,
    const bool & traverse_unknown);

  /**
   * @brief Check if a cell index is in collision (center cost only).
   * @param i Linear cell index
   * @param traverse_unknown Whether to allow traversal of unknown cells
   * @return true if in collision, false otherwise.
   */
  bool inCollision(const unsigned int & i, const bool & traverse_unknown);

  /**
   * @brief Get the cost at the last-checked pose.
   * @return Cost value (0-255).
   */
  float getCost();

  /**
   * @brief Get the precomputed angle bins.
   * @return Reference to the angles vector.
   */
  std::vector<float> & getPrecomputedAngles() { return angles_; }

  /**
   * @brief Get the costmap pointer.
   * @return Costmap pointer.
   */
  Costmap2D * getCostmap() { return this->costmap_; }

  /**
   * @brief Check if a value is outside the valid range [0, max].
   * @param max Maximum valid value
   * @param value Value to check
   * @return true if value < 0 or value > max.
   */
  bool outsideRange(const unsigned int & max, const float & value);

protected:
  std::vector<Footprint> oriented_footprints_;
  Footprint unoriented_footprint_;
  float center_cost_{0.0f};
  bool footprint_is_radius_{false};
  std::vector<float> angles_;
  float possible_collision_cost_{-1.0f};
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__COLLISION__COLLISION_CHECKER_HPP_
