// Lattice Planner - Path Smoother
// Ported from nav2_smac_planner/smoother.hpp with ROS message types replaced by Pose2D.

#ifndef LATTICE_PLANNER__SMOOTHER__SMOOTHER_HPP_
#define LATTICE_PLANNER__SMOOTHER__SMOOTHER_HPP_

#include <vector>

#include "lattice_planner/core/types.hpp"
#include "lattice_planner/core/config.hpp"
#include "lattice_planner/core/constants.hpp"
#include "lattice_planner/costmap/costmap_2d.hpp"

#include "ompl/base/StateSpace.h"

namespace lattice_planner {

/**
 * @struct BoundaryPoints
 * @brief A 2D pose used for boundary expansion points.
 */
struct BoundaryPoints
{
  BoundaryPoints(double x_in, double y_in, double theta_in)
  : x(x_in), y(y_in), theta(theta_in)
  {}

  double x;
  double y;
  double theta;
};

/**
 * @struct BoundaryExpansion
 * @brief Boundary expansion state for smoother boundary conditions.
 */
struct BoundaryExpansion
{
  double path_end_idx{0.0};
  double expansion_path_length{0.0};
  double original_path_length{0.0};
  std::vector<BoundaryPoints> pts;
  bool in_collision{false};
};

typedef std::vector<BoundaryExpansion> BoundaryExpansions;
typedef std::vector<Pose2D>::iterator PathIterator;
typedef std::vector<Pose2D>::reverse_iterator ReversePathIterator;

/**
 * @class Smoother
 * @brief Conjugate-gradient path smoother with curvature boundary conditions.
 *        Replaces nav2_smac_planner::Smoother with no ROS dependencies.
 */
class Smoother
{
public:
  explicit Smoother(const SmootherParams & params);
  ~Smoother() {}

  /**
   * @brief Initialize the smoother with minimum turning radius.
   * @param min_turning_radius Minimum turning radius in world units (meters).
   */
  void initialize(const double & min_turning_radius);

  /**
   * @brief Smooth a path using conjugate gradient with boundary conditions.
   * @param path Path to smooth (modified in place).
   * @param costmap Costmap for collision checking (may be nullptr).
   * @param max_time Maximum smoothing time in seconds.
   * @return true if smoothing succeeded.
   */
  bool smooth(
    std::vector<Pose2D> & path,
    const Costmap2D * costmap,
    const double & max_time);

protected:
  bool smoothImpl(
    std::vector<Pose2D> & path,
    bool & reversing_segment,
    const Costmap2D * costmap,
    const double & max_time);

  inline double getFieldByDim(const Pose2D & pose, const unsigned int & dim);
  inline void setFieldByDim(Pose2D & pose, const unsigned int dim, const double & value);

  void enforceStartBoundaryConditions(
    const Pose2D & start_pose,
    std::vector<Pose2D> & path,
    const Costmap2D * costmap,
    const bool & reversing_segment);

  void enforceEndBoundaryConditions(
    const Pose2D & end_pose,
    std::vector<Pose2D> & path,
    const Costmap2D * costmap,
    const bool & reversing_segment);

  unsigned int findShortestBoundaryExpansionIdx(const BoundaryExpansions & boundary_expansions);

  void findBoundaryExpansion(
    const Pose2D & start,
    const Pose2D & end,
    BoundaryExpansion & expansion,
    const Costmap2D * costmap);

  template<typename IteratorT>
  BoundaryExpansions generateBoundaryExpansionPoints(IteratorT start, IteratorT end);

  double min_turning_rad_, tolerance_, data_w_, smooth_w_;
  int max_its_, refinement_ctr_, refinement_num_;
  bool is_holonomic_, do_refinement_;
  MotionModel motion_model_;
  ompl::base::StateSpacePtr state_space_;
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__SMOOTHER__SMOOTHER_HPP_
