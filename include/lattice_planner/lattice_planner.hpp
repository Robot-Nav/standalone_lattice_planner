// Lattice Planner - Main Facade Class
// Replaces nav2_smac_planner::SmacPlannerLatticeT with a standalone, ROS-free
// interface. Provides configure/createPlan/cleanup lifecycle for the State
// Lattice A* planner with optional CG smoothing.

#ifndef LATTICE_PLANNER__LATTICE_PLANNER_HPP_
#define LATTICE_PLANNER__LATTICE_PLANNER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "lattice_planner/core/config.hpp"
#include "lattice_planner/core/constants.hpp"
#include "lattice_planner/core/exceptions.hpp"
#include "lattice_planner/core/logger.hpp"
#include "lattice_planner/core/types.hpp"
#include "lattice_planner/costmap/costmap_2d.hpp"
#include "lattice_planner/collision/collision_checker.hpp"
#include "lattice_planner/search/a_star.hpp"
#include "lattice_planner/smoother/smoother.hpp"
#include "lattice_planner/state_space/node_lattice.hpp"

namespace lattice_planner {

/**
 * @class LatticePlanner
 * @brief Standalone State Lattice planner facade.
 *
 * Replaces nav2_smac_planner::SmacPlannerLatticeT<NodeLattice> with a
 * ROS-free interface. Owns an A* search engine, collision checker, and
 * optional path smoother. Thread-safe: createPlan is guarded by an internal
 * mutex.
 *
 * Typical usage:
 * @code
 *   LatticePlannerConfig config = LatticePlannerConfig::loadFromFile("config.json");
 *   auto costmap = std::make_unique<Costmap2D>(...);
 *   LatticePlanner planner;
 *   planner.configure(config, costmap.get());
 *   Pose2D start{1.0, 1.0, 0.0};
 *   Pose2D goal{5.0, 5.0, 1.57};
 *   Path path = planner.createPlan(start, goal);
 *   planner.cleanup();
 * @endcode
 */
class LatticePlanner
{
public:
  LatticePlanner();
  ~LatticePlanner();

  /**
   * @brief Configure the planner with parameters and a costmap.
   * @param config Planner configuration (search penalties, smoother params,
   *               footprint, lattice filepath, etc.)
   * @param costmap Pointer to an external Costmap2D. Ownership is NOT
   *                transferred; the caller must keep it alive until cleanup().
   * @throws PlannerException if the lattice file cannot be loaded or
   *                          configuration is invalid.
   */
  void configure(const LatticePlannerConfig & config, Costmap2D * costmap);

  /**
   * @brief Plan a path from start to goal.
   * @param start Start pose in world coordinates (theta in radians).
   * @param goal  Goal pose in world coordinates (theta in radians).
   * @param cancel_checker Optional callback; if it returns true, planning
   *                       aborts with PlannerCancelled.
   * @return Path containing world-coordinate poses (start-to-goal order).
   * @throws StartOutsideMapBounds if start is outside the costmap.
   * @throws GoalOutsideMapBounds if goal is outside the costmap.
   * @throws StartOccupied if the start cell is lethal.
   * @throws NoValidPathCouldBeFound if no path exists.
   * @throws PlannerTimedOut if max iterations exceeded.
   * @throws PlannerCancelled if cancel_checker returned true.
   * @throws PlannerException if configure() was not called.
   */
  Path createPlan(
    const Pose2D & start,
    const Pose2D & goal,
    std::function<bool()> cancel_checker = nullptr);

  /**
   * @brief Release internal resources (A*, smoother, collision checker).
   *        The costmap pointer is NOT freed (external ownership).
   */
  void cleanup();

  /**
   * @brief Check if the planner has been configured.
   * @return true if configure() was called successfully.
   */
  bool isConfigured() const { return _configured; }

private:
  std::unique_ptr<AStarAlgorithm<NodeLattice>> _a_star;
  std::unique_ptr<GridCollisionChecker> _collision_checker;
  std::unique_ptr<Smoother> _smoother;

  Costmap2D * _costmap;
  LatticePlannerConfig _config;
  LatticeMetadata _metadata;

  MotionModel _motion_model;
  GoalHeadingMode _goal_heading_mode;
  int _coarse_search_resolution;
  float _tolerance;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  int _terminal_checking_interval;
  double _max_planning_time;
  double _lookup_table_size;
  bool _smooth_path;
  double _smoothing_max_time;

  std::mutex _mutex;
  bool _configured;
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__LATTICE_PLANNER_HPP_
