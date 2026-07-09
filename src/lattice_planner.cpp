// Lattice Planner - Main Facade Implementation
// Ported from nav2_smac_planner/smac_planner_lattice_impl.hpp with all ROS
// lifecycle, parameter, publisher, and TF dependencies removed.

#include "lattice_planner/lattice_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lattice_planner {

using namespace std::chrono;  // NOLINT

LatticePlanner::LatticePlanner()
: _a_star(nullptr),
  _collision_checker(nullptr),
  _smoother(nullptr),
  _costmap(nullptr),
  _motion_model(MotionModel::STATE_LATTICE),
  _goal_heading_mode(GoalHeadingMode::DEFAULT),
  _coarse_search_resolution(1),
  _tolerance(0.25f),
  _allow_unknown(true),
  _max_iterations(1000000),
  _max_on_approach_iterations(1000),
  _terminal_checking_interval(5000),
  _max_planning_time(5.0),
  _lookup_table_size(20.0),
  _smooth_path(true),
  _smoothing_max_time(2.0),
  _configured(false)
{
}

LatticePlanner::~LatticePlanner()
{
  cleanup();
}

void LatticePlanner::configure(const LatticePlannerConfig & config, Costmap2D * costmap)
{
  if (!costmap) {
    throw PlannerException("Cannot configure: costmap pointer is null");
  }

  _config = config;
  _costmap = costmap;

  // Load lattice metadata from the primitives file
  if (_config.search_info.lattice_filepath.empty()) {
    throw PlannerException("Cannot configure: lattice_filepath is empty");
  }
  _metadata = LatticeMotionTable::getLatticeMetadata(_config.search_info.lattice_filepath);

  if (_metadata.number_of_headings == 0) {
    throw PlannerException(
            "Invalid lattice metadata: number_of_headings is 0. "
            "Check lattice file: " + _config.search_info.lattice_filepath);
  }

  // Convert turning radius from meters (metadata) to grid cells
  double resolution = _costmap->getResolution();
  if (resolution <= 0.0) {
    throw PlannerException("Cannot configure: costmap resolution must be > 0");
  }
  _config.search_info.minimum_turning_radius =
    _metadata.min_turning_radius / static_cast<float>(resolution);

  // Convert analytic_expansion_max_length from meters to grid cells
  _config.search_info.analytic_expansion_max_length =
    _config.search_info.analytic_expansion_max_length / static_cast<float>(resolution);

  // Goal heading mode
  _goal_heading_mode = _config.goal_heading_mode;
  if (_goal_heading_mode == GoalHeadingMode::UNKNOWN) {
    throw PlannerException(
            "Invalid goal_heading_mode. Valid options: DEFAULT, BIDIRECTIONAL, ALL_DIRECTION");
  }

  _coarse_search_resolution = _config.coarse_search_resolution;
  if (_coarse_search_resolution <= 0) {
    LP_LOG_WARN("coarse_search_resolution <= 0, disabling coarse search");
    _coarse_search_resolution = 1;
  }

  if (_metadata.number_of_headings % static_cast<unsigned int>(_coarse_search_resolution) != 0) {
    throw PlannerException(
            "coarse_search_resolution must be a divisor of number_of_headings (" +
            std::to_string(_metadata.number_of_headings) + ")");
  }

  // Handle iteration limits
  _max_iterations = _config.max_iterations;
  if (_max_iterations <= 0) {
    LP_LOG_INFO("max_iterations <= 0, disabling maximum iterations");
    _max_iterations = std::numeric_limits<int>::max();
  }

  _max_on_approach_iterations = _config.max_on_approach_iterations;
  if (_max_on_approach_iterations <= 0) {
    LP_LOG_INFO("max_on_approach_iterations <= 0, disabling on-approach iterations");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  _terminal_checking_interval = _config.terminal_checking_interval;
  _max_planning_time = _config.max_planning_time;
  _lookup_table_size = _config.lookup_table_size;
  _tolerance = _config.tolerance;
  _allow_unknown = _config.allow_unknown;
  _smooth_path = _config.smooth_path;
  _smoothing_max_time = _config.smoothing_max_time;
  _motion_model = MotionModel::STATE_LATTICE;

  // Omni robots are holonomic
  if (_metadata.motion_model == "omni") {
    _config.smoother_params.holonomic_ = true;
    if (_config.search_info.allow_reverse_expansion) {
      LP_LOG_WARN(
        "allow_reverse_expansion is not applicable for omnidirectional robots. Disabling.");
      _config.search_info.allow_reverse_expansion = false;
    }
  }

  // Compute lookup table dimension in grid cells (odd integer)
  float lookup_table_dim =
    static_cast<float>(_lookup_table_size) / static_cast<float>(resolution);
  lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));
  if (static_cast<int>(lookup_table_dim) % 2 == 0) {
    LP_LOG_INFO("Even lookup table size, increasing by 1 to make odd");
    lookup_table_dim += 1.0f;
  }

  // Set costmap inflation parameters (static replacement for nav2 inflation layer)
  _costmap->setInflationParameters(
    _config.inflation_radius, _config.inscribed_radius, _config.circumscribed_radius);

  // Determine possible_collision_cost: use config value if > 0, else auto-compute
  double possible_collision_cost = _config.possible_collision_cost;
  if (possible_collision_cost <= 0.0) {
    double circ_cost = _costmap->getCircumscribedCost();
    if (circ_cost > 0.0) {
      possible_collision_cost = circ_cost;
    }
  }

  // Create collision checker with 72 evenly-spaced angle bins (5-degree resolution)
  // This matches the original nav2_smac_planner behavior for stable collision checking
  // of intermediary primitive points.
  unsigned int num_quant = _config.num_quantizations;
  if (num_quant == 0) {
    num_quant = 72;
  }
  _collision_checker = std::make_unique<GridCollisionChecker>(_costmap, num_quant);
  _collision_checker->setFootprint(
    _config.footprint, _config.footprint_is_radius, possible_collision_cost);

  // Create A* search engine
  _a_star = std::make_unique<AStarAlgorithm<NodeLattice>>(_motion_model, _config.search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _terminal_checking_interval,
    _max_planning_time,
    lookup_table_dim,
    _metadata.number_of_headings);

  // Create smoother if enabled
  if (_smooth_path) {
    _smoother = std::make_unique<Smoother>(_config.smoother_params);
    _smoother->initialize(_metadata.min_turning_radius);
  } else {
    _smoother.reset();
  }

  _configured = true;

  LP_LOG_INFO(
    "LatticePlanner configured: max_iterations=" << _max_iterations <<
    ", tolerance=" << _tolerance <<
    ", allow_unknown=" << (_allow_unknown ? "true" : "false") <<
    ", motion_model=" << toString(_motion_model) <<
    ", lattice=" << _config.search_info.lattice_filepath);
}

Path LatticePlanner::createPlan(
  const Pose2D & start,
  const Pose2D & goal,
  std::function<bool()> cancel_checker)
{
  if (!_configured) {
    throw PlannerException("LatticePlanner not configured. Call configure() first.");
  }

  std::lock_guard<std::mutex> lock(_mutex);

  steady_clock::time_point start_time = steady_clock::now();

  // Update collision checker footprint (supports dynamic footprint changes)
  double possible_collision_cost = _config.possible_collision_cost;
  if (possible_collision_cost <= 0.0) {
    double circ_cost = _costmap->getCircumscribedCost();
    if (circ_cost > 0.0) {
      possible_collision_cost = circ_cost;
    }
  }
  _collision_checker->setFootprint(
    _config.footprint, _config.footprint_is_radius, possible_collision_cost);
  _a_star->setCollisionChecker(_collision_checker.get());

  // Convert start to grid coordinates
  double mx_start_d, my_start_d;
  if (!_costmap->worldToMapContinuous(start.x, start.y, mx_start_d, my_start_d)) {
    throw StartOutsideMapBounds(
            "Start coordinates (" + std::to_string(start.x) + ", " +
            std::to_string(start.y) + ") outside map bounds");
  }
  float mx_start = static_cast<float>(mx_start_d);
  float my_start = static_cast<float>(my_start_d);
  unsigned int start_bin =
    _a_star->getContext()->motion_table.getClosestAngularBin(start.theta);
  _a_star->setStart(mx_start, my_start, start_bin);

  // Convert goal to grid coordinates
  double mx_goal_d, my_goal_d;
  if (!_costmap->worldToMapContinuous(goal.x, goal.y, mx_goal_d, my_goal_d)) {
    throw GoalOutsideMapBounds(
            "Goal coordinates (" + std::to_string(goal.x) + ", " +
            std::to_string(goal.y) + ") outside map bounds");
  }
  float mx_goal = static_cast<float>(mx_goal_d);
  float my_goal = static_cast<float>(my_goal_d);
  unsigned int goal_bin =
    _a_star->getContext()->motion_table.getClosestAngularBin(goal.theta);
  _a_star->setGoal(
    mx_goal, my_goal, goal_bin, _goal_heading_mode, _coarse_search_resolution);

  // Corner case: start and goal in the same cell with the same heading
  if (std::floor(mx_start) == std::floor(mx_goal) &&
    std::floor(my_start) == std::floor(my_goal) &&
    start_bin == goal_bin)
  {
    Path plan;
    plan.poses.push_back(start);
    plan.poses.back().theta = goal.theta;
    return plan;
  }

  // Run A* search
  CoordinateVector path;
  int num_iterations = 0;
  float tolerance_grid = _tolerance / static_cast<float>(_costmap->getResolution());

  if (!_a_star->createPath(path, num_iterations, tolerance_grid, cancel_checker, nullptr)) {
    if (num_iterations == 1) {
      throw StartOccupied("Start cell is occupied");
    }
    if (num_iterations < _max_iterations) {
      throw NoValidPathCouldBeFound(
              "No valid path found after " + std::to_string(num_iterations) + " iterations");
    }
    throw PlannerTimedOut(
            "Exceeded maximum iterations (" + std::to_string(_max_iterations) + ")");
  }

  // Convert grid coordinates to world coordinates (reverse order: goal→start becomes start→goal)
  Path plan;
  plan.poses.reserve(path.size());
  double resolution = _costmap->getResolution();
  double origin_x = _costmap->getOriginX();
  double origin_y = _costmap->getOriginY();

  double last_x = std::numeric_limits<double>::quiet_NaN();
  double last_y = std::numeric_limits<double>::quiet_NaN();
  double last_theta = std::numeric_limits<double>::quiet_NaN();

  for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i) {
    double wx = origin_x + path[i].x * resolution;
    double wy = origin_y + path[i].y * resolution;
    double wtheta = path[i].theta;

    // Deduplicate: skip if essentially identical to the previous point
    if (!std::isnan(last_x)) {
      if (std::fabs(wx - last_x) < 1e-4 &&
        std::fabs(wy - last_y) < 1e-4 &&
        std::fabs(shortestAngularDistance(wtheta, last_theta)) < 1e-4)
      {
        continue;
      }
    }

    plan.poses.emplace_back(wx, wy, wtheta);
    last_x = wx;
    last_y = wy;
    last_theta = wtheta;
  }

  // Smooth the path if enabled and more than one point
  if (_smoother && num_iterations > 1 && plan.poses.size() > 1) {
    steady_clock::time_point now = steady_clock::now();
    double elapsed = duration_cast<duration<double>>(now - start_time).count();
    double time_remaining = _smoothing_max_time;
    if (_max_planning_time > elapsed) {
      time_remaining = std::min(_smoothing_max_time, _max_planning_time - elapsed);
    }
    _smoother->smooth(plan.poses, _costmap, time_remaining);
  }

  return plan;
}

void LatticePlanner::cleanup()
{
  if (!_configured) {
    return;
  }
  _a_star.reset();
  _smoother.reset();
  _collision_checker.reset();
  _costmap = nullptr;
  _configured = false;
  LP_LOG_INFO("LatticePlanner cleaned up");
}

}  // namespace lattice_planner
