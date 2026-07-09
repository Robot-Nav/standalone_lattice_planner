// Lattice Planner - Configuration Structures
// Replaces ROS parameter declarations with plain C++ structs.

#ifndef LATTICE_PLANNER__CORE__CONFIG_HPP_
#define LATTICE_PLANNER__CORE__CONFIG_HPP_

#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include "lattice_planner/core/constants.hpp"
#include "lattice_planner/core/types.hpp"
#include "lattice_planner/core/exceptions.hpp"
#include "nlohmann/json.hpp"

namespace lattice_planner {

typedef std::pair<float, uint64_t> NodeHeuristicPair;
typedef std::vector<float> LookupTable;
typedef std::pair<double, double> TrigValues;

/**
 * @struct SearchInfo
 * @brief Search properties and penalties. Ported from nav2_smac_planner::SearchInfo
 *        with no ROS dependencies.
 */
struct SearchInfo {
  float minimum_turning_radius{8.0f};
  float non_straight_penalty{1.05f};
  float change_penalty{0.0f};
  float reverse_penalty{2.0f};
  float cost_penalty{2.0f};
  float retrospective_penalty{0.015f};
  float rotation_penalty{5.0f};
  float analytic_expansion_ratio{3.5f};
  float analytic_expansion_max_length{60.0f};
  float analytic_expansion_max_cost{200.0f};
  bool analytic_expansion_max_cost_override{false};
  std::string lattice_filepath;
  bool cache_obstacle_heuristic{false};
  bool allow_reverse_expansion{false};
  bool allow_primitive_interpolation{false};
  bool downsample_obstacle_heuristic{true};
  bool use_quadratic_cost_penalty{false};
};

/**
 * @struct SmootherParams
 * @brief Parameters for the smoother. Ported from nav2_smac_planner::SmootherParams
 *        with the ROS get() method removed. Use setDefaults() or set fields directly.
 */
struct SmootherParams {
  SmootherParams() : holonomic_(false) {}

  void setDefaults() {
    tolerance_ = 1e-10;
    max_its_ = 1000;
    w_data_ = 0.2;
    w_smooth_ = 0.3;
    do_refinement_ = true;
    refinement_num_ = 2;
    holonomic_ = false;
  }

  double tolerance_{1e-10};
  int max_its_{1000};
  double w_data_{0.2};
  double w_smooth_{0.3};
  bool holonomic_{false};
  bool do_refinement_{true};
  int refinement_num_{2};
};

/**
 * @struct LatticePlannerConfig
 * @brief Top-level configuration for the standalone LatticePlanner.
 *        Consolidates all parameters that were previously declared as ROS parameters.
 */
struct LatticePlannerConfig {
  // Motion model
  MotionModel motion_model{MotionModel::STATE_LATTICE};

  // Lattice primitives file path
  std::string lattice_filepath;

  // Search info (penalties, turning radius, analytic expansion params)
  SearchInfo search_info;

  // Smoother params
  SmootherParams smoother_params;

  // A* search parameters
  bool allow_unknown{false};
  int max_iterations{1000000};
  int max_on_approach_iterations{std::numeric_limits<int>::max()};
  int terminal_checking_interval{5000};
  double max_planning_time{5.0};
  double lookup_table_size{400.0};  // in costmap pixels, as float-compatible
  unsigned int angle_quantization{72};
  float tolerance{2.0f};

  // Goal heading
  GoalHeadingMode goal_heading_mode{GoalHeadingMode::DEFAULT};
  int coarse_search_resolution{1};

  // Smoother enable
  bool smooth_path{true};
  double smoothing_max_time{2.0};

  // Robot footprint (polygon in world coordinates, meters)
  std::vector<Point2D> footprint;
  bool footprint_is_radius{false};
  double possible_collision_cost{0.0};

  // Number of footprint angle quantizations for precomputation
  unsigned int num_quantizations{72};

  // Costmap inflation parameters (static replacement for nav2 inflation layer)
  double inflation_radius{0.0};
  double inscribed_radius{0.0};
  double circumscribed_radius{0.0};
  double circumscribed_cost{-1.0};  // -1 = auto-compute from costmap

  // Debug
  bool debug_visualizations{false};

  /**
   * @brief Load configuration from a JSON file.
   *
   * JSON format (all fields optional, defaults used if missing):
   * {
   *   "motion_model": "STATE_LATTICE",
   *   "lattice_filepath": "/path/to/output.json",
   *   "search_info": {
   *     "minimum_turning_radius": 0.5,
   *     "non_straight_penalty": 1.05,
   *     ...
   *   },
   *   "smoother_params": {
   *     "tolerance": 1e-10,
   *     "max_iterations": 1000,
   *     ...
   *   },
   *   "max_iterations": 1000000,
   *   "max_planning_time": 5.0,
   *   "tolerance": 2.0,
   *   "allow_unknown": false,
   *   "footprint": [[x1,y1], [x2,y2], ...],
   *   "footprint_is_radius": false,
   *   ...
   * }
   */
  static LatticePlannerConfig loadFromFile(const std::string & json_path) {
    std::ifstream ifs(json_path);
    if (!ifs.is_open()) {
      throw PlannerException("Cannot open config file: " + json_path);
    }
    nlohmann::json j;
    ifs >> j;
    return loadFromJson(j);
  }

  static LatticePlannerConfig loadFromJson(const nlohmann::json & j) {
    LatticePlannerConfig cfg;

    if (j.contains("motion_model")) {
      cfg.motion_model = fromString(j["motion_model"].get<std::string>());
    }
    if (j.contains("lattice_filepath")) {
      cfg.lattice_filepath = j["lattice_filepath"].get<std::string>();
    }

    if (j.contains("search_info")) {
      const auto & si = j["search_info"];
      auto & dst = cfg.search_info;
      if (si.contains("lattice_filepath")) {
        dst.lattice_filepath = si["lattice_filepath"].get<std::string>();
      } else {
        dst.lattice_filepath = cfg.lattice_filepath;
      }
      if (si.contains("minimum_turning_radius")) dst.minimum_turning_radius = si["minimum_turning_radius"].get<float>();
      if (si.contains("non_straight_penalty")) dst.non_straight_penalty = si["non_straight_penalty"].get<float>();
      if (si.contains("change_penalty")) dst.change_penalty = si["change_penalty"].get<float>();
      if (si.contains("reverse_penalty")) dst.reverse_penalty = si["reverse_penalty"].get<float>();
      if (si.contains("cost_penalty")) dst.cost_penalty = si["cost_penalty"].get<float>();
      if (si.contains("retrospective_penalty")) dst.retrospective_penalty = si["retrospective_penalty"].get<float>();
      if (si.contains("rotation_penalty")) dst.rotation_penalty = si["rotation_penalty"].get<float>();
      if (si.contains("analytic_expansion_ratio")) dst.analytic_expansion_ratio = si["analytic_expansion_ratio"].get<float>();
      if (si.contains("analytic_expansion_max_length")) dst.analytic_expansion_max_length = si["analytic_expansion_max_length"].get<float>();
      if (si.contains("analytic_expansion_max_cost")) dst.analytic_expansion_max_cost = si["analytic_expansion_max_cost"].get<float>();
      if (si.contains("analytic_expansion_max_cost_override")) dst.analytic_expansion_max_cost_override = si["analytic_expansion_max_cost_override"].get<bool>();
      if (si.contains("cache_obstacle_heuristic")) dst.cache_obstacle_heuristic = si["cache_obstacle_heuristic"].get<bool>();
      if (si.contains("allow_reverse_expansion")) dst.allow_reverse_expansion = si["allow_reverse_expansion"].get<bool>();
      if (si.contains("allow_primitive_interpolation")) dst.allow_primitive_interpolation = si["allow_primitive_interpolation"].get<bool>();
      if (si.contains("downsample_obstacle_heuristic")) dst.downsample_obstacle_heuristic = si["downsample_obstacle_heuristic"].get<bool>();
      if (si.contains("use_quadratic_cost_penalty")) dst.use_quadratic_cost_penalty = si["use_quadratic_cost_penalty"].get<bool>();
    } else {
      cfg.search_info.lattice_filepath = cfg.lattice_filepath;
    }

    if (j.contains("smoother_params")) {
      const auto & sp = j["smoother_params"];
      auto & dst = cfg.smoother_params;
      if (sp.contains("tolerance")) dst.tolerance_ = sp["tolerance"].get<double>();
      if (sp.contains("max_iterations")) dst.max_its_ = sp["max_iterations"].get<int>();
      if (sp.contains("w_data")) dst.w_data_ = sp["w_data"].get<double>();
      if (sp.contains("w_smooth")) dst.w_smooth_ = sp["w_smooth"].get<double>();
      if (sp.contains("holonomic")) dst.holonomic_ = sp["holonomic"].get<bool>();
      if (sp.contains("do_refinement")) dst.do_refinement_ = sp["do_refinement"].get<bool>();
      if (sp.contains("refinement_num")) dst.refinement_num_ = sp["refinement_num"].get<int>();
    }

    if (j.contains("max_iterations")) cfg.max_iterations = j["max_iterations"].get<int>();
    if (j.contains("max_on_approach_iterations")) cfg.max_on_approach_iterations = j["max_on_approach_iterations"].get<int>();
    if (j.contains("terminal_checking_interval")) cfg.terminal_checking_interval = j["terminal_checking_interval"].get<int>();
    if (j.contains("max_planning_time")) cfg.max_planning_time = j["max_planning_time"].get<double>();
    if (j.contains("lookup_table_size")) cfg.lookup_table_size = j["lookup_table_size"].get<double>();
    if (j.contains("angle_quantization")) cfg.angle_quantization = j["angle_quantization"].get<unsigned int>();
    if (j.contains("tolerance")) cfg.tolerance = j["tolerance"].get<float>();
    if (j.contains("allow_unknown")) cfg.allow_unknown = j["allow_unknown"].get<bool>();
    if (j.contains("goal_heading_mode")) cfg.goal_heading_mode = fromStringToGH(j["goal_heading_mode"].get<std::string>());
    if (j.contains("coarse_search_resolution")) cfg.coarse_search_resolution = j["coarse_search_resolution"].get<int>();
    if (j.contains("smooth_path")) cfg.smooth_path = j["smooth_path"].get<bool>();
    if (j.contains("smoothing_max_time")) cfg.smoothing_max_time = j["smoothing_max_time"].get<double>();
    if (j.contains("num_quantizations")) cfg.num_quantizations = j["num_quantizations"].get<unsigned int>();
    if (j.contains("footprint_is_radius")) cfg.footprint_is_radius = j["footprint_is_radius"].get<bool>();
    if (j.contains("possible_collision_cost")) cfg.possible_collision_cost = j["possible_collision_cost"].get<double>();
    if (j.contains("inflation_radius")) cfg.inflation_radius = j["inflation_radius"].get<double>();
    if (j.contains("inscribed_radius")) cfg.inscribed_radius = j["inscribed_radius"].get<double>();
    if (j.contains("circumscribed_radius")) cfg.circumscribed_radius = j["circumscribed_radius"].get<double>();
    if (j.contains("circumscribed_cost")) cfg.circumscribed_cost = j["circumscribed_cost"].get<double>();
    if (j.contains("debug_visualizations")) cfg.debug_visualizations = j["debug_visualizations"].get<bool>();

    if (j.contains("footprint")) {
      for (const auto & pt : j["footprint"]) {
        cfg.footprint.emplace_back(pt[0].get<double>(), pt[1].get<double>());
      }
    }

    return cfg;
  }
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__CORE__CONFIG_HPP_
