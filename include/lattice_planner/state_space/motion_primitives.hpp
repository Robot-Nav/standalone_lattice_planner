// Lattice Planner - Motion Primitives Types and JSON Parsing
// Ported from nav2_smac_planner types.hpp and utils.hpp with no ROS dependencies.

#ifndef LATTICE_PLANNER__STATE_SPACE__MOTION_PRIMITIVES_HPP_
#define LATTICE_PLANNER__STATE_SPACE__MOTION_PRIMITIVES_HPP_

#include <string>
#include <vector>
#include "nlohmann/json.hpp"

namespace lattice_planner {

/**
 * @enum TurnDirection
 * @brief Direction of a motion primitive's turn.
 */
enum class TurnDirection {
  UNKNOWN = 0,
  FORWARD = 1,
  LEFT = 2,
  RIGHT = 3,
  REVERSE = 4,
  REV_LEFT = 5,
  REV_RIGHT = 6
};

/**
 * @struct MotionPose
 * @brief A single pose within a motion primitive (metric coordinates).
 */
struct MotionPose {
  MotionPose() = default;

  MotionPose(const float & x, const float & y, const float & theta, const TurnDirection & turn_dir)
  : _x(x), _y(y), _theta(theta), _turn_dir(turn_dir) {}

  MotionPose operator-(const MotionPose & p2) {
    return MotionPose(
      this->_x - p2._x, this->_y - p2._y, this->_theta - p2._theta, TurnDirection::UNKNOWN);
  }

  float _x{0.0f};
  float _y{0.0f};
  float _theta{0.0f};
  TurnDirection _turn_dir{TurnDirection::UNKNOWN};
};

typedef std::vector<MotionPose> MotionPoses;

/**
 * @struct LatticeMetadata
 * @brief Metadata about a lattice primitive set.
 */
struct LatticeMetadata {
  float min_turning_radius{0.0f};
  float grid_resolution{0.0f};
  unsigned int number_of_headings{0};
  std::vector<float> heading_angles;
  unsigned int number_of_trajectories{0};
  std::string motion_model;
};

/**
 * @struct MotionPrimitive
 * @brief A single motion primitive trajectory.
 */
struct MotionPrimitive {
  unsigned int trajectory_id{0};
  float start_angle{0.0f};
  float end_angle{0.0f};
  float turning_radius{0.0f};
  float trajectory_length{0.0f};
  float arc_length{0.0f};
  float straight_length{0.0f};
  bool left_turn{false};
  MotionPoses poses;
};

typedef std::vector<MotionPrimitive> MotionPrimitives;
typedef std::vector<MotionPrimitive *> MotionPrimitivePtrs;

// ---- JSON parsing functions (ported from nav2_smac_planner/utils.hpp) ----

/**
 * @brief Convert JSON to lattice metadata.
 */
inline void fromJsonToMetaData(const nlohmann::json & json, LatticeMetadata & lattice_metadata)
{
  json.at("turning_radius").get_to(lattice_metadata.min_turning_radius);
  json.at("grid_resolution").get_to(lattice_metadata.grid_resolution);
  json.at("num_of_headings").get_to(lattice_metadata.number_of_headings);
  json.at("heading_angles").get_to(lattice_metadata.heading_angles);
  json.at("number_of_trajectories").get_to(lattice_metadata.number_of_trajectories);
  json.at("motion_model").get_to(lattice_metadata.motion_model);
}

/**
 * @brief Convert JSON to a motion pose.
 */
inline void fromJsonToPose(const nlohmann::json & json, MotionPose & pose)
{
  pose._x = json[0];
  pose._y = json[1];
  pose._theta = json[2];
}

/**
 * @brief Convert JSON to a motion primitive.
 */
inline void fromJsonToMotionPrimitive(
  const nlohmann::json & json, MotionPrimitive & motion_primitive)
{
  json.at("trajectory_id").get_to(motion_primitive.trajectory_id);
  json.at("start_angle_index").get_to(motion_primitive.start_angle);
  json.at("end_angle_index").get_to(motion_primitive.end_angle);
  json.at("trajectory_radius").get_to(motion_primitive.turning_radius);
  json.at("trajectory_length").get_to(motion_primitive.trajectory_length);
  json.at("arc_length").get_to(motion_primitive.arc_length);
  json.at("straight_length").get_to(motion_primitive.straight_length);
  json.at("left_turn").get_to(motion_primitive.left_turn);

  for (unsigned int i = 0; i < json["poses"].size(); ++i) {
    MotionPose pose;
    fromJsonToPose(json["poses"][i], pose);
    motion_primitive.poses.push_back(pose);
  }
}

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__STATE_SPACE__MOTION_PRIMITIVES_HPP_
