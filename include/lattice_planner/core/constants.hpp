// Lattice Planner - Constants and Enums
// Decoupled from ROS nav2_smac_planner. No ROS dependencies.

#ifndef LATTICE_PLANNER__CORE__CONSTANTS_HPP_
#define LATTICE_PLANNER__CORE__CONSTANTS_HPP_

#include <string>

namespace lattice_planner {

/** @brief Motion model enumeration. */
enum class MotionModel {
  UNKNOWN = 0,
  TWOD = 1,
  DUBIN = 2,
  REEDS_SHEPP = 3,
  STATE_LATTICE = 4,
  OMNI = 5,
};

/** @brief Goal heading mode enumeration. */
enum class GoalHeadingMode {
  UNKNOWN = 0,
  DEFAULT = 1,
  BIDIRECTIONAL = 2,
  ALL_DIRECTION = 3,
};

/** @brief Convert MotionModel to string. */
inline std::string toString(const MotionModel & n) {
  switch (n) {
    case MotionModel::TWOD:          return "2D";
    case MotionModel::DUBIN:         return "Dubin";
    case MotionModel::REEDS_SHEPP:   return "Reeds-Shepp";
    case MotionModel::STATE_LATTICE: return "State Lattice";
    case MotionModel::OMNI:          return "Omni";
    default:                         return "Unknown";
  }
}

/** @brief Convert string to MotionModel. */
inline MotionModel fromString(const std::string & n) {
  if (n == "2D")             return MotionModel::TWOD;
  if (n == "DUBIN")          return MotionModel::DUBIN;
  if (n == "REEDS_SHEPP")    return MotionModel::REEDS_SHEPP;
  if (n == "STATE_LATTICE")  return MotionModel::STATE_LATTICE;
  if (n == "OMNI")           return MotionModel::OMNI;
  return MotionModel::UNKNOWN;
}

/** @brief Convert GoalHeadingMode to string. */
inline std::string toString(const GoalHeadingMode & n) {
  switch (n) {
    case GoalHeadingMode::DEFAULT:       return "DEFAULT";
    case GoalHeadingMode::BIDIRECTIONAL: return "BIDIRECTIONAL";
    case GoalHeadingMode::ALL_DIRECTION: return "ALL_DIRECTION";
    default:                             return "Unknown";
  }
}

/** @brief Convert string to GoalHeadingMode. */
inline GoalHeadingMode fromStringToGH(const std::string & n) {
  if (n == "DEFAULT")        return GoalHeadingMode::DEFAULT;
  if (n == "BIDIRECTIONAL")  return GoalHeadingMode::BIDIRECTIONAL;
  if (n == "ALL_DIRECTION")  return GoalHeadingMode::ALL_DIRECTION;
  return GoalHeadingMode::UNKNOWN;
}

// Costmap cost thresholds (match nav2_costmap_2d conventions)
const float UNKNOWN_COST = 255.0;
const float OCCUPIED_COST = 254.0;
const float INSCRIBED_COST = 253.0;
const float MAX_NON_OBSTACLE_COST = 252.0;
const float FREE_COST = 0.0;

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__CORE__CONSTANTS_HPP_
