// Lattice Planner - Core Geometry Types
// Replaces geometry_msgs, nav_msgs, tf2 types with pure C++ structs.

#ifndef LATTICE_PLANNER__CORE__TYPES_HPP_
#define LATTICE_PLANNER__CORE__TYPES_HPP_

#include <cmath>
#include <vector>
#include <string>

namespace lattice_planner {

/** @brief 2D point (double precision, world coordinates). */
struct Point2D {
  double x{0.0};
  double y{0.0};
  Point2D() = default;
  Point2D(double x_in, double y_in) : x(x_in), y(y_in) {}

  inline bool operator==(const Point2D & rhs) const {
    return x == rhs.x && y == rhs.y;
  }
  inline bool operator!=(const Point2D & rhs) const {
    return !(*this == rhs);
  }
};

/** @brief 2D pose with heading (world coordinates, theta in radians). */
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  Pose2D() = default;
  Pose2D(double x_in, double y_in, double theta_in)
    : x(x_in), y(y_in), theta(theta_in) {}
};

/** @brief Path as a sequence of poses. */
struct Path {
  std::vector<Pose2D> poses;
  std::string frame_id;  // optional, for user convenience
};

/** @brief Search-space coordinates (float, grid cell units). */
struct Coordinates {
  float x{0.0f};
  float y{0.0f};
  float theta{0.0f};
  Coordinates() = default;
  Coordinates(float x_in, float y_in, float theta_in)
    : x(x_in), y(y_in), theta(theta_in) {}

  inline bool operator==(const Coordinates & rhs) const {
    return x == rhs.x && y == rhs.y && theta == rhs.theta;
  }
  inline bool operator!=(const Coordinates & rhs) const {
    return !(*this == rhs);
  }
};

/** @brief 2D search-space coordinates (float, grid cell units). */
struct Coordinates2D {
  float x{0.0f};
  float y{0.0f};
  Coordinates2D() = default;
  Coordinates2D(float x_in, float y_in) : x(x_in), y(y_in) {}

  inline bool operator==(const Coordinates2D & rhs) const {
    return x == rhs.x && y == rhs.y;
  }
  inline bool operator!=(const Coordinates2D & rhs) const {
    return !(*this == rhs);
  }
};

typedef std::vector<Coordinates> CoordinateVector;

/**
 * @brief Template forward declaration for goal state tracking.
 * @tparam NodeT Node type (e.g. NodeLattice).
 */
template<typename NodeT>
struct GoalState {
  NodeT * goal{nullptr};
  bool is_valid{true};
};

// ---- Angle / quaternion utilities (replace tf2 + angles) ----

/** @brief Normalize angle to [0, 2*PI). */
inline double normalizeAngle(double angle) {
  double a = std::fmod(angle, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a;
}

/** @brief Shortest signed angular distance from a to b, result in [-PI, PI].
 *  Replaces angles::shortest_angular_distance. */
inline double shortestAngularDistance(double a, double b) {
  double diff = b - a;
  while (diff > M_PI)  diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  return diff;
}

/** @brief Convert yaw (radians) to quaternion (z, w components only, x=y=0).
 *  Replaces tf2::Quaternion::setRPY + tf2::toMsg for planar robots. */
inline void yawToQuaternion(double theta, double & qz, double & qw) {
  qw = std::cos(theta * 0.5);
  qz = std::sin(theta * 0.5);
}

/** @brief Convert quaternion (z, w) to yaw angle in radians.
 *  Replaces tf2::getYaw. */
inline double quaternionToYaw(double qz, double qw) {
  return std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * qz * qz);
}

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__CORE__TYPES_HPP_
