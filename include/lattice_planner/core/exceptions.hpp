// Lattice Planner - Exception Hierarchy
// Replaces nav2_core planner exceptions with pure C++ equivalents.

#ifndef LATTICE_PLANNER__CORE__EXCEPTIONS_HPP_
#define LATTICE_PLANNER__CORE__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace lattice_planner {

/** @brief Base class for all planner exceptions. Replaces nav2_core::PlannerException. */
class PlannerException : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

/** @brief Start pose is outside costmap bounds. */
class StartOutsideMapBounds : public PlannerException {
public:
  using PlannerException::PlannerException;
};

/** @brief Goal pose is outside costmap bounds. */
class GoalOutsideMapBounds : public PlannerException {
public:
  using PlannerException::PlannerException;
};

/** @brief Start cell is occupied by an obstacle. */
class StartOccupied : public PlannerException {
public:
  using PlannerException::PlannerException;
};

/** @brief Goal cell is occupied by an obstacle. */
class GoalOccupied : public PlannerException {
public:
  using PlannerException::PlannerException;
};

/** @brief No valid path could be found within iteration limit. */
class NoValidPathCouldBeFound : public PlannerException {
public:
  using PlannerException::PlannerException;
};

/** @brief Planner exceeded maximum allowed time. */
class PlannerTimedOut : public PlannerException {
public:
  using PlannerException::PlannerException;
};

/** @brief Planning was cancelled by the cancel_checker callback. */
class PlannerCancelled : public PlannerException {
public:
  using PlannerException::PlannerException;
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__CORE__EXCEPTIONS_HPP_
