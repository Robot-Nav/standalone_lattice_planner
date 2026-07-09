// Lattice Planner - GoalManager
// Ported from nav2_smac_planner/goal_manager.hpp with Node2D branches removed.

#ifndef LATTICE_PLANNER__SEARCH__GOAL_MANAGER_HPP_
#define LATTICE_PLANNER__SEARCH__GOAL_MANAGER_HPP_

#include <algorithm>
#include <functional>
#include <unordered_set>
#include <vector>

#include "lattice_planner/core/types.hpp"
#include "lattice_planner/state_space/node_lattice.hpp"
#include "lattice_planner/collision/collision_checker.hpp"

namespace lattice_planner {

/**
 * @class GoalManager
 * @brief Manages goal state, goal sets, and goal coordinates for A* search.
 *        Supports multiple goals, analytic expansion preparation, and
 *        zone validity checking within a tolerance radius.
 */
template<typename NodeT>
class GoalManager
{
public:
  typedef NodeT * NodePtr;
  typedef std::vector<NodePtr> NodeVector;
  typedef std::unordered_set<NodePtr> NodeSet;
  typedef std::vector<GoalState<NodeT>> GoalStateVector;
  typedef typename NodeT::Coordinates Coordinates;
  typedef typename NodeT::CoordinateVector CoordinateVector;
  using NodeContext = typename NodeT::NodeContext;

  GoalManager()
  : _goals_set(NodeSet()),
    _goals_state(GoalStateVector()),
    _goals_coordinate(CoordinateVector()),
    _ref_goal_coord(Coordinates())
  {
  }

  ~GoalManager() = default;

  void setContext(NodeContext * ctx)
  {
    _ctx = ctx;
  }

  bool goalsIsEmpty()
  {
    return _goals_state.empty();
  }

  void addGoal(NodePtr & goal)
  {
    _goals_state.push_back({goal, true});
  }

  void clear()
  {
    _goals_set.clear();
    _goals_state.clear();
    _goals_coordinate.clear();
  }

  /**
   * @brief Split valid goals into coarse and fine lists for analytic expansion.
   * @param coarse_check_goals Output: every coarse_search_resolution-th valid goal.
   * @param fine_check_goals Output: remaining valid goals.
   * @param coarse_search_resolution Number of fine goals per coarse goal.
   */
  void prepareGoalsForAnalyticExpansion(
    NodeVector & coarse_check_goals, NodeVector & fine_check_goals,
    int coarse_search_resolution)
  {
    for (unsigned int i = 0; i < _goals_state.size(); i++) {
      if (_goals_state[i].is_valid) {
        if (i % coarse_search_resolution == 0) {
          coarse_check_goals.push_back(_goals_state[i].goal);
        } else {
          fine_check_goals.push_back(_goals_state[i].goal);
        }
      }
    }
  }

  /**
   * @brief Check if there is at least one non-lethal cell within radius of node.
   * @param node Center node.
   * @param radius Search radius in cell units.
   * @param collision_checker Collision checker.
   * @param traverse_unknown Whether unknown space is traversable.
   * @return true if at least one valid cell found.
   */
  bool isZoneValid(
    const NodePtr node, const float & radius, GridCollisionChecker * collision_checker,
    const bool & traverse_unknown) const
  {
    if (radius < 1) {
      return false;
    }

    const auto size_x = collision_checker->getCostmap()->getSizeInCellsX();
    const auto size_y = collision_checker->getCostmap()->getSizeInCellsY();

    auto getIndexFromPoint = [this](const Coordinates & point) {
        const auto mx = static_cast<unsigned int>(point.x);
        const auto my = static_cast<unsigned int>(point.y);
        const auto angle = static_cast<unsigned int>(point.theta);
        return NodeT::getIndex(mx, my, angle, _ctx->motion_table.size_x,
                               _ctx->motion_table.num_angle_quantization);
      };

    const Coordinates & center_point = node->pose;
    const float min_x = std::max(0.0f, std::floor(center_point.x - radius));
    const float min_y = std::max(0.0f, std::floor(center_point.y - radius));
    const float max_x =
      std::min(static_cast<float>(size_x - 1), std::ceil(center_point.x + radius));
    const float max_y =
      std::min(static_cast<float>(size_y - 1), std::ceil(center_point.y + radius));
    const float radius_sq = radius * radius;

    Coordinates m;
    for (m.x = min_x; m.x <= max_x; m.x += 1.0f) {
      for (m.y = min_y; m.y <= max_y; m.y += 1.0f) {
        const float dx = m.x - center_point.x;
        const float dy = m.y - center_point.y;

        if (dx * dx + dy * dy > radius_sq) {
          continue;
        }

        NodeT current_node(getIndexFromPoint(m), _ctx);
        current_node.setPose(m);

        if (current_node.isNodeValid(traverse_unknown, collision_checker)) {
          return true;
        }
      }
    }

    return false;
  }

  /**
   * @brief Filter goals: keep those that are directly valid or have a valid zone
   *        within tolerance. Store valid goals into the internal set and
   *        coordinate list.
   * @param tolerance Tolerance radius in cell units.
   * @param collision_checker Collision checker.
   * @param traverse_unknown Whether unknown space is traversable.
   */
  void removeInvalidGoals(
    const float & tolerance,
    GridCollisionChecker * collision_checker,
    const bool & traverse_unknown)
  {
    if (!_goals_set.empty() || !_goals_coordinate.empty()) {
      throw std::runtime_error(
              "Goal set should be cleared before calling removeInvalidGoals");
    }
    for (unsigned int i = 0; i < _goals_state.size(); i++) {
      if (_goals_state[i].goal->isNodeValid(traverse_unknown, collision_checker) ||
        isZoneValid(_goals_state[i].goal, tolerance, collision_checker, traverse_unknown))
      {
        _goals_state[i].is_valid = true;
        _goals_set.insert(_goals_state[i].goal);
        _goals_coordinate.push_back(_goals_state[i].goal->pose);
      } else {
        _goals_state[i].is_valid = false;
      }
    }
  }

  inline bool isGoal(const NodePtr & node)
  {
    return _goals_set.find(node) != _goals_set.end();
  }

  inline NodeSet & getGoalsSet()
  {
    return _goals_set;
  }

  inline GoalStateVector & getGoalsState()
  {
    return _goals_state;
  }

  inline CoordinateVector & getGoalsCoordinates()
  {
    return _goals_coordinate;
  }

  inline void setRefGoalCoordinates(const Coordinates & coord)
  {
    _ref_goal_coord = coord;
  }

  inline bool hasGoalChanged(const Coordinates & coord)
  {
    return _ref_goal_coord != coord;
  }

protected:
  NodeSet _goals_set;
  GoalStateVector _goals_state;
  CoordinateVector _goals_coordinate;
  Coordinates _ref_goal_coord;
  NodeContext * _ctx = nullptr;
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__SEARCH__GOAL_MANAGER_HPP_
