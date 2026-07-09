// Lattice Planner - Analytic Expansion
// Ported from nav2_smac_planner/analytic_expansion.hpp with Node2D/NodeHybrid removed.

#ifndef LATTICE_PLANNER__SEARCH__ANALYTIC_EXPANSION_HPP_
#define LATTICE_PLANNER__SEARCH__ANALYTIC_EXPANSION_HPP_

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <functional>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "lattice_planner/state_space/node_lattice.hpp"
#include "lattice_planner/collision/collision_checker.hpp"
#include "lattice_planner/core/config.hpp"
#include "lattice_planner/core/constants.hpp"
#include "lattice_planner/core/types.hpp"

namespace lattice_planner {

/**
 * @class AnalyticExpansion
 * @brief Attempts analytic path expansions (Dubins/Reeds-Shepp) from a node
 *        to the goal during A* search, providing shortcut paths when possible.
 */
template<typename NodeT>
class AnalyticExpansion
{
public:
  typedef NodeT * NodePtr;
  typedef std::vector<NodePtr> NodeVector;
  typedef typename NodeT::Coordinates Coordinates;
  typedef std::function<bool(const uint64_t &, NodeT *&)> NodeGetter;
  typedef typename NodeT::CoordinateVector CoordinateVector;
  using NodeContext = typename NodeT::NodeContext;

  /**
   * @struct AnalyticExpansionNode
   * @brief A single node in an analytic expansion, with initial and proposed coords.
   */
  struct AnalyticExpansionNode
  {
    AnalyticExpansionNode(
      NodePtr & node_in,
      Coordinates & initial_coords_in,
      Coordinates & proposed_coords_in)
    : node(node_in),
      initial_coords(initial_coords_in),
      proposed_coords(proposed_coords_in)
    {
    }

    NodePtr node;
    Coordinates initial_coords;
    Coordinates proposed_coords;
  };

  /**
   * @struct AnalyticExpansionNodes
   * @brief Collection of analytic expansion nodes and direction change count.
   */
  struct AnalyticExpansionNodes
  {
    AnalyticExpansionNodes() = default;

    void add(
      NodePtr & node,
      Coordinates & initial_coords,
      Coordinates & proposed_coords)
    {
      nodes.emplace_back(node, initial_coords, proposed_coords);
    }

    void setDirectionChanges(int changes)
    {
      direction_changes = changes;
    }

    std::vector<AnalyticExpansionNode> nodes;
    int direction_changes{0};
  };

  AnalyticExpansion(
    const MotionModel & motion_model,
    const SearchInfo & search_info,
    const bool & traverse_unknown,
    const unsigned int & dim_3_size);

  void setCollisionChecker(GridCollisionChecker * collision_checker);

  void setContext(NodeContext * ctx);

  /**
   * @brief Attempt an analytic path completion to the goal.
   * @param current_node Node to expand from.
   * @param coarse_check_goals Coarse goal list.
   * @param fine_check_goals Fine goal list.
   * @param goals_coords Goal coordinates.
   * @param getter Function to get a node by index.
   * @param iterations Ref: analytic iteration counter.
   * @param closest_distance Ref: closest distance to goal so far.
   * @return Goal node pointer if successful, nullptr otherwise.
   */
  NodePtr tryAnalyticExpansion(
    const NodePtr & current_node,
    const NodeVector & coarse_check_goals,
    const NodeVector & fine_check_goals,
    const CoordinateVector & goals_coords,
    const NodeGetter & getter, int & iterations,
    int & closest_distance);

  /**
   * @brief Compute an analytic path from node to goal via OMPL interpolation.
   */
  AnalyticExpansionNodes getAnalyticPath(
    const NodePtr & node, const NodePtr & goal,
    const NodeGetter & getter, const ompl::base::StateSpacePtr & state_space);

  /**
   * @brief Refine an analytic path by trying different start points and turning radii.
   * @return Score of the best refined path.
   */
  float refineAnalyticPath(
    NodePtr & node,
    const NodePtr & goal_node,
    const NodeGetter & getter,
    AnalyticExpansionNodes & analytic_nodes);

  /**
   * @brief Set the analytic path nodes into the search graph (parent links, poses).
   * @return Goal node pointer if successful, nullptr otherwise.
   */
  NodePtr setAnalyticPath(
    const NodePtr & node, const NodePtr & goal,
    const AnalyticExpansionNodes & expanded_nodes);

  /**
   * @brief Count direction changes in a Reeds-Shepp path.
   */
  int countDirectionChanges(const ompl::base::ReedsSheppStateSpace::ReedsSheppPath & path);

  /**
   * @brief Clean up a node's state after a failed or completed expansion.
   */
  void cleanNode(const NodePtr & node);

protected:
  MotionModel _motion_model;
  SearchInfo _search_info;
  bool _traverse_unknown;
  unsigned int _dim_3_size;
  GridCollisionChecker * _collision_checker;
  std::list<std::unique_ptr<NodeT>> _detached_nodes;
  NodeContext * _ctx = nullptr;
};

}  // namespace lattice_planner

#include "lattice_planner/search/analytic_expansion_impl.hpp"  // NOLINT

#endif  // LATTICE_PLANNER__SEARCH__ANALYTIC_EXPANSION_HPP_
