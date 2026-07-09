// Lattice Planner - NodeBasic for priority queue insertion
// Ported from nav2_smac_planner/node_basic.hpp with Node2D/NodeHybrid branches removed.

#ifndef LATTICE_PLANNER__SEARCH__NODE_BASIC_HPP_
#define LATTICE_PLANNER__SEARCH__NODE_BASIC_HPP_

#include <cstdint>

#include "lattice_planner/state_space/node_lattice.hpp"
#include "lattice_planner/state_space/motion_primitives.hpp"
#include "lattice_planner/core/types.hpp"

namespace lattice_planner {

/**
 * @class NodeBasic
 * @brief NodeBasic implementation for priority queue insertion.
 *        Caches search-node state so it can be restored if a lower-cost
 *        branch overwrites a queued-but-unvisited node.
 */
template<typename NodeT>
class NodeBasic
{
public:
  typedef NodeT * NodePtr;
  typedef typename NodeT::Coordinates Coordinates;

  explicit NodeBasic(const uint64_t new_index)
  : graph_node_ptr(nullptr),
    index(new_index),
    prim_ptr(nullptr),
    backward(false)
  {
  }

  /**
   * @brief Cache state from a NodeT into this NodeBasic for queue storage.
   * @param node NodeT ptr to populate metadata from.
   */
  void populateSearchNode(NodeT * & node)
  {
    this->pose = node->pose;
    this->graph_node_ptr = node;
    this->prim_ptr = node->getMotionPrimitive();
    this->backward = node->isBackward();
  }

  /**
   * @brief Restore cached state into the internal graph node, but only
   *        if it has not yet been visited. This prevents a queued-but-
   *        superseded branch from clobbering a lower-cost visited node.
   */
  void processSearchNode()
  {
    if (!this->graph_node_ptr->wasVisited()) {
      this->graph_node_ptr->pose = this->pose;
      this->graph_node_ptr->setMotionPrimitive(this->prim_ptr);
      this->graph_node_ptr->backwards(this->backward);
    }
  }

  Coordinates pose;
  NodeT * graph_node_ptr;
  uint64_t index;
  MotionPrimitive * prim_ptr;
  bool backward;
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__SEARCH__NODE_BASIC_HPP_
