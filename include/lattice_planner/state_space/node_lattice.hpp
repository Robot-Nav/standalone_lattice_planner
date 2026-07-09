// Lattice Planner - NodeLattice and LatticeMotionTable
// Ported from nav2_smac_planner/node_lattice.hpp with no ROS dependencies.

#ifndef LATTICE_PLANNER__STATE_SPACE__NODE_LATTICE_HPP_
#define LATTICE_PLANNER__STATE_SPACE__NODE_LATTICE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "ompl/base/StateSpace.h"

#include "lattice_planner/core/constants.hpp"
#include "lattice_planner/core/config.hpp"
#include "lattice_planner/core/types.hpp"
#include "lattice_planner/collision/collision_checker.hpp"
#include "lattice_planner/state_space/motion_primitives.hpp"

namespace lattice_planner {

// Forward declarations
class ObstacleHeuristic;
template<typename NodeT> class DistanceHeuristic;
class NodeLattice;

/**
 * @struct LatticeMotionTable
 * @brief A table of motion primitives and related functions.
 */
struct LatticeMotionTable {
  LatticeMotionTable() = default;

  void initMotionModel(unsigned int & size_x_in, SearchInfo & search_info);

  MotionPrimitivePtrs getMotionPrimitives(
    const NodeLattice * node,
    unsigned int & direction_change_index);

  static LatticeMetadata getLatticeMetadata(const std::string & lattice_filepath);

  unsigned int getClosestAngularBin(const double & theta);

  float & getAngleFromBin(const unsigned int & bin_idx);

  double getAngle(const double & theta);

  unsigned int size_x{0};
  unsigned int num_angle_quantization{0};
  float change_penalty{0.0f};
  float non_straight_penalty{1.0f};
  float cost_penalty{1.0f};
  float reverse_penalty{1.0f};
  float travel_distance_reward{1.0f};
  float rotation_penalty{1.0f};
  float min_turning_radius{0.0f};
  bool allow_reverse_expansion{false};
  bool downsample_obstacle_heuristic{true};
  bool use_quadratic_cost_penalty{false};
  std::vector<std::vector<MotionPrimitive>> motion_primitives;
  ompl::base::StateSpacePtr state_space;
  std::vector<TrigValues> trig_values;
  std::string current_lattice_filepath;
  LatticeMetadata lattice_metadata;
  MotionModel motion_model{MotionModel::UNKNOWN};
};

/**
 * @class NodeLattice
 * @brief NodeLattice implementation for State Lattice A* search.
 */
class NodeLattice {
public:
  typedef NodeLattice * NodePtr;
  typedef std::unique_ptr<std::vector<NodeLattice>> Graph;
  typedef std::vector<NodePtr> NodeVector;
  using Coordinates = ::lattice_planner::Coordinates;
  using CoordinateVector = ::lattice_planner::CoordinateVector;

  /**
   * @struct NodeContext
   * @brief Shared context containing motion table and heuristics.
   *        Constructor/destructor defined in .cpp to avoid circular includes.
   */
  struct NodeContext {
    NodeContext();
    ~NodeContext();

    LatticeMotionTable motion_table;
    std::unique_ptr<ObstacleHeuristic> obstacle_heuristic;
    std::unique_ptr<DistanceHeuristic<NodeLattice>> distance_heuristic;
  };

  explicit NodeLattice(const uint64_t index, NodeContext * ctx);

  ~NodeLattice();

  bool operator==(const NodeLattice & rhs) const {
    return this->_index == rhs._index;
  }

  inline void setPose(const Coordinates & pose_in) { pose = pose_in; }

  void reset();

  inline void setMotionPrimitive(MotionPrimitive * prim) { _motion_primitive = prim; }

  inline MotionPrimitive * & getMotionPrimitive() { return _motion_primitive; }

  inline float getAccumulatedCost() { return _accumulated_cost; }

  inline void setAccumulatedCost(const float & cost_in) { _accumulated_cost = cost_in; }

  inline float getCost() { return _cell_cost; }

  inline bool wasVisited() { return _was_visited; }

  inline void visited() { _was_visited = true; }

  inline uint64_t getIndex() { return _index; }

  inline void backwards(bool back = true) { _backwards = back; }

  inline bool isBackward() { return _backwards; }

  bool isNodeValid(
    const bool & traverse_unknown,
    GridCollisionChecker * collision_checker,
    MotionPrimitive * primitive = nullptr,
    bool is_backwards = false);

  float getTraversalCost(const NodePtr & child);

  /**
   * @brief Get index at coordinates.
   * Index = theta + x * num_theta + y * width * num_theta
   */
  static inline uint64_t getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & angle,
    const unsigned int & width, const unsigned int & angle_quantization)
  {
    return angle + x * angle_quantization + y * angle_quantization * width;
  }

  /**
   * @brief Get coordinates at index.
   */
  static inline Coordinates getCoords(
    const uint64_t & index,
    const unsigned int & width, const unsigned int & angle_quantization)
  {
    return Coordinates(
      (index / angle_quantization) % width,    // x
      index / (angle_quantization * width),    // y
      index % angle_quantization);             // theta
  }

  float getHeuristicCost(
    const Coordinates & node_coords,
    const CoordinateVector & goals_coords);

  static void initMotionModel(
    NodeContext * ctx,
    const MotionModel & motion_model,
    unsigned int & size_x,
    unsigned int & size_y,
    unsigned int & angle_quantization,
    SearchInfo & search_info);

  void getNeighbors(
    std::function<bool(const uint64_t &, NodeLattice * &)> & validity_checker,
    GridCollisionChecker * collision_checker,
    const bool & traverse_unknown,
    NodeVector & neighbors);

  bool backtracePath(CoordinateVector & path);

  void addNodeToPath(NodePtr current_node, CoordinateVector & path);

  NodeLattice * parent{nullptr};
  Coordinates pose;

private:
  float _cell_cost;
  float _accumulated_cost;
  uint64_t _index;
  bool _was_visited;
  MotionPrimitive * _motion_primitive;
  bool _backwards;
  bool _is_node_valid{false};
  NodeContext * _ctx{nullptr};
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__STATE_SPACE__NODE_LATTICE_HPP_
