// Lattice Planner - A* Search Algorithm
// Ported from nav2_smac_planner/a_star.hpp with ROS dependencies removed.

#ifndef LATTICE_PLANNER__SEARCH__A_STAR_HPP_
#define LATTICE_PLANNER__SEARCH__A_STAR_HPP_

#include <functional>
#include <memory>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#include "robin_hood.h"

#include "lattice_planner/costmap/costmap_2d.hpp"
#include "lattice_planner/core/exceptions.hpp"
#include "lattice_planner/core/config.hpp"
#include "lattice_planner/core/constants.hpp"
#include "lattice_planner/core/types.hpp"
#include "lattice_planner/state_space/node_lattice.hpp"
#include "lattice_planner/collision/collision_checker.hpp"
#include "lattice_planner/heuristic/obstacle_heuristic.hpp"
#include "lattice_planner/heuristic/distance_heuristic.hpp"
#include "lattice_planner/search/analytic_expansion.hpp"
#include "lattice_planner/search/node_basic.hpp"
#include "lattice_planner/search/goal_manager.hpp"

namespace lattice_planner {

/**
 * @class AStarAlgorithm
 * @brief A* search implementation templated on node type.
 *        Uses a priority queue, double heuristic (obstacle wavefront +
 *        SE2 distance), and analytic expansion via OMPL.
 */
template<typename NodeT>
class AStarAlgorithm
{
public:
  typedef NodeT * NodePtr;
  typedef robin_hood::unordered_node_map<uint64_t, NodeT> Graph;
  typedef std::vector<NodePtr> NodeVector;
  typedef std::pair<float, NodeBasic<NodeT>> NodeElement;
  typedef typename NodeT::Coordinates Coordinates;
  typedef typename NodeT::CoordinateVector CoordinateVector;
  typedef typename NodeVector::iterator NeighborIterator;
  typedef std::function<bool(const uint64_t &, NodeT *&)> NodeGetter;
  typedef GoalManager<NodeT> GoalManagerT;
  using NodeContext = typename NodeT::NodeContext;

  /**
   * @struct NodeComparator
   * @brief Min-heap comparator: lower f-cost has higher priority.
   */
  struct NodeComparator
  {
    bool operator()(const NodeElement & a, const NodeElement & b) const
    {
      return a.first > b.first;
    }
  };

  typedef std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator> NodeQueue;

  explicit AStarAlgorithm(const MotionModel & motion_model, const SearchInfo & search_info);
  ~AStarAlgorithm();

  /**
   * @brief Initialize the planner.
   * @param allow_unknown Allow search through unknown cells.
   * @param max_iterations Maximum search iterations.
   * @param max_on_approach_iterations Max iterations once within tolerance.
   * @param terminal_checking_interval Check timeout/cancel every N iterations.
   * @param max_planning_time Maximum planning time in seconds.
   * @param lookup_table_size Size of the distance heuristic lookup table.
   * @param dim_3_size Number of angle quantization bins.
   */
  void initialize(
    const bool & allow_unknown,
    int & max_iterations,
    const int & max_on_approach_iterations,
    const int & terminal_checking_interval,
    const double & max_planning_time,
    const float & lookup_table_size,
    const unsigned int & dim_3_size);

  /**
   * @brief Create a path from start to goal.
   * @param path Output: vector of coordinates.
   * @param iterations Output: number of iterations used.
   * @param tolerance Tolerance in cell units for goal approach.
   * @param cancel_checker Function returning true if planning should cancel.
   * @param expansions_log Optional debug log of expanded nodes.
   * @return true if path found.
   */
  bool createPath(
    CoordinateVector & path, int & iterations, const float & tolerance,
    std::function<bool()> cancel_checker,
    std::vector<std::tuple<float, float, float>> * expansions_log = nullptr);

  void setSearchInfo(const SearchInfo & search_info) {_search_info = search_info;}

  void setCollisionChecker(GridCollisionChecker * collision_checker);

  /**
   * @brief Set the goal position and heading.
   * @param mx X cell coordinate (float for sub-cell precision).
   * @param my Y cell coordinate.
   * @param dim_3 Angle bin index.
   * @param goal_heading_mode Heading constraint mode.
   * @param coarse_search_resolution Goal heading search resolution.
   */
  void setGoal(
    const float & mx,
    const float & my,
    const unsigned int & dim_3,
    const GoalHeadingMode & goal_heading_mode = GoalHeadingMode::DEFAULT,
    const int & coarse_search_resolution = 1);

  /**
   * @brief Set the start position and heading.
   */
  void setStart(
    const float & mx,
    const float & my,
    const unsigned int & dim_3);

  int & getMaxIterations();
  NodePtr & getStart();
  int & getOnApproachMaxIterations();
  float & getToleranceHeuristic();
  unsigned int & getSizeX();
  unsigned int & getSizeY();
  unsigned int & getSizeDim3();
  unsigned int getCoarseSearchResolution();
  GoalManagerT getGoalManager();
  NodeContext * getContext();

protected:
  inline NodePtr getNextNode();
  inline void addNode(const float & cost, NodePtr & node);
  inline NodePtr addToGraph(const uint64_t & index);
  inline float getHeuristicCost(const NodePtr & node);
  inline bool areInputsValid();
  inline bool getClosestPathWithinTolerance(CoordinateVector & path);
  inline void clearQueue();
  inline void clearGraph();
  inline uint64_t getIndex(
    const unsigned int & x, const unsigned int & y, const unsigned int & dim3);
  inline bool onVisitationCheckNode(const NodePtr & node);
  inline void populateExpansionsLog(
    const NodePtr & node, std::vector<std::tuple<float, float, float>> * expansions_log);

  bool _traverse_unknown;
  bool _is_initialized;
  int _max_iterations;
  int _max_on_approach_iterations;
  int _terminal_checking_interval;
  double _max_planning_time;
  float _tolerance;
  unsigned int _x_size;
  unsigned int _y_size;
  unsigned int _dim3_size;
  unsigned int _coarse_search_resolution;
  SearchInfo _search_info;

  NodePtr _start;
  GoalManagerT _goal_manager;
  Graph _graph;
  NodeQueue _queue;

  MotionModel _motion_model;
  NodeHeuristicPair _best_heuristic_node;

  GridCollisionChecker * _collision_checker;
  Costmap2D * _costmap;
  std::unique_ptr<AnalyticExpansion<NodeT>> _expander;
  std::shared_ptr<NodeContext> _shared_ctx;
};

}  // namespace lattice_planner

#include "lattice_planner/search/a_star_impl.hpp"  // NOLINT

#endif  // LATTICE_PLANNER__SEARCH__A_STAR_HPP_
