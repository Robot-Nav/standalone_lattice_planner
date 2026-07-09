// Lattice Planner - NodeLattice Implementation
// Ported from nav2_smac_planner/src/node_lattice.cpp with no ROS dependencies.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"

#include "lattice_planner/state_space/node_lattice.hpp"
#include "lattice_planner/heuristic/obstacle_heuristic.hpp"
#include "lattice_planner/heuristic/distance_heuristic.hpp"
#include "lattice_planner/core/logger.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lattice_planner {

// ---- NodeContext implementation (requires complete heuristic types) ----

NodeLattice::NodeContext::NodeContext()
{
  obstacle_heuristic = std::make_unique<ObstacleHeuristic>();
  distance_heuristic = std::make_unique<DistanceHeuristic<NodeLattice>>();
}

NodeLattice::NodeContext::~NodeContext() = default;

// ---- LatticeMotionTable implementation ----

void LatticeMotionTable::initMotionModel(
  unsigned int & size_x_in,
  SearchInfo & search_info)
{
  size_x = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  allow_reverse_expansion = search_info.allow_reverse_expansion;
  rotation_penalty = search_info.rotation_penalty;
  min_turning_radius = search_info.minimum_turning_radius;
  downsample_obstacle_heuristic = search_info.downsample_obstacle_heuristic;
  use_quadratic_cost_penalty = search_info.use_quadratic_cost_penalty;

  if (current_lattice_filepath == search_info.lattice_filepath) {
    return;
  }
  current_lattice_filepath = search_info.lattice_filepath;

  lattice_metadata = getLatticeMetadata(current_lattice_filepath);
  std::ifstream latticeFile(current_lattice_filepath);
  if (!latticeFile.is_open()) {
    throw std::runtime_error("Could not open lattice file");
  }
  nlohmann::json json;
  latticeFile >> json;
  num_angle_quantization = lattice_metadata.number_of_headings;

  if (!state_space) {
    if (lattice_metadata.motion_model == "omni") {
      state_space = std::make_shared<ompl::base::SE2StateSpace>();
      motion_model = MotionModel::OMNI;
    } else if (!allow_reverse_expansion) {
      state_space = std::make_shared<ompl::base::DubinsStateSpace>(
        lattice_metadata.min_turning_radius);
      motion_model = MotionModel::DUBIN;
    } else {
      state_space = std::make_shared<ompl::base::ReedsSheppStateSpace>(
        lattice_metadata.min_turning_radius);
      motion_model = MotionModel::REEDS_SHEPP;
    }
  }

  float prev_start_angle = 0.0f;
  std::vector<MotionPrimitive> primitives;
  nlohmann::json json_primitives = json["primitives"];
  for (unsigned int i = 0; i < json_primitives.size(); ++i) {
    MotionPrimitive new_primitive;
    fromJsonToMotionPrimitive(json_primitives[i], new_primitive);

    if (prev_start_angle != new_primitive.start_angle) {
      motion_primitives.push_back(primitives);
      primitives.clear();
      prev_start_angle = new_primitive.start_angle;
    }
    primitives.push_back(new_primitive);
  }
  motion_primitives.push_back(primitives);

  trig_values.reserve(lattice_metadata.number_of_headings);
  for (unsigned int i = 0; i < lattice_metadata.heading_angles.size(); ++i) {
    trig_values.emplace_back(
      cos(lattice_metadata.heading_angles[i]),
      sin(lattice_metadata.heading_angles[i]));
  }
}

MotionPrimitivePtrs LatticeMotionTable::getMotionPrimitives(
  const NodeLattice * node,
  unsigned int & direction_change_index)
{
  MotionPrimitives & prims_at_heading = motion_primitives[node->pose.theta];
  MotionPrimitivePtrs primitive_projection_list;
  for (unsigned int i = 0; i != prims_at_heading.size(); ++i) {
    primitive_projection_list.push_back(&prims_at_heading[i]);
  }

  direction_change_index = static_cast<unsigned int>(primitive_projection_list.size());

  if (allow_reverse_expansion) {
    double reserve_heading = node->pose.theta - (num_angle_quantization / 2.0);
    if (reserve_heading < 0) {
      reserve_heading += num_angle_quantization;
    }
    if (reserve_heading > num_angle_quantization) {
      reserve_heading -= num_angle_quantization;
    }

    MotionPrimitives & prims_at_reverse_heading = motion_primitives[reserve_heading];
    for (unsigned int i = 0; i != prims_at_reverse_heading.size(); ++i) {
      primitive_projection_list.push_back(&prims_at_reverse_heading[i]);
    }
  }

  return primitive_projection_list;
}

LatticeMetadata LatticeMotionTable::getLatticeMetadata(const std::string & lattice_filepath)
{
  std::ifstream lattice_file(lattice_filepath);
  if (!lattice_file.is_open()) {
    throw std::runtime_error("Could not open lattice file!");
  }

  nlohmann::json j;
  lattice_file >> j;
  LatticeMetadata metadata;
  fromJsonToMetaData(j["lattice_metadata"], metadata);
  return metadata;
}

unsigned int LatticeMotionTable::getClosestAngularBin(const double & theta)
{
  float min_dist = std::numeric_limits<float>::max();
  unsigned int closest_idx = 0;
  for (unsigned int i = 0; i != lattice_metadata.heading_angles.size(); ++i) {
    float dist = static_cast<float>(std::fabs(
      shortestAngularDistance(theta, lattice_metadata.heading_angles[i])));
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

float & LatticeMotionTable::getAngleFromBin(const unsigned int & bin_idx)
{
  return lattice_metadata.heading_angles[bin_idx];
}

double LatticeMotionTable::getAngle(const double & theta)
{
  return getClosestAngularBin(theta);
}

// ---- NodeLattice implementation ----

NodeLattice::NodeLattice(const uint64_t index, NodeContext * ctx)
: parent(nullptr),
  pose(0.0f, 0.0f, 0.0f),
  _cell_cost(std::numeric_limits<float>::quiet_NaN()),
  _accumulated_cost(std::numeric_limits<float>::max()),
  _index(index),
  _was_visited(false),
  _motion_primitive(nullptr),
  _backwards(false),
  _is_node_valid(false),
  _ctx(ctx)
{
}

NodeLattice::~NodeLattice()
{
  parent = nullptr;
}

void NodeLattice::reset()
{
  parent = nullptr;
  _cell_cost = std::numeric_limits<float>::quiet_NaN();
  _accumulated_cost = std::numeric_limits<float>::max();
  _was_visited = false;
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;
  _motion_primitive = nullptr;
  _backwards = false;
  _is_node_valid = false;
}

bool NodeLattice::isNodeValid(
  const bool & traverse_unknown,
  GridCollisionChecker * collision_checker,
  MotionPrimitive * motion_primitive,
  bool is_backwards)
{
  if (!std::isnan(_cell_cost)) {
    return _is_node_valid;
  }

  const double bin_size = 2.0 * M_PI / collision_checker->getPrecomputedAngles().size();
  const double angle = std::fmod(
    _ctx->motion_table.getAngleFromBin(this->pose.theta),
    2.0 * M_PI) / bin_size;
  if (collision_checker->inCollision(
      this->pose.x, this->pose.y, angle, traverse_unknown))
  {
    _is_node_valid = false;
    _cell_cost = collision_checker->getCost();
    return false;
  }

  float max_cell_cost = collision_checker->getCost();

  if (motion_primitive) {
    const float & grid_resolution = _ctx->motion_table.lattice_metadata.grid_resolution;
    const float resolution_diag_sq = 2.0f * grid_resolution * grid_resolution;
    MotionPose last_pose(1e9, 1e9, 1e9, TurnDirection::UNKNOWN);
    MotionPose pose_dist(0.0f, 0.0f, 0.0f, TurnDirection::UNKNOWN);

    MotionPose initial_pose, prim_pose;
    initial_pose._x = this->pose.x - (motion_primitive->poses.back()._x / grid_resolution);
    initial_pose._y = this->pose.y - (motion_primitive->poses.back()._y / grid_resolution);
    initial_pose._theta = _ctx->motion_table.getAngleFromBin(motion_primitive->start_angle);

    for (auto it = motion_primitive->poses.begin(); it != motion_primitive->poses.end(); ++it) {
      pose_dist = *it - last_pose;
      if (pose_dist._x * pose_dist._x + pose_dist._y * pose_dist._y > resolution_diag_sq) {
        last_pose = *it;
        prim_pose._x = initial_pose._x + (it->_x / grid_resolution);
        prim_pose._y = initial_pose._y + (it->_y / grid_resolution);
        if (is_backwards) {
          prim_pose._theta = std::fmod(it->_theta + M_PI, 2.0 * M_PI);
        } else {
          prim_pose._theta = std::fmod(it->_theta, 2.0 * M_PI);
        }
        if (collision_checker->inCollision(
            prim_pose._x,
            prim_pose._y,
            prim_pose._theta / bin_size,
            traverse_unknown))
        {
          _is_node_valid = false;
          _cell_cost = std::max(max_cell_cost, collision_checker->getCost());
          return false;
        }
        max_cell_cost = std::max(max_cell_cost, collision_checker->getCost());
      }
    }
  }

  _cell_cost = max_cell_cost;
  _is_node_valid = true;
  return _is_node_valid;
}

float NodeLattice::getTraversalCost(const NodePtr & child)
{
  const float normalized_cost = child->getCost() / 252.0f;
  if (std::isnan(normalized_cost)) {
    throw std::runtime_error(
      "Node attempted to get traversal cost without a known collision cost!");
  }

  const MotionPrimitive * prim = this->getMotionPrimitive();
  const MotionPrimitive * transition_prim = child->getMotionPrimitive();
  const float prim_length =
    transition_prim->trajectory_length / _ctx->motion_table.lattice_metadata.grid_resolution;
  if (prim == nullptr) {
    return prim_length;
  }

  if (transition_prim->trajectory_length < 1e-4f) {
    return _ctx->motion_table.rotation_penalty *
           (1.0f + _ctx->motion_table.cost_penalty * normalized_cost);
  }

  float travel_cost = 0.0f;
  float travel_cost_raw = 0.0f;
  if (_ctx->motion_table.use_quadratic_cost_penalty) {
    travel_cost_raw = prim_length *
      (_ctx->motion_table.travel_distance_reward +
      _ctx->motion_table.cost_penalty * normalized_cost * normalized_cost);
  } else {
    travel_cost_raw = prim_length *
      (_ctx->motion_table.travel_distance_reward +
      _ctx->motion_table.cost_penalty * normalized_cost);
  }

  if (transition_prim->arc_length < 0.001f) {
    travel_cost = travel_cost_raw;
  } else {
    if (prim->left_turn == transition_prim->left_turn) {
      travel_cost = travel_cost_raw * _ctx->motion_table.non_straight_penalty;
    } else {
      travel_cost = travel_cost_raw *
        (_ctx->motion_table.non_straight_penalty + _ctx->motion_table.change_penalty);
    }
  }

  if (child->isBackward()) {
    travel_cost *= _ctx->motion_table.reverse_penalty;
  }

  return travel_cost;
}

float NodeLattice::getHeuristicCost(
  const Coordinates & node_coords,
  const CoordinateVector & goals_coords)
{
  const float obstacle_heuristic = _ctx->obstacle_heuristic->getObstacleHeuristic(
    node_coords, _ctx->motion_table.cost_penalty,
    _ctx->motion_table.use_quadratic_cost_penalty,
    _ctx->motion_table.downsample_obstacle_heuristic);
  float distance_heuristic = std::numeric_limits<float>::max();
  for (unsigned int i = 0; i < goals_coords.size(); ++i) {
    distance_heuristic = std::min(
      distance_heuristic,
      _ctx->distance_heuristic->getDistanceHeuristic(
        node_coords, goals_coords[i], obstacle_heuristic, _ctx->motion_table));
  }
  return std::max(obstacle_heuristic, distance_heuristic);
}

void NodeLattice::initMotionModel(
  NodeContext * ctx,
  const MotionModel & motion_model,
  unsigned int & size_x,
  unsigned int & /*size_y*/,
  unsigned int & /*num_angle_quantization*/,
  SearchInfo & search_info)
{
  if (motion_model != MotionModel::STATE_LATTICE) {
    throw std::runtime_error(
      "Invalid motion model for Lattice node. Please select "
      "STATE_LATTICE and provide a valid lattice file.");
  }

  ctx->motion_table.initMotionModel(size_x, search_info);
}

void NodeLattice::getNeighbors(
  std::function<bool(const uint64_t &, NodeLattice * &)> & validity_checker,
  GridCollisionChecker * collision_checker,
  const bool & traverse_unknown,
  NodeVector & neighbors)
{
  uint64_t index = 0;
  bool backwards = false;
  NodePtr neighbor = nullptr;
  Coordinates initial_node_coords, motion_projection;
  unsigned int direction_change_index = 0;
  MotionPrimitivePtrs motion_primitives = _ctx->motion_table.getMotionPrimitives(
    this, direction_change_index);
  const float & grid_resolution = _ctx->motion_table.lattice_metadata.grid_resolution;

  for (unsigned int i = 0; i != motion_primitives.size(); ++i) {
    const MotionPose & end_pose = motion_primitives[i]->poses.back();
    motion_projection.x = this->pose.x + (end_pose._x / grid_resolution);
    motion_projection.y = this->pose.y + (end_pose._y / grid_resolution);
    motion_projection.theta = motion_primitives[i]->end_angle;

    backwards = false;
    if (i >= direction_change_index) {
      backwards = true;
      float opposite_heading_theta =
        motion_projection.theta - (_ctx->motion_table.num_angle_quantization / 2.0f);
      if (opposite_heading_theta < 0) {
        opposite_heading_theta += _ctx->motion_table.num_angle_quantization;
      }
      if (opposite_heading_theta > _ctx->motion_table.num_angle_quantization) {
        opposite_heading_theta -= _ctx->motion_table.num_angle_quantization;
      }
      motion_projection.theta = opposite_heading_theta;
    }

    index = NodeLattice::getIndex(
      static_cast<unsigned int>(motion_projection.x),
      static_cast<unsigned int>(motion_projection.y),
      static_cast<unsigned int>(motion_projection.theta),
      _ctx->motion_table.size_x, _ctx->motion_table.num_angle_quantization);

    if (validity_checker(index, neighbor) && !neighbor->wasVisited()) {
      initial_node_coords = neighbor->pose;

      neighbor->setPose(
        Coordinates(
          motion_projection.x,
          motion_projection.y,
          motion_projection.theta));

      if (neighbor->isNodeValid(
          traverse_unknown, collision_checker, motion_primitives[i], backwards))
      {
        neighbor->setMotionPrimitive(motion_primitives[i]);
        neighbor->backwards(backwards);
        neighbors.push_back(neighbor);
      } else {
        neighbor->setPose(initial_node_coords);
      }
    }
  }
}

bool NodeLattice::backtracePath(CoordinateVector & path)
{
  if (!this->parent) {
    return false;
  }

  NodePtr current_node = this;

  while (current_node->parent) {
    addNodeToPath(current_node, path);
    current_node = current_node->parent;
  }

  addNodeToPath(current_node, path);

  return true;
}

void NodeLattice::addNodeToPath(
  NodeLattice::NodePtr current_node,
  NodeLattice::CoordinateVector & path)
{
  Coordinates initial_pose, prim_pose;
  const MotionPrimitive * prim = current_node->getMotionPrimitive();
  const float & grid_resolution = _ctx->motion_table.lattice_metadata.grid_resolution;

  if (prim) {
    initial_pose.x = current_node->pose.x - (prim->poses.back()._x / grid_resolution);
    initial_pose.y = current_node->pose.y - (prim->poses.back()._y / grid_resolution);
    initial_pose.theta = _ctx->motion_table.getAngleFromBin(prim->start_angle);

    for (auto it = prim->poses.crbegin(); it != prim->poses.crend(); ++it) {
      prim_pose.x = initial_pose.x + (it->_x / grid_resolution);
      prim_pose.y = initial_pose.y + (it->_y / grid_resolution);
      if (current_node->isBackward()) {
        prim_pose.theta = std::fmod(it->_theta + M_PI, 2.0 * M_PI);
      } else {
        prim_pose.theta = it->_theta;
      }
      path.push_back(prim_pose);
    }
  } else {
    path.push_back(current_node->pose);
    path.back().theta = _ctx->motion_table.getAngleFromBin(path.back().theta);
  }
}

}  // namespace lattice_planner
