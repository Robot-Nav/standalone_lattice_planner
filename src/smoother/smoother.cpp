// Lattice Planner - Path Smoother Implementation
// Ported from nav2_smac_planner/smoother.cpp with ROS dependencies replaced.

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "lattice_planner/smoother/smoother.hpp"
#include "lattice_planner/core/logger.hpp"

namespace lattice_planner {

using namespace std::chrono;  // NOLINT

// ---- Local replacements for nav2_util types and functions ----

struct PathSegment
{
  unsigned int start{0};
  unsigned int end{0};
};

static std::vector<PathSegment> findDirectionalPathSegments(
  const std::vector<Pose2D> & path, bool holonomic)
{
  std::vector<PathSegment> segments;
  if (path.size() < 2) {
    if (!path.empty()) {
      PathSegment seg;
      seg.start = 0;
      seg.end = 0;
      segments.push_back(seg);
    }
    return segments;
  }

  PathSegment current;
  current.start = 0;
  current.end = 0;
  bool prev_reversing = false;

  for (unsigned int i = 1; i < path.size(); i++) {
    double dx = path[i].x - path[i - 1].x;
    double dy = path[i].y - path[i - 1].y;
    double theta = path[i - 1].theta;
    double motion_angle = std::atan2(dy, dx);
    double angle_diff = std::fabs(shortestAngularDistance(theta, motion_angle));
    bool is_reversing = (angle_diff > M_PI / 2.0);

    if (i == 1) {
      prev_reversing = is_reversing;
    } else if (is_reversing != prev_reversing) {
      current.end = i - 1;
      segments.push_back(current);
      current.start = i - 1;
      prev_reversing = is_reversing;
    }
    current.end = i;
  }
  segments.push_back(current);
  return segments;
}

static void updateApproximatePathOrientations(
  std::vector<Pose2D> & path, bool reversing_segment, bool holonomic)
{
  if (path.size() < 2) {
    return;
  }

  for (unsigned int i = 0; i < path.size() - 1; i++) {
    double dx = path[i + 1].x - path[i].x;
    double dy = path[i + 1].y - path[i].y;
    double angle = std::atan2(dy, dx);
    if (reversing_segment) {
      angle += M_PI;
      angle = std::fmod(angle, 2.0 * M_PI);
    }
    path[i].theta = angle;
  }
  // Last pose: copy from previous
  path.back().theta = path[path.size() - 2].theta;
}

// ---- Smoother implementation ----

Smoother::Smoother(const SmootherParams & params)
{
  tolerance_ = params.tolerance_;
  max_its_ = params.max_its_;
  data_w_ = params.w_data_;
  smooth_w_ = params.w_smooth_;
  is_holonomic_ = params.holonomic_;
  do_refinement_ = params.do_refinement_;
  refinement_num_ = params.refinement_num_;
  refinement_ctr_ = 0;
  motion_model_ = MotionModel::UNKNOWN;
}

void Smoother::initialize(const double & min_turning_radius)
{
  min_turning_rad_ = min_turning_radius;
  state_space_ = std::make_unique<ompl::base::DubinsStateSpace>(min_turning_rad_);
}

bool Smoother::smooth(
  std::vector<Pose2D> & path,
  const Costmap2D * costmap,
  const double & max_time)
{
  if (max_its_ == 0) {
    return false;
  }

  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time;
  bool success = true, reversing_segment;
  std::vector<Pose2D> curr_path_segment;
  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path, is_holonomic_);

  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > 10) {
      // Populate path segment
      curr_path_segment.clear();
      std::copy(
        path.begin() + path_segments[i].start,
        path.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment));

      // Make sure we're still able to smooth with time remaining
      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time - duration_cast<duration<double>>(now - start).count();
      refinement_ctr_ = 0;

      // Smooth path segment naively
      Pose2D start_pose = curr_path_segment.front();
      Pose2D goal_pose = curr_path_segment.back();
      bool local_success =
        smoothImpl(curr_path_segment, reversing_segment, costmap, time_remaining);
      success = success && local_success;

      // Enforce boundary conditions
      if (!is_holonomic_ && local_success) {
        enforceStartBoundaryConditions(start_pose, curr_path_segment, costmap, reversing_segment);
        enforceEndBoundaryConditions(goal_pose, curr_path_segment, costmap, reversing_segment);
      }

      // Assemble the path changes to the main path
      std::copy(
        curr_path_segment.begin(),
        curr_path_segment.end(),
        path.begin() + path_segments[i].start);
    }
  }

  return success;
}

bool Smoother::smoothImpl(
  std::vector<Pose2D> & path,
  bool & reversing_segment,
  const Costmap2D * costmap,
  const double & max_time)
{
  steady_clock::time_point a = steady_clock::now();

  int its = 0;
  double change = tolerance_;
  const unsigned int path_size = path.size();
  double x_i, y_i, y_m1, y_ip1, y_i_org;
  unsigned int mx, my;

  std::vector<Pose2D> new_path = path;
  std::vector<Pose2D> last_path = path;

  while (change >= tolerance_) {
    its += 1;
    change = 0.0;

    // Make sure the smoothing function will converge
    if (its >= max_its_) {
      LP_LOG_DEBUG("Number of iterations has exceeded limit.");
      path = last_path;
      updateApproximatePathOrientations(path, reversing_segment, is_holonomic_);
      return false;
    }

    // Make sure still have time left to process
    steady_clock::time_point b = steady_clock::now();
    double timespan = duration_cast<duration<double>>(b - a).count();
    if (timespan > max_time) {
      LP_LOG_DEBUG("Smoothing time exceeded allowed duration.");
      path = last_path;
      updateApproximatePathOrientations(path, reversing_segment, is_holonomic_);
      return false;
    }

    for (unsigned int i = 1; i != path_size - 1; i++) {
      for (unsigned int j = 0; j != 2; j++) {
        x_i = getFieldByDim(path[i], j);
        y_i = getFieldByDim(new_path[i], j);
        y_m1 = getFieldByDim(new_path[i - 1], j);
        y_ip1 = getFieldByDim(new_path[i + 1], j);
        y_i_org = y_i;

        // Smooth based on local 3 point neighborhood and original data locations
        y_i += data_w_ * (x_i - y_i) + smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
        setFieldByDim(new_path[i], j, y_i);
        change += abs(y_i - y_i_org);
      }

      // validate update is admissible, only checks cost if a valid costmap pointer is provided
      float cost = 0.0;
      if (costmap) {
        costmap->worldToMap(
          getFieldByDim(new_path[i], 0),
          getFieldByDim(new_path[i], 1),
          mx, my);
        cost = static_cast<float>(costmap->getCost(mx, my));
      }

      if (cost > MAX_NON_OBSTACLE_COST && cost != UNKNOWN_COST) {
        LP_LOG_DEBUG("Smoothing resulted in collision. Returning last path.");
        path = last_path;
        updateApproximatePathOrientations(path, reversing_segment, is_holonomic_);
        return false;
      }
    }

    last_path = new_path;
  }

  // Let's do additional refinement
  if (do_refinement_ && refinement_ctr_ < refinement_num_) {
    refinement_ctr_++;
    smoothImpl(new_path, reversing_segment, costmap, max_time);
  }

  updateApproximatePathOrientations(new_path, reversing_segment, is_holonomic_);
  path = new_path;
  return true;
}

double Smoother::getFieldByDim(const Pose2D & pose, const unsigned int & dim)
{
  if (dim == 0) {
    return pose.x;
  } else if (dim == 1) {
    return pose.y;
  } else {
    return pose.theta;
  }
}

void Smoother::setFieldByDim(Pose2D & pose, const unsigned int dim, const double & value)
{
  if (dim == 0) {
    pose.x = value;
  } else if (dim == 1) {
    pose.y = value;
  } else {
    pose.theta = value;
  }
}

unsigned int Smoother::findShortestBoundaryExpansionIdx(
  const BoundaryExpansions & boundary_expansions)
{
  double min_length = 1e9;
  int shortest_boundary_expansion_idx = 1e9;
  for (unsigned int idx = 0; idx != boundary_expansions.size(); idx++) {
    if (boundary_expansions[idx].expansion_path_length < min_length &&
      !boundary_expansions[idx].in_collision &&
      boundary_expansions[idx].path_end_idx > 0.0 &&
      boundary_expansions[idx].expansion_path_length > 0.0)
    {
      min_length = boundary_expansions[idx].expansion_path_length;
      shortest_boundary_expansion_idx = idx;
    }
  }

  return shortest_boundary_expansion_idx;
}

void Smoother::findBoundaryExpansion(
  const Pose2D & start,
  const Pose2D & end,
  BoundaryExpansion & expansion,
  const Costmap2D * costmap)
{
  ompl::base::ScopedState<> from(state_space_), to(state_space_), s(state_space_);

  from[0] = start.x;
  from[1] = start.y;
  from[2] = start.theta;
  to[0] = end.x;
  to[1] = end.y;
  to[2] = end.theta;

  double d = state_space_->distance(from(), to());
  // If this path is too long compared to the original, then this is probably
  // a loop-de-loop, treat as invalid.
  if (d > 2.0 * expansion.original_path_length) {
    return;
  }

  std::vector<double> reals;
  double theta(0.0), x(0.0), y(0.0);
  double x_m = start.x;
  double y_m = start.y;

  // Get intermediary poses
  for (double i = 0; i <= expansion.path_end_idx; i++) {
    state_space_->interpolate(from(), to(), i / expansion.path_end_idx, s());
    reals = s.reals();
    // Make sure in range [0, 2PI)
    theta = (reals[2] < 0.0) ? (reals[2] + 2.0 * M_PI) : reals[2];
    theta = (theta > 2.0 * M_PI) ? (theta - 2.0 * M_PI) : theta;
    x = reals[0];
    y = reals[1];

    // Check for collision
    unsigned int mx, my;
    costmap->worldToMap(x, y, mx, my);
    if (static_cast<float>(costmap->getCost(mx, my)) >= INSCRIBED_COST) {
      expansion.in_collision = true;
    }

    // Integrate path length
    expansion.expansion_path_length += hypot(x - x_m, y - y_m);
    x_m = x;
    y_m = y;

    // Store point
    expansion.pts.emplace_back(x, y, theta);
  }
}

template<typename IteratorT>
BoundaryExpansions Smoother::generateBoundaryExpansionPoints(IteratorT start, IteratorT end)
{
  std::vector<double> distances = {
    min_turning_rad_,                    // Radius
    2.0 * min_turning_rad_,              // Diameter
    M_PI * min_turning_rad_,             // 50% Circumference
    2.0 * M_PI * min_turning_rad_        // Circumference
  };

  BoundaryExpansions boundary_expansions;
  boundary_expansions.resize(distances.size());
  double curr_dist = 0.0;
  double x_last = start->x;
  double y_last = start->y;
  unsigned int curr_dist_idx = 0;

  for (IteratorT iter = start; iter != end; iter++) {
    curr_dist += hypot(iter->x - x_last, iter->y - y_last);
    x_last = iter->x;
    y_last = iter->y;

    if (curr_dist >= distances[curr_dist_idx]) {
      boundary_expansions[curr_dist_idx].path_end_idx = iter - start;
      boundary_expansions[curr_dist_idx].original_path_length = curr_dist;
      curr_dist_idx++;
    }

    if (curr_dist_idx == boundary_expansions.size()) {
      break;
    }
  }

  return boundary_expansions;
}

void Smoother::enforceStartBoundaryConditions(
  const Pose2D & start_pose,
  std::vector<Pose2D> & path,
  const Costmap2D * costmap,
  const bool & reversing_segment)
{
  // Find range of points for testing
  BoundaryExpansions boundary_expansions =
    generateBoundaryExpansionPoints<PathIterator>(path.begin(), path.end());

  // Generate the motion model and metadata from start -> test points
  for (unsigned int i = 0; i != boundary_expansions.size(); i++) {
    BoundaryExpansion & expansion = boundary_expansions[i];
    if (expansion.path_end_idx == 0.0) {
      continue;
    }

    if (!reversing_segment) {
      findBoundaryExpansion(
        start_pose, path[expansion.path_end_idx], expansion, costmap);
    } else {
      findBoundaryExpansion(
        path[expansion.path_end_idx], start_pose, expansion, costmap);
    }
  }

  // Find the shortest kinematically feasible boundary expansion
  unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
  if (best_expansion_idx > boundary_expansions.size()) {
    return;
  }

  // Override values to match curve
  BoundaryExpansion & best_expansion = boundary_expansions[best_expansion_idx];
  if (reversing_segment) {
    std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
  }
  for (unsigned int i = 0; i != best_expansion.pts.size(); i++) {
    path[i].x = best_expansion.pts[i].x;
    path[i].y = best_expansion.pts[i].y;
    path[i].theta = best_expansion.pts[i].theta;
  }
}

void Smoother::enforceEndBoundaryConditions(
  const Pose2D & end_pose,
  std::vector<Pose2D> & path,
  const Costmap2D * costmap,
  const bool & reversing_segment)
{
  // Find range of points for testing
  BoundaryExpansions boundary_expansions =
    generateBoundaryExpansionPoints<ReversePathIterator>(path.rbegin(), path.rend());

  // Generate the motion model and metadata from start -> test points
  unsigned int expansion_starting_idx;
  for (unsigned int i = 0; i != boundary_expansions.size(); i++) {
    BoundaryExpansion & expansion = boundary_expansions[i];
    if (expansion.path_end_idx == 0.0) {
      continue;
    }
    expansion_starting_idx = path.size() - expansion.path_end_idx - 1;
    if (!reversing_segment) {
      findBoundaryExpansion(path[expansion_starting_idx], end_pose, expansion, costmap);
    } else {
      findBoundaryExpansion(end_pose, path[expansion_starting_idx], expansion, costmap);
    }
  }

  // Find the shortest kinematically feasible boundary expansion
  unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
  if (best_expansion_idx > boundary_expansions.size()) {
    return;
  }

  // Override values to match curve
  BoundaryExpansion & best_expansion = boundary_expansions[best_expansion_idx];
  if (reversing_segment) {
    std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
  }
  expansion_starting_idx = path.size() - best_expansion.path_end_idx - 1;
  for (unsigned int i = 0; i != best_expansion.pts.size(); i++) {
    path[expansion_starting_idx + i].x = best_expansion.pts[i].x;
    path[expansion_starting_idx + i].y = best_expansion.pts[i].y;
    path[expansion_starting_idx + i].theta = best_expansion.pts[i].theta;
  }
}

// Explicit template instantiation
template BoundaryExpansions Smoother::generateBoundaryExpansionPoints<PathIterator>(
  PathIterator, PathIterator);
template BoundaryExpansions Smoother::generateBoundaryExpansionPoints<ReversePathIterator>(
  ReversePathIterator, ReversePathIterator);

}  // namespace lattice_planner
