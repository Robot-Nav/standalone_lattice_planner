// Lattice Planner - Grid Collision Checker Implementation

#include "lattice_planner/collision/collision_checker.hpp"

#include <cmath>
#include "lattice_planner/core/logger.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lattice_planner {

GridCollisionChecker::GridCollisionChecker(Costmap2D * costmap, unsigned int num_quantizations)
: FootprintCollisionChecker(costmap)
{
  float bin_size = 2.0f * static_cast<float>(M_PI) / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; ++i) {
    angles_.push_back(bin_size * static_cast<float>(i));
  }
}

GridCollisionChecker::GridCollisionChecker(Costmap2D * costmap, const std::vector<float> & angles)
: FootprintCollisionChecker(costmap),
  angles_(angles)
{
}

void GridCollisionChecker::setFootprint(
  const Footprint & footprint,
  const bool & radius,
  const double & possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  if (possible_collision_cost_ <= 0.0f) {
    LP_LOG_WARN(
      "Inflation layer not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking. It is HIGHLY recommended to set "
      "the inflation radius to be at MINIMUM half of the robot's largest cross-section. "
      "This will substantially impact run-time performance.");
  }

  footprint_is_radius_ = radius;

  if (radius) {
    return;
  }

  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.clear();
  oriented_footprints_.reserve(angles_.size());
  const unsigned int footprint_size = static_cast<unsigned int>(footprint.size());

  for (unsigned int i = 0; i != angles_.size(); ++i) {
    double sin_th = std::sin(angles_[i]);
    double cos_th = std::cos(angles_[i]);
    Footprint oriented_footprint;
    oriented_footprint.reserve(footprint_size);

    for (unsigned int j = 0; j < footprint_size; ++j) {
      Point2D new_pt;
      new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
      new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
      oriented_footprint.push_back(new_pt);
    }
    oriented_footprints_.push_back(std::move(oriented_footprint));
  }

  unoriented_footprint_ = footprint;
}

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return true;
  }

  center_cost_ = static_cast<float>(costmap_->getCost(
      static_cast<unsigned int>(x + 0.5f), static_cast<unsigned int>(y + 0.5f)));

  if (!footprint_is_radius_) {
    if (center_cost_ < possible_collision_cost_ && possible_collision_cost_ > 0.0f) {
      return false;
    }

    if (center_cost_ == UNKNOWN_COST && !traverse_unknown) {
      return true;
    }

    if (center_cost_ == INSCRIBED_COST || center_cost_ == OCCUPIED_COST) {
      return true;
    }

    double wx, wy;
    costmap_->mapToWorld(
      static_cast<unsigned int>(x), static_cast<unsigned int>(y), wx, wy);

    unsigned int bin_idx = static_cast<unsigned int>(angle_bin);
    if (bin_idx >= oriented_footprints_.size()) {
      return true;
    }

    const Footprint & oriented_footprint = oriented_footprints_[bin_idx];
    Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
      Point2D new_pt;
      new_pt.x = wx + oriented_footprint[i].x;
      new_pt.y = wy + oriented_footprint[i].y;
      current_footprint.push_back(new_pt);
    }

    float footprint_cost = static_cast<float>(footprintCost(current_footprint));

    if (footprint_cost == UNKNOWN_COST && traverse_unknown) {
      return false;
    }

    return footprint_cost >= OCCUPIED_COST;
  } else {
    if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
      return false;
    }
    return center_cost_ >= INSCRIBED_COST;
  }
}

bool GridCollisionChecker::inCollision(const unsigned int & i, const bool & traverse_unknown)
{
  center_cost_ = costmap_->getCost(i);
  if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
    return false;
  }
  return center_cost_ >= INSCRIBED_COST;
}

float GridCollisionChecker::getCost()
{
  return center_cost_;
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > static_cast<float>(max);
}

}  // namespace lattice_planner
