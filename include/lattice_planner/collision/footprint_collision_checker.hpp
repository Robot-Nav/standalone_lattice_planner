// Lattice Planner - Footprint Collision Checker
// Replaces nav2_costmap_2d::FootprintCollisionChecker with standalone implementation.

#ifndef LATTICE_PLANNER__COLLISION__FOOTPRINT_COLLISION_CHECKER_HPP_
#define LATTICE_PLANNER__COLLISION__FOOTPRINT_COLLISION_CHECKER_HPP_

#include <cmath>
#include <vector>
#include "lattice_planner/core/constants.hpp"
#include "lattice_planner/core/types.hpp"

namespace lattice_planner {

typedef std::vector<Point2D> Footprint;

/**
 * @class FootprintCollisionChecker
 * @brief Template class for checking footprint collision against a costmap.
 *        Replaces nav2_costmap_2d::FootprintCollisionChecker<CostmapT>.
 *
 * CostmapT must provide:
 *   - unsigned char getCost(unsigned int x, unsigned int y) const
 *   - unsigned int getSizeInCellsX() const
 *   - unsigned int getSizeInCellsY() const
 *   - double getResolution() const
 *   - double getOriginX() const
 *   - double getOriginY() const
 *   - bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
 */
template<typename CostmapT>
class FootprintCollisionChecker
{
public:
  explicit FootprintCollisionChecker(CostmapT costmap)
  : costmap_(costmap) {}

  FootprintCollisionChecker()
  : costmap_(nullptr) {}

  /**
   * @brief Get the cost of a single point in the costmap.
   * @param x Cell x index
   * @param y Cell y index
   * @return Cost at (x, y), or OCCUPIED_COST if out of bounds.
   */
  unsigned char pointCost(int x, int y) const
  {
    if (x < 0 || y < 0 ||
        x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
        y >= static_cast<int>(costmap_->getSizeInCellsY()))
    {
      return static_cast<unsigned char>(OCCUPIED_COST);
    }
    return costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
  }

  /**
   * @brief Get the maximum cost along a line segment between two cells (Bresenham).
   * @param x0 Start cell x
   * @param y0 Start cell y
   * @param x1 End cell x
   * @param y1 End cell y
   * @return Maximum cost along the line, or OCCUPIED_COST if any point is lethal.
   */
  unsigned char lineCost(int x0, int y0, int x1, int y1) const
  {
    unsigned char line_cost = 0;
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;

    while (true) {
      unsigned char point_cost = pointCost(x, y);
      if (point_cost >= static_cast<unsigned char>(OCCUPIED_COST)) {
        return point_cost;
      }
      if (point_cost > line_cost) {
        line_cost = point_cost;
      }
      if (x == x1 && y == y1) {
        break;
      }
      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x += sx; }
      if (e2 < dx) { err += dx; y += sy; }
    }
    return line_cost;
  }

  /**
   * @brief Get the maximum cost across all edges of a footprint polygon.
   *        The footprint is given in world coordinates.
   * @param footprint Vector of world-coordinate points forming the footprint polygon.
   * @return Maximum cost found along the footprint edges.
   */
  unsigned char footprintCost(const Footprint & footprint) const
  {
    unsigned int x0, x1, y0, y1;
    unsigned char footprint_cost = 0;

    for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
      if (!costmap_->worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
        return static_cast<unsigned char>(OCCUPIED_COST);
      }
      if (!costmap_->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
        return static_cast<unsigned char>(OCCUPIED_COST);
      }
      unsigned char line_cost = lineCost(
        static_cast<int>(x0), static_cast<int>(y0),
        static_cast<int>(x1), static_cast<int>(y1));
      if (line_cost >= static_cast<unsigned char>(OCCUPIED_COST)) {
        return line_cost;
      }
      if (line_cost > footprint_cost) {
        footprint_cost = line_cost;
      }
    }

    // Close the polygon: last point to first point
    if (footprint.size() > 2) {
      if (!costmap_->worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
        return static_cast<unsigned char>(OCCUPIED_COST);
      }
      if (!costmap_->worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
        return static_cast<unsigned char>(OCCUPIED_COST);
      }
      unsigned char line_cost = lineCost(
        static_cast<int>(x0), static_cast<int>(y0),
        static_cast<int>(x1), static_cast<int>(y1));
      if (line_cost >= static_cast<unsigned char>(OCCUPIED_COST)) {
        return line_cost;
      }
      if (line_cost > footprint_cost) {
        footprint_cost = line_cost;
      }
    }

    return footprint_cost;
  }

  /**
   * @brief Get the cost of a footprint at a specific pose (world coordinates).
   * @param x World x
   * @param y World y
   * @param theta Yaw angle (radians)
   * @param footprint Unoriented footprint (robot-frame points)
   * @return Maximum cost along the transformed footprint edges.
   */
  unsigned char footprintCost(
    double x, double y, double theta, const Footprint & footprint) const
  {
    double cos_th = std::cos(theta);
    double sin_th = std::sin(theta);
    Footprint oriented_footprint;
    oriented_footprint.reserve(footprint.size());

    for (const auto & pt : footprint) {
      Point2D new_pt;
      new_pt.x = x + pt.x * cos_th - pt.y * sin_th;
      new_pt.y = y + pt.x * sin_th + pt.y * cos_th;
      oriented_footprint.push_back(new_pt);
    }
    return footprintCost(oriented_footprint);
  }

  void setCostmap(CostmapT costmap) { costmap_ = costmap; }
  CostmapT getCostmap() const { return costmap_; }

protected:
  CostmapT costmap_;
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__COLLISION__FOOTPRINT_COLLISION_CHECKER_HPP_
