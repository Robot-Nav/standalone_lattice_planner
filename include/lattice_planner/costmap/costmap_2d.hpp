// Lattice Planner - Costmap2D
// Replaces nav2_costmap_2d::Costmap2D with a standalone grid costmap.
// No ROS dependencies. Supports PGM map loading for standalone demos.

#ifndef LATTICE_PLANNER__COSTMAP__COSTMAP_2D_HPP_
#define LATTICE_PLANNER__COSTMAP__COSTMAP_2D_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include "lattice_planner/core/exceptions.hpp"

namespace lattice_planner {

/**
 * @class Costmap2D
 * @brief A 2D grid costmap storing obstacle costs (0-255).
 *        Replaces nav2_costmap_2d::Costmap2D with no ROS dependencies.
 *
 * Coordinate conventions (same as nav2_costmap_2d):
 *   - World coordinates: meters, origin at (origin_x_, origin_y_)
 *   - Map coordinates: cell indices [0, size_x_) x [0, size_y_)
 *   - Cost values: 0=free, 254=occupied, 255=unknown
 */
class Costmap2D {
public:
  Costmap2D();

  Costmap2D(
    unsigned int size_x, unsigned int size_y, double resolution,
    double origin_x, double origin_y, unsigned char default_value = 0);

  Costmap2D(const Costmap2D & other);

  Costmap2D & operator=(const Costmap2D & other);

  ~Costmap2D();

  // ---- Size / geometry accessors ----

  unsigned int getSizeInCellsX() const { return size_x_; }
  unsigned int getSizeInCellsY() const { return size_y_; }
  double getSizeInMetersX() const { return (size_x_ - 1 + 0.5) * resolution_; }
  double getSizeInMetersY() const { return (size_y_ - 1 + 0.5) * resolution_; }
  double getResolution() const { return resolution_; }
  double getOriginX() const { return origin_x_; }
  double getOriginY() const { return origin_y_; }

  // ---- Cost access ----

  unsigned char getCost(unsigned int mx, unsigned int my) const;

  unsigned char getCost(unsigned int index) const;

  void setCost(unsigned int mx, unsigned int my, unsigned char cost);

  void setCost(unsigned int index, unsigned char cost);

  unsigned char * getCharMap() { return costmap_; }
  const unsigned char * getCharMap() const { return costmap_; }

  unsigned int getIndex(unsigned int mx, unsigned int my) const {
    return my * size_x_ + mx;
  }

  // ---- Coordinate conversions ----

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;

  bool worldToMapContinuous(double wx, double wy, double & mx_d, double & my_d) const;

  // ---- Map manipulation ----

  void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);

  void clearEntirely();

  // ---- Inflation parameters (static replacement for nav2 inflation layer) ----

  void setInflationParameters(
    double inflation_radius, double inscribed_radius, double circumscribed_radius);

  double getInflationRadius() const { return inflation_radius_; }
  double getInscribedRadius() const { return inscribed_radius_; }
  double getCircumscribedRadius() const { return circumscribed_radius_; }

  /**
   * @brief Compute the circumscribed cost from the inflation parameters.
   *        Replaces nav2_smac_planner::findCircumscribedCost.
   * @return The cost at the circumscribed radius, or -1 if no inflation configured.
   */
  double getCircumscribedCost() const;

  /**
   * @brief Set the circumscribed cost directly (override auto-computation).
   */
  void setCircumscribedCost(double cost) { circumscribed_cost_ = cost; }

private:
  unsigned int size_x_{0};
  unsigned int size_y_{0};
  double resolution_{1.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  unsigned char default_value_{0};
  unsigned char * costmap_{nullptr};

  // Inflation layer replacement (static parameters)
  double inflation_radius_{0.0};
  double inscribed_radius_{0.0};
  double circumscribed_radius_{0.0};
  double circumscribed_cost_{-1.0};

  void deleteMap();
  void initMaps();
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__COSTMAP__COSTMAP_2D_HPP_
