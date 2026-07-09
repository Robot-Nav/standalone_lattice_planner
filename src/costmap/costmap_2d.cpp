// Lattice Planner - Costmap2D Implementation

#include "lattice_planner/costmap/costmap_2d.hpp"

#include <cstring>
#include <cmath>
#include "lattice_planner/core/logger.hpp"

namespace lattice_planner {

Costmap2D::Costmap2D() = default;

Costmap2D::Costmap2D(
  unsigned int size_x, unsigned int size_y, double resolution,
  double origin_x, double origin_y, unsigned char default_value)
: size_x_(size_x),
  size_y_(size_y),
  resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  default_value_(default_value)
{
  initMaps();
}

Costmap2D::Costmap2D(const Costmap2D & other)
: size_x_(other.size_x_),
  size_y_(other.size_y_),
  resolution_(other.resolution_),
  origin_x_(other.origin_x_),
  origin_y_(other.origin_y_),
  default_value_(other.default_value_),
  inflation_radius_(other.inflation_radius_),
  inscribed_radius_(other.inscribed_radius_),
  circumscribed_radius_(other.circumscribed_radius_),
  circumscribed_cost_(other.circumscribed_cost_)
{
  initMaps();
  if (costmap_ && other.costmap_) {
    std::memcpy(costmap_, other.costmap_, size_x_ * size_y_ * sizeof(unsigned char));
  }
}

Costmap2D & Costmap2D::operator=(const Costmap2D & other)
{
  if (this == &other) {
    return *this;
  }
  deleteMap();
  size_x_ = other.size_x_;
  size_y_ = other.size_y_;
  resolution_ = other.resolution_;
  origin_x_ = other.origin_x_;
  origin_y_ = other.origin_y_;
  default_value_ = other.default_value_;
  inflation_radius_ = other.inflation_radius_;
  inscribed_radius_ = other.inscribed_radius_;
  circumscribed_radius_ = other.circumscribed_radius_;
  circumscribed_cost_ = other.circumscribed_cost_;
  initMaps();
  if (costmap_ && other.costmap_) {
    std::memcpy(costmap_, other.costmap_, size_x_ * size_y_ * sizeof(unsigned char));
  }
  return *this;
}

Costmap2D::~Costmap2D()
{
  deleteMap();
}

void Costmap2D::initMaps()
{
  if (size_x_ > 0 && size_y_ > 0) {
    costmap_ = new unsigned char[size_x_ * size_y_];
    clearEntirely();
  }
}

void Costmap2D::deleteMap()
{
  delete[] costmap_;
  costmap_ = nullptr;
}

void Costmap2D::clearEntirely()
{
  if (costmap_) {
    std::memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  }
}

void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  if (!costmap_) {
    return;
  }
  for (unsigned int y = y0; y < yn && y < size_y_; ++y) {
    for (unsigned int x = x0; x < xn && x < size_x_; ++x) {
      costmap_[getIndex(x, y)] = default_value_;
    }
  }
}

unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
  return costmap_[getIndex(mx, my)];
}

unsigned char Costmap2D::getCost(unsigned int index) const
{
  return costmap_[index];
}

void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  costmap_[getIndex(mx, my)] = cost;
}

void Costmap2D::setCost(unsigned int index, unsigned char cost)
{
  costmap_[index] = cost;
}

bool Costmap2D::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }
  mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
  if (mx >= size_x_ || my >= size_y_) {
    return false;
  }
  return true;
}

void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool Costmap2D::worldToMapContinuous(double wx, double wy, double & mx_d, double & my_d) const
{
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }
  mx_d = (wx - origin_x_) / resolution_;
  my_d = (wy - origin_y_) / resolution_;
  if (mx_d >= size_x_ || my_d >= size_y_) {
    return false;
  }
  return true;
}

void Costmap2D::setInflationParameters(
  double inflation_radius, double inscribed_radius, double circumscribed_radius)
{
  inflation_radius_ = inflation_radius;
  inscribed_radius_ = inscribed_radius;
  circumscribed_radius_ = circumscribed_radius;
}

double Costmap2D::getCircumscribedCost() const
{
  if (circumscribed_cost_ >= 0.0) {
    return circumscribed_cost_;
  }
  // Auto-compute from inflation parameters using the nav2 inflation model.
  // In nav2, the cost at distance d from an obstacle is:
  //   cost = 252 * exp(-a * (d - inscribed_radius))
  // where a = ln(252/253) / (inflation_radius - inscribed_radius)
  // At the circumscribed radius, this gives the circumscribed cost.
  if (inflation_radius_ <= 0.0 || circumscribed_radius_ <= 0.0 ||
    inflation_radius_ <= inscribed_radius_)
  {
    return -1.0;
  }
  if (circumscribed_radius_ > inflation_radius_) {
    return 0.0;
  }
  double scale_factor = std::log(252.0 / 253.0) / (inflation_radius_ - inscribed_radius_);
  double dist = circumscribed_radius_;
  double cost = 252.0 * std::exp(scale_factor * (dist - inscribed_radius_));
  return cost;
}

}  // namespace lattice_planner
