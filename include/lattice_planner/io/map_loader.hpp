// Lattice Planner - Map Loader
// Loads PGM (P5/P2) image files into Costmap2D. No ROS dependencies.

#ifndef LATTICE_PLANNER__IO__MAP_LOADER_HPP_
#define LATTICE_PLANNER__IO__MAP_LOADER_HPP_

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "lattice_planner/costmap/costmap_2d.hpp"
#include "lattice_planner/core/constants.hpp"
#include "lattice_planner/core/exceptions.hpp"
#include "lattice_planner/core/logger.hpp"

namespace lattice_planner {

/**
 * @class MapLoader
 * @brief Utility for loading PGM maps and creating test maps.
 *        Supports P5 (binary) and P2 (ASCII) PGM formats.
 */
class MapLoader
{
public:
  /**
   * @brief Load a PGM file into a Costmap2D.
   *
   * Pixel value mapping:
   *   - pixel >= occupied_threshold  → OCCUPIED_COST (254)
   *   - pixel <= free_threshold       → FREE_COST (0)
   *   - otherwise                     → scaled cost (pixel * 254 / maxval)
   *
   * @param filepath Path to the PGM file.
   * @param resolution World size per pixel (meters).
   * @param origin_x World X coordinate of the map origin (lower-left).
   * @param origin_y World Y coordinate of the map origin (lower-left).
   * @param occupied_threshold Pixel value at/above which a cell is lethal.
   * @param free_threshold Pixel value at/below which a cell is free.
   * @return Owned Costmap2D.
   * @throws PlannerException if the file cannot be opened or is invalid.
   */
  static std::unique_ptr<Costmap2D> loadFromPGM(
    const std::string & filepath,
    double resolution = 0.05,
    double origin_x = 0.0,
    double origin_y = 0.0,
    unsigned char occupied_threshold = 200,
    unsigned char free_threshold = 50)
  {
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs.is_open()) {
      throw PlannerException("Cannot open PGM file: " + filepath);
    }

    // Read magic number
    std::string magic;
    ifs >> magic;
    if (magic != "P5" && magic != "P2") {
      throw PlannerException(
              "Unsupported PGM format '" + magic + "'. Only P5 and P2 are supported.");
    }
    bool is_binary = (magic == "P5");

    // Read width, height, maxval (skip comments starting with #)
    unsigned int width = readPGMInt(ifs);
    unsigned int height = readPGMInt(ifs);
    int maxval = readPGMInt(ifs);
    if (maxval <= 0 || maxval > 65535) {
      throw PlannerException("Invalid PGM maxval: " + std::to_string(maxval));
    }

    // Consume single whitespace after maxval (for binary mode)
    if (is_binary) {
      ifs.get();  // single whitespace
    }

    auto costmap = std::make_unique<Costmap2D>(
      width, height, resolution, origin_x, origin_y, 0);

    for (unsigned int row = 0; row < height; ++row) {
      for (unsigned int col = 0; col < width; ++col) {
        int pixel;
        if (is_binary) {
          if (maxval < 256) {
            pixel = static_cast<unsigned char>(ifs.get());
          } else {
            unsigned char hi = static_cast<unsigned char>(ifs.get());
            unsigned char lo = static_cast<unsigned char>(ifs.get());
            pixel = (hi << 8) | lo;
          }
        } else {
          ifs >> pixel;
        }

        unsigned char cost;
        if (pixel >= static_cast<int>(occupied_threshold)) {
          cost = static_cast<unsigned char>(OCCUPIED_COST);
        } else if (pixel <= static_cast<int>(free_threshold)) {
          cost = static_cast<unsigned char>(FREE_COST);
        } else {
          cost = static_cast<unsigned char>(
            static_cast<int>(pixel) * 254 / maxval);
        }

        // PGM rows are top-to-bottom; costmap rows are bottom-to-top
        unsigned int my = height - 1 - row;
        costmap->setCost(col, my, cost);
      }
    }

    LP_LOG_INFO(
      "Loaded PGM map: " << filepath << " (" << width << "x" << height <<
      ", resolution=" << resolution << ")");

    return costmap;
  }

  /**
   * @brief Create an empty (all-free) costmap.
   */
  static std::unique_ptr<Costmap2D> createEmptyMap(
    unsigned int width,
    unsigned int height,
    double resolution = 0.05,
    double origin_x = 0.0,
    double origin_y = 0.0)
  {
    return std::make_unique<Costmap2D>(width, height, resolution, origin_x, origin_y, 0);
  }

  /**
   * @brief Add a rectangular obstacle to the costmap.
   * @param costmap Costmap to modify.
   * @param center_x World X of rectangle center.
   * @param center_y World Y of rectangle center.
   * @param size_x World size in X (meters).
   * @param size_y World size in Y (meters).
   * @param cost Cost value to fill (default: OCCUPIED_COST).
   */
  static void addRectangle(
    Costmap2D * costmap,
    double center_x, double center_y,
    double size_x, double size_y,
    unsigned char cost = static_cast<unsigned char>(OCCUPIED_COST))
  {
    if (!costmap) return;

    unsigned int x0, y0, x1, y1;
    double half_x = size_x / 2.0;
    double half_y = size_y / 2.0;

    if (!costmap->worldToMap(center_x - half_x, center_y - half_y, x0, y0) ||
        !costmap->worldToMap(center_x + half_x, center_y + half_y, x1, y1))
    {
      LP_LOG_WARN("Rectangle partially outside map bounds, clipping.");
    }

    // Clamp to map bounds
    x0 = std::min(x0, costmap->getSizeInCellsX() - 1);
    y0 = std::min(y0, costmap->getSizeInCellsY() - 1);
    x1 = std::min(x1, costmap->getSizeInCellsX() - 1);
    y1 = std::min(y1, costmap->getSizeInCellsY() - 1);

    for (unsigned int y = y0; y <= y1; ++y) {
      for (unsigned int x = x0; x <= x1; ++x) {
        costmap->setCost(x, y, cost);
      }
    }
  }

private:
  /**
   * @brief Read the next integer from a PGM stream, skipping comments (#...).
   */
  static unsigned int readPGMInt(std::ifstream & ifs)
  {
    std::string token;
    while (true) {
      ifs >> token;
      if (token[0] == '#') {
        std::string dummy;
        std::getline(ifs, dummy);
        continue;
      }
      break;
    }
    return static_cast<unsigned int>(std::stoul(token));
  }
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__IO__MAP_LOADER_HPP_
