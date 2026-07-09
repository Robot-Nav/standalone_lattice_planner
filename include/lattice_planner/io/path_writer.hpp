// Lattice Planner - Path Writer
// Outputs Path data to CSV, JSON, or stdout. No ROS dependencies.

#ifndef LATTICE_PLANNER__IO__PATH_WRITER_HPP_
#define LATTICE_PLANNER__IO__PATH_WRITER_HPP_

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "lattice_planner/core/types.hpp"

namespace lattice_planner {

/**
 * @class PathWriter
 * @brief Utility for writing a Path to CSV, JSON, or stdout.
 */
class PathWriter
{
public:
  /**
   * @brief Write path to a CSV file (x,y,theta per line).
   * @param path Path to write.
   * @param filepath Output file path.
   * @return true on success.
   */
  static bool writeToFile(const Path & path, const std::string & filepath)
  {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
      std::cerr << "[ERROR] Cannot open output file: " << filepath << std::endl;
      return false;
    }
    ofs << "x,y,theta\n";
    ofs << std::fixed << std::setprecision(6);
    for (const auto & pose : path.poses) {
      ofs << pose.x << "," << pose.y << "," << pose.theta << "\n";
    }
    ofs.close();
    std::cout << "[INFO] Path written to " << filepath
              << " (" << path.poses.size() << " points)" << std::endl;
    return true;
  }

  /**
   * @brief Write path to a JSON array file.
   * @param path Path to write.
   * @param filepath Output file path.
   * @return true on success.
   */
  static bool writeToJSON(const Path & path, const std::string & filepath)
  {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
      std::cerr << "[ERROR] Cannot open output file: " << filepath << std::endl;
      return false;
    }
    ofs << "[\n";
    ofs << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < path.poses.size(); ++i) {
      ofs << "  {\"x\": " << path.poses[i].x
          << ", \"y\": " << path.poses[i].y
          << ", \"theta\": " << path.poses[i].theta << "}";
      if (i + 1 < path.poses.size()) ofs << ",";
      ofs << "\n";
    }
    ofs << "]\n";
    ofs.close();
    std::cout << "[INFO] Path (JSON) written to " << filepath
              << " (" << path.poses.size() << " points)" << std::endl;
    return true;
  }

  /**
   * @brief Print path to stdout.
   * @param path Path to print.
   */
  static void writeToConsole(const Path & path)
  {
    std::cout << "Path (" << path.poses.size() << " points):" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto & p = path.poses[i];
      std::cout << "  [" << i << "] x=" << p.x
                << ", y=" << p.y
                << ", theta=" << p.theta << std::endl;
    }
  }

  /**
   * @brief Print a summary of the path (length, point count).
   */
  static void printSummary(const Path & path)
  {
    double total_length = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      double dx = path.poses[i].x - path.poses[i - 1].x;
      double dy = path.poses[i].y - path.poses[i - 1].y;
      total_length += std::sqrt(dx * dx + dy * dy);
    }
    std::cout << "[INFO] Path summary: " << path.poses.size() << " points, "
              << "total length = " << total_length << " m" << std::endl;
  }
};

}  // namespace lattice_planner

#endif  // LATTICE_PLANNER__IO__PATH_WRITER_HPP_
