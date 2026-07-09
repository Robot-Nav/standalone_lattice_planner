// Lattice Planner - Standalone Example Executable
// Demonstrates the full planning pipeline without any ROS dependencies.
//
// Usage:
//   lattice_planner_example [options]
//
// Options:
//   --map <path>          PGM map file (P5/P2). If omitted, a test map is generated.
//   --start <x,y,theta>   Start pose in world coordinates (default: 1.0,1.0,0.0)
//   --goal <x,y,theta>    Goal pose in world coordinates (default: 5.0,5.0,1.57)
//   --lattice <path>      Lattice primitives JSON file
//   --config <path>       Configuration JSON file (optional)
//   --output <path>       Output path CSV file (default: stdout)
//   --output-json <path>  Output path JSON file
//   --resolution <r>      Map resolution in meters (default: 0.05)
//   --origin <x,y>        Map origin in world coordinates (default: 0,0)
//   --no-smooth           Disable path smoothing
//   --help                Show this help message

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "lattice_planner/lattice_planner.hpp"
#include "lattice_planner/io/map_loader.hpp"
#include "lattice_planner/io/path_writer.hpp"
#include "lattice_planner/core/exceptions.hpp"
#include "lattice_planner/core/logger.hpp"

using namespace lattice_planner;
using namespace std::chrono;

// ---- Command-line argument parsing ----

struct CliArgs {
  std::string map_path;
  std::string lattice_path;
  std::string config_path;
  std::string output_csv;
  std::string output_json;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  Pose2D start{1.0, 1.0, 0.0};
  Pose2D goal{5.0, 5.0, 1.57};
  bool no_smooth = false;
  bool show_help = false;
};

static void printHelp()
{
  std::cout <<
    "Lattice Planner - Standalone Example\n"
    "Usage: lattice_planner_example [options]\n\n"
    "Options:\n"
    "  --map <path>          PGM map file (P5/P2). If omitted, a test map is generated.\n"
    "  --start <x,y,theta>   Start pose in world coordinates (default: 1.0,1.0,0.0)\n"
    "  --goal <x,y,theta>    Goal pose in world coordinates (default: 5.0,5.0,1.57)\n"
    "  --lattice <path>      Lattice primitives JSON file\n"
    "  --config <path>       Configuration JSON file (optional)\n"
    "  --output <path>       Output path CSV file (default: stdout)\n"
    "  --output-json <path>  Output path JSON file\n"
    "  --resolution <r>      Map resolution in meters (default: 0.05)\n"
    "  --origin <x,y>        Map origin in world coordinates (default: 0,0)\n"
    "  --no-smooth           Disable path smoothing\n"
    "  --help                Show this help message\n";
}

static bool parsePose(const std::string & s, Pose2D & pose)
{
  size_t comma1 = s.find(',');
  if (comma1 == std::string::npos) return false;
  size_t comma2 = s.find(',', comma1 + 1);
  if (comma2 == std::string::npos) return false;
  try {
    pose.x = std::stod(s.substr(0, comma1));
    pose.y = std::stod(s.substr(comma1 + 1, comma2 - comma1 - 1));
    pose.theta = std::stod(s.substr(comma2 + 1));
    return true;
  } catch (...) {
    return false;
  }
}

static bool parseOrigin(const std::string & s, double & ox, double & oy)
{
  size_t comma = s.find(',');
  if (comma == std::string::npos) return false;
  try {
    ox = std::stod(s.substr(0, comma));
    oy = std::stod(s.substr(comma + 1));
    return true;
  } catch (...) {
    return false;
  }
}

static CliArgs parseArgs(int argc, char * argv[])
{
  CliArgs args;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      args.show_help = true;
    } else if (arg == "--map" && i + 1 < argc) {
      args.map_path = argv[++i];
    } else if (arg == "--lattice" && i + 1 < argc) {
      args.lattice_path = argv[++i];
    } else if (arg == "--config" && i + 1 < argc) {
      args.config_path = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      args.output_csv = argv[++i];
    } else if (arg == "--output-json" && i + 1 < argc) {
      args.output_json = argv[++i];
    } else if (arg == "--resolution" && i + 1 < argc) {
      args.resolution = std::stod(argv[++i]);
    } else if (arg == "--origin" && i + 1 < argc) {
      if (!parseOrigin(argv[++i], args.origin_x, args.origin_y)) {
        std::cerr << "Error: invalid --origin format. Use: x,y" << std::endl;
        std::exit(1);
      }
    } else if (arg == "--start" && i + 1 < argc) {
      if (!parsePose(argv[++i], args.start)) {
        std::cerr << "Error: invalid --start format. Use: x,y,theta" << std::endl;
        std::exit(1);
      }
    } else if (arg == "--goal" && i + 1 < argc) {
      if (!parsePose(argv[++i], args.goal)) {
        std::cerr << "Error: invalid --goal format. Use: x,y,theta" << std::endl;
        std::exit(1);
      }
    } else if (arg == "--no-smooth") {
      args.no_smooth = true;
    } else {
      std::cerr << "Warning: unknown argument '" << arg << "'" << std::endl;
    }
  }
  return args;
}

// ---- Create a test map with obstacles ----

static std::unique_ptr<Costmap2D> createTestMap(double resolution, double origin_x, double origin_y)
{
  // 200x200 cells at 0.05m = 10m x 10m map
  unsigned int width = 200;
  unsigned int height = 200;
  auto costmap = MapLoader::createEmptyMap(width, height, resolution, origin_x, origin_y);

  // Vertical wall in the middle with a gap (forces planning around it)
  // Wall from y=2.0 to y=4.0 at x=5.0, thickness 0.2m
  MapLoader::addRectangle(costmap.get(), 5.0, 3.0, 0.2, 2.0);

  // Wall from y=6.0 to y=8.0 at x=5.0 (gap between y=4.0 and y=6.0)
  MapLoader::addRectangle(costmap.get(), 5.0, 7.0, 0.2, 2.0);

  // A small obstacle block
  MapLoader::addRectangle(costmap.get(), 3.0, 6.0, 0.5, 0.5);

  // Another obstacle
  MapLoader::addRectangle(costmap.get(), 7.0, 2.0, 0.5, 0.5);

  return costmap;
}

// ---- Main ----

int main(int argc, char * argv[])
{
  CliArgs args = parseArgs(argc, argv);

  if (args.show_help) {
    printHelp();
    return 0;
  }

  try {
    // 1. Load or create map
    std::unique_ptr<Costmap2D> costmap;
    if (!args.map_path.empty()) {
      std::cout << "[INFO] Loading map from: " << args.map_path << std::endl;
      costmap = MapLoader::loadFromPGM(
        args.map_path, args.resolution, args.origin_x, args.origin_y);
    } else {
      std::cout << "[INFO] No map specified, generating test map (200x200, "
                << args.resolution << "m/cell)" << std::endl;
      costmap = createTestMap(args.resolution, args.origin_x, args.origin_y);
    }

    std::cout << "[INFO] Map size: " << costmap->getSizeInCellsX() << "x"
              << costmap->getSizeInCellsY() << ", resolution="
              << costmap->getResolution() << "m" << std::endl;

    // 2. Load configuration
    LatticePlannerConfig config;
    if (!args.config_path.empty()) {
      std::cout << "[INFO] Loading config from: " << args.config_path << std::endl;
      config = LatticePlannerConfig::loadFromFile(args.config_path);
    } else {
      std::cout << "[INFO] Using default configuration" << std::endl;
      config.search_info.minimum_turning_radius = 0.5f;
      config.search_info.non_straight_penalty = 1.05f;
      config.search_info.change_penalty = 0.05f;
      config.search_info.reverse_penalty = 2.0f;
      config.search_info.cost_penalty = 2.0f;
      config.search_info.retrospective_penalty = 0.015f;
      config.search_info.rotation_penalty = 5.0f;
      config.search_info.analytic_expansion_ratio = 3.5f;
      config.search_info.analytic_expansion_max_length = 3.0f;  // meters
      config.search_info.analytic_expansion_max_cost = 200.0f;
      config.search_info.cache_obstacle_heuristic = false;
      config.search_info.allow_reverse_expansion = false;
      config.search_info.downsample_obstacle_heuristic = true;
      config.smoother_params.setDefaults();
      config.tolerance = 0.25f;
      config.allow_unknown = true;
      config.max_iterations = 1000000;
      config.max_on_approach_iterations = 1000;
      config.terminal_checking_interval = 5000;
      config.max_planning_time = 5.0;
      config.lookup_table_size = 20.0;
      config.smooth_path = !args.no_smooth;
      config.smoothing_max_time = 2.0;
      config.num_quantizations = 72;
      // Square footprint 0.2m x 0.2m
      config.footprint = {{0.1, 0.1}, {-0.1, 0.1}, {-0.1, -0.1}, {0.1, -0.1}};
      config.footprint_is_radius = false;
      config.possible_collision_cost = 0.0;
    }

    // Override lattice path if specified on command line
    if (!args.lattice_path.empty()) {
      config.search_info.lattice_filepath = args.lattice_path;
    } else if (config.search_info.lattice_filepath.empty()) {
      std::cerr << "[ERROR] No lattice file specified. Use --lattice <path> or "
                   "set lattice_filepath in config." << std::endl;
      return 1;
    }

    // Override smoothing if --no-smooth
    if (args.no_smooth) {
      config.smooth_path = false;
    }

    std::cout << "[INFO] Lattice file: " << config.search_info.lattice_filepath << std::endl;
    std::cout << "[INFO] Start: (" << args.start.x << ", " << args.start.y
              << ", " << args.start.theta << ")" << std::endl;
    std::cout << "[INFO] Goal:  (" << args.goal.x << ", " << args.goal.y
              << ", " << args.goal.theta << ")" << std::endl;

    // 3. Configure planner
    LatticePlanner planner;
    planner.configure(config, costmap.get());

    // 4. Plan path
    std::cout << "[INFO] Planning..." << std::endl;
    auto t_start = steady_clock::now();
    Path path = planner.createPlan(args.start, args.goal);
    auto t_end = steady_clock::now();
    double elapsed_ms = duration_cast<duration<double, std::milli>>(t_end - t_start).count();

    std::cout << "[INFO] Planning completed in " << elapsed_ms << " ms" << std::endl;
    PathWriter::printSummary(path);

    // 5. Output path
    if (!args.output_csv.empty()) {
      PathWriter::writeToFile(path, args.output_csv);
    } else if (!args.output_json.empty()) {
      PathWriter::writeToJSON(path, args.output_json);
    } else {
      PathWriter::writeToConsole(path);
    }

    // 6. Cleanup
    planner.cleanup();
    std::cout << "[INFO] Done." << std::endl;
    return 0;

  } catch (const StartOutsideMapBounds & e) {
    std::cerr << "[ERROR] StartOutsideMapBounds: " << e.what() << std::endl;
    return 2;
  } catch (const GoalOutsideMapBounds & e) {
    std::cerr << "[ERROR] GoalOutsideMapBounds: " << e.what() << std::endl;
    return 3;
  } catch (const StartOccupied & e) {
    std::cerr << "[ERROR] StartOccupied: " << e.what() << std::endl;
    return 4;
  } catch (const NoValidPathCouldBeFound & e) {
    std::cerr << "[ERROR] NoValidPathCouldBeFound: " << e.what() << std::endl;
    return 5;
  } catch (const PlannerTimedOut & e) {
    std::cerr << "[ERROR] PlannerTimedOut: " << e.what() << std::endl;
    return 6;
  } catch (const PlannerCancelled & e) {
    std::cerr << "[ERROR] PlannerCancelled: " << e.what() << std::endl;
    return 7;
  } catch (const PlannerException & e) {
    std::cerr << "[ERROR] PlannerException: " << e.what() << std::endl;
    return 8;
  } catch (const std::exception & e) {
    std::cerr << "[ERROR] " << e.what() << std::endl;
    return 99;
  }
}
