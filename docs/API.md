# Lattice Planner — API Reference

> 纯 C++ State Lattice 规划器 API 文档。所有类型与函数位于 `lattice_planner` 命名空间下，无任何 ROS 依赖。

## 目录

- [快速开始](#快速开始)
- [核心类 LatticePlanner](#核心类-latticeplanner)
- [配置结构体](#配置结构体)
  - [LatticePlannerConfig](#latticeplannerconfig)
  - [SearchInfo](#searchinfo)
  - [SmootherParams](#smootherparams)
- [Costmap2D](#costmap2d)
- [GridCollisionChecker](#gridcollisionchecker)
- [MapLoader](#maploader)
- [PathWriter](#pathwriter)
- [基础类型](#基础类型)
- [异常层次](#异常层次)
- [日志系统](#日志系统)
- [枚举常量](#枚举常量)

---

## 快速开始

```cpp
#include "lattice_planner/lattice_planner.hpp"
#include "lattice_planner/io/map_loader.hpp"
#include "lattice_planner/io/path_writer.hpp"

using namespace lattice_planner;

// 1. 创建地图（外部拥有所有权）
auto costmap = MapLoader::createEmptyMap(200, 200, 0.05, 0.0, 0.0);
MapLoader::addRectangle(costmap.get(), 5.0, 3.0, 0.2, 2.0);  // 添加障碍

// 2. 配置规划器
LatticePlannerConfig config = LatticePlannerConfig::loadFromFile("config.json");
// 或手动填充 config 字段...

LatticePlanner planner;
planner.configure(config, costmap.get());

// 3. 规划路径
Pose2D start{1.0, 1.0, 0.0};
Pose2D goal{8.0, 8.0, 1.57};
Path path = planner.createPlan(start, goal);

// 4. 输出
PathWriter::writeToConsole(path);
PathWriter::writeToFile(path, "path.csv");

// 5. 清理
planner.cleanup();
```

---

## 核心类 LatticePlanner

`LatticePlanner` 是规划器的主门面类，替代原 ROS 版本的 `SmacPlannerLatticeT<NodeLattice>`。内部拥有 A* 搜索引擎、碰撞检测器和可选的路径平滑器。`createPlan` 方法通过内部 `std::mutex` 保证线程安全。

头文件：`lattice_planner/lattice_planner.hpp`

### 构造与析构

```cpp
LatticePlanner();
~LatticePlanner();
```

默认构造的实例处于未配置状态，需调用 `configure()` 后才能使用。析构函数自动调用 `cleanup()`。

### configure

```cpp
void configure(const LatticePlannerConfig & config, Costmap2D * costmap);
```

配置规划器。替代 ROS 的 `on_configure` 生命周期回调。

| 参数 | 类型 | 说明 |
|------|------|------|
| `config` | `const LatticePlannerConfig &` | 规划器配置（搜索参数、平滑参数、footprint、lattice 路径等） |
| `costmap` | `Costmap2D *` | 指向外部 costmap。**不获取所有权**，调用者需保证其在 `cleanup()` 前有效 |

**内部流程**：
1. 从 `lattice_filepath` 加载 LatticeMetadata
2. 将 `minimum_turning_radius` 从米转换为网格单元
3. 将 `analytic_expansion_max_length` 从米转换为网格单元
4. 验证 `coarse_search_resolution` 是 `number_of_headings` 的因子
5. 创建 `GridCollisionChecker`（72 角度分箱）
6. 创建 `AStarAlgorithm<NodeLattice>` 搜索引擎并初始化
7. 若 `smooth_path` 为真，创建 `Smoother`

**异常**：若 lattice 文件无法加载或配置无效，抛出 `PlannerException`。

### createPlan

```cpp
Path createPlan(
    const Pose2D & start,
    const Pose2D & goal,
    std::function<bool()> cancel_checker = nullptr);
```

规划从 `start` 到 `goal` 的路径。线程安全（内部 mutex 保护）。

| 参数 | 类型 | 说明 |
|------|------|------|
| `start` | `const Pose2D &` | 起点位姿（世界坐标，theta 为弧度） |
| `goal` | `const Pose2D &` | 目标位姿（世界坐标） |
| `cancel_checker` | `std::function<bool()>` | 可选取消回调，返回 `true` 则中止规划 |

**返回值**：`Path` 对象，包含世界坐标路径点序列（start→goal 顺序）。

**异常**：

| 异常类型 | 触发条件 |
|---------|---------|
| `PlannerException` | 未调用 `configure()` |
| `StartOutsideMapBounds` | 起点在 costmap 边界外 |
| `GoalOutsideMapBounds` | 目标在 costmap 边界外 |
| `StartOccupied` | 起点格子被障碍占据 |
| `NoValidPathCouldBeFound` | 在迭代限制内未找到路径 |
| `PlannerTimedOut` | 超过最大迭代次数 |
| `PlannerCancelled` | `cancel_checker` 返回 true |

### cleanup

```cpp
void cleanup();
```

释放内部资源（A* 引擎、平滑器、碰撞检测器）。costmap 指针不被释放（外部所有权）。可安全重复调用（幂等）。

### isConfigured

```cpp
bool isConfigured() const;
```

返回 `true` 若 `configure()` 已成功调用且未 `cleanup()`。

---

## 配置结构体

### LatticePlannerConfig

头文件：`lattice_planner/core/config.hpp`

顶层配置结构体，整合原 ROS 参数声明。

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `motion_model` | `MotionModel` | `STATE_LATTICE` | 运动模型 |
| `lattice_filepath` | `std::string` | `""` | Lattice 基元 JSON 文件路径 |
| `search_info` | `SearchInfo` | — | 搜索惩罚参数 |
| `smoother_params` | `SmootherParams` | — | 平滑器参数 |
| `allow_unknown` | `bool` | `false` | 是否允许穿越未知区域 |
| `max_iterations` | `int` | `1000000` | A* 最大迭代次数 |
| `max_on_approach_iterations` | `int` | `INT_MAX` | 接近目标时的最大迭代 |
| `terminal_checking_interval` | `int` | `5000` | 终端检查间隔 |
| `max_planning_time` | `double` | `5.0` | 最大规划时间（秒） |
| `lookup_table_size` | `double` | `400.0` | 启发式查表尺寸（像素） |
| `angle_quantization` | `unsigned int` | `72` | 角度量化数 |
| `tolerance` | `float` | `2.0` | 目标容差（米） |
| `goal_heading_mode` | `GoalHeadingMode` | `DEFAULT` | 目标朝向模式 |
| `coarse_search_resolution` | `int` | `1` | 粗搜索分辨率 |
| `smooth_path` | `bool` | `true` | 是否启用平滑 |
| `smoothing_max_time` | `double` | `2.0` | 平滑最大时间（秒） |
| `footprint` | `std::vector<Point2D>` | `{}` | 机器人足迹多边形（机器人坐标系，米） |
| `footprint_is_radius` | `bool` | `false` | 若为 true，足迹视为圆形 |
| `possible_collision_cost` | `double` | `0.0` | 碰撞代价阈值（0=自动计算） |
| `num_quantizations` | `unsigned int` | `72` | 足迹角度预计算分箱数 |
| `inflation_radius` | `double` | `0.0` | 膨胀半径（米） |
| `inscribed_radius` | `double` | `0.0` | 内切半径（米） |
| `circumscribed_radius` | `double` | `0.0` | 外接半径（米） |

**静态方法**：

```cpp
static LatticePlannerConfig loadFromFile(const std::string & json_path);
static LatticePlannerConfig loadFromJson(const nlohmann::json & j);
```

从 JSON 文件或 JSON 对象加载配置。所有字段可选，缺失时使用默认值。

**JSON 示例**：
```json
{
  "motion_model": "STATE_LATTICE",
  "lattice_filepath": "examples/data/lattice/output.json",
  "search_info": {
    "minimum_turning_radius": 0.5,
    "non_straight_penalty": 1.05,
    "change_penalty": 0.05,
    "reverse_penalty": 2.0,
    "cost_penalty": 2.0,
    "analytic_expansion_ratio": 3.5,
    "analytic_expansion_max_length": 3.0
  },
  "smoother_params": {
    "tolerance": 1e-10,
    "max_iterations": 1000,
    "w_data": 0.2,
    "w_smooth": 0.3
  },
  "max_iterations": 1000000,
  "max_planning_time": 5.0,
  "tolerance": 0.25,
  "allow_unknown": true,
  "smooth_path": true,
  "footprint": [[0.1,0.1], [-0.1,0.1], [-0.1,-0.1], [0.1,-0.1]],
  "footprint_is_radius": false,
  "num_quantizations": 72
}
```

### SearchInfo

搜索属性与惩罚参数。

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `minimum_turning_radius` | `float` | `8.0` | 最小转弯半径（网格单元，configure 时自动转换） |
| `non_straight_penalty` | `float` | `1.05` | 非直线运动惩罚 |
| `change_penalty` | `float` | `0.0` | 转向变更惩罚 |
| `reverse_penalty` | `float` | `2.0` | 倒车惩罚 |
| `cost_penalty` | `float` | `2.0` | 代价惩罚因子 |
| `retrospective_penalty` | `float` | `0.015` | 回顾惩罚 |
| `rotation_penalty` | `float` | `5.0` | 旋转惩罚 |
| `analytic_expansion_ratio` | `float` | `3.5` | 解析展开比率 |
| `analytic_expansion_max_length` | `float` | `60.0` | 解析展开最大长度（米，configure 时转网格） |
| `analytic_expansion_max_cost` | `float` | `200.0` | 解析展开最大代价 |
| `lattice_filepath` | `std::string` | `""` | Lattice 基元文件路径 |
| `cache_obstacle_heuristic` | `bool` | `false` | 是否缓存障碍启发式 |
| `allow_reverse_expansion` | `bool` | `false` | 是否允许倒车扩展 |
| `downsample_obstacle_heuristic` | `bool` | `true` | 是否下采样障碍启发式 |

### SmootherParams

```cpp
struct SmootherParams {
  void setDefaults();
  double tolerance_{1e-10};
  int max_its_{1000};
  double w_data_{0.2};      // 数据项权重
  double w_smooth_{0.3};    // 平滑项权重
  bool holonomic_{false};   // 全向移动模式
  bool do_refinement_{true};
  int refinement_num_{2};
};
```

---

## Costmap2D

头文件：`lattice_planner/costmap/costmap_2d.hpp`

2D 网格代价地图，替代 `nav2_costmap_2d::Costmap2D`。

### 构造函数

```cpp
Costmap2D();
Costmap2D(unsigned int size_x, unsigned int size_y, double resolution,
          double origin_x, double origin_y, unsigned char default_value = 0);
Costmap2D(const Costmap2D & other);  // 拷贝构造
```

| 参数 | 说明 |
|------|------|
| `size_x`, `size_y` | 地图宽高（格子数） |
| `resolution` | 分辨率（米/格子） |
| `origin_x`, `origin_y` | 地图原点世界坐标 |
| `default_value` | 默认代价值（0=自由） |

### 坐标转换

```cpp
// 世界 → 地图（离散）。越界返回 false。
bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;

// 地图 → 世界（cell-center 约定：wx = origin + (mx + 0.5) * resolution）
void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;

// 世界 → 地图（连续浮点）。越界返回 false。
bool worldToMapContinuous(double wx, double wy, double & mx_d, double & my_d) const;
```

### 代价访问

```cpp
unsigned char getCost(unsigned int mx, unsigned int my) const;
unsigned char getCost(unsigned int index) const;
void setCost(unsigned int mx, unsigned int my, unsigned char cost);
void setCost(unsigned int index, unsigned char cost);
unsigned char * getCharMap();               // 直接访问底层数组
unsigned int getIndex(unsigned int mx, unsigned int my) const;  // mx,my → 线性索引
```

### 尺寸/几何

```cpp
unsigned int getSizeInCellsX() const;
unsigned int getSizeInCellsY() const;
double getSizeInMetersX() const;
double getSizeInMetersY() const;
double getResolution() const;
double getOriginX() const;
double getOriginY() const;
```

### 地图操作

```cpp
void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);
void clearEntirely();  // 全部重置为 default_value
```

### 膨胀参数

```cpp
void setInflationParameters(double inflation_radius,
                            double inscribed_radius,
                            double circumscribed_radius);
double getInflationRadius() const;
double getInscribedRadius() const;
double getCircumscribedRadius() const;
double getCircumscribedCost() const;  // 自动计算或返回 -1
void setCircumscribedCost(double cost);
```

`getCircumscribedCost()` 使用 nav2 膨胀代价模型自动计算：
```
cost = 252 * exp(-a * (d - inscribed_radius))
a = ln(252/253) / (inflation_radius - inscribed_radius)
```

---

## GridCollisionChecker

头文件：`lattice_planner/collision/collision_checker.hpp`

网格碰撞检测器，支持圆形和多边形足迹，预计算多角度足迹。

### 构造

```cpp
GridCollisionChecker(Costmap2D * costmap, unsigned int num_quantizations);
GridCollisionChecker(Costmap2D * costmap, const std::vector<float> & angles);
```

### 设置足迹

```cpp
void setFootprint(const Footprint & footprint,
                  const bool & radius,
                  const double & possible_collision_cost);
```

| 参数 | 说明 |
|------|------|
| `footprint` | 足迹多边形点集（机器人坐标系） |
| `radius` | true=圆形足迹（仅检查中心代价），false=多边形 |
| `possible_collision_cost` | 低于此代价时不做完整足迹检查（加速） |

### 碰撞检测

```cpp
bool inCollision(const float & x, const float & y,
                 const float & theta_bin, const bool & traverse_unknown);
bool inCollision(const unsigned int & index, const bool & traverse_unknown);
float getCost();  // 返回最后一次检测的代价
```

---

## MapLoader

头文件：`lattice_planner/io/map_loader.hpp`

PGM 地图加载与测试地图生成工具。所有方法为静态。

```cpp
static std::unique_ptr<Costmap2D> loadFromPGM(
    const std::string & filepath,
    double resolution = 0.05,
    double origin_x = 0.0, double origin_y = 0.0,
    unsigned char occupied_threshold = 200,
    unsigned char free_threshold = 50);

static std::unique_ptr<Costmap2D> createEmptyMap(
    unsigned int width, unsigned int height,
    double resolution = 0.05,
    double origin_x = 0.0, double origin_y = 0.0);

static void addRectangle(Costmap2D * costmap,
    double center_x, double center_y,
    double size_x, double size_y,
    unsigned char cost = OCCUPIED_COST);
```

**PGM 像素映射**：
- 像素值 ≥ `occupied_threshold` → `OCCUPIED_COST`(254)
- 像素值 ≤ `free_threshold` → `FREE_COST`(0)
- 其他 → 按比例缩放 `pixel * 254 / maxval`

PGM 行从上到下存储，costmap 行从下到上（加载时翻转）。

---

## PathWriter

头文件：`lattice_planner/io/path_writer.hpp`

路径输出工具。所有方法为静态。

```cpp
static bool writeToFile(const Path & path, const std::string & filepath);    // CSV
static bool writeToJSON(const Path & path, const std::string & filepath);    // JSON 数组
static void writeToConsole(const Path & path);                               // 标准输出
static void printSummary(const Path & path);                                  // 摘要
```

**CSV 格式**：
```
x,y,theta
1.000000,1.000000,0.000000
1.050716,1.002579,0.101606
...
```

**JSON 格式**：
```json
[
  {"x": 1.000000, "y": 1.000000, "theta": 0.000000},
  ...
]
```

---

## 基础类型

头文件：`lattice_planner/core/types.hpp`

```cpp
struct Point2D { double x, y; };
struct Pose2D { double x, y, theta; };        // theta 弧度
struct Path { std::vector<Pose2D> poses; std::string frame_id; };
struct Coordinates { float x, y, theta; };     // 网格坐标（搜索空间）
struct Coordinates2D { float x, y; };
typedef std::vector<Coordinates> CoordinateVector;
typedef std::vector<Point2D> Footprint;
```

**角度工具函数**：
```cpp
inline double normalizeAngle(double angle);              // → [0, 2π)
inline double shortestAngularDistance(double a, double b); // → [-π, π]
inline void yawToQuaternion(double theta, double & qz, double & qw);
inline double quaternionToYaw(double qz, double qw);
```

---

## 异常层次

头文件：`lattice_planner/core/exceptions.hpp`

```
std::runtime_error
  └─ PlannerException              基类
       ├─ StartOutsideMapBounds    起点越界
       ├─ GoalOutsideMapBounds     目标越界
       ├─ StartOccupied            起点被占据
       ├─ GoalOccupied             目标被占据
       ├─ NoValidPathCouldBeFound  无可行路径
       ├─ PlannerTimedOut          规划超时
       └─ PlannerCancelled         被取消
```

所有异常继承 `std::runtime_error`，可通过 `what()` 获取错误描述。

---

## 日志系统

头文件：`lattice_planner/core/logger.hpp`

流式日志宏（替代 `RCLCPP_*`），支持 `<<` 链式调用：

```cpp
LP_LOG_DEBUG("debug info " << value);
LP_LOG_INFO("planning from " << start.x << " to " << goal.x);
LP_LOG_WARN("inflation not set: radius=" << radius);
LP_LOG_ERROR("failed: " << e.what());
```

**日志级别控制**：
```cpp
Logger::instance().setLevel(LogLevel::WARN);  // 仅输出 WARN 和 ERROR
```

级别枚举：`DEBUG=0, INFO=1, WARN=2, ERROR=3`。默认 `INFO`。线程安全。

---

## 枚举常量

头文件：`lattice_planner/core/constants.hpp`

**MotionModel**：
```cpp
enum class MotionModel { UNKNOWN, TWOD, DUBIN, REEDS_SHEPP, STATE_LATTICE, OMNI };
std::string toString(MotionModel m);
MotionModel fromString(const std::string & s);
```

**GoalHeadingMode**：
```cpp
enum class GoalHeadingMode { UNKNOWN, DEFAULT, BIDIRECTIONAL, ALL_DIRECTION };
```

**代价常量**：
| 常量 | 值 | 说明 |
|------|----|------|
| `FREE_COST` | 0 | 自由空间 |
| `UNKNOWN_COST` | 255 | 未知区域 |
| `OCCUPIED_COST` | 254 | 障碍 |
| `INSCRIBED_COST` | 253 | 内切区域 |
| `MAX_NON_OBSTACLE_COST` | 252 | 最大非障碍代价 |
