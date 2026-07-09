# Standalone Lattice Planner

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-%3E%3D3.14-green.svg)](https://cmake.org/)
[![OMPL](https://img.shields.io/badge/OMPL-%3E%3D1.5-orange.svg)](https://ompl.kavrakilab.org/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Windows%20%7C%20macOS-lightgrey.svg)](https://github.com/Robot-Nav/standalone_lattice_planner)
[![ROS-Free](https://img.shields.io/badge/ROS-Free-success.svg)](https://github.com/Robot-Nav/standalone_lattice_planner)

独立 State Lattice 规划器 — 从 ROS `nav2_smac_planner` 解耦的纯 C++ 实现。

本包提供基于 A* 搜索 + 离散运动基元的 Ackermann 车辆路径规划，支持 Dubins/Reeds-Shepp 解析扩展、双启发式 (2D 障碍物波前 + SE2 距离) 和共轭梯度路径平滑，**完全不依赖 ROS**。

（待测试，有兴趣的可以先测试一下然后方便的话可以提交一下issue）

---

## 特性

- **纯 C++17 实现**: 无 rclcpp / nav2 / tf2 / geometry_msgs 依赖
- **State Lattice A***: 72 方位量化，Ackermann 运动基元
- **双启发式**: 2D 障碍物波前 (Dijkstra BFS) + SE2 距离 (Dubins/RS 查表)
- **解析扩展**: 周期性尝试 Dubins/Reeds-Shepp 直达目标
- **路径平滑**: 共轭梯度法 + 运动学边界条件
- **碰撞检测**: 多边形足迹预计算 (支持 72 个旋转角度)
- **跨平台**: 支持 Linux / Windows / macOS (CMake 构建)
- **JSON 配置**: 无 ROS 参数服务器，纯 JSON 文件配置
- **PGM 地图加载**: 支持 P5/P2 格式
- **完整测试**: GoogleTest 单元测试 (52+ 用例，100% 通过)
- **线程安全**: `createPlan()` 内部互斥锁保护

---

## 依赖

### 必需依赖

| 依赖 | 版本 | 说明 |
|------|------|------|
| C++ 编译器 | C++17 | GCC 7+ / MSVC 2019+ / Clang 7+ |
| CMake | ≥ 3.14 | 构建系统 |
| OMPL | ≥ 1.5 | Dubins/Reeds-Shepp/SE2 状态空间 |
| Eigen3 | ≥ 3.3 | OMPL 间接依赖 |
| nlohmann/json | ≥ 3.0 | 配置文件和运动基元 JSON 解析 |

### 可选依赖

| 依赖 | 用途 |
|------|------|
| GoogleTest | 单元测试 (未安装时自动 FetchContent 下载) |

### 内置依赖

- `thirdparty/robin_hood.h` — 高性能哈希表 (header-only，已包含)

---

## 安装依赖

### Ubuntu / Debian

```bash
# OMPL (从 conda 或源码安装更佳)
sudo apt install libompl-dev

# Eigen3
sudo apt install libeigen3-dev

# nlohmann/json
sudo apt install nlohmann-json3-dev

# GoogleTest (可选，用于单元测试)
sudo apt install libgtest-dev

# CMake
sudo apt install cmake
```

### Conda (推荐，避免系统包冲突)

```bash
conda install -c conda-forge ompl eigen nlohmann_json gtest cmake
```

### 源码安装 OMPL

```bash
git clone https://github.com/ompl/ompl.git
cd ompl
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

---

## 编译

### 基本编译

```bash
cd standalone_lattice_planner
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 指定 OMPL 路径

若 OMPL 安装在非标准路径 (如 conda):

```bash
cmake .. -DOMPL_ROOT=/home/user/miniconda3
```

### Release 模式 (默认，带 -O2 优化)

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
```

### Debug 模式

```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug
```

### 仅编译库 (不编译示例和测试)

示例和测试默认在对应文件存在时自动编译。若需禁用:

```bash
cmake .. -DBUILD_TESTING=OFF
```

### 编译产物

| 产物 | 路径 | 说明 |
|------|------|------|
| `liblattice_planner.so` | `build/liblattice_planner.so` | 共享库 |
| `lattice_planner_example` | `build/lattice_planner_example` | CLI 示例程序 |
| `test_*` | `build/test/test_*` | 单元测试可执行文件 |

---

## 快速开始

### 运行示例 (默认测试地图)

```bash
cd build

./lattice_planner_example \
  --lattice ../examples/data/lattice/output.json \
  --config ../examples/data/default_config.json \
  --start 1.0,1.0,0.0 \
  --goal 5.0,5.0,1.57
```

输出示例:
```
[INFO] No map specified, generating test map (200x200, 0.05m/cell)
[INFO] Map size: 200x200, resolution=0.05m
[INFO] Loading config from: ../examples/data/default_config.json
[INFO] Lattice file: ../examples/data/lattice/output.json
[INFO] Start: (1, 1, 0)
[INFO] Goal:  (5, 5, 1.57)
[INFO] Planning...
[INFO] Planning completed in 2.09 ms
[INFO] Path summary: 176 points, total length = 9.99696 m
Path (176 points):
  [0] x=1.050000, y=1.050000, theta=0.000000
  [1] x=1.099976, y=1.050043, theta=0.002510
  ...
[INFO] Done.
```

### 加载 PGM 地图

```bash
./lattice_planner_example \
  --map /path/to/map.pgm \
  --resolution 0.05 \
  --origin 0,0 \
  --lattice ../examples/data/lattice/output.json \
  --start 1.0,1.0,0.0 \
  --goal 10.0,10.0,1.57 \
  --output /path/to/output.csv
```

### 输出路径到文件

```bash
# CSV 格式
./lattice_planner_example ... --output path.csv

# JSON 格式
./lattice_planner_example ... --output-json path.json
```

### 禁用平滑

```bash
./lattice_planner_example ... --no-smooth
```

### 查看帮助

```bash
./lattice_planner_example --help
```

---

## 命令行参数

| 参数 | 格式 | 默认值 | 说明 |
|------|------|--------|------|
| `--map` | `<path>` | (无，生成测试地图) | PGM 地图文件 (P5/P2) |
| `--start` | `x,y,theta` | `1.0,1.0,0.0` | 起点位姿 (世界坐标，rad) |
| `--goal` | `x,y,theta` | `5.0,5.0,1.57` | 目标位姿 (世界坐标，rad) |
| `--lattice` | `<path>` | (必填) | 运动基元 JSON 文件 |
| `--config` | `<path>` | (默认参数) | 配置 JSON 文件 |
| `--output` | `<path>` | stdout | 输出 CSV 路径文件 |
| `--output-json` | `<path>` | (无) | 输出 JSON 路径文件 |
| `--resolution` | `<r>` | `0.05` | 地图分辨率 (m/cell) |
| `--origin` | `x,y` | `0,0` | 地图原点世界坐标 |
| `--no-smooth` | (flag) | false | 禁用路径平滑 |
| `--help` | (flag) | — | 显示帮助 |

---

## 编程接口

### 最简示例

```cpp
#include "lattice_planner/lattice_planner.hpp"
#include "lattice_planner/io/map_loader.hpp"

using namespace lattice_planner;

int main() {
    // 1. 创建代价图
    auto costmap = MapLoader::createEmptyMap(200, 200, 0.05, 0.0, 0.0);
    MapLoader::addRectangle(costmap.get(), 5.0, 3.0, 0.2, 2.0);

    // 2. 加载配置
    LatticePlannerConfig config =
        LatticePlannerConfig::loadFromFile("examples/data/default_config.json");

    // 3. 配置规划器
    LatticePlanner planner;
    planner.configure(config, costmap.get());

    // 4. 规划路径
    Pose2D start{1.0, 1.0, 0.0};
    Pose2D goal{5.0, 5.0, 1.57};
    Path path = planner.createPlan(start, goal);

    // 5. 使用路径
    std::cout << "Path: " << path.poses.size() << " points" << std::endl;

    // 6. 清理
    planner.cleanup();
    return 0;
}
```

### 异常处理

```cpp
try {
    Path path = planner.createPlan(start, goal);
} catch (const StartOutsideMapBounds & e) {
    // 起点在地图外
} catch (const GoalOutsideMapBounds & e) {
    // 目标在地图外
} catch (const StartOccupied & e) {
    // 起点被障碍物占据
} catch (const NoValidPathCouldBeFound & e) {
    // 找不到路径
} catch (const PlannerTimedOut & e) {
    // 超时
} catch (const PlannerException & e) {
    // 其他规划错误
}
```

完整 API 文档见 [docs/API.md](docs/API.md)。

---

## 配置文件

配置使用 JSON 格式，完整示例见 [examples/data/default_config.json](examples/data/default_config.json)。

关键字段:

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
    "allow_reverse_expansion": false,
    "downsample_obstacle_heuristic": true
  },
  "max_planning_time": 5.0,
  "tolerance": 0.25,
  "smooth_path": true,
  "footprint": [[0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1], [0.1, -0.1]],
  "footprint_is_radius": false
}
```

所有字段均可省略，省略时使用默认值。详见 [docs/API.md](docs/API.md) 中的 `LatticePlannerConfig` 章节。

---

## 单元测试

### 运行测试

```bash
cd build
cmake .. && make -j$(nproc)
ctest --output-on-failure
```

### 测试套件

| 测试文件 | 覆盖范围 | 用例数 |
|----------|----------|--------|
| `test_costmap.cpp` | Costmap2D 构造、坐标变换、代价读写、膨胀参数 | 11 |
| `test_collision_checker.cpp` | 圆形/多边形足迹碰撞、预计算角度、越界检测 | 9 |
| `test_motion_primitives.cpp` | JSON 加载、元数据一致性、方位 bin 转换 | 6 |
| `test_node_lattice.cpp` | 节点构造、重置、代价、索引、运动模型初始化 | 12 |
| `test_a_star.cpp` | 端到端规划、避障、异常场景、配置状态 | 8 |
| `test_smoother.cpp` | 平滑器参数、空路径、端点保持、锯齿消除 | 7 |

---

## 目录结构

```
standalone_lattice_planner/
├── include/lattice_planner/     # 公共头文件
├── src/                         # 实现文件
├── examples/                    # CLI 示例 + 数据
├── test/                        # 单元测试
├── thirdparty/                  # robin_hood.h
├── scripts/                     # 测试地图生成脚本
├── docs/                        # 文档
│   ├── API.md                   # API 参考手册
│   ├── ARCHITECTURE.md          # 架构与算法原理
│   └── PERFORMANCE.md           # 性能测试报告
├── CMakeLists.txt               # 构建系统
└── README.md                    # 本文件
```

---

## 安装

```bash
cd build
sudo make install
```

安装位置 (遵循 GNUInstallDirs):

| 产物 | 安装路径 |
|------|----------|
| 共享库 | `/usr/local/lib/liblattice_planner.so` |
| 头文件 | `/usr/local/include/lattice_planner/` |
| 示例程序 | `/usr/local/bin/lattice_planner_example` |

安装后可在其他 CMake 项目中使用:

```cmake
find_package(lattice_planner REQUIRED)
target_link_libraries(your_target lattice_planner)
```

---

## 文档

- [API 参考手册](docs/API.md) — 所有公共类、结构、函数的详细说明
- [架构文档](docs/ARCHITECTURE.md) — 模块依赖、数据流、算法原理、关键公式
- [性能测试报告](docs/PERFORMANCE.md) — 规划时间、路径质量、与 ROS 版本对比

---

## 与原 nav2_smac_planner 的关系

本项目从 [nav2_smac_planner](https://github.com/ros-navigation/navigation2) 的 State Lattice 规划器移植而来，主要变更:

1. **移除全部 ROS 依赖**: rclcpp、nav2_costmap_2d、nav2_core、tf2、geometry_msgs 等
2. **门面模式**: `LatticePlanner` 替代 `SmacPlannerLatticeT<NodeLattice>`，简化生命周期
3. **JSON 配置**: 替代 ROS 参数服务器
4. **纯 C++ 类型**: `Pose2D`、`Path`、`Point2D` 替代 ROS 消息类型
5. **流式日志**: `LP_LOG_*` 宏替代 `rclcpp::Logger`
6. **PGM 地图加载**: 内置 `MapLoader` 替代 nav2 地图服务器
7. **保留核心算法**: A* 搜索、运动基元、双启发式、解析扩展、碰撞检测、路径平滑算法完全一致

详细对应关系见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) 第 8 节。
