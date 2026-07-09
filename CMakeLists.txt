cmake_minimum_required(VERSION 3.14)
project(lattice_planner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Cross-platform compiler options
if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES NOMINMAX)
  add_compile_options(/W3 /wd4996 /wd4267 /wd4244)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O2)
  endif()
endif()

# ---- Dependencies ----

# OMPL (prefer conda, fall back to system)
set(OMPL_ROOT "/home/fatu08/miniconda3" CACHE PATH "OMPL installation root")
find_path(OMPL_INCLUDE_DIR
  NAMES ompl/base/StateSpace.h
  PATHS ${OMPL_ROOT}/include/ompl-1.7 /usr/include/ompl-1.5 /usr/include/ompl-1.6
  DOC "OMPL include directory")
find_library(OMPL_LIBRARY
  NAMES ompl
  PATHS ${OMPL_ROOT}/lib /usr/lib /usr/lib/x86_64-linux-gnu
  DOC "OMPL shared library")

if(NOT OMPL_INCLUDE_DIR OR NOT OMPL_LIBRARY)
  message(FATAL_ERROR "OMPL not found. Set OMPL_ROOT to your OMPL installation.")
endif()
message(STATUS "OMPL includes: ${OMPL_INCLUDE_DIR}")
message(STATUS "OMPL library:  ${OMPL_LIBRARY}")

# Eigen3
find_path(EIGEN3_INCLUDE_DIR
  NAMES Eigen/Core
  PATHS /usr/include/eigen3 ${OMPL_ROOT}/include/eigen3
  DOC "Eigen3 include directory")
if(NOT EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "Eigen3 not found.")
endif()
message(STATUS "Eigen3 includes: ${EIGEN3_INCLUDE_DIR}")

# nlohmann/json
find_path(NLOHMANN_JSON_INCLUDE_DIR
  NAMES nlohmann/json.hpp
  PATHS ${OMPL_ROOT}/include /usr/include
  DOC "nlohmann/json include directory")
if(NOT NLOHMANN_JSON_INCLUDE_DIR)
  message(FATAL_ERROR "nlohmann/json not found.")
endif()
message(STATUS "nlohmann/json:   ${NLOHMANN_JSON_INCLUDE_DIR}")

# ---- Include directories ----
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
  ${OMPL_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${NLOHMANN_JSON_INCLUDE_DIR}
)

# ---- Library sources ----
set(LATTICE_PLANNER_SOURCES
  src/costmap/costmap_2d.cpp
  src/collision/collision_checker.cpp
  src/state_space/node_lattice.cpp
  src/heuristic/obstacle_heuristic.cpp
  src/heuristic/distance_heuristic.cpp
  src/smoother/smoother.cpp
  src/lattice_planner.cpp
)

# ---- Shared library ----
add_library(lattice_planner SHARED ${LATTICE_PLANNER_SOURCES})
target_link_libraries(lattice_planner ${OMPL_LIBRARY})
set_target_properties(lattice_planner PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1)

# ---- Example executable ----
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/main.cpp")
  add_executable(lattice_planner_example examples/main.cpp)
  target_link_libraries(lattice_planner_example lattice_planner)
endif()

# ---- Tests ----
enable_testing()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()

# ---- Install ----
include(GNUInstallDirs)
install(TARGETS lattice_planner
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/lattice_planner
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp")

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/main.cpp")
  install(TARGETS lattice_planner_example
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
