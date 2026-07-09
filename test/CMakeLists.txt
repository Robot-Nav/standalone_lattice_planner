# Lattice Planner - Unit Tests (GoogleTest)
#
# Tests link against the `lattice_planner` shared library built by the parent
# project. GoogleTest is preferred via find_package (system-installed
# libgtest-dev); if unavailable, FetchContent pulls it from GitHub.

find_package(GTest QUIET CONFIG)

if(NOT GTest_FOUND)
  message(STATUS "System GTest not found, fetching via FetchContent")
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

# Data directory macro for tests that load the lattice primitives JSON.
set(LATTICE_DATA_DIR "${CMAKE_SOURCE_DIR}/examples/data/lattice")

# Helper: register one test executable
function(add_lattice_test name src)
  add_executable(${name} ${src})
  target_link_libraries(${name} PRIVATE lattice_planner GTest::gtest_main)
  target_compile_definitions(${name} PRIVATE
    LATTICE_DATA_DIR="${LATTICE_DATA_DIR}")
  add_test(NAME ${name} COMMAND ${name})
endfunction()

add_lattice_test(test_costmap          test_costmap.cpp)
add_lattice_test(test_collision        test_collision_checker.cpp)
add_lattice_test(test_motion_prims     test_motion_primitives.cpp)
add_lattice_test(test_node_lattice     test_node_lattice.cpp)
add_lattice_test(test_planner          test_a_star.cpp)
add_lattice_test(test_smoother         test_smoother.cpp)
