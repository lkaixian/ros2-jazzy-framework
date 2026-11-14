cmake_minimum_required(VERSION 3.8)
project(dstar_lite_planner)
# colcon passes no build type; without this the plugin was compiled with no optimisation (about 6x slower)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

add_library(dstar_lite_core src/core.cpp)
# a STATIC library linked into the SHARED plugin must be position independent. Without -fPIC it only links because
# Ubuntu's GCC defaults to PIE, and the optimised plugin then segfaulted planner_server (Core::reset -> insert).
set_target_properties(dstar_lite_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(dstar_lite_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(dstar_lite_planner SHARED src/planner.cpp)
target_link_libraries(dstar_lite_planner dstar_lite_core)
ament_target_dependencies(dstar_lite_planner rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d
  nav2_util nav_msgs geometry_msgs tf2_ros pluginlib)
pluginlib_export_plugin_description_file(nav2_core dstar_lite_planner.xml)

install(TARGETS dstar_lite_core dstar_lite_planner
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core dstar_lite_core)
endif()
ament_package()
