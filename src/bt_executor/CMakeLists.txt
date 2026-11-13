cmake_minimum_required(VERSION 3.8)
project(bt_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(bt_pipeline_msgs REQUIRED)
find_package(object_detection_msgr REQUIRED)
find_package(person_detection_msgr REQUIRED)
find_package(face_recognition_msgr REQUIRED)

add_library(bt_executor_lib src/registry.cpp)
target_include_directories(bt_executor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bt_executor_lib PUBLIC behaviortree_cpp::behaviortree_cpp)
ament_target_dependencies(bt_executor_lib PUBLIC rclcpp rclcpp_action bt_pipeline_msgs
  object_detection_msgr person_detection_msgr face_recognition_msgr)

add_executable(bt_executor_node src/bt_executor_node.cpp)
target_link_libraries(bt_executor_node bt_executor_lib)
ament_target_dependencies(bt_executor_node std_msgs std_srvs)
target_link_libraries(bt_executor_node nlohmann_json::nlohmann_json)

install(TARGETS bt_executor_lib bt_executor_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_registry test/test_registry.cpp)
  target_link_libraries(test_registry bt_executor_lib)
endif()

ament_package()
