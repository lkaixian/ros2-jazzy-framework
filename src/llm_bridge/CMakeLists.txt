cmake_minimum_required(VERSION 3.8)
project(llm_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bt_pipeline_msgs REQUIRED)
find_package(cluster_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(llama_client src/llama_client.cpp src/failover.cpp)
target_include_directories(llama_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(llama_client PUBLIC CURL::libcurl nlohmann_json::nlohmann_json)

add_executable(llm_bridge_node src/llm_bridge_node.cpp)
target_link_libraries(llm_bridge_node llama_client)
ament_target_dependencies(llm_bridge_node rclcpp ament_index_cpp std_msgs std_srvs bt_pipeline_msgs cluster_msgs)

install(TARGETS llama_client llm_bridge_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_llama_client test/test_llama_client.cpp)
  target_link_libraries(test_llama_client llama_client)
  ament_add_gtest(test_failover test/test_failover.cpp)
  target_link_libraries(test_failover llama_client)
endif()

ament_package()
