cmake_minimum_required(VERSION 3.8)
project(cluster_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JobStatus.msg"
  "msg/ServiceEndpoint.msg"
  "msg/NodeInfo.msg"
  "msg/Assignment.msg"
  "msg/AssignmentSet.msg"
  "msg/JobPlacement.msg"
  "msg/ClusterStatus.msg"
  "srv/HostInfo.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
