        cmake_minimum_required(VERSION 3.8)
        project(bt_pipeline_msgs)

        find_package(ament_cmake REQUIRED)
        find_package(rosidl_default_generators REQUIRED)
        find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

        rosidl_generate_interfaces(${PROJECT_NAME}
          "msg/BtNodeSpec.msg" "msg/ExtensionManifest.msg" "msg/BtPlan.msg" "srv/LlmQuery.srv" "srv/GenerateXml.srv" "srv/ValidateTree.srv" "srv/GetCatalog.srv" "srv/RehearseTask.srv" "srv/ListMemories.srv" "action/ExecuteNode.action"
          DEPENDENCIES std_msgs action_msgs
        )

        ament_export_dependencies(rosidl_default_runtime)
        ament_package()
