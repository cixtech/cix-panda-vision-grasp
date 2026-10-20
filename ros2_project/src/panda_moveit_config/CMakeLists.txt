cmake_minimum_required(VERSION 3.22)
project(panda_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(PROGRAMS
  scripts/arm_control_from_UI.py
  scripts/grasp_candidate_bridge.py
  scripts/target_point_from_obb.py
  scripts/world_object_poses.py
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS config/ros2_dds_localhost.sh DESTINATION share/${PROJECT_NAME}/config)
