cmake_minimum_required(VERSION 3.22)
project(d1_550_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

include_directories(include)

add_executable(d1_550
  src/main.cpp
  src/tasks/pick_and_place.cpp
)

ament_target_dependencies(d1_550
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_task_constructor_core
)

install(TARGETS d1_550
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()