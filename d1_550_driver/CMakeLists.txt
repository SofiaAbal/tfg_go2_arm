cmake_minimum_required(VERSION 3.8)
project(d1_550_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

set(D1_SDK_SRC /home/sofi/Documents/unitree_d1_teleoperation/d1_sdk/src)

# --- Plugin (no SDK, no DDS conflict) ---
add_library(d1_550_driver SHARED src/d1_hardware.cpp)
target_include_directories(d1_550_driver PUBLIC include)
ament_target_dependencies(d1_550_driver hardware_interface pluginlib rclcpp sensor_msgs)
pluginlib_export_plugin_description_file(hardware_interface d1_550_driver_plugin.xml)
install(TARGETS d1_550_driver DESTINATION lib)

# --- Bridge (raw CycloneDDS C/C++ API, no Unitree SDK wrapper) ---
# libunitree_sdk2.a is NOT needed: we bypass ChannelFactory entirely.
# The only SDK dependency is the IDL-generated message types (ArmString_, PubServoInfo_).
#add_executable(d1_bridge
#  src/d1_bridge.cpp
#  ${D1_SDK_SRC}/msg/ArmString_.cpp
#  ${D1_SDK_SRC}/msg/PubServoInfo_.cpp
#)
#target_include_directories(d1_bridge PRIVATE
#  /usr/local/include        # dds/dds.h (CycloneDDS C API)
#  /usr/local/include/ddscxx # dds/dds.hpp (CycloneDDS C++ API)
#  ${D1_SDK_SRC}             # msg/ArmString_.hpp, msg/PubServoInfo_.hpp
#)

#ament_target_dependencies(d1_bridge rclcpp sensor_msgs)
#target_link_libraries(d1_bridge ddsc ddscxx pthread)
#install(TARGETS d1_bridge DESTINATION lib/${PROJECT_NAME})

# Python bridge (uses bundled cyclonedds — no iceoryx conflict)
install(PROGRAMS 
  scripts/custom_d1_driver.py
  scripts/d1_driver.py
  scripts/d1_arm.py
  DESTINATION lib/${PROJECT_NAME}
)

# CycloneDDS config (network interface to the physical arm)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
