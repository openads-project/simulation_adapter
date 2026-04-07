cmake_minimum_required(VERSION 3.8)
project(simulation_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
   add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(perception_msgs_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_perception_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_trajectory_planning_msgs REQUIRED)
find_package(trajectory_planning_msgs REQUIRED)
find_package(trajectory_planning_msgs_utils REQUIRED)

set(TARGET_NAME simulation_adapter_component)

add_library(${TARGET_NAME} SHARED
  src/simulation_adapter.cpp
)

rclcpp_components_register_node(${TARGET_NAME}
  PLUGIN "simulation_adapter::SimulationAdapter"
  EXECUTABLE simulation_adapter
  EXECUTOR SingleThreadedExecutor
)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

target_link_libraries(${TARGET_NAME} PUBLIC
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${perception_msgs_TARGETS}
  ${perception_msgs_utils_TARGETS}
  ${std_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${tf2_perception_msgs_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_trajectory_planning_msgs_TARGETS}
  ${trajectory_planning_msgs_TARGETS}
  ${trajectory_planning_msgs_utils_TARGETS}
)

install(TARGETS ${TARGET_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
