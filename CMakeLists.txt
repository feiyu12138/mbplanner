cmake_minimum_required(VERSION 3.8)
project(mbplanner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(include)
add_library(${PROJECT_NAME} SHARED src/MBPlanner.cpp src/MBRemapper.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  Eigen3
  moveit_core
  sensor_msgs
)
add_executable(remapping src/remapping.cpp)
target_include_directories(remapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(remapping PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  remapping
  rclcpp
  sensor_msgs
)
target_link_libraries(remapping ${PROJECT_NAME})

install (TARGETS
  ${PROJECT_NAME} LIBRARY
  DESTINATION lib
)
install(TARGETS remapping
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp moveit_ros_planning_interface moveit_visual_tools Eigen3 moveit_core)
ament_package()
