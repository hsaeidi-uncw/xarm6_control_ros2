cmake_minimum_required(VERSION 3.8)
project(xarm6_control_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ruckig REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Plan.msg"
  DEPENDENCIES geometry_msgs std_msgs
)


add_executable(xarm6_controller src/xarm6_controller.cpp)


target_link_libraries(xarm6_controller 
  ${orocos_kdl_LIBRARIES}
)

# Ensure the include directories are present
target_include_directories(xarm6_controller PUBLIC
  ${orocos_kdl_INCLUDE_DIRS}
)

ament_target_dependencies(xarm6_controller
  rclcpp
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_default_runtime
)

install(TARGETS xarm6_controller
  DESTINATION lib/${PROJECT_NAME})
  
install(PROGRAMS
  scripts/simple_planner.py
  scripts/simple_pos_control.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
