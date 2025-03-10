cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  DEPENDENCIES geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs rclcpp rclcpp_action # Add packages that the custom interfaces depend on
)

add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node rclcpp std_msgs nav_msgs sensor_msgs)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp std_msgs sensor_msgs std_srvs custom_interfaces)

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp std_msgs sensor_msgs custom_interfaces)

add_executable(patrol_with_service_node src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service_node rclcpp std_msgs nav_msgs sensor_msgs custom_interfaces)

add_executable(go_to_pose_action_node src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action_node rclcpp rclcpp_action std_msgs nav_msgs sensor_msgs custom_interfaces geometry_msgs tf2 tf2_geometry_msgs rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(go_to_pose_action_node "${cpp_typesupport_target}")

install(TARGETS
	patrol_node
	direction_service_node
	test_service_node
	patrol_with_service_node
	go_to_pose_action_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()