cmake_minimum_required(VERSION 3.8)
project(x30_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Source files
set(SOURCE_FILES
  src/command.cpp
  src/command_list.cpp
  src/main.cpp
  src/motion_sdk_move_robot.cpp
  src/parse_cmd.cpp
  src/send_to_robot.cpp
)

# Create executable
add_executable(${PROJECT_NAME}_node ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs sensor_msgs Boost trajectory_msgs)
# target_link_libraries(${PROJECT_NAME}_node deeprobotics_${ROBOT_NAME}_motion_sdk_${CMAKE_SYSTEM_PROCESSOR})

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
