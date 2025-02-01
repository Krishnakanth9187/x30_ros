cmake_minimum_required(VERSION 3.5)
project(axis_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        # srv/SetInt.srv
        msg/Ptz.msg
)

ament_package()
