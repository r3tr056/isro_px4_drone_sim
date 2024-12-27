cmake_minimum_required(VERSION 3.8)
project(px4_drone_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(xacro REQUIRED)


# set(PX4_PATH "/home/retro/PX4-Autopilot")
# list(APPEND CMAKE_PREFIX_PATH ${PX4_PATH})
# find_package(px4 REQUIRED)

# install directories
install (DIRECTORY
  launch
  worlds
  models
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# install python scripts
install(PROGRAMS
  scripts/launch_simulation.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
