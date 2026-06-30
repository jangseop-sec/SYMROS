cmake_minimum_required(VERSION 3.8)
project(symros_exec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# target app

# SYMROS find target application
find_package(robot_localization REQUIRED)
# SYMROS find target application

# SYMROS find target's dependencies
include_directories("/home/ubuntu/symros/external/geographiclib/include")
find_library(GEOGRAPHICLIB_LIB NAMES GeographicLib Geographic
  HINTS "/home/ubuntu/symros/external/geographiclib/lib" NO_DEFAULT_PATH REQUIRED)
link_libraries(${GEOGRAPHICLIB_LIB})
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
# find_package(GeographicLib REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_cppcheck REQUIRED)
find_package(ament_cmake_cpplint REQUIRED)
find_package(ament_cmake_lint_cmake REQUIRED)
find_package(ament_cmake_uncrustify REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(statistics_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rmw_dds_common REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rosbag2_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(bond REQUIRED)
find_package(turtlesim REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

get_filename_component(RL_PREFIX "${robot_localization_DIR}/../../.." ABSOLUTE)

find_library(RL_LIB rl_lib
  HINTS "${RL_PREFIX}/lib" REQUIRED)
find_library(RL_TS_CPP robot_localization__rosidl_typesupport_cpp
  HINTS "${RL_PREFIX}/lib" REQUIRED)
find_library(RL_TS_INTROSPECTION robot_localization__rosidl_typesupport_introspection_cpp
  HINTS "${RL_PREFIX}/lib" REQUIRED)

include_directories("${RL_PREFIX}/include")
link_libraries(${RL_LIB} ${RL_TS_CPP} ${RL_TS_INTROSPECTION})
# SYMROS find target's dependencies

set(dependencies
  # SYMROS set dependencies
ament_cmake
angles
rclcpp
diagnostic_msgs
diagnostic_updater
geographic_msgs
geometry_msgs
message_filters
nav_msgs
rosidl_default_generators
sensor_msgs
std_srvs
tf2
tf2_eigen
tf2_geometry_msgs
tf2_ros
Eigen3
Boost
yaml_cpp_vendor
# GeographicLib
ament_cmake_gtest
ament_cmake_cppcheck
ament_cmake_cpplint
ament_cmake_lint_cmake
ament_cmake_uncrustify
launch_testing_ament_cmake
unique_identifier_msgs
map_msgs
actionlib_msgs
visualization_msgs
statistics_msgs
action_msgs
rmw_dds_common
trajectory_msgs
example_interfaces
tf2_msgs
stereo_msgs
shape_msgs
std_msgs
nav_2d_msgs
rosgraph_msgs
rosbag2_interfaces
rcl_interfaces
bond
turtlesim
builtin_interfaces
dwb_msgs
lifecycle_msgs
nav2_msgs
robot_localization
# SYMROS set dependencies
)

# TODO ros2-nodds dependency
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-sym)
link_libraries(ros2_no_dds)
include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)

# build executable
add_executable(exec src/main.cpp)
ament_target_dependencies(exec ${dependencies})



# install
install(TARGETS
  exec
  DESTINATION lib/${PROJECT_NAME})

# test
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

ament_package()