cmake_minimum_required(VERSION 3.5)
project(tf2_msgs)

# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-conc)
link_libraries(ros2_no_dds)
# ros2-nodds dependency



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TF2Error.msg"
  "msg/TFMessage.msg"
  "srv/FrameGraph.srv"
  "action/LookupTransform.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs action_msgs
  ADD_LINTER_TESTS
)

install(
  FILES tfmessage_bridge_mapping_rule.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
