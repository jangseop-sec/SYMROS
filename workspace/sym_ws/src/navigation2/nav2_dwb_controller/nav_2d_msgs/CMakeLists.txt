cmake_minimum_required(VERSION 3.5)
project(nav_2d_msgs)

# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-sym)
link_libraries(ros2_no_dds)
include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)
# ros2-nodds dependency



find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(nav_2d_msgs
    "msg/Path2D.msg"
    "msg/Pose2D32.msg"
    "msg/Pose2DStamped.msg"
    "msg/Twist2D.msg"
    "msg/Twist2D32.msg"
    "msg/Twist2DStamped.msg"
    DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
