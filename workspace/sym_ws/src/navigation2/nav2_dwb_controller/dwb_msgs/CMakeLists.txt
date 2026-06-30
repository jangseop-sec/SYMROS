cmake_minimum_required(VERSION 3.5)
project(dwb_msgs)

# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-sym)
link_libraries(ros2_no_dds)
include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)
# ros2-nodds dependency



find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(dwb_msgs
    "msg/CriticScore.msg"
    "msg/LocalPlanEvaluation.msg"
    "msg/Trajectory2D.msg"
    "msg/TrajectoryScore.msg"
    "srv/DebugLocalPlan.srv"
    "srv/GenerateTrajectory.srv"
    "srv/GenerateTwists.srv"
    "srv/GetCriticScore.srv"
    "srv/ScoreTrajectory.srv"
    DEPENDENCIES geometry_msgs std_msgs nav_2d_msgs nav_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
