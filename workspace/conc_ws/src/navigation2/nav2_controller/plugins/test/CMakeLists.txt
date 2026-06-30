ament_add_gtest(pctest progress_checker.cpp)
target_link_libraries(pctest simple_progress_checker pose_progress_checker)
ament_add_gtest(gctest goal_checker.cpp)
target_link_libraries(gctest simple_goal_checker stopped_goal_checker)






# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-conc)
link_libraries(ros2_no_dds)
# ros2-nodds dependency
