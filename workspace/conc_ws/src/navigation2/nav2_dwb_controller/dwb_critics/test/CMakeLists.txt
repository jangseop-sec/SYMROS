ament_add_gtest(prefer_forward_tests prefer_forward_test.cpp)
target_link_libraries(prefer_forward_tests dwb_critics)

ament_add_gtest(base_obstacle_tests base_obstacle_test.cpp)
target_link_libraries(base_obstacle_tests dwb_critics)

ament_add_gtest(obstacle_footprint_tests obstacle_footprint_test.cpp)
target_link_libraries(obstacle_footprint_tests dwb_critics)

ament_add_gtest(alignment_util_tests alignment_util_test.cpp)
target_link_libraries(alignment_util_tests dwb_critics)

ament_add_gtest(twirling_tests twirling_test.cpp)
target_link_libraries(twirling_tests dwb_critics)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-conc)
link_libraries(ros2_no_dds)
# ros2-nodds dependency
