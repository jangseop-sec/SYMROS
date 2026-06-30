ament_add_gtest(2d_utils_tests 2d_utils_test.cpp)
target_link_libraries(2d_utils_tests conversions)

ament_add_gtest(path_ops_tests path_ops_test.cpp)
target_link_libraries(path_ops_tests path_ops)

ament_add_gtest(tf_help_tests tf_help_test.cpp)
target_link_libraries(tf_help_tests tf_help conversions)




# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-sym)
link_libraries(ros2_no_dds)
include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)
# ros2-nodds dependency
