# tests for regulated PP
ament_add_gtest(test_regulated_pp
  test_regulated_pp.cpp
  path_utils/path_utils.cpp
)
ament_target_dependencies(test_regulated_pp
  ${dependencies}
)
target_link_libraries(test_regulated_pp
  ${library_name}
)

# Path utils test
ament_add_gtest(test_path_utils path_utils/test_path_utils.cpp path_utils/path_utils.cpp)
ament_target_dependencies(test_path_utils nav_msgs geometry_msgs tf2_geometry_msgs)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-sym)
link_libraries(ros2_no_dds)
include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)
# ros2-nodds dependency
