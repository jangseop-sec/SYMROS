# Tests
ament_add_gtest(velocity_smoother_tests
  test_velocity_smoother.cpp
)
ament_target_dependencies(velocity_smoother_tests
  ${dependencies}
)
target_link_libraries(velocity_smoother_tests
  ${library_name}
)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-sym)
link_libraries(ros2_no_dds)
include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)
# ros2-nodds dependency
