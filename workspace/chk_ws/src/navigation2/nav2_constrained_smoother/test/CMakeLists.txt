ament_add_gtest(test_constrained_smoother
  test_constrained_smoother.cpp
)

target_link_libraries(test_constrained_smoother
  ${library_name}
)
ament_target_dependencies(test_constrained_smoother
  ${dependencies}
)

ament_add_gtest(test_smoother_cost_function
  test_smoother_cost_function.cpp
)
target_link_libraries(test_smoother_cost_function
  ${library_name}
)
ament_target_dependencies(test_smoother_cost_function
  ${dependencies}
)


# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-sym)
link_libraries(ros2_no_dds)
include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)
# ros2-nodds dependency
