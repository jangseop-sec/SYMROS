ament_add_gtest(test_behaviors
  test_behaviors.cpp
)

ament_target_dependencies(test_behaviors
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
