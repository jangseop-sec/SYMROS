# tests
find_package(nav2_controller REQUIRED)

ament_add_gtest(test_shim_controller
  test_shim_controller.cpp
)
ament_target_dependencies(test_shim_controller
  ${dependencies}
  nav2_controller
)
target_link_libraries(test_shim_controller
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
