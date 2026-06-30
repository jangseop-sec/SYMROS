ament_add_gtest(test_bt_conversions test_bt_conversions.cpp)
ament_target_dependencies(test_bt_conversions ${dependencies})

add_subdirectory(plugins/condition)
add_subdirectory(plugins/decorator)
add_subdirectory(plugins/control)
add_subdirectory(plugins/action)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-conc)
link_libraries(ros2_no_dds)
# ros2-nodds dependency
