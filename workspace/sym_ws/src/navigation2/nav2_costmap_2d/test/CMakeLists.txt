set(TEST_MAP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/map)
set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)

add_subdirectory(unit)
add_subdirectory(integration)
add_subdirectory(regression)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-sym)
link_libraries(ros2_no_dds)
include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)
# ros2-nodds dependency
