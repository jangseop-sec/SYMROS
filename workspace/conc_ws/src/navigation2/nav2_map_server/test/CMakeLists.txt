set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)
set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions( -DTEST_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}\")

add_subdirectory(unit)
add_subdirectory(component)
add_subdirectory(map_saver_cli)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-conc)
link_libraries(ros2_no_dds)
# ros2-nodds dependency
