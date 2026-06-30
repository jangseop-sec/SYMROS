ament_add_gtest(voxel_grid_tests voxel_grid_tests.cpp)
target_link_libraries(voxel_grid_tests voxel_grid)

ament_add_gtest(voxel_grid_bresenham_3d voxel_grid_bresenham_3d.cpp)
target_link_libraries(voxel_grid_bresenham_3d voxel_grid)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-conc)
link_libraries(ros2_no_dds)
# ros2-nodds dependency
