add_library(map_lib SHARED
  map.c
  map_range.c
  map_draw.c
  map_cspace.cpp
)

install(TARGETS
  map_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-sym)
link_libraries(ros2_no_dds)
include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)
# ros2-nodds dependency
