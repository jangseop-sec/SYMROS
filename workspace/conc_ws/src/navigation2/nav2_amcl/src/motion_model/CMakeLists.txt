add_library(motions_lib SHARED
  omni_motion_model.cpp
  differential_motion_model.cpp
)
target_link_libraries(motions_lib pf_lib)
ament_target_dependencies(motions_lib
  pluginlib
  nav2_util
)

install(TARGETS
  motions_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-conc)
link_libraries(ros2_no_dds)
# ros2-nodds dependency
