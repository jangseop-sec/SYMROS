# Bresenham2D corner cases test
ament_add_gtest(costmap_bresenham_2d costmap_bresenham_2d.cpp)
target_link_libraries(costmap_bresenham_2d
  ${PROJECT_NAME}::nav2_costmap_2d_core
)

# OrderLayer for checking Costmap2D plugins API calling order
add_library(order_layer SHARED
            order_layer.cpp)
ament_target_dependencies(order_layer
  ${dependencies}
)
target_link_libraries(order_layer
  ${PROJECT_NAME}::nav2_costmap_2d_core
)
install(TARGETS
  order_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Costmap2D plugins API calling order test
ament_add_gtest(plugin_api_order plugin_api_order.cpp)
target_link_libraries(plugin_api_order
  ${PROJECT_NAME}::nav2_costmap_2d_core
)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-conc)
link_libraries(ros2_no_dds)
# ros2-nodds dependency
