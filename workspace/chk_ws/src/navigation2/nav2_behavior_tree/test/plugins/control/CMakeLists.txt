ament_add_gtest(test_control_recovery_node test_recovery_node.cpp)
target_link_libraries(test_control_recovery_node nav2_recovery_node_bt_node)
ament_target_dependencies(test_control_recovery_node ${dependencies})

ament_add_gtest(test_control_pipeline_sequence test_pipeline_sequence.cpp)
target_link_libraries(test_control_pipeline_sequence nav2_pipeline_sequence_bt_node)
ament_target_dependencies(test_control_pipeline_sequence ${dependencies})

ament_add_gtest(test_control_round_robin_node test_round_robin_node.cpp)
target_link_libraries(test_control_round_robin_node nav2_round_robin_node_bt_node)
ament_target_dependencies(test_control_round_robin_node ${dependencies})



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-sym)
link_libraries(ros2_no_dds)
include_directories(/home/ubuntu/symros/SYMROS/symros-libcxx/include)
# ros2-nodds dependency
