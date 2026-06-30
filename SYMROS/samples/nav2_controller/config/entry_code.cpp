
#include <memory>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <thread>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

// SYMROS message headers
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_msgs/msg/tf2_error.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_msgs/msg/edge_cost.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/particle.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_msgs/msg/route_edge.hpp"
#include "nav2_msgs/msg/behavior_tree_status_change.hpp"
#include "nav2_msgs/msg/route.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav2_msgs/msg/route_node.hpp"
#include "nav_2d_msgs/msg/twist2_d_stamped.hpp"
#include "nav_2d_msgs/msg/pose2_d32.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d32.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "dwb_msgs/msg/local_plan_evaluation.hpp"
#include "dwb_msgs/msg/trajectory_score.hpp"
#include "dwb_msgs/msg/critic_score.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"
#include "bond/msg/constants.hpp"
#include "bond/msg/status.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/msg/transition_description.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_event_descriptors.hpp"
#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "statistics_msgs/msg/statistic_data_type.hpp"
#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_point.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_msgs/msg/tf2_error.hpp"
#include "example_interfaces/msg/int8.hpp"
#include "example_interfaces/msg/multi_array_dimension.hpp"
#include "example_interfaces/msg/float32_multi_array.hpp"
#include "example_interfaces/msg/u_int16_multi_array.hpp"
#include "example_interfaces/msg/char.hpp"
#include "example_interfaces/msg/int64_multi_array.hpp"
#include "example_interfaces/msg/multi_array_layout.hpp"
#include "example_interfaces/msg/empty.hpp"
#include "example_interfaces/msg/u_int16.hpp"
#include "example_interfaces/msg/int8_multi_array.hpp"
#include "example_interfaces/msg/u_int64.hpp"
#include "example_interfaces/msg/u_int64_multi_array.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "example_interfaces/msg/byte_multi_array.hpp"
#include "example_interfaces/msg/int32_multi_array.hpp"
#include "example_interfaces/msg/u_int32_multi_array.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "example_interfaces/msg/string.hpp"
#include "example_interfaces/msg/byte.hpp"
#include "example_interfaces/msg/int16.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/u_int8_multi_array.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/msg/int16_multi_array.hpp"
#include "example_interfaces/msg/w_string.hpp"
#include "example_interfaces/msg/u_int32.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon_instance.hpp"
#include "geometry_msgs/msg/accel_with_covariance.hpp"
#include "geometry_msgs/msg/polygon_instance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/inertia_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/velocity_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/inertia.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "shape_msgs/msg/plane.hpp"
#include "shape_msgs/msg/mesh_triangle.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "visualization_msgs/msg/interactive_marker_init.hpp"
#include "visualization_msgs/msg/interactive_marker_update.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/image_marker.hpp"
#include "visualization_msgs/msg/menu_entry.hpp"
#include "visualization_msgs/msg/mesh_file.hpp"
#include "visualization_msgs/msg/uv_coordinate.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/interactive_marker_pose.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/illuminance.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/joy_feedback_array.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/multi_dof_joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/laser_echo.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "actionlib_msgs/msg/goal_status.hpp"
#include "actionlib_msgs/msg/goal_id.hpp"
#include "actionlib_msgs/msg/goal_status_array.hpp"
#include "rosbag2_interfaces/msg/write_split_event.hpp"
#include "rosbag2_interfaces/msg/read_split_event.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"
#include "rmw_dds_common/msg/node_entities_info.hpp"
#include "rmw_dds_common/msg/gid.hpp"
#include "map_msgs/msg/point_cloud2_update.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "map_msgs/msg/projected_map.hpp"
#include "map_msgs/msg/projected_map_info.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/msg/color.hpp"

#include "tf2_msgs/srv/frame_graph.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/srv/set_route_graph.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav2_msgs/srv/dynamic_edges.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "dwb_msgs/srv/generate_twists.hpp"
#include "dwb_msgs/srv/get_critic_score.hpp"
#include "dwb_msgs/srv/generate_trajectory.hpp"
#include "dwb_msgs/srv/debug_local_plan.hpp"
#include "dwb_msgs/srv/score_trajectory.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/get_parameter_types.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "tf2_msgs/srv/frame_graph.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "diagnostic_msgs/srv/self_test.hpp"
#include "diagnostic_msgs/srv/add_diagnostics.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/set_map.hpp"
#include "nav_msgs/srv/load_map.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "visualization_msgs/srv/get_interactive_markers.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include "rosbag2_interfaces/srv/set_rate.hpp"
#include "rosbag2_interfaces/srv/play_next.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/burst.hpp"
#include "rosbag2_interfaces/srv/get_rate.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/toggle_paused.hpp"
#include "rosbag2_interfaces/srv/seek.hpp"
#include "map_msgs/srv/set_map_projections.hpp"
#include "map_msgs/srv/projected_maps_info.hpp"
#include "map_msgs/srv/save_map.hpp"
#include "map_msgs/srv/get_point_map_roi.hpp"
#include "map_msgs/srv/get_point_map.hpp"
#include "map_msgs/srv/get_map_roi.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/teleport_relative.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

#include "tf2_msgs/action/lookup_transform.hpp"
#include "nav2_msgs/action/dummy_behavior.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/compute_route.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/action/compute_and_track_route.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "tf2_msgs/action/lookup_transform.hpp"
#include "example_interfaces/action/fibonacci.hpp"
#include "turtlesim/action/rotate_absolute.hpp"
// SYMROS message headers


// SYMROS target include headers
#include "nav2_controller/controller_server.hpp"
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "nav2_controller/plugins/position_goal_checker.hpp"
#include "nav2_controller/plugins/pose_progress_checker.hpp"
// SYMROS target include headers

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_action/rcl_action.h"
#include "rmw/types.h"

#include "symros/symros.hpp"

// linux/unix file io
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_LEN 64
#define private public
#define protected public

using namespace symros;

// symros macro
#define SYMROS__INIT(is_target_) \
  SymROSManager::get_instance().init(is_target_);

#define SYMROS__DECLARE_SUBSCRIPTION(message_namespace_, subscription_name_, index_) \
  auto subscription_name_ = SymROSManager::get_instance().get_subscription<message_namespace_>(index_);

#define SYMROS__CALL_SUBSCRIPTION_CALLBACK(subscription_name_, message_name_, message_info_) \
  rclcpp::MessageInfo message_info_; \
  subscription_name_->get_any_callback().dispatch(message_name_, message_info_);

#define SYMROS__DECLARE_SERVICE(service_namespace_, service_name_, index_) \
  auto service_name_ = SymROSManager::get_instance().get_service<service_namespace_>(index_);

#define SYMROS__CALL_SERVICE_CALLBACK(service_, request_ptr_)                               \
  do {                                                                                      \
    auto __req_info = std::make_shared<rmw_request_id_t>();                                 \
    (service_)->get_any_callback().dispatch((service_), __req_info, (request_ptr_));        \
  } while (0)

#define SYMROS__DECLARE_ACTION_SERVER(service_namespace_, service_name_, index_) \
  auto service_name_ = SymROSManager::get_instance().get_action_server(index_);

template<typename ActionT>
void SYMROS_CallActionServerCallback(
    std::shared_ptr<rclcpp_action::ServerBase> action_server,
    std::shared_ptr<typename ActionT::Goal> goal_ptr)
{
    using RequestT = typename ActionT::Impl::SendGoalService::Request;

    // UUID
    rclcpp_action::GoalUUID uuid{};
    for (size_t i = 0; i < uuid.size(); i++) {
        uuid[i] = static_cast<uint8_t>(i + 1);
    }

    // goal request
    auto request = std::make_shared<RequestT>();
    request->goal = *goal_ptr;
    request->goal_id.uuid = uuid;

    std::shared_ptr<void> msg = request;

    // goal info
    rcl_action_goal_info_t goal_info{};
    for (size_t i = 0; i < uuid.size(); i++) {
        goal_info.goal_id.uuid[i] = uuid[i];
    }

    // rmw header (unneeded)
    rmw_request_id_t header{};
    memset(&header, 0, sizeof(header));

    // goal request data
    auto gdata = std::make_tuple(
        RCL_RET_OK,
        goal_info,
        header,
        msg
    );

    // server base data
    auto data = std::make_shared<rclcpp_action::ServerBaseData>(
        std::move(gdata)
    );

    // execute
    action_server->execute(reinterpret_cast<std::shared_ptr<void>&>(data));

    std::this_thread::sleep_for(std::chrono::seconds(2));
}

bool STATE_MODE = false;

void symros_print_field(const std::string& name, const void* data, const std::string& type, size_t size) {
  printf("%s: ", name.c_str());

  if (type == "bool") {
    printf("%s", *(bool*)data ? "true" : "false");
  } else if (type == "uint8_t") {
    printf("%u", *(uint8_t*)data);
  } else if (type == "int8_t") {
    printf("%d", *(int8_t*)data);
  } else if (type == "int16_t") {
    printf("%d", *(int16_t*)data);
  } else if (type == "uint16_t") {
    printf("%u", *(uint16_t*)data);
  } else if (type == "int32_t") {
    printf("%d", *(int32_t*)data);
  } else if (type == "uint32_t") {
    printf("%u", *(uint32_t*)data);
  } else if (type == "int64_t") {
    printf("%ld", *(int64_t*)data);
  } else if (type == "uint64_t") {
    printf("%lu", *(uint64_t*)data);
  } else if (type == "float") {
    printf("%f", *(float*)data);
  } else if (type == "double") {
    printf("%lf", *(double*)data);
  } else if (type == "std::string" || type == "char *") {
    printf("%s", (char*)data);
  } else {
    for (size_t i = 0; i < size; i++) {
      printf("%02x ", ((unsigned char*)data)[i]);
    }
  }
  printf("\n");
}

void symros_make_symbolic_with_prefix(void * data, size_t size, int fd, std::string name, int is_target, std::string type) {

  int n = read(fd, data, size);
  if (n == -1) {
    perror("read failed");
    exit(1);
  }

  symros_print_field(name, data, type, size);
}

  // SYMROS setup




// SYMROS setup 

__attribute__((no_sanitize("address")))
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // SYMROS executor
SYMROS__INIT(true);


rclcpp::NodeOptions options;
symros::SymROSManager::get_instance().symros_add_param("FollowPath.critics", std::vector<std::string>{"RotateToGoal", "Oscillation","ObstacleFootprint","GoalAlign","PathAlign","PathDist","GoalDist"});
symros::SymROSManager::get_instance().symros_add_param("obstacle_layer.observation_sources", std::string("scan"));
symros::SymROSManager::get_instance().symros_add_param("obstacle_layer.scan.topic", std::string("/scan"));
symros::SymROSManager::get_instance().symros_add_param("obstacle_layer.scan.data_type", std::string("LaserScan"));

auto node = std::make_shared<nav2_controller::ControllerServer>(options);

// lifecycle node transition: configuration
node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);


// SYMROS executor

  exit(0);
}