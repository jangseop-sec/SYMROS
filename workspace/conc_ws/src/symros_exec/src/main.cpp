
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
#include "robot_localization/srv/get_state.hpp"
#include "robot_localization/srv/set_datum.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "robot_localization/srv/to_ll.hpp"
#include "robot_localization/srv/set_pose.hpp"
#include "robot_localization/srv/toggle_filter_processing.hpp"
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
#include "robot_localization/srv/get_state.hpp"
#include "robot_localization/srv/set_datum.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "robot_localization/srv/to_ll.hpp"
#include "robot_localization/srv/set_pose.hpp"
#include "robot_localization/srv/toggle_filter_processing.hpp"

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
#include "robot_localization/filter_base.hpp"
#include "robot_localization/filter_state.hpp"
#include "robot_localization/ekf.hpp"
#include "robot_localization/measurement.hpp"
#include "robot_localization/filter_common.hpp"
#include "robot_localization/ros_filter_utilities.hpp"
#include "robot_localization/navsat_transform.hpp"
#include "robot_localization/ukf.hpp"
#include "robot_localization/navsat_conversions.hpp"
#include "robot_localization/ros_filter_types.hpp"
#include "robot_localization/robot_localization_estimator.hpp"
#include "robot_localization/ros_filter.hpp"
#include "robot_localization/ros_robot_localization_listener.hpp"
#include "robot_localization/filter_utilities.hpp"
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
int fd = open(argv[1], O_RDONLY);

STATE_MODE = argv[2][0] == '1';


SYMROS__INIT(false);


printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
bool param__use_sim_time;
symros_make_symbolic_with_prefix(&param__use_sim_time, sizeof(bool), fd, "param__use_sim_time", 2, "bool");

double param__frequency;
symros_make_symbolic_with_prefix(&param__frequency, sizeof(double), fd, "param__frequency", 2, "double");

double param__sensor_timeout;
symros_make_symbolic_with_prefix(&param__sensor_timeout, sizeof(double), fd, "param__sensor_timeout", 2, "double");

bool param__two_d_mode;
symros_make_symbolic_with_prefix(&param__two_d_mode, sizeof(bool), fd, "param__two_d_mode", 2, "bool");

double param__transform_time_offset;
symros_make_symbolic_with_prefix(&param__transform_time_offset, sizeof(double), fd, "param__transform_time_offset", 2, "double");

double param__transform_timeout;
symros_make_symbolic_with_prefix(&param__transform_timeout, sizeof(double), fd, "param__transform_timeout", 2, "double");

bool param__print_diagnostics;
symros_make_symbolic_with_prefix(&param__print_diagnostics, sizeof(bool), fd, "param__print_diagnostics", 2, "bool");

bool param__debug;
symros_make_symbolic_with_prefix(&param__debug, sizeof(bool), fd, "param__debug", 2, "bool");

bool param__publish_tf;
symros_make_symbolic_with_prefix(&param__publish_tf, sizeof(bool), fd, "param__publish_tf", 2, "bool");

uint8_t param__map_frame_string_size;
symros_make_symbolic_with_prefix(&param__map_frame_string_size, sizeof(uint8_t), fd, "param__map_frame_string_size", 2, "uint8_t");
char param__map_frame_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__map_frame_string_data, MAX_LEN, fd, "param__map_frame_string_data", 2, "char *");
std::string param__map_frame(param__map_frame_string_data, param__map_frame_string_size);

uint8_t param__odom_frame_string_size;
symros_make_symbolic_with_prefix(&param__odom_frame_string_size, sizeof(uint8_t), fd, "param__odom_frame_string_size", 2, "uint8_t");
char param__odom_frame_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__odom_frame_string_data, MAX_LEN, fd, "param__odom_frame_string_data", 2, "char *");
std::string param__odom_frame(param__odom_frame_string_data, param__odom_frame_string_size);

uint8_t param__base_link_frame_string_size;
symros_make_symbolic_with_prefix(&param__base_link_frame_string_size, sizeof(uint8_t), fd, "param__base_link_frame_string_size", 2, "uint8_t");
char param__base_link_frame_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__base_link_frame_string_data, MAX_LEN, fd, "param__base_link_frame_string_data", 2, "char *");
std::string param__base_link_frame(param__base_link_frame_string_data, param__base_link_frame_string_size);

uint8_t param__world_frame_string_size;
symros_make_symbolic_with_prefix(&param__world_frame_string_size, sizeof(uint8_t), fd, "param__world_frame_string_size", 2, "uint8_t");
char param__world_frame_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__world_frame_string_data, MAX_LEN, fd, "param__world_frame_string_data", 2, "char *");
std::string param__world_frame(param__world_frame_string_data, param__world_frame_string_size);

uint8_t param__odom0_string_size;
symros_make_symbolic_with_prefix(&param__odom0_string_size, sizeof(uint8_t), fd, "param__odom0_string_size", 2, "uint8_t");
char param__odom0_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__odom0_string_data, MAX_LEN, fd, "param__odom0_string_data", 2, "char *");
std::string param__odom0(param__odom0_string_data, param__odom0_string_size);

std::vector<bool> param__odom0_config;
bool param__odom0_config_0;
symros_make_symbolic_with_prefix(&param__odom0_config_0, sizeof(bool), fd, "param__odom0_config_0", 2, "bool");
param__odom0_config.push_back(param__odom0_config_0);

bool param__odom0_config_1;
symros_make_symbolic_with_prefix(&param__odom0_config_1, sizeof(bool), fd, "param__odom0_config_1", 2, "bool");
param__odom0_config.push_back(param__odom0_config_1);

bool param__odom0_config_2;
symros_make_symbolic_with_prefix(&param__odom0_config_2, sizeof(bool), fd, "param__odom0_config_2", 2, "bool");
param__odom0_config.push_back(param__odom0_config_2);

bool param__odom0_config_3;
symros_make_symbolic_with_prefix(&param__odom0_config_3, sizeof(bool), fd, "param__odom0_config_3", 2, "bool");
param__odom0_config.push_back(param__odom0_config_3);

bool param__odom0_config_4;
symros_make_symbolic_with_prefix(&param__odom0_config_4, sizeof(bool), fd, "param__odom0_config_4", 2, "bool");
param__odom0_config.push_back(param__odom0_config_4);

bool param__odom0_config_5;
symros_make_symbolic_with_prefix(&param__odom0_config_5, sizeof(bool), fd, "param__odom0_config_5", 2, "bool");
param__odom0_config.push_back(param__odom0_config_5);

bool param__odom0_config_6;
symros_make_symbolic_with_prefix(&param__odom0_config_6, sizeof(bool), fd, "param__odom0_config_6", 2, "bool");
param__odom0_config.push_back(param__odom0_config_6);

bool param__odom0_config_7;
symros_make_symbolic_with_prefix(&param__odom0_config_7, sizeof(bool), fd, "param__odom0_config_7", 2, "bool");
param__odom0_config.push_back(param__odom0_config_7);

bool param__odom0_config_8;
symros_make_symbolic_with_prefix(&param__odom0_config_8, sizeof(bool), fd, "param__odom0_config_8", 2, "bool");
param__odom0_config.push_back(param__odom0_config_8);

bool param__odom0_config_9;
symros_make_symbolic_with_prefix(&param__odom0_config_9, sizeof(bool), fd, "param__odom0_config_9", 2, "bool");
param__odom0_config.push_back(param__odom0_config_9);

bool param__odom0_config_10;
symros_make_symbolic_with_prefix(&param__odom0_config_10, sizeof(bool), fd, "param__odom0_config_10", 2, "bool");
param__odom0_config.push_back(param__odom0_config_10);

bool param__odom0_config_11;
symros_make_symbolic_with_prefix(&param__odom0_config_11, sizeof(bool), fd, "param__odom0_config_11", 2, "bool");
param__odom0_config.push_back(param__odom0_config_11);

bool param__odom0_config_12;
symros_make_symbolic_with_prefix(&param__odom0_config_12, sizeof(bool), fd, "param__odom0_config_12", 2, "bool");
param__odom0_config.push_back(param__odom0_config_12);

bool param__odom0_config_13;
symros_make_symbolic_with_prefix(&param__odom0_config_13, sizeof(bool), fd, "param__odom0_config_13", 2, "bool");
param__odom0_config.push_back(param__odom0_config_13);

bool param__odom0_config_14;
symros_make_symbolic_with_prefix(&param__odom0_config_14, sizeof(bool), fd, "param__odom0_config_14", 2, "bool");
param__odom0_config.push_back(param__odom0_config_14);

int64_t param__odom0_queue_size;
symros_make_symbolic_with_prefix(&param__odom0_queue_size, sizeof(int64_t), fd, "param__odom0_queue_size", 2, "int64_t");

bool param__odom0_nodelay;
symros_make_symbolic_with_prefix(&param__odom0_nodelay, sizeof(bool), fd, "param__odom0_nodelay", 2, "bool");

bool param__odom0_differential;
symros_make_symbolic_with_prefix(&param__odom0_differential, sizeof(bool), fd, "param__odom0_differential", 2, "bool");

bool param__odom0_relative;
symros_make_symbolic_with_prefix(&param__odom0_relative, sizeof(bool), fd, "param__odom0_relative", 2, "bool");

uint8_t param__pose0_string_size;
symros_make_symbolic_with_prefix(&param__pose0_string_size, sizeof(uint8_t), fd, "param__pose0_string_size", 2, "uint8_t");
char param__pose0_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__pose0_string_data, MAX_LEN, fd, "param__pose0_string_data", 2, "char *");
std::string param__pose0(param__pose0_string_data, param__pose0_string_size);

std::vector<bool> param__pose0_config;
bool param__pose0_config_0;
symros_make_symbolic_with_prefix(&param__pose0_config_0, sizeof(bool), fd, "param__pose0_config_0", 2, "bool");
param__pose0_config.push_back(param__pose0_config_0);

bool param__pose0_config_1;
symros_make_symbolic_with_prefix(&param__pose0_config_1, sizeof(bool), fd, "param__pose0_config_1", 2, "bool");
param__pose0_config.push_back(param__pose0_config_1);

bool param__pose0_config_2;
symros_make_symbolic_with_prefix(&param__pose0_config_2, sizeof(bool), fd, "param__pose0_config_2", 2, "bool");
param__pose0_config.push_back(param__pose0_config_2);

bool param__pose0_config_3;
symros_make_symbolic_with_prefix(&param__pose0_config_3, sizeof(bool), fd, "param__pose0_config_3", 2, "bool");
param__pose0_config.push_back(param__pose0_config_3);

bool param__pose0_config_4;
symros_make_symbolic_with_prefix(&param__pose0_config_4, sizeof(bool), fd, "param__pose0_config_4", 2, "bool");
param__pose0_config.push_back(param__pose0_config_4);

bool param__pose0_config_5;
symros_make_symbolic_with_prefix(&param__pose0_config_5, sizeof(bool), fd, "param__pose0_config_5", 2, "bool");
param__pose0_config.push_back(param__pose0_config_5);

bool param__pose0_config_6;
symros_make_symbolic_with_prefix(&param__pose0_config_6, sizeof(bool), fd, "param__pose0_config_6", 2, "bool");
param__pose0_config.push_back(param__pose0_config_6);

bool param__pose0_config_7;
symros_make_symbolic_with_prefix(&param__pose0_config_7, sizeof(bool), fd, "param__pose0_config_7", 2, "bool");
param__pose0_config.push_back(param__pose0_config_7);

bool param__pose0_config_8;
symros_make_symbolic_with_prefix(&param__pose0_config_8, sizeof(bool), fd, "param__pose0_config_8", 2, "bool");
param__pose0_config.push_back(param__pose0_config_8);

bool param__pose0_config_9;
symros_make_symbolic_with_prefix(&param__pose0_config_9, sizeof(bool), fd, "param__pose0_config_9", 2, "bool");
param__pose0_config.push_back(param__pose0_config_9);

bool param__pose0_config_10;
symros_make_symbolic_with_prefix(&param__pose0_config_10, sizeof(bool), fd, "param__pose0_config_10", 2, "bool");
param__pose0_config.push_back(param__pose0_config_10);

bool param__pose0_config_11;
symros_make_symbolic_with_prefix(&param__pose0_config_11, sizeof(bool), fd, "param__pose0_config_11", 2, "bool");
param__pose0_config.push_back(param__pose0_config_11);

bool param__pose0_config_12;
symros_make_symbolic_with_prefix(&param__pose0_config_12, sizeof(bool), fd, "param__pose0_config_12", 2, "bool");
param__pose0_config.push_back(param__pose0_config_12);

bool param__pose0_config_13;
symros_make_symbolic_with_prefix(&param__pose0_config_13, sizeof(bool), fd, "param__pose0_config_13", 2, "bool");
param__pose0_config.push_back(param__pose0_config_13);

bool param__pose0_config_14;
symros_make_symbolic_with_prefix(&param__pose0_config_14, sizeof(bool), fd, "param__pose0_config_14", 2, "bool");
param__pose0_config.push_back(param__pose0_config_14);

int64_t param__pose0_queue_size;
symros_make_symbolic_with_prefix(&param__pose0_queue_size, sizeof(int64_t), fd, "param__pose0_queue_size", 2, "int64_t");

bool param__pose0_differential;
symros_make_symbolic_with_prefix(&param__pose0_differential, sizeof(bool), fd, "param__pose0_differential", 2, "bool");

bool param__pose0_relative;
symros_make_symbolic_with_prefix(&param__pose0_relative, sizeof(bool), fd, "param__pose0_relative", 2, "bool");

uint8_t param__twist0_string_size;
symros_make_symbolic_with_prefix(&param__twist0_string_size, sizeof(uint8_t), fd, "param__twist0_string_size", 2, "uint8_t");
char param__twist0_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__twist0_string_data, MAX_LEN, fd, "param__twist0_string_data", 2, "char *");
std::string param__twist0(param__twist0_string_data, param__twist0_string_size);

std::vector<bool> param__twist0_config;
bool param__twist0_config_0;
symros_make_symbolic_with_prefix(&param__twist0_config_0, sizeof(bool), fd, "param__twist0_config_0", 2, "bool");
param__twist0_config.push_back(param__twist0_config_0);

bool param__twist0_config_1;
symros_make_symbolic_with_prefix(&param__twist0_config_1, sizeof(bool), fd, "param__twist0_config_1", 2, "bool");
param__twist0_config.push_back(param__twist0_config_1);

bool param__twist0_config_2;
symros_make_symbolic_with_prefix(&param__twist0_config_2, sizeof(bool), fd, "param__twist0_config_2", 2, "bool");
param__twist0_config.push_back(param__twist0_config_2);

bool param__twist0_config_3;
symros_make_symbolic_with_prefix(&param__twist0_config_3, sizeof(bool), fd, "param__twist0_config_3", 2, "bool");
param__twist0_config.push_back(param__twist0_config_3);

bool param__twist0_config_4;
symros_make_symbolic_with_prefix(&param__twist0_config_4, sizeof(bool), fd, "param__twist0_config_4", 2, "bool");
param__twist0_config.push_back(param__twist0_config_4);

bool param__twist0_config_5;
symros_make_symbolic_with_prefix(&param__twist0_config_5, sizeof(bool), fd, "param__twist0_config_5", 2, "bool");
param__twist0_config.push_back(param__twist0_config_5);

bool param__twist0_config_6;
symros_make_symbolic_with_prefix(&param__twist0_config_6, sizeof(bool), fd, "param__twist0_config_6", 2, "bool");
param__twist0_config.push_back(param__twist0_config_6);

bool param__twist0_config_7;
symros_make_symbolic_with_prefix(&param__twist0_config_7, sizeof(bool), fd, "param__twist0_config_7", 2, "bool");
param__twist0_config.push_back(param__twist0_config_7);

bool param__twist0_config_8;
symros_make_symbolic_with_prefix(&param__twist0_config_8, sizeof(bool), fd, "param__twist0_config_8", 2, "bool");
param__twist0_config.push_back(param__twist0_config_8);

bool param__twist0_config_9;
symros_make_symbolic_with_prefix(&param__twist0_config_9, sizeof(bool), fd, "param__twist0_config_9", 2, "bool");
param__twist0_config.push_back(param__twist0_config_9);

bool param__twist0_config_10;
symros_make_symbolic_with_prefix(&param__twist0_config_10, sizeof(bool), fd, "param__twist0_config_10", 2, "bool");
param__twist0_config.push_back(param__twist0_config_10);

bool param__twist0_config_11;
symros_make_symbolic_with_prefix(&param__twist0_config_11, sizeof(bool), fd, "param__twist0_config_11", 2, "bool");
param__twist0_config.push_back(param__twist0_config_11);

bool param__twist0_config_12;
symros_make_symbolic_with_prefix(&param__twist0_config_12, sizeof(bool), fd, "param__twist0_config_12", 2, "bool");
param__twist0_config.push_back(param__twist0_config_12);

bool param__twist0_config_13;
symros_make_symbolic_with_prefix(&param__twist0_config_13, sizeof(bool), fd, "param__twist0_config_13", 2, "bool");
param__twist0_config.push_back(param__twist0_config_13);

bool param__twist0_config_14;
symros_make_symbolic_with_prefix(&param__twist0_config_14, sizeof(bool), fd, "param__twist0_config_14", 2, "bool");
param__twist0_config.push_back(param__twist0_config_14);

int64_t param__twist0_queue_size;
symros_make_symbolic_with_prefix(&param__twist0_queue_size, sizeof(int64_t), fd, "param__twist0_queue_size", 2, "int64_t");

uint8_t param__imu0_string_size;
symros_make_symbolic_with_prefix(&param__imu0_string_size, sizeof(uint8_t), fd, "param__imu0_string_size", 2, "uint8_t");
char param__imu0_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__imu0_string_data, MAX_LEN, fd, "param__imu0_string_data", 2, "char *");
std::string param__imu0(param__imu0_string_data, param__imu0_string_size);

std::vector<bool> param__imu0_config;
bool param__imu0_config_0;
symros_make_symbolic_with_prefix(&param__imu0_config_0, sizeof(bool), fd, "param__imu0_config_0", 2, "bool");
param__imu0_config.push_back(param__imu0_config_0);

bool param__imu0_config_1;
symros_make_symbolic_with_prefix(&param__imu0_config_1, sizeof(bool), fd, "param__imu0_config_1", 2, "bool");
param__imu0_config.push_back(param__imu0_config_1);

bool param__imu0_config_2;
symros_make_symbolic_with_prefix(&param__imu0_config_2, sizeof(bool), fd, "param__imu0_config_2", 2, "bool");
param__imu0_config.push_back(param__imu0_config_2);

bool param__imu0_config_3;
symros_make_symbolic_with_prefix(&param__imu0_config_3, sizeof(bool), fd, "param__imu0_config_3", 2, "bool");
param__imu0_config.push_back(param__imu0_config_3);

bool param__imu0_config_4;
symros_make_symbolic_with_prefix(&param__imu0_config_4, sizeof(bool), fd, "param__imu0_config_4", 2, "bool");
param__imu0_config.push_back(param__imu0_config_4);

bool param__imu0_config_5;
symros_make_symbolic_with_prefix(&param__imu0_config_5, sizeof(bool), fd, "param__imu0_config_5", 2, "bool");
param__imu0_config.push_back(param__imu0_config_5);

bool param__imu0_config_6;
symros_make_symbolic_with_prefix(&param__imu0_config_6, sizeof(bool), fd, "param__imu0_config_6", 2, "bool");
param__imu0_config.push_back(param__imu0_config_6);

bool param__imu0_config_7;
symros_make_symbolic_with_prefix(&param__imu0_config_7, sizeof(bool), fd, "param__imu0_config_7", 2, "bool");
param__imu0_config.push_back(param__imu0_config_7);

bool param__imu0_config_8;
symros_make_symbolic_with_prefix(&param__imu0_config_8, sizeof(bool), fd, "param__imu0_config_8", 2, "bool");
param__imu0_config.push_back(param__imu0_config_8);

bool param__imu0_config_9;
symros_make_symbolic_with_prefix(&param__imu0_config_9, sizeof(bool), fd, "param__imu0_config_9", 2, "bool");
param__imu0_config.push_back(param__imu0_config_9);

bool param__imu0_config_10;
symros_make_symbolic_with_prefix(&param__imu0_config_10, sizeof(bool), fd, "param__imu0_config_10", 2, "bool");
param__imu0_config.push_back(param__imu0_config_10);

bool param__imu0_config_11;
symros_make_symbolic_with_prefix(&param__imu0_config_11, sizeof(bool), fd, "param__imu0_config_11", 2, "bool");
param__imu0_config.push_back(param__imu0_config_11);

bool param__imu0_config_12;
symros_make_symbolic_with_prefix(&param__imu0_config_12, sizeof(bool), fd, "param__imu0_config_12", 2, "bool");
param__imu0_config.push_back(param__imu0_config_12);

bool param__imu0_config_13;
symros_make_symbolic_with_prefix(&param__imu0_config_13, sizeof(bool), fd, "param__imu0_config_13", 2, "bool");
param__imu0_config.push_back(param__imu0_config_13);

bool param__imu0_config_14;
symros_make_symbolic_with_prefix(&param__imu0_config_14, sizeof(bool), fd, "param__imu0_config_14", 2, "bool");
param__imu0_config.push_back(param__imu0_config_14);

int64_t param__imu0_queue_size;
symros_make_symbolic_with_prefix(&param__imu0_queue_size, sizeof(int64_t), fd, "param__imu0_queue_size", 2, "int64_t");

bool param__imu0_nodelay;
symros_make_symbolic_with_prefix(&param__imu0_nodelay, sizeof(bool), fd, "param__imu0_nodelay", 2, "bool");

bool param__imu0_differential;
symros_make_symbolic_with_prefix(&param__imu0_differential, sizeof(bool), fd, "param__imu0_differential", 2, "bool");

bool param__imu0_relative;
symros_make_symbolic_with_prefix(&param__imu0_relative, sizeof(bool), fd, "param__imu0_relative", 2, "bool");

bool param__imu0_remove_gravitational_acceleration;
symros_make_symbolic_with_prefix(&param__imu0_remove_gravitational_acceleration, sizeof(bool), fd, "param__imu0_remove_gravitational_acceleration", 2, "bool");

bool param__use_control;
symros_make_symbolic_with_prefix(&param__use_control, sizeof(bool), fd, "param__use_control", 2, "bool");

bool param__stamped_control;
symros_make_symbolic_with_prefix(&param__stamped_control, sizeof(bool), fd, "param__stamped_control", 2, "bool");

double param__control_timeout;
symros_make_symbolic_with_prefix(&param__control_timeout, sizeof(double), fd, "param__control_timeout", 2, "double");

std::vector<bool> param__control_config;
bool param__control_config_0;
symros_make_symbolic_with_prefix(&param__control_config_0, sizeof(bool), fd, "param__control_config_0", 2, "bool");
param__control_config.push_back(param__control_config_0);

bool param__control_config_1;
symros_make_symbolic_with_prefix(&param__control_config_1, sizeof(bool), fd, "param__control_config_1", 2, "bool");
param__control_config.push_back(param__control_config_1);

bool param__control_config_2;
symros_make_symbolic_with_prefix(&param__control_config_2, sizeof(bool), fd, "param__control_config_2", 2, "bool");
param__control_config.push_back(param__control_config_2);

bool param__control_config_3;
symros_make_symbolic_with_prefix(&param__control_config_3, sizeof(bool), fd, "param__control_config_3", 2, "bool");
param__control_config.push_back(param__control_config_3);

bool param__control_config_4;
symros_make_symbolic_with_prefix(&param__control_config_4, sizeof(bool), fd, "param__control_config_4", 2, "bool");
param__control_config.push_back(param__control_config_4);

bool param__control_config_5;
symros_make_symbolic_with_prefix(&param__control_config_5, sizeof(bool), fd, "param__control_config_5", 2, "bool");
param__control_config.push_back(param__control_config_5);

std::vector<double> param__acceleration_limits;
double param__acceleration_limits_0;
symros_make_symbolic_with_prefix(&param__acceleration_limits_0, sizeof(double), fd, "param__acceleration_limits_0", 2, "double");
param__acceleration_limits.push_back(param__acceleration_limits_0);

double param__acceleration_limits_1;
symros_make_symbolic_with_prefix(&param__acceleration_limits_1, sizeof(double), fd, "param__acceleration_limits_1", 2, "double");
param__acceleration_limits.push_back(param__acceleration_limits_1);

double param__acceleration_limits_2;
symros_make_symbolic_with_prefix(&param__acceleration_limits_2, sizeof(double), fd, "param__acceleration_limits_2", 2, "double");
param__acceleration_limits.push_back(param__acceleration_limits_2);

double param__acceleration_limits_3;
symros_make_symbolic_with_prefix(&param__acceleration_limits_3, sizeof(double), fd, "param__acceleration_limits_3", 2, "double");
param__acceleration_limits.push_back(param__acceleration_limits_3);

double param__acceleration_limits_4;
symros_make_symbolic_with_prefix(&param__acceleration_limits_4, sizeof(double), fd, "param__acceleration_limits_4", 2, "double");
param__acceleration_limits.push_back(param__acceleration_limits_4);

double param__acceleration_limits_5;
symros_make_symbolic_with_prefix(&param__acceleration_limits_5, sizeof(double), fd, "param__acceleration_limits_5", 2, "double");
param__acceleration_limits.push_back(param__acceleration_limits_5);

std::vector<double> param__deceleration_limits;
double param__deceleration_limits_0;
symros_make_symbolic_with_prefix(&param__deceleration_limits_0, sizeof(double), fd, "param__deceleration_limits_0", 2, "double");
param__deceleration_limits.push_back(param__deceleration_limits_0);

double param__deceleration_limits_1;
symros_make_symbolic_with_prefix(&param__deceleration_limits_1, sizeof(double), fd, "param__deceleration_limits_1", 2, "double");
param__deceleration_limits.push_back(param__deceleration_limits_1);

double param__deceleration_limits_2;
symros_make_symbolic_with_prefix(&param__deceleration_limits_2, sizeof(double), fd, "param__deceleration_limits_2", 2, "double");
param__deceleration_limits.push_back(param__deceleration_limits_2);

double param__deceleration_limits_3;
symros_make_symbolic_with_prefix(&param__deceleration_limits_3, sizeof(double), fd, "param__deceleration_limits_3", 2, "double");
param__deceleration_limits.push_back(param__deceleration_limits_3);

double param__deceleration_limits_4;
symros_make_symbolic_with_prefix(&param__deceleration_limits_4, sizeof(double), fd, "param__deceleration_limits_4", 2, "double");
param__deceleration_limits.push_back(param__deceleration_limits_4);

double param__deceleration_limits_5;
symros_make_symbolic_with_prefix(&param__deceleration_limits_5, sizeof(double), fd, "param__deceleration_limits_5", 2, "double");
param__deceleration_limits.push_back(param__deceleration_limits_5);



symros::SymROSManager::get_instance().symros_add_param("use_sim_time", param__use_sim_time);
symros::SymROSManager::get_instance().symros_add_param("frequency", param__frequency);
symros::SymROSManager::get_instance().symros_add_param("sensor_timeout", param__sensor_timeout);
symros::SymROSManager::get_instance().symros_add_param("two_d_mode", param__two_d_mode);
symros::SymROSManager::get_instance().symros_add_param("transform_time_offset", param__transform_time_offset);
symros::SymROSManager::get_instance().symros_add_param("transform_timeout", param__transform_timeout);
symros::SymROSManager::get_instance().symros_add_param("print_diagnostics", param__print_diagnostics);
symros::SymROSManager::get_instance().symros_add_param("debug", param__debug);
symros::SymROSManager::get_instance().symros_add_param("publish_tf", param__publish_tf);
symros::SymROSManager::get_instance().symros_add_param("map_frame", param__map_frame);
symros::SymROSManager::get_instance().symros_add_param("odom_frame", param__odom_frame);
symros::SymROSManager::get_instance().symros_add_param("base_link_frame", param__base_link_frame);
symros::SymROSManager::get_instance().symros_add_param("world_frame", param__world_frame);
symros::SymROSManager::get_instance().symros_add_param("odom0", param__odom0);
symros::SymROSManager::get_instance().symros_add_param("odom0_config", param__odom0_config);
symros::SymROSManager::get_instance().symros_add_param("odom0_queue_size", param__odom0_queue_size);
symros::SymROSManager::get_instance().symros_add_param("odom0_nodelay", param__odom0_nodelay);
symros::SymROSManager::get_instance().symros_add_param("odom0_differential", param__odom0_differential);
symros::SymROSManager::get_instance().symros_add_param("odom0_relative", param__odom0_relative);
symros::SymROSManager::get_instance().symros_add_param("pose0", param__pose0);
symros::SymROSManager::get_instance().symros_add_param("pose0_config", param__pose0_config);
symros::SymROSManager::get_instance().symros_add_param("pose0_queue_size", param__pose0_queue_size);
symros::SymROSManager::get_instance().symros_add_param("pose0_differential", param__pose0_differential);
symros::SymROSManager::get_instance().symros_add_param("pose0_relative", param__pose0_relative);
symros::SymROSManager::get_instance().symros_add_param("twist0", param__twist0);
symros::SymROSManager::get_instance().symros_add_param("twist0_config", param__twist0_config);
symros::SymROSManager::get_instance().symros_add_param("twist0_queue_size", param__twist0_queue_size);
symros::SymROSManager::get_instance().symros_add_param("imu0", param__imu0);
symros::SymROSManager::get_instance().symros_add_param("imu0_config", param__imu0_config);
symros::SymROSManager::get_instance().symros_add_param("imu0_queue_size", param__imu0_queue_size);
symros::SymROSManager::get_instance().symros_add_param("imu0_nodelay", param__imu0_nodelay);
symros::SymROSManager::get_instance().symros_add_param("imu0_differential", param__imu0_differential);
symros::SymROSManager::get_instance().symros_add_param("imu0_relative", param__imu0_relative);
symros::SymROSManager::get_instance().symros_add_param("imu0_remove_gravitational_acceleration", param__imu0_remove_gravitational_acceleration);
symros::SymROSManager::get_instance().symros_add_param("use_control", param__use_control);
symros::SymROSManager::get_instance().symros_add_param("stamped_control", param__stamped_control);
symros::SymROSManager::get_instance().symros_add_param("control_timeout", param__control_timeout);
symros::SymROSManager::get_instance().symros_add_param("control_config", param__control_config);
symros::SymROSManager::get_instance().symros_add_param("acceleration_limits", param__acceleration_limits);
symros::SymROSManager::get_instance().symros_add_param("deceleration_limits", param__deceleration_limits);



printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);

using symros::SymROSManager;
auto & mgr = SymROSManager::get_instance();

// ===== Odometry =====
// mgr.symros_add_param("odom0",                         std::string("/odom"));                       // std::string
// mgr.symros_add_param("odom0_config",                  std::vector<bool>{                           // std::vector<bool> (15)
//     true, true, false,  false, false, true,
//     false, false, false,  false, false, true,
//     false, false, false});
// mgr.symros_add_param("odom0_differential",            false);                                      // bool
// mgr.symros_add_param("odom0_relative",                false);                                      // bool
// mgr.symros_add_param("odom0_queue_size",              static_cast<int64_t>(10));                    // int64_t
// mgr.symros_add_param("odom0_pose_rejection_threshold",  5.0);                                      // double
// mgr.symros_add_param("odom0_twist_rejection_threshold", 1.0);                                      // double

// // ===== Pose =====
// mgr.symros_add_param("pose0",                         std::string("/pose"));                       // std::string
// mgr.symros_add_param("pose0_config",                  std::vector<bool>{                           // std::vector<bool> (15)
//     true, true, false,  false, false, true,
//     false, false, false,  false, false, false,
//     false, false, false});
// mgr.symros_add_param("pose0_differential",            false);                                      // bool
// mgr.symros_add_param("pose0_relative",                false);                                      // bool
// mgr.symros_add_param("pose0_queue_size",              static_cast<int64_t>(10));                    // int64_t
// mgr.symros_add_param("pose0_rejection_threshold",     2.0);                                         // double

// // ===== Twist =====
// mgr.symros_add_param("twist0",                        std::string("/twist"));                      // std::string
// mgr.symros_add_param("twist0_config",                 std::vector<bool>{                           // std::vector<bool> (15)
//     false, false, false,  false, false, false,
//     true, true, false,  false, false, true,
//     false, false, false});
// mgr.symros_add_param("twist0_queue_size",             static_cast<int64_t>(10));                    // int64_t
// mgr.symros_add_param("twist0_rejection_threshold",    1.0);                                         // double

// // ===== IMU =====
// mgr.symros_add_param("imu0",                          std::string("/imu"));                        // std::string
// mgr.symros_add_param("imu0_config",                   std::vector<bool>{                           // std::vector<bool> (15)
//     false, false, false,  true, true, true,
//     false, false, false,  true, true, true,
//     true, true, false});
// mgr.symros_add_param("imu0_differential",                          false);                         // bool
// mgr.symros_add_param("imu0_relative",                              true);                          // bool
// mgr.symros_add_param("imu0_queue_size",                            static_cast<int64_t>(10));      // int64_t
// mgr.symros_add_param("imu0_remove_gravitational_acceleration",     true);                          // bool
// mgr.symros_add_param("imu0_pose_rejection_threshold",              0.8);                           // double
// mgr.symros_add_param("imu0_twist_rejection_threshold",             0.8);                           // double
// mgr.symros_add_param("imu0_linear_acceleration_rejection_threshold", 0.8);                         // double

// // ===== Control =====
// mgr.symros_add_param("use_control",     true);                                                      // bool
// mgr.symros_add_param("stamped_control", true);                                                     // bool
// mgr.symros_add_param("control_config",  std::vector<bool>{true, false, false, false, false, true}); // std::vector<bool> (6)

rclcpp::NodeOptions options;
options.arguments({"ekf_filter_node"});
std::shared_ptr<robot_localization::RosEkf> filter = std::make_shared<robot_localization::RosEkf>(options);
filter->initialize();


SYMROS__INIT(true);

SYMROS__DECLARE_SUBSCRIPTION(tf2_msgs::msg::TFMessage, sub_0, 3);
SYMROS__DECLARE_SUBSCRIPTION(tf2_msgs::msg::TFMessage, sub_1, 2);
SYMROS__DECLARE_SERVICE(std_srvs::srv::Empty, srv_2, 7);
SYMROS__DECLARE_SERVICE(robot_localization::srv::ToggleFilterProcessing, srv_3, 8);
SYMROS__DECLARE_SUBSCRIPTION(geometry_msgs::msg::TwistStamped, sub_4, 9);
SYMROS__DECLARE_SUBSCRIPTION(nav_msgs::msg::Odometry, sub_5, 5);
SYMROS__DECLARE_SUBSCRIPTION(geometry_msgs::msg::PoseWithCovarianceStamped, sub_6, 6);
SYMROS__DECLARE_SUBSCRIPTION(geometry_msgs::msg::TwistWithCovarianceStamped, sub_7, 7);
SYMROS__DECLARE_SUBSCRIPTION(sensor_msgs::msg::Imu, sub_8, 8);
SYMROS__DECLARE_SERVICE(robot_localization::srv::SetPose, srv_9, 6);

printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto msg_0 = std::make_shared<tf2_msgs::msg::TFMessage>();
auto msg_0_transforms_0 = std::make_shared<geometry_msgs::msg::TransformStamped>();
auto msg_0_transforms_0_header = std::make_shared<std_msgs::msg::Header>();
auto msg_0_transforms_0_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_0_transforms_0_header_stamp_sec;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_header_stamp_sec, sizeof(int32_t), fd, "msg_0_transforms_0_header_stamp_sec", 0, "int32_t");
msg_0_transforms_0_header_stamp->sec = msg_0_transforms_0_header_stamp_sec;

uint32_t msg_0_transforms_0_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_header_stamp_nanosec, sizeof(uint32_t), fd, "msg_0_transforms_0_header_stamp_nanosec", 0, "uint32_t");
msg_0_transforms_0_header_stamp->nanosec = msg_0_transforms_0_header_stamp_nanosec;

msg_0_transforms_0_header->stamp = *msg_0_transforms_0_header_stamp;

uint8_t msg_0_transforms_0_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_header_frame_id_string_size, sizeof(uint8_t), fd, "msg_0_transforms_0_header_frame_id_string_size", 0, "uint8_t");
char msg_0_transforms_0_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_0_transforms_0_header_frame_id_string_data, MAX_LEN, fd, "msg_0_transforms_0_header_frame_id_string_data", 0, "char *");
std::string msg_0_transforms_0_header_frame_id(msg_0_transforms_0_header_frame_id_string_data, msg_0_transforms_0_header_frame_id_string_size);
msg_0_transforms_0_header->frame_id = msg_0_transforms_0_header_frame_id;

msg_0_transforms_0->header = *msg_0_transforms_0_header;

uint8_t msg_0_transforms_0_child_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_child_frame_id_string_size, sizeof(uint8_t), fd, "msg_0_transforms_0_child_frame_id_string_size", 0, "uint8_t");
char msg_0_transforms_0_child_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_0_transforms_0_child_frame_id_string_data, MAX_LEN, fd, "msg_0_transforms_0_child_frame_id_string_data", 0, "char *");
std::string msg_0_transforms_0_child_frame_id(msg_0_transforms_0_child_frame_id_string_data, msg_0_transforms_0_child_frame_id_string_size);
msg_0_transforms_0->child_frame_id = msg_0_transforms_0_child_frame_id;

auto msg_0_transforms_0_transform = std::make_shared<geometry_msgs::msg::Transform>();
auto msg_0_transforms_0_transform_translation = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_0_transforms_0_transform_translation_x;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_transform_translation_x, sizeof(double), fd, "msg_0_transforms_0_transform_translation_x", 0, "double");
msg_0_transforms_0_transform_translation->x = msg_0_transforms_0_transform_translation_x;

double msg_0_transforms_0_transform_translation_y;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_transform_translation_y, sizeof(double), fd, "msg_0_transforms_0_transform_translation_y", 0, "double");
msg_0_transforms_0_transform_translation->y = msg_0_transforms_0_transform_translation_y;

double msg_0_transforms_0_transform_translation_z;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_transform_translation_z, sizeof(double), fd, "msg_0_transforms_0_transform_translation_z", 0, "double");
msg_0_transforms_0_transform_translation->z = msg_0_transforms_0_transform_translation_z;

msg_0_transforms_0_transform->translation = *msg_0_transforms_0_transform_translation;

auto msg_0_transforms_0_transform_rotation = std::make_shared<geometry_msgs::msg::Quaternion>();
double msg_0_transforms_0_transform_rotation_x;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_transform_rotation_x, sizeof(double), fd, "msg_0_transforms_0_transform_rotation_x", 0, "double");
msg_0_transforms_0_transform_rotation->x = msg_0_transforms_0_transform_rotation_x;

double msg_0_transforms_0_transform_rotation_y;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_transform_rotation_y, sizeof(double), fd, "msg_0_transforms_0_transform_rotation_y", 0, "double");
msg_0_transforms_0_transform_rotation->y = msg_0_transforms_0_transform_rotation_y;

double msg_0_transforms_0_transform_rotation_z;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_transform_rotation_z, sizeof(double), fd, "msg_0_transforms_0_transform_rotation_z", 0, "double");
msg_0_transforms_0_transform_rotation->z = msg_0_transforms_0_transform_rotation_z;

double msg_0_transforms_0_transform_rotation_w;
symros_make_symbolic_with_prefix(&msg_0_transforms_0_transform_rotation_w, sizeof(double), fd, "msg_0_transforms_0_transform_rotation_w", 0, "double");
msg_0_transforms_0_transform_rotation->w = msg_0_transforms_0_transform_rotation_w;

msg_0_transforms_0_transform->rotation = *msg_0_transforms_0_transform_rotation;

msg_0_transforms_0->transform = *msg_0_transforms_0_transform;

bool empty_msg_0_transforms;
symros_make_symbolic_with_prefix(&empty_msg_0_transforms, sizeof(bool), fd, "empty_msg_0_transforms", 2, "bool");

if (!empty_msg_0_transforms) {

msg_0->transforms.push_back(*msg_0_transforms_0);


}


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);



if (empty_msg_0_transforms) {}


SYMROS__CALL_SUBSCRIPTION_CALLBACK(sub_0, msg_0, msg_0_info_);



printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto msg_1 = std::make_shared<tf2_msgs::msg::TFMessage>();
auto msg_1_transforms_0 = std::make_shared<geometry_msgs::msg::TransformStamped>();
auto msg_1_transforms_0_header = std::make_shared<std_msgs::msg::Header>();
auto msg_1_transforms_0_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_1_transforms_0_header_stamp_sec;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_header_stamp_sec, sizeof(int32_t), fd, "msg_1_transforms_0_header_stamp_sec", 0, "int32_t");
msg_1_transforms_0_header_stamp->sec = msg_1_transforms_0_header_stamp_sec;

uint32_t msg_1_transforms_0_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_header_stamp_nanosec, sizeof(uint32_t), fd, "msg_1_transforms_0_header_stamp_nanosec", 0, "uint32_t");
msg_1_transforms_0_header_stamp->nanosec = msg_1_transforms_0_header_stamp_nanosec;

msg_1_transforms_0_header->stamp = *msg_1_transforms_0_header_stamp;

uint8_t msg_1_transforms_0_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_header_frame_id_string_size, sizeof(uint8_t), fd, "msg_1_transforms_0_header_frame_id_string_size", 0, "uint8_t");
char msg_1_transforms_0_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_1_transforms_0_header_frame_id_string_data, MAX_LEN, fd, "msg_1_transforms_0_header_frame_id_string_data", 0, "char *");
std::string msg_1_transforms_0_header_frame_id(msg_1_transforms_0_header_frame_id_string_data, msg_1_transforms_0_header_frame_id_string_size);
msg_1_transforms_0_header->frame_id = msg_1_transforms_0_header_frame_id;

msg_1_transforms_0->header = *msg_1_transforms_0_header;

uint8_t msg_1_transforms_0_child_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_child_frame_id_string_size, sizeof(uint8_t), fd, "msg_1_transforms_0_child_frame_id_string_size", 0, "uint8_t");
char msg_1_transforms_0_child_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_1_transforms_0_child_frame_id_string_data, MAX_LEN, fd, "msg_1_transforms_0_child_frame_id_string_data", 0, "char *");
std::string msg_1_transforms_0_child_frame_id(msg_1_transforms_0_child_frame_id_string_data, msg_1_transforms_0_child_frame_id_string_size);
msg_1_transforms_0->child_frame_id = msg_1_transforms_0_child_frame_id;

auto msg_1_transforms_0_transform = std::make_shared<geometry_msgs::msg::Transform>();
auto msg_1_transforms_0_transform_translation = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_1_transforms_0_transform_translation_x;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_transform_translation_x, sizeof(double), fd, "msg_1_transforms_0_transform_translation_x", 0, "double");
msg_1_transforms_0_transform_translation->x = msg_1_transforms_0_transform_translation_x;

double msg_1_transforms_0_transform_translation_y;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_transform_translation_y, sizeof(double), fd, "msg_1_transforms_0_transform_translation_y", 0, "double");
msg_1_transforms_0_transform_translation->y = msg_1_transforms_0_transform_translation_y;

double msg_1_transforms_0_transform_translation_z;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_transform_translation_z, sizeof(double), fd, "msg_1_transforms_0_transform_translation_z", 0, "double");
msg_1_transforms_0_transform_translation->z = msg_1_transforms_0_transform_translation_z;

msg_1_transforms_0_transform->translation = *msg_1_transforms_0_transform_translation;

auto msg_1_transforms_0_transform_rotation = std::make_shared<geometry_msgs::msg::Quaternion>();
double msg_1_transforms_0_transform_rotation_x;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_transform_rotation_x, sizeof(double), fd, "msg_1_transforms_0_transform_rotation_x", 0, "double");
msg_1_transforms_0_transform_rotation->x = msg_1_transforms_0_transform_rotation_x;

double msg_1_transforms_0_transform_rotation_y;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_transform_rotation_y, sizeof(double), fd, "msg_1_transforms_0_transform_rotation_y", 0, "double");
msg_1_transforms_0_transform_rotation->y = msg_1_transforms_0_transform_rotation_y;

double msg_1_transforms_0_transform_rotation_z;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_transform_rotation_z, sizeof(double), fd, "msg_1_transforms_0_transform_rotation_z", 0, "double");
msg_1_transforms_0_transform_rotation->z = msg_1_transforms_0_transform_rotation_z;

double msg_1_transforms_0_transform_rotation_w;
symros_make_symbolic_with_prefix(&msg_1_transforms_0_transform_rotation_w, sizeof(double), fd, "msg_1_transforms_0_transform_rotation_w", 0, "double");
msg_1_transforms_0_transform_rotation->w = msg_1_transforms_0_transform_rotation_w;

msg_1_transforms_0_transform->rotation = *msg_1_transforms_0_transform_rotation;

msg_1_transforms_0->transform = *msg_1_transforms_0_transform;

bool empty_msg_1_transforms;
symros_make_symbolic_with_prefix(&empty_msg_1_transforms, sizeof(bool), fd, "empty_msg_1_transforms", 2, "bool");

if (!empty_msg_1_transforms) {

msg_1->transforms.push_back(*msg_1_transforms_0);


}


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);



if (empty_msg_1_transforms) {}


SYMROS__CALL_SUBSCRIPTION_CALLBACK(sub_1, msg_1, msg_1_info_);


bool call_option_5;


printf("\n===== START CALLBACK TEST =====\n");fflush(stdout);


printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto req_2 = std::make_shared<std_srvs::srv::Empty::Request>();

printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




symros_make_symbolic_with_prefix(&call_option_5, sizeof(bool), fd, "call_option_5", 2, "bool");
if (call_option_5) {

SYMROS__CALL_SERVICE_CALLBACK(srv_2, req_2);
}



bool call_option_6;

printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto req_3 = std::make_shared<robot_localization::srv::ToggleFilterProcessing::Request>();
bool req_3_on;
symros_make_symbolic_with_prefix(&req_3_on, sizeof(bool), fd, "req_3_on", 1, "bool");
req_3->on = req_3_on;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




symros_make_symbolic_with_prefix(&call_option_6, sizeof(bool), fd, "call_option_6", 2, "bool");
if (call_option_6) {

SYMROS__CALL_SERVICE_CALLBACK(srv_3, req_3);
}



bool call_option_11;

printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto msg_4 = std::make_shared<geometry_msgs::msg::TwistStamped>();
auto msg_4_header = std::make_shared<std_msgs::msg::Header>();
auto msg_4_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_4_header_stamp_sec;
symros_make_symbolic_with_prefix(&msg_4_header_stamp_sec, sizeof(int32_t), fd, "msg_4_header_stamp_sec", 1, "int32_t");
msg_4_header_stamp->sec = msg_4_header_stamp_sec;

uint32_t msg_4_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&msg_4_header_stamp_nanosec, sizeof(uint32_t), fd, "msg_4_header_stamp_nanosec", 1, "uint32_t");
msg_4_header_stamp->nanosec = msg_4_header_stamp_nanosec;

msg_4_header->stamp = *msg_4_header_stamp;

uint8_t msg_4_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_4_header_frame_id_string_size, sizeof(uint8_t), fd, "msg_4_header_frame_id_string_size", 1, "uint8_t");
char msg_4_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_4_header_frame_id_string_data, MAX_LEN, fd, "msg_4_header_frame_id_string_data", 1, "char *");
std::string msg_4_header_frame_id(msg_4_header_frame_id_string_data, msg_4_header_frame_id_string_size);
msg_4_header->frame_id = msg_4_header_frame_id;

msg_4->header = *msg_4_header;

auto msg_4_twist = std::make_shared<geometry_msgs::msg::Twist>();
auto msg_4_twist_linear = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_4_twist_linear_x;
symros_make_symbolic_with_prefix(&msg_4_twist_linear_x, sizeof(double), fd, "msg_4_twist_linear_x", 1, "double");
msg_4_twist_linear->x = msg_4_twist_linear_x;

double msg_4_twist_linear_y;
symros_make_symbolic_with_prefix(&msg_4_twist_linear_y, sizeof(double), fd, "msg_4_twist_linear_y", 1, "double");
msg_4_twist_linear->y = msg_4_twist_linear_y;

double msg_4_twist_linear_z;
symros_make_symbolic_with_prefix(&msg_4_twist_linear_z, sizeof(double), fd, "msg_4_twist_linear_z", 1, "double");
msg_4_twist_linear->z = msg_4_twist_linear_z;

msg_4_twist->linear = *msg_4_twist_linear;

auto msg_4_twist_angular = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_4_twist_angular_x;
symros_make_symbolic_with_prefix(&msg_4_twist_angular_x, sizeof(double), fd, "msg_4_twist_angular_x", 1, "double");
msg_4_twist_angular->x = msg_4_twist_angular_x;

double msg_4_twist_angular_y;
symros_make_symbolic_with_prefix(&msg_4_twist_angular_y, sizeof(double), fd, "msg_4_twist_angular_y", 1, "double");
msg_4_twist_angular->y = msg_4_twist_angular_y;

double msg_4_twist_angular_z;
symros_make_symbolic_with_prefix(&msg_4_twist_angular_z, sizeof(double), fd, "msg_4_twist_angular_z", 1, "double");
msg_4_twist_angular->z = msg_4_twist_angular_z;

msg_4_twist->angular = *msg_4_twist_angular;

msg_4->twist = *msg_4_twist;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




symros_make_symbolic_with_prefix(&call_option_11, sizeof(bool), fd, "call_option_11", 2, "bool");
if (call_option_11) {

SYMROS__CALL_SUBSCRIPTION_CALLBACK(sub_4, msg_4, msg_4_info_);
}




printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto msg_5 = std::make_shared<nav_msgs::msg::Odometry>();
auto msg_5_header = std::make_shared<std_msgs::msg::Header>();
auto msg_5_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_5_header_stamp_sec;
symros_make_symbolic_with_prefix(&msg_5_header_stamp_sec, sizeof(int32_t), fd, "msg_5_header_stamp_sec", 1, "int32_t");
msg_5_header_stamp->sec = msg_5_header_stamp_sec;

uint32_t msg_5_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&msg_5_header_stamp_nanosec, sizeof(uint32_t), fd, "msg_5_header_stamp_nanosec", 1, "uint32_t");
msg_5_header_stamp->nanosec = msg_5_header_stamp_nanosec;

msg_5_header->stamp = *msg_5_header_stamp;

uint8_t msg_5_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_5_header_frame_id_string_size, sizeof(uint8_t), fd, "msg_5_header_frame_id_string_size", 1, "uint8_t");
char msg_5_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_5_header_frame_id_string_data, MAX_LEN, fd, "msg_5_header_frame_id_string_data", 1, "char *");
std::string msg_5_header_frame_id(msg_5_header_frame_id_string_data, msg_5_header_frame_id_string_size);
msg_5_header->frame_id = msg_5_header_frame_id;

msg_5->header = *msg_5_header;

uint8_t msg_5_child_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_5_child_frame_id_string_size, sizeof(uint8_t), fd, "msg_5_child_frame_id_string_size", 1, "uint8_t");
char msg_5_child_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_5_child_frame_id_string_data, MAX_LEN, fd, "msg_5_child_frame_id_string_data", 1, "char *");
std::string msg_5_child_frame_id(msg_5_child_frame_id_string_data, msg_5_child_frame_id_string_size);
msg_5->child_frame_id = msg_5_child_frame_id;

auto msg_5_pose = std::make_shared<geometry_msgs::msg::PoseWithCovariance>();
auto msg_5_pose_pose = std::make_shared<geometry_msgs::msg::Pose>();
auto msg_5_pose_pose_position = std::make_shared<geometry_msgs::msg::Point>();
double msg_5_pose_pose_position_x;
symros_make_symbolic_with_prefix(&msg_5_pose_pose_position_x, sizeof(double), fd, "msg_5_pose_pose_position_x", 1, "double");
msg_5_pose_pose_position->x = msg_5_pose_pose_position_x;

double msg_5_pose_pose_position_y;
symros_make_symbolic_with_prefix(&msg_5_pose_pose_position_y, sizeof(double), fd, "msg_5_pose_pose_position_y", 1, "double");
msg_5_pose_pose_position->y = msg_5_pose_pose_position_y;

double msg_5_pose_pose_position_z;
symros_make_symbolic_with_prefix(&msg_5_pose_pose_position_z, sizeof(double), fd, "msg_5_pose_pose_position_z", 1, "double");
msg_5_pose_pose_position->z = msg_5_pose_pose_position_z;

msg_5_pose_pose->position = *msg_5_pose_pose_position;

auto msg_5_pose_pose_orientation = std::make_shared<geometry_msgs::msg::Quaternion>();
double msg_5_pose_pose_orientation_x;
symros_make_symbolic_with_prefix(&msg_5_pose_pose_orientation_x, sizeof(double), fd, "msg_5_pose_pose_orientation_x", 1, "double");
msg_5_pose_pose_orientation->x = msg_5_pose_pose_orientation_x;

double msg_5_pose_pose_orientation_y;
symros_make_symbolic_with_prefix(&msg_5_pose_pose_orientation_y, sizeof(double), fd, "msg_5_pose_pose_orientation_y", 1, "double");
msg_5_pose_pose_orientation->y = msg_5_pose_pose_orientation_y;

double msg_5_pose_pose_orientation_z;
symros_make_symbolic_with_prefix(&msg_5_pose_pose_orientation_z, sizeof(double), fd, "msg_5_pose_pose_orientation_z", 1, "double");
msg_5_pose_pose_orientation->z = msg_5_pose_pose_orientation_z;

double msg_5_pose_pose_orientation_w;
symros_make_symbolic_with_prefix(&msg_5_pose_pose_orientation_w, sizeof(double), fd, "msg_5_pose_pose_orientation_w", 1, "double");
msg_5_pose_pose_orientation->w = msg_5_pose_pose_orientation_w;

msg_5_pose_pose->orientation = *msg_5_pose_pose_orientation;

msg_5_pose->pose = *msg_5_pose_pose;

double msg_5_pose_covariance_0;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_0, sizeof(double), fd, "msg_5_pose_covariance_0", 1, "double");
double msg_5_pose_covariance_1;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_1, sizeof(double), fd, "msg_5_pose_covariance_1", 1, "double");
double msg_5_pose_covariance_2;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_2, sizeof(double), fd, "msg_5_pose_covariance_2", 1, "double");
double msg_5_pose_covariance_3;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_3, sizeof(double), fd, "msg_5_pose_covariance_3", 1, "double");
double msg_5_pose_covariance_4;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_4, sizeof(double), fd, "msg_5_pose_covariance_4", 1, "double");
double msg_5_pose_covariance_5;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_5, sizeof(double), fd, "msg_5_pose_covariance_5", 1, "double");
double msg_5_pose_covariance_6;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_6, sizeof(double), fd, "msg_5_pose_covariance_6", 1, "double");
double msg_5_pose_covariance_7;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_7, sizeof(double), fd, "msg_5_pose_covariance_7", 1, "double");
double msg_5_pose_covariance_8;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_8, sizeof(double), fd, "msg_5_pose_covariance_8", 1, "double");
double msg_5_pose_covariance_9;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_9, sizeof(double), fd, "msg_5_pose_covariance_9", 1, "double");
double msg_5_pose_covariance_10;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_10, sizeof(double), fd, "msg_5_pose_covariance_10", 1, "double");
double msg_5_pose_covariance_11;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_11, sizeof(double), fd, "msg_5_pose_covariance_11", 1, "double");
double msg_5_pose_covariance_12;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_12, sizeof(double), fd, "msg_5_pose_covariance_12", 1, "double");
double msg_5_pose_covariance_13;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_13, sizeof(double), fd, "msg_5_pose_covariance_13", 1, "double");
double msg_5_pose_covariance_14;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_14, sizeof(double), fd, "msg_5_pose_covariance_14", 1, "double");
double msg_5_pose_covariance_15;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_15, sizeof(double), fd, "msg_5_pose_covariance_15", 1, "double");
double msg_5_pose_covariance_16;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_16, sizeof(double), fd, "msg_5_pose_covariance_16", 1, "double");
double msg_5_pose_covariance_17;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_17, sizeof(double), fd, "msg_5_pose_covariance_17", 1, "double");
double msg_5_pose_covariance_18;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_18, sizeof(double), fd, "msg_5_pose_covariance_18", 1, "double");
double msg_5_pose_covariance_19;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_19, sizeof(double), fd, "msg_5_pose_covariance_19", 1, "double");
double msg_5_pose_covariance_20;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_20, sizeof(double), fd, "msg_5_pose_covariance_20", 1, "double");
double msg_5_pose_covariance_21;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_21, sizeof(double), fd, "msg_5_pose_covariance_21", 1, "double");
double msg_5_pose_covariance_22;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_22, sizeof(double), fd, "msg_5_pose_covariance_22", 1, "double");
double msg_5_pose_covariance_23;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_23, sizeof(double), fd, "msg_5_pose_covariance_23", 1, "double");
double msg_5_pose_covariance_24;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_24, sizeof(double), fd, "msg_5_pose_covariance_24", 1, "double");
double msg_5_pose_covariance_25;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_25, sizeof(double), fd, "msg_5_pose_covariance_25", 1, "double");
double msg_5_pose_covariance_26;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_26, sizeof(double), fd, "msg_5_pose_covariance_26", 1, "double");
double msg_5_pose_covariance_27;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_27, sizeof(double), fd, "msg_5_pose_covariance_27", 1, "double");
double msg_5_pose_covariance_28;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_28, sizeof(double), fd, "msg_5_pose_covariance_28", 1, "double");
double msg_5_pose_covariance_29;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_29, sizeof(double), fd, "msg_5_pose_covariance_29", 1, "double");
double msg_5_pose_covariance_30;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_30, sizeof(double), fd, "msg_5_pose_covariance_30", 1, "double");
double msg_5_pose_covariance_31;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_31, sizeof(double), fd, "msg_5_pose_covariance_31", 1, "double");
double msg_5_pose_covariance_32;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_32, sizeof(double), fd, "msg_5_pose_covariance_32", 1, "double");
double msg_5_pose_covariance_33;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_33, sizeof(double), fd, "msg_5_pose_covariance_33", 1, "double");
double msg_5_pose_covariance_34;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_34, sizeof(double), fd, "msg_5_pose_covariance_34", 1, "double");
double msg_5_pose_covariance_35;
symros_make_symbolic_with_prefix(&msg_5_pose_covariance_35, sizeof(double), fd, "msg_5_pose_covariance_35", 1, "double");
msg_5_pose->covariance[0] = msg_5_pose_covariance_0;

msg_5_pose->covariance[1] = msg_5_pose_covariance_1;

msg_5_pose->covariance[2] = msg_5_pose_covariance_2;

msg_5_pose->covariance[3] = msg_5_pose_covariance_3;

msg_5_pose->covariance[4] = msg_5_pose_covariance_4;

msg_5_pose->covariance[5] = msg_5_pose_covariance_5;

msg_5_pose->covariance[6] = msg_5_pose_covariance_6;

msg_5_pose->covariance[7] = msg_5_pose_covariance_7;

msg_5_pose->covariance[8] = msg_5_pose_covariance_8;

msg_5_pose->covariance[9] = msg_5_pose_covariance_9;

msg_5_pose->covariance[10] = msg_5_pose_covariance_10;

msg_5_pose->covariance[11] = msg_5_pose_covariance_11;

msg_5_pose->covariance[12] = msg_5_pose_covariance_12;

msg_5_pose->covariance[13] = msg_5_pose_covariance_13;

msg_5_pose->covariance[14] = msg_5_pose_covariance_14;

msg_5_pose->covariance[15] = msg_5_pose_covariance_15;

msg_5_pose->covariance[16] = msg_5_pose_covariance_16;

msg_5_pose->covariance[17] = msg_5_pose_covariance_17;

msg_5_pose->covariance[18] = msg_5_pose_covariance_18;

msg_5_pose->covariance[19] = msg_5_pose_covariance_19;

msg_5_pose->covariance[20] = msg_5_pose_covariance_20;

msg_5_pose->covariance[21] = msg_5_pose_covariance_21;

msg_5_pose->covariance[22] = msg_5_pose_covariance_22;

msg_5_pose->covariance[23] = msg_5_pose_covariance_23;

msg_5_pose->covariance[24] = msg_5_pose_covariance_24;

msg_5_pose->covariance[25] = msg_5_pose_covariance_25;

msg_5_pose->covariance[26] = msg_5_pose_covariance_26;

msg_5_pose->covariance[27] = msg_5_pose_covariance_27;

msg_5_pose->covariance[28] = msg_5_pose_covariance_28;

msg_5_pose->covariance[29] = msg_5_pose_covariance_29;

msg_5_pose->covariance[30] = msg_5_pose_covariance_30;

msg_5_pose->covariance[31] = msg_5_pose_covariance_31;

msg_5_pose->covariance[32] = msg_5_pose_covariance_32;

msg_5_pose->covariance[33] = msg_5_pose_covariance_33;

msg_5_pose->covariance[34] = msg_5_pose_covariance_34;

msg_5_pose->covariance[35] = msg_5_pose_covariance_35;

msg_5->pose = *msg_5_pose;

auto msg_5_twist = std::make_shared<geometry_msgs::msg::TwistWithCovariance>();
auto msg_5_twist_twist = std::make_shared<geometry_msgs::msg::Twist>();
auto msg_5_twist_twist_linear = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_5_twist_twist_linear_x;
symros_make_symbolic_with_prefix(&msg_5_twist_twist_linear_x, sizeof(double), fd, "msg_5_twist_twist_linear_x", 1, "double");
msg_5_twist_twist_linear->x = msg_5_twist_twist_linear_x;

double msg_5_twist_twist_linear_y;
symros_make_symbolic_with_prefix(&msg_5_twist_twist_linear_y, sizeof(double), fd, "msg_5_twist_twist_linear_y", 1, "double");
msg_5_twist_twist_linear->y = msg_5_twist_twist_linear_y;

double msg_5_twist_twist_linear_z;
symros_make_symbolic_with_prefix(&msg_5_twist_twist_linear_z, sizeof(double), fd, "msg_5_twist_twist_linear_z", 1, "double");
msg_5_twist_twist_linear->z = msg_5_twist_twist_linear_z;

msg_5_twist_twist->linear = *msg_5_twist_twist_linear;

auto msg_5_twist_twist_angular = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_5_twist_twist_angular_x;
symros_make_symbolic_with_prefix(&msg_5_twist_twist_angular_x, sizeof(double), fd, "msg_5_twist_twist_angular_x", 1, "double");
msg_5_twist_twist_angular->x = msg_5_twist_twist_angular_x;

double msg_5_twist_twist_angular_y;
symros_make_symbolic_with_prefix(&msg_5_twist_twist_angular_y, sizeof(double), fd, "msg_5_twist_twist_angular_y", 1, "double");
msg_5_twist_twist_angular->y = msg_5_twist_twist_angular_y;

double msg_5_twist_twist_angular_z;
symros_make_symbolic_with_prefix(&msg_5_twist_twist_angular_z, sizeof(double), fd, "msg_5_twist_twist_angular_z", 1, "double");
msg_5_twist_twist_angular->z = msg_5_twist_twist_angular_z;

msg_5_twist_twist->angular = *msg_5_twist_twist_angular;

msg_5_twist->twist = *msg_5_twist_twist;

double msg_5_twist_covariance_0;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_0, sizeof(double), fd, "msg_5_twist_covariance_0", 1, "double");
double msg_5_twist_covariance_1;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_1, sizeof(double), fd, "msg_5_twist_covariance_1", 1, "double");
double msg_5_twist_covariance_2;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_2, sizeof(double), fd, "msg_5_twist_covariance_2", 1, "double");
double msg_5_twist_covariance_3;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_3, sizeof(double), fd, "msg_5_twist_covariance_3", 1, "double");
double msg_5_twist_covariance_4;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_4, sizeof(double), fd, "msg_5_twist_covariance_4", 1, "double");
double msg_5_twist_covariance_5;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_5, sizeof(double), fd, "msg_5_twist_covariance_5", 1, "double");
double msg_5_twist_covariance_6;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_6, sizeof(double), fd, "msg_5_twist_covariance_6", 1, "double");
double msg_5_twist_covariance_7;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_7, sizeof(double), fd, "msg_5_twist_covariance_7", 1, "double");
double msg_5_twist_covariance_8;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_8, sizeof(double), fd, "msg_5_twist_covariance_8", 1, "double");
double msg_5_twist_covariance_9;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_9, sizeof(double), fd, "msg_5_twist_covariance_9", 1, "double");
double msg_5_twist_covariance_10;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_10, sizeof(double), fd, "msg_5_twist_covariance_10", 1, "double");
double msg_5_twist_covariance_11;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_11, sizeof(double), fd, "msg_5_twist_covariance_11", 1, "double");
double msg_5_twist_covariance_12;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_12, sizeof(double), fd, "msg_5_twist_covariance_12", 1, "double");
double msg_5_twist_covariance_13;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_13, sizeof(double), fd, "msg_5_twist_covariance_13", 1, "double");
double msg_5_twist_covariance_14;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_14, sizeof(double), fd, "msg_5_twist_covariance_14", 1, "double");
double msg_5_twist_covariance_15;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_15, sizeof(double), fd, "msg_5_twist_covariance_15", 1, "double");
double msg_5_twist_covariance_16;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_16, sizeof(double), fd, "msg_5_twist_covariance_16", 1, "double");
double msg_5_twist_covariance_17;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_17, sizeof(double), fd, "msg_5_twist_covariance_17", 1, "double");
double msg_5_twist_covariance_18;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_18, sizeof(double), fd, "msg_5_twist_covariance_18", 1, "double");
double msg_5_twist_covariance_19;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_19, sizeof(double), fd, "msg_5_twist_covariance_19", 1, "double");
double msg_5_twist_covariance_20;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_20, sizeof(double), fd, "msg_5_twist_covariance_20", 1, "double");
double msg_5_twist_covariance_21;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_21, sizeof(double), fd, "msg_5_twist_covariance_21", 1, "double");
double msg_5_twist_covariance_22;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_22, sizeof(double), fd, "msg_5_twist_covariance_22", 1, "double");
double msg_5_twist_covariance_23;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_23, sizeof(double), fd, "msg_5_twist_covariance_23", 1, "double");
double msg_5_twist_covariance_24;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_24, sizeof(double), fd, "msg_5_twist_covariance_24", 1, "double");
double msg_5_twist_covariance_25;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_25, sizeof(double), fd, "msg_5_twist_covariance_25", 1, "double");
double msg_5_twist_covariance_26;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_26, sizeof(double), fd, "msg_5_twist_covariance_26", 1, "double");
double msg_5_twist_covariance_27;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_27, sizeof(double), fd, "msg_5_twist_covariance_27", 1, "double");
double msg_5_twist_covariance_28;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_28, sizeof(double), fd, "msg_5_twist_covariance_28", 1, "double");
double msg_5_twist_covariance_29;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_29, sizeof(double), fd, "msg_5_twist_covariance_29", 1, "double");
double msg_5_twist_covariance_30;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_30, sizeof(double), fd, "msg_5_twist_covariance_30", 1, "double");
double msg_5_twist_covariance_31;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_31, sizeof(double), fd, "msg_5_twist_covariance_31", 1, "double");
double msg_5_twist_covariance_32;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_32, sizeof(double), fd, "msg_5_twist_covariance_32", 1, "double");
double msg_5_twist_covariance_33;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_33, sizeof(double), fd, "msg_5_twist_covariance_33", 1, "double");
double msg_5_twist_covariance_34;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_34, sizeof(double), fd, "msg_5_twist_covariance_34", 1, "double");
double msg_5_twist_covariance_35;
symros_make_symbolic_with_prefix(&msg_5_twist_covariance_35, sizeof(double), fd, "msg_5_twist_covariance_35", 1, "double");
msg_5_twist->covariance[0] = msg_5_twist_covariance_0;

msg_5_twist->covariance[1] = msg_5_twist_covariance_1;

msg_5_twist->covariance[2] = msg_5_twist_covariance_2;

msg_5_twist->covariance[3] = msg_5_twist_covariance_3;

msg_5_twist->covariance[4] = msg_5_twist_covariance_4;

msg_5_twist->covariance[5] = msg_5_twist_covariance_5;

msg_5_twist->covariance[6] = msg_5_twist_covariance_6;

msg_5_twist->covariance[7] = msg_5_twist_covariance_7;

msg_5_twist->covariance[8] = msg_5_twist_covariance_8;

msg_5_twist->covariance[9] = msg_5_twist_covariance_9;

msg_5_twist->covariance[10] = msg_5_twist_covariance_10;

msg_5_twist->covariance[11] = msg_5_twist_covariance_11;

msg_5_twist->covariance[12] = msg_5_twist_covariance_12;

msg_5_twist->covariance[13] = msg_5_twist_covariance_13;

msg_5_twist->covariance[14] = msg_5_twist_covariance_14;

msg_5_twist->covariance[15] = msg_5_twist_covariance_15;

msg_5_twist->covariance[16] = msg_5_twist_covariance_16;

msg_5_twist->covariance[17] = msg_5_twist_covariance_17;

msg_5_twist->covariance[18] = msg_5_twist_covariance_18;

msg_5_twist->covariance[19] = msg_5_twist_covariance_19;

msg_5_twist->covariance[20] = msg_5_twist_covariance_20;

msg_5_twist->covariance[21] = msg_5_twist_covariance_21;

msg_5_twist->covariance[22] = msg_5_twist_covariance_22;

msg_5_twist->covariance[23] = msg_5_twist_covariance_23;

msg_5_twist->covariance[24] = msg_5_twist_covariance_24;

msg_5_twist->covariance[25] = msg_5_twist_covariance_25;

msg_5_twist->covariance[26] = msg_5_twist_covariance_26;

msg_5_twist->covariance[27] = msg_5_twist_covariance_27;

msg_5_twist->covariance[28] = msg_5_twist_covariance_28;

msg_5_twist->covariance[29] = msg_5_twist_covariance_29;

msg_5_twist->covariance[30] = msg_5_twist_covariance_30;

msg_5_twist->covariance[31] = msg_5_twist_covariance_31;

msg_5_twist->covariance[32] = msg_5_twist_covariance_32;

msg_5_twist->covariance[33] = msg_5_twist_covariance_33;

msg_5_twist->covariance[34] = msg_5_twist_covariance_34;

msg_5_twist->covariance[35] = msg_5_twist_covariance_35;

msg_5->twist = *msg_5_twist;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




SYMROS__CALL_SUBSCRIPTION_CALLBACK(sub_5, msg_5, msg_5_info_);



printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto msg_6 = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
auto msg_6_header = std::make_shared<std_msgs::msg::Header>();
auto msg_6_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_6_header_stamp_sec;
symros_make_symbolic_with_prefix(&msg_6_header_stamp_sec, sizeof(int32_t), fd, "msg_6_header_stamp_sec", 1, "int32_t");
msg_6_header_stamp->sec = msg_6_header_stamp_sec;

uint32_t msg_6_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&msg_6_header_stamp_nanosec, sizeof(uint32_t), fd, "msg_6_header_stamp_nanosec", 1, "uint32_t");
msg_6_header_stamp->nanosec = msg_6_header_stamp_nanosec;

msg_6_header->stamp = *msg_6_header_stamp;

uint8_t msg_6_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_6_header_frame_id_string_size, sizeof(uint8_t), fd, "msg_6_header_frame_id_string_size", 1, "uint8_t");
char msg_6_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_6_header_frame_id_string_data, MAX_LEN, fd, "msg_6_header_frame_id_string_data", 1, "char *");
std::string msg_6_header_frame_id(msg_6_header_frame_id_string_data, msg_6_header_frame_id_string_size);
msg_6_header->frame_id = msg_6_header_frame_id;

msg_6->header = *msg_6_header;

auto msg_6_pose = std::make_shared<geometry_msgs::msg::PoseWithCovariance>();
auto msg_6_pose_pose = std::make_shared<geometry_msgs::msg::Pose>();
auto msg_6_pose_pose_position = std::make_shared<geometry_msgs::msg::Point>();
double msg_6_pose_pose_position_x;
symros_make_symbolic_with_prefix(&msg_6_pose_pose_position_x, sizeof(double), fd, "msg_6_pose_pose_position_x", 1, "double");
msg_6_pose_pose_position->x = msg_6_pose_pose_position_x;

double msg_6_pose_pose_position_y;
symros_make_symbolic_with_prefix(&msg_6_pose_pose_position_y, sizeof(double), fd, "msg_6_pose_pose_position_y", 1, "double");
msg_6_pose_pose_position->y = msg_6_pose_pose_position_y;

double msg_6_pose_pose_position_z;
symros_make_symbolic_with_prefix(&msg_6_pose_pose_position_z, sizeof(double), fd, "msg_6_pose_pose_position_z", 1, "double");
msg_6_pose_pose_position->z = msg_6_pose_pose_position_z;

msg_6_pose_pose->position = *msg_6_pose_pose_position;

auto msg_6_pose_pose_orientation = std::make_shared<geometry_msgs::msg::Quaternion>();
double msg_6_pose_pose_orientation_x;
symros_make_symbolic_with_prefix(&msg_6_pose_pose_orientation_x, sizeof(double), fd, "msg_6_pose_pose_orientation_x", 1, "double");
msg_6_pose_pose_orientation->x = msg_6_pose_pose_orientation_x;

double msg_6_pose_pose_orientation_y;
symros_make_symbolic_with_prefix(&msg_6_pose_pose_orientation_y, sizeof(double), fd, "msg_6_pose_pose_orientation_y", 1, "double");
msg_6_pose_pose_orientation->y = msg_6_pose_pose_orientation_y;

double msg_6_pose_pose_orientation_z;
symros_make_symbolic_with_prefix(&msg_6_pose_pose_orientation_z, sizeof(double), fd, "msg_6_pose_pose_orientation_z", 1, "double");
msg_6_pose_pose_orientation->z = msg_6_pose_pose_orientation_z;

double msg_6_pose_pose_orientation_w;
symros_make_symbolic_with_prefix(&msg_6_pose_pose_orientation_w, sizeof(double), fd, "msg_6_pose_pose_orientation_w", 1, "double");
msg_6_pose_pose_orientation->w = msg_6_pose_pose_orientation_w;

msg_6_pose_pose->orientation = *msg_6_pose_pose_orientation;

msg_6_pose->pose = *msg_6_pose_pose;

double msg_6_pose_covariance_0;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_0, sizeof(double), fd, "msg_6_pose_covariance_0", 1, "double");
double msg_6_pose_covariance_1;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_1, sizeof(double), fd, "msg_6_pose_covariance_1", 1, "double");
double msg_6_pose_covariance_2;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_2, sizeof(double), fd, "msg_6_pose_covariance_2", 1, "double");
double msg_6_pose_covariance_3;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_3, sizeof(double), fd, "msg_6_pose_covariance_3", 1, "double");
double msg_6_pose_covariance_4;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_4, sizeof(double), fd, "msg_6_pose_covariance_4", 1, "double");
double msg_6_pose_covariance_5;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_5, sizeof(double), fd, "msg_6_pose_covariance_5", 1, "double");
double msg_6_pose_covariance_6;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_6, sizeof(double), fd, "msg_6_pose_covariance_6", 1, "double");
double msg_6_pose_covariance_7;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_7, sizeof(double), fd, "msg_6_pose_covariance_7", 1, "double");
double msg_6_pose_covariance_8;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_8, sizeof(double), fd, "msg_6_pose_covariance_8", 1, "double");
double msg_6_pose_covariance_9;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_9, sizeof(double), fd, "msg_6_pose_covariance_9", 1, "double");
double msg_6_pose_covariance_10;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_10, sizeof(double), fd, "msg_6_pose_covariance_10", 1, "double");
double msg_6_pose_covariance_11;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_11, sizeof(double), fd, "msg_6_pose_covariance_11", 1, "double");
double msg_6_pose_covariance_12;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_12, sizeof(double), fd, "msg_6_pose_covariance_12", 1, "double");
double msg_6_pose_covariance_13;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_13, sizeof(double), fd, "msg_6_pose_covariance_13", 1, "double");
double msg_6_pose_covariance_14;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_14, sizeof(double), fd, "msg_6_pose_covariance_14", 1, "double");
double msg_6_pose_covariance_15;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_15, sizeof(double), fd, "msg_6_pose_covariance_15", 1, "double");
double msg_6_pose_covariance_16;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_16, sizeof(double), fd, "msg_6_pose_covariance_16", 1, "double");
double msg_6_pose_covariance_17;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_17, sizeof(double), fd, "msg_6_pose_covariance_17", 1, "double");
double msg_6_pose_covariance_18;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_18, sizeof(double), fd, "msg_6_pose_covariance_18", 1, "double");
double msg_6_pose_covariance_19;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_19, sizeof(double), fd, "msg_6_pose_covariance_19", 1, "double");
double msg_6_pose_covariance_20;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_20, sizeof(double), fd, "msg_6_pose_covariance_20", 1, "double");
double msg_6_pose_covariance_21;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_21, sizeof(double), fd, "msg_6_pose_covariance_21", 1, "double");
double msg_6_pose_covariance_22;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_22, sizeof(double), fd, "msg_6_pose_covariance_22", 1, "double");
double msg_6_pose_covariance_23;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_23, sizeof(double), fd, "msg_6_pose_covariance_23", 1, "double");
double msg_6_pose_covariance_24;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_24, sizeof(double), fd, "msg_6_pose_covariance_24", 1, "double");
double msg_6_pose_covariance_25;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_25, sizeof(double), fd, "msg_6_pose_covariance_25", 1, "double");
double msg_6_pose_covariance_26;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_26, sizeof(double), fd, "msg_6_pose_covariance_26", 1, "double");
double msg_6_pose_covariance_27;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_27, sizeof(double), fd, "msg_6_pose_covariance_27", 1, "double");
double msg_6_pose_covariance_28;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_28, sizeof(double), fd, "msg_6_pose_covariance_28", 1, "double");
double msg_6_pose_covariance_29;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_29, sizeof(double), fd, "msg_6_pose_covariance_29", 1, "double");
double msg_6_pose_covariance_30;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_30, sizeof(double), fd, "msg_6_pose_covariance_30", 1, "double");
double msg_6_pose_covariance_31;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_31, sizeof(double), fd, "msg_6_pose_covariance_31", 1, "double");
double msg_6_pose_covariance_32;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_32, sizeof(double), fd, "msg_6_pose_covariance_32", 1, "double");
double msg_6_pose_covariance_33;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_33, sizeof(double), fd, "msg_6_pose_covariance_33", 1, "double");
double msg_6_pose_covariance_34;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_34, sizeof(double), fd, "msg_6_pose_covariance_34", 1, "double");
double msg_6_pose_covariance_35;
symros_make_symbolic_with_prefix(&msg_6_pose_covariance_35, sizeof(double), fd, "msg_6_pose_covariance_35", 1, "double");
msg_6_pose->covariance[0] = msg_6_pose_covariance_0;

msg_6_pose->covariance[1] = msg_6_pose_covariance_1;

msg_6_pose->covariance[2] = msg_6_pose_covariance_2;

msg_6_pose->covariance[3] = msg_6_pose_covariance_3;

msg_6_pose->covariance[4] = msg_6_pose_covariance_4;

msg_6_pose->covariance[5] = msg_6_pose_covariance_5;

msg_6_pose->covariance[6] = msg_6_pose_covariance_6;

msg_6_pose->covariance[7] = msg_6_pose_covariance_7;

msg_6_pose->covariance[8] = msg_6_pose_covariance_8;

msg_6_pose->covariance[9] = msg_6_pose_covariance_9;

msg_6_pose->covariance[10] = msg_6_pose_covariance_10;

msg_6_pose->covariance[11] = msg_6_pose_covariance_11;

msg_6_pose->covariance[12] = msg_6_pose_covariance_12;

msg_6_pose->covariance[13] = msg_6_pose_covariance_13;

msg_6_pose->covariance[14] = msg_6_pose_covariance_14;

msg_6_pose->covariance[15] = msg_6_pose_covariance_15;

msg_6_pose->covariance[16] = msg_6_pose_covariance_16;

msg_6_pose->covariance[17] = msg_6_pose_covariance_17;

msg_6_pose->covariance[18] = msg_6_pose_covariance_18;

msg_6_pose->covariance[19] = msg_6_pose_covariance_19;

msg_6_pose->covariance[20] = msg_6_pose_covariance_20;

msg_6_pose->covariance[21] = msg_6_pose_covariance_21;

msg_6_pose->covariance[22] = msg_6_pose_covariance_22;

msg_6_pose->covariance[23] = msg_6_pose_covariance_23;

msg_6_pose->covariance[24] = msg_6_pose_covariance_24;

msg_6_pose->covariance[25] = msg_6_pose_covariance_25;

msg_6_pose->covariance[26] = msg_6_pose_covariance_26;

msg_6_pose->covariance[27] = msg_6_pose_covariance_27;

msg_6_pose->covariance[28] = msg_6_pose_covariance_28;

msg_6_pose->covariance[29] = msg_6_pose_covariance_29;

msg_6_pose->covariance[30] = msg_6_pose_covariance_30;

msg_6_pose->covariance[31] = msg_6_pose_covariance_31;

msg_6_pose->covariance[32] = msg_6_pose_covariance_32;

msg_6_pose->covariance[33] = msg_6_pose_covariance_33;

msg_6_pose->covariance[34] = msg_6_pose_covariance_34;

msg_6_pose->covariance[35] = msg_6_pose_covariance_35;

msg_6->pose = *msg_6_pose;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




SYMROS__CALL_SUBSCRIPTION_CALLBACK(sub_6, msg_6, msg_6_info_);



printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto msg_7 = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
auto msg_7_header = std::make_shared<std_msgs::msg::Header>();
auto msg_7_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_7_header_stamp_sec;
symros_make_symbolic_with_prefix(&msg_7_header_stamp_sec, sizeof(int32_t), fd, "msg_7_header_stamp_sec", 1, "int32_t");
msg_7_header_stamp->sec = msg_7_header_stamp_sec;

uint32_t msg_7_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&msg_7_header_stamp_nanosec, sizeof(uint32_t), fd, "msg_7_header_stamp_nanosec", 1, "uint32_t");
msg_7_header_stamp->nanosec = msg_7_header_stamp_nanosec;

msg_7_header->stamp = *msg_7_header_stamp;

uint8_t msg_7_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_7_header_frame_id_string_size, sizeof(uint8_t), fd, "msg_7_header_frame_id_string_size", 1, "uint8_t");
char msg_7_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_7_header_frame_id_string_data, MAX_LEN, fd, "msg_7_header_frame_id_string_data", 1, "char *");
std::string msg_7_header_frame_id(msg_7_header_frame_id_string_data, msg_7_header_frame_id_string_size);
msg_7_header->frame_id = msg_7_header_frame_id;

msg_7->header = *msg_7_header;

auto msg_7_twist = std::make_shared<geometry_msgs::msg::TwistWithCovariance>();
auto msg_7_twist_twist = std::make_shared<geometry_msgs::msg::Twist>();
auto msg_7_twist_twist_linear = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_7_twist_twist_linear_x;
symros_make_symbolic_with_prefix(&msg_7_twist_twist_linear_x, sizeof(double), fd, "msg_7_twist_twist_linear_x", 1, "double");
msg_7_twist_twist_linear->x = msg_7_twist_twist_linear_x;

double msg_7_twist_twist_linear_y;
symros_make_symbolic_with_prefix(&msg_7_twist_twist_linear_y, sizeof(double), fd, "msg_7_twist_twist_linear_y", 1, "double");
msg_7_twist_twist_linear->y = msg_7_twist_twist_linear_y;

double msg_7_twist_twist_linear_z;
symros_make_symbolic_with_prefix(&msg_7_twist_twist_linear_z, sizeof(double), fd, "msg_7_twist_twist_linear_z", 1, "double");
msg_7_twist_twist_linear->z = msg_7_twist_twist_linear_z;

msg_7_twist_twist->linear = *msg_7_twist_twist_linear;

auto msg_7_twist_twist_angular = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_7_twist_twist_angular_x;
symros_make_symbolic_with_prefix(&msg_7_twist_twist_angular_x, sizeof(double), fd, "msg_7_twist_twist_angular_x", 1, "double");
msg_7_twist_twist_angular->x = msg_7_twist_twist_angular_x;

double msg_7_twist_twist_angular_y;
symros_make_symbolic_with_prefix(&msg_7_twist_twist_angular_y, sizeof(double), fd, "msg_7_twist_twist_angular_y", 1, "double");
msg_7_twist_twist_angular->y = msg_7_twist_twist_angular_y;

double msg_7_twist_twist_angular_z;
symros_make_symbolic_with_prefix(&msg_7_twist_twist_angular_z, sizeof(double), fd, "msg_7_twist_twist_angular_z", 1, "double");
msg_7_twist_twist_angular->z = msg_7_twist_twist_angular_z;

msg_7_twist_twist->angular = *msg_7_twist_twist_angular;

msg_7_twist->twist = *msg_7_twist_twist;

double msg_7_twist_covariance_0;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_0, sizeof(double), fd, "msg_7_twist_covariance_0", 1, "double");
double msg_7_twist_covariance_1;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_1, sizeof(double), fd, "msg_7_twist_covariance_1", 1, "double");
double msg_7_twist_covariance_2;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_2, sizeof(double), fd, "msg_7_twist_covariance_2", 1, "double");
double msg_7_twist_covariance_3;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_3, sizeof(double), fd, "msg_7_twist_covariance_3", 1, "double");
double msg_7_twist_covariance_4;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_4, sizeof(double), fd, "msg_7_twist_covariance_4", 1, "double");
double msg_7_twist_covariance_5;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_5, sizeof(double), fd, "msg_7_twist_covariance_5", 1, "double");
double msg_7_twist_covariance_6;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_6, sizeof(double), fd, "msg_7_twist_covariance_6", 1, "double");
double msg_7_twist_covariance_7;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_7, sizeof(double), fd, "msg_7_twist_covariance_7", 1, "double");
double msg_7_twist_covariance_8;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_8, sizeof(double), fd, "msg_7_twist_covariance_8", 1, "double");
double msg_7_twist_covariance_9;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_9, sizeof(double), fd, "msg_7_twist_covariance_9", 1, "double");
double msg_7_twist_covariance_10;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_10, sizeof(double), fd, "msg_7_twist_covariance_10", 1, "double");
double msg_7_twist_covariance_11;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_11, sizeof(double), fd, "msg_7_twist_covariance_11", 1, "double");
double msg_7_twist_covariance_12;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_12, sizeof(double), fd, "msg_7_twist_covariance_12", 1, "double");
double msg_7_twist_covariance_13;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_13, sizeof(double), fd, "msg_7_twist_covariance_13", 1, "double");
double msg_7_twist_covariance_14;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_14, sizeof(double), fd, "msg_7_twist_covariance_14", 1, "double");
double msg_7_twist_covariance_15;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_15, sizeof(double), fd, "msg_7_twist_covariance_15", 1, "double");
double msg_7_twist_covariance_16;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_16, sizeof(double), fd, "msg_7_twist_covariance_16", 1, "double");
double msg_7_twist_covariance_17;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_17, sizeof(double), fd, "msg_7_twist_covariance_17", 1, "double");
double msg_7_twist_covariance_18;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_18, sizeof(double), fd, "msg_7_twist_covariance_18", 1, "double");
double msg_7_twist_covariance_19;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_19, sizeof(double), fd, "msg_7_twist_covariance_19", 1, "double");
double msg_7_twist_covariance_20;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_20, sizeof(double), fd, "msg_7_twist_covariance_20", 1, "double");
double msg_7_twist_covariance_21;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_21, sizeof(double), fd, "msg_7_twist_covariance_21", 1, "double");
double msg_7_twist_covariance_22;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_22, sizeof(double), fd, "msg_7_twist_covariance_22", 1, "double");
double msg_7_twist_covariance_23;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_23, sizeof(double), fd, "msg_7_twist_covariance_23", 1, "double");
double msg_7_twist_covariance_24;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_24, sizeof(double), fd, "msg_7_twist_covariance_24", 1, "double");
double msg_7_twist_covariance_25;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_25, sizeof(double), fd, "msg_7_twist_covariance_25", 1, "double");
double msg_7_twist_covariance_26;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_26, sizeof(double), fd, "msg_7_twist_covariance_26", 1, "double");
double msg_7_twist_covariance_27;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_27, sizeof(double), fd, "msg_7_twist_covariance_27", 1, "double");
double msg_7_twist_covariance_28;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_28, sizeof(double), fd, "msg_7_twist_covariance_28", 1, "double");
double msg_7_twist_covariance_29;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_29, sizeof(double), fd, "msg_7_twist_covariance_29", 1, "double");
double msg_7_twist_covariance_30;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_30, sizeof(double), fd, "msg_7_twist_covariance_30", 1, "double");
double msg_7_twist_covariance_31;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_31, sizeof(double), fd, "msg_7_twist_covariance_31", 1, "double");
double msg_7_twist_covariance_32;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_32, sizeof(double), fd, "msg_7_twist_covariance_32", 1, "double");
double msg_7_twist_covariance_33;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_33, sizeof(double), fd, "msg_7_twist_covariance_33", 1, "double");
double msg_7_twist_covariance_34;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_34, sizeof(double), fd, "msg_7_twist_covariance_34", 1, "double");
double msg_7_twist_covariance_35;
symros_make_symbolic_with_prefix(&msg_7_twist_covariance_35, sizeof(double), fd, "msg_7_twist_covariance_35", 1, "double");
msg_7_twist->covariance[0] = msg_7_twist_covariance_0;

msg_7_twist->covariance[1] = msg_7_twist_covariance_1;

msg_7_twist->covariance[2] = msg_7_twist_covariance_2;

msg_7_twist->covariance[3] = msg_7_twist_covariance_3;

msg_7_twist->covariance[4] = msg_7_twist_covariance_4;

msg_7_twist->covariance[5] = msg_7_twist_covariance_5;

msg_7_twist->covariance[6] = msg_7_twist_covariance_6;

msg_7_twist->covariance[7] = msg_7_twist_covariance_7;

msg_7_twist->covariance[8] = msg_7_twist_covariance_8;

msg_7_twist->covariance[9] = msg_7_twist_covariance_9;

msg_7_twist->covariance[10] = msg_7_twist_covariance_10;

msg_7_twist->covariance[11] = msg_7_twist_covariance_11;

msg_7_twist->covariance[12] = msg_7_twist_covariance_12;

msg_7_twist->covariance[13] = msg_7_twist_covariance_13;

msg_7_twist->covariance[14] = msg_7_twist_covariance_14;

msg_7_twist->covariance[15] = msg_7_twist_covariance_15;

msg_7_twist->covariance[16] = msg_7_twist_covariance_16;

msg_7_twist->covariance[17] = msg_7_twist_covariance_17;

msg_7_twist->covariance[18] = msg_7_twist_covariance_18;

msg_7_twist->covariance[19] = msg_7_twist_covariance_19;

msg_7_twist->covariance[20] = msg_7_twist_covariance_20;

msg_7_twist->covariance[21] = msg_7_twist_covariance_21;

msg_7_twist->covariance[22] = msg_7_twist_covariance_22;

msg_7_twist->covariance[23] = msg_7_twist_covariance_23;

msg_7_twist->covariance[24] = msg_7_twist_covariance_24;

msg_7_twist->covariance[25] = msg_7_twist_covariance_25;

msg_7_twist->covariance[26] = msg_7_twist_covariance_26;

msg_7_twist->covariance[27] = msg_7_twist_covariance_27;

msg_7_twist->covariance[28] = msg_7_twist_covariance_28;

msg_7_twist->covariance[29] = msg_7_twist_covariance_29;

msg_7_twist->covariance[30] = msg_7_twist_covariance_30;

msg_7_twist->covariance[31] = msg_7_twist_covariance_31;

msg_7_twist->covariance[32] = msg_7_twist_covariance_32;

msg_7_twist->covariance[33] = msg_7_twist_covariance_33;

msg_7_twist->covariance[34] = msg_7_twist_covariance_34;

msg_7_twist->covariance[35] = msg_7_twist_covariance_35;

msg_7->twist = *msg_7_twist;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




SYMROS__CALL_SUBSCRIPTION_CALLBACK(sub_7, msg_7, msg_7_info_);



printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto msg_8 = std::make_shared<sensor_msgs::msg::Imu>();
auto msg_8_header = std::make_shared<std_msgs::msg::Header>();
auto msg_8_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_8_header_stamp_sec;
symros_make_symbolic_with_prefix(&msg_8_header_stamp_sec, sizeof(int32_t), fd, "msg_8_header_stamp_sec", 1, "int32_t");
msg_8_header_stamp->sec = msg_8_header_stamp_sec;

uint32_t msg_8_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&msg_8_header_stamp_nanosec, sizeof(uint32_t), fd, "msg_8_header_stamp_nanosec", 1, "uint32_t");
msg_8_header_stamp->nanosec = msg_8_header_stamp_nanosec;

msg_8_header->stamp = *msg_8_header_stamp;

uint8_t msg_8_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_8_header_frame_id_string_size, sizeof(uint8_t), fd, "msg_8_header_frame_id_string_size", 1, "uint8_t");
char msg_8_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_8_header_frame_id_string_data, MAX_LEN, fd, "msg_8_header_frame_id_string_data", 1, "char *");
std::string msg_8_header_frame_id(msg_8_header_frame_id_string_data, msg_8_header_frame_id_string_size);
msg_8_header->frame_id = msg_8_header_frame_id;

msg_8->header = *msg_8_header;

auto msg_8_orientation = std::make_shared<geometry_msgs::msg::Quaternion>();
double msg_8_orientation_x;
symros_make_symbolic_with_prefix(&msg_8_orientation_x, sizeof(double), fd, "msg_8_orientation_x", 1, "double");
msg_8_orientation->x = msg_8_orientation_x;

double msg_8_orientation_y;
symros_make_symbolic_with_prefix(&msg_8_orientation_y, sizeof(double), fd, "msg_8_orientation_y", 1, "double");
msg_8_orientation->y = msg_8_orientation_y;

double msg_8_orientation_z;
symros_make_symbolic_with_prefix(&msg_8_orientation_z, sizeof(double), fd, "msg_8_orientation_z", 1, "double");
msg_8_orientation->z = msg_8_orientation_z;

double msg_8_orientation_w;
symros_make_symbolic_with_prefix(&msg_8_orientation_w, sizeof(double), fd, "msg_8_orientation_w", 1, "double");
msg_8_orientation->w = msg_8_orientation_w;

msg_8->orientation = *msg_8_orientation;

double msg_8_orientation_covariance_0;
symros_make_symbolic_with_prefix(&msg_8_orientation_covariance_0, sizeof(double), fd, "msg_8_orientation_covariance_0", 1, "double");
double msg_8_orientation_covariance_1;
symros_make_symbolic_with_prefix(&msg_8_orientation_covariance_1, sizeof(double), fd, "msg_8_orientation_covariance_1", 1, "double");
double msg_8_orientation_covariance_2;
symros_make_symbolic_with_prefix(&msg_8_orientation_covariance_2, sizeof(double), fd, "msg_8_orientation_covariance_2", 1, "double");
double msg_8_orientation_covariance_3;
symros_make_symbolic_with_prefix(&msg_8_orientation_covariance_3, sizeof(double), fd, "msg_8_orientation_covariance_3", 1, "double");
double msg_8_orientation_covariance_4;
symros_make_symbolic_with_prefix(&msg_8_orientation_covariance_4, sizeof(double), fd, "msg_8_orientation_covariance_4", 1, "double");
double msg_8_orientation_covariance_5;
symros_make_symbolic_with_prefix(&msg_8_orientation_covariance_5, sizeof(double), fd, "msg_8_orientation_covariance_5", 1, "double");
double msg_8_orientation_covariance_6;
symros_make_symbolic_with_prefix(&msg_8_orientation_covariance_6, sizeof(double), fd, "msg_8_orientation_covariance_6", 1, "double");
double msg_8_orientation_covariance_7;
symros_make_symbolic_with_prefix(&msg_8_orientation_covariance_7, sizeof(double), fd, "msg_8_orientation_covariance_7", 1, "double");
double msg_8_orientation_covariance_8;
symros_make_symbolic_with_prefix(&msg_8_orientation_covariance_8, sizeof(double), fd, "msg_8_orientation_covariance_8", 1, "double");
msg_8->orientation_covariance[0] = msg_8_orientation_covariance_0;

msg_8->orientation_covariance[1] = msg_8_orientation_covariance_1;

msg_8->orientation_covariance[2] = msg_8_orientation_covariance_2;

msg_8->orientation_covariance[3] = msg_8_orientation_covariance_3;

msg_8->orientation_covariance[4] = msg_8_orientation_covariance_4;

msg_8->orientation_covariance[5] = msg_8_orientation_covariance_5;

msg_8->orientation_covariance[6] = msg_8_orientation_covariance_6;

msg_8->orientation_covariance[7] = msg_8_orientation_covariance_7;

msg_8->orientation_covariance[8] = msg_8_orientation_covariance_8;

auto msg_8_angular_velocity = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_8_angular_velocity_x;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_x, sizeof(double), fd, "msg_8_angular_velocity_x", 1, "double");
msg_8_angular_velocity->x = msg_8_angular_velocity_x;

double msg_8_angular_velocity_y;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_y, sizeof(double), fd, "msg_8_angular_velocity_y", 1, "double");
msg_8_angular_velocity->y = msg_8_angular_velocity_y;

double msg_8_angular_velocity_z;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_z, sizeof(double), fd, "msg_8_angular_velocity_z", 1, "double");
msg_8_angular_velocity->z = msg_8_angular_velocity_z;

msg_8->angular_velocity = *msg_8_angular_velocity;

double msg_8_angular_velocity_covariance_0;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_covariance_0, sizeof(double), fd, "msg_8_angular_velocity_covariance_0", 1, "double");
double msg_8_angular_velocity_covariance_1;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_covariance_1, sizeof(double), fd, "msg_8_angular_velocity_covariance_1", 1, "double");
double msg_8_angular_velocity_covariance_2;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_covariance_2, sizeof(double), fd, "msg_8_angular_velocity_covariance_2", 1, "double");
double msg_8_angular_velocity_covariance_3;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_covariance_3, sizeof(double), fd, "msg_8_angular_velocity_covariance_3", 1, "double");
double msg_8_angular_velocity_covariance_4;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_covariance_4, sizeof(double), fd, "msg_8_angular_velocity_covariance_4", 1, "double");
double msg_8_angular_velocity_covariance_5;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_covariance_5, sizeof(double), fd, "msg_8_angular_velocity_covariance_5", 1, "double");
double msg_8_angular_velocity_covariance_6;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_covariance_6, sizeof(double), fd, "msg_8_angular_velocity_covariance_6", 1, "double");
double msg_8_angular_velocity_covariance_7;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_covariance_7, sizeof(double), fd, "msg_8_angular_velocity_covariance_7", 1, "double");
double msg_8_angular_velocity_covariance_8;
symros_make_symbolic_with_prefix(&msg_8_angular_velocity_covariance_8, sizeof(double), fd, "msg_8_angular_velocity_covariance_8", 1, "double");
msg_8->angular_velocity_covariance[0] = msg_8_angular_velocity_covariance_0;

msg_8->angular_velocity_covariance[1] = msg_8_angular_velocity_covariance_1;

msg_8->angular_velocity_covariance[2] = msg_8_angular_velocity_covariance_2;

msg_8->angular_velocity_covariance[3] = msg_8_angular_velocity_covariance_3;

msg_8->angular_velocity_covariance[4] = msg_8_angular_velocity_covariance_4;

msg_8->angular_velocity_covariance[5] = msg_8_angular_velocity_covariance_5;

msg_8->angular_velocity_covariance[6] = msg_8_angular_velocity_covariance_6;

msg_8->angular_velocity_covariance[7] = msg_8_angular_velocity_covariance_7;

msg_8->angular_velocity_covariance[8] = msg_8_angular_velocity_covariance_8;

auto msg_8_linear_acceleration = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_8_linear_acceleration_x;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_x, sizeof(double), fd, "msg_8_linear_acceleration_x", 1, "double");
msg_8_linear_acceleration->x = msg_8_linear_acceleration_x;

double msg_8_linear_acceleration_y;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_y, sizeof(double), fd, "msg_8_linear_acceleration_y", 1, "double");
msg_8_linear_acceleration->y = msg_8_linear_acceleration_y;

double msg_8_linear_acceleration_z;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_z, sizeof(double), fd, "msg_8_linear_acceleration_z", 1, "double");
msg_8_linear_acceleration->z = msg_8_linear_acceleration_z;

msg_8->linear_acceleration = *msg_8_linear_acceleration;

double msg_8_linear_acceleration_covariance_0;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_covariance_0, sizeof(double), fd, "msg_8_linear_acceleration_covariance_0", 1, "double");
double msg_8_linear_acceleration_covariance_1;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_covariance_1, sizeof(double), fd, "msg_8_linear_acceleration_covariance_1", 1, "double");
double msg_8_linear_acceleration_covariance_2;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_covariance_2, sizeof(double), fd, "msg_8_linear_acceleration_covariance_2", 1, "double");
double msg_8_linear_acceleration_covariance_3;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_covariance_3, sizeof(double), fd, "msg_8_linear_acceleration_covariance_3", 1, "double");
double msg_8_linear_acceleration_covariance_4;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_covariance_4, sizeof(double), fd, "msg_8_linear_acceleration_covariance_4", 1, "double");
double msg_8_linear_acceleration_covariance_5;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_covariance_5, sizeof(double), fd, "msg_8_linear_acceleration_covariance_5", 1, "double");
double msg_8_linear_acceleration_covariance_6;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_covariance_6, sizeof(double), fd, "msg_8_linear_acceleration_covariance_6", 1, "double");
double msg_8_linear_acceleration_covariance_7;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_covariance_7, sizeof(double), fd, "msg_8_linear_acceleration_covariance_7", 1, "double");
double msg_8_linear_acceleration_covariance_8;
symros_make_symbolic_with_prefix(&msg_8_linear_acceleration_covariance_8, sizeof(double), fd, "msg_8_linear_acceleration_covariance_8", 1, "double");
msg_8->linear_acceleration_covariance[0] = msg_8_linear_acceleration_covariance_0;

msg_8->linear_acceleration_covariance[1] = msg_8_linear_acceleration_covariance_1;

msg_8->linear_acceleration_covariance[2] = msg_8_linear_acceleration_covariance_2;

msg_8->linear_acceleration_covariance[3] = msg_8_linear_acceleration_covariance_3;

msg_8->linear_acceleration_covariance[4] = msg_8_linear_acceleration_covariance_4;

msg_8->linear_acceleration_covariance[5] = msg_8_linear_acceleration_covariance_5;

msg_8->linear_acceleration_covariance[6] = msg_8_linear_acceleration_covariance_6;

msg_8->linear_acceleration_covariance[7] = msg_8_linear_acceleration_covariance_7;

msg_8->linear_acceleration_covariance[8] = msg_8_linear_acceleration_covariance_8;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




SYMROS__CALL_SUBSCRIPTION_CALLBACK(sub_8, msg_8, msg_8_info_);


bool call_option_4;

printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto req_9 = std::make_shared<robot_localization::srv::SetPose::Request>();
auto req_9_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
auto req_9_pose_header = std::make_shared<std_msgs::msg::Header>();
auto req_9_pose_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t req_9_pose_header_stamp_sec;
symros_make_symbolic_with_prefix(&req_9_pose_header_stamp_sec, sizeof(int32_t), fd, "req_9_pose_header_stamp_sec", 1, "int32_t");
req_9_pose_header_stamp->sec = req_9_pose_header_stamp_sec;

uint32_t req_9_pose_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&req_9_pose_header_stamp_nanosec, sizeof(uint32_t), fd, "req_9_pose_header_stamp_nanosec", 1, "uint32_t");
req_9_pose_header_stamp->nanosec = req_9_pose_header_stamp_nanosec;

req_9_pose_header->stamp = *req_9_pose_header_stamp;

uint8_t req_9_pose_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&req_9_pose_header_frame_id_string_size, sizeof(uint8_t), fd, "req_9_pose_header_frame_id_string_size", 1, "uint8_t");
char req_9_pose_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&req_9_pose_header_frame_id_string_data, MAX_LEN, fd, "req_9_pose_header_frame_id_string_data", 1, "char *");
std::string req_9_pose_header_frame_id(req_9_pose_header_frame_id_string_data, req_9_pose_header_frame_id_string_size);
req_9_pose_header->frame_id = req_9_pose_header_frame_id;

req_9_pose->header = *req_9_pose_header;

auto req_9_pose_pose = std::make_shared<geometry_msgs::msg::PoseWithCovariance>();
auto req_9_pose_pose_pose = std::make_shared<geometry_msgs::msg::Pose>();
auto req_9_pose_pose_pose_position = std::make_shared<geometry_msgs::msg::Point>();
double req_9_pose_pose_pose_position_x;
symros_make_symbolic_with_prefix(&req_9_pose_pose_pose_position_x, sizeof(double), fd, "req_9_pose_pose_pose_position_x", 1, "double");
req_9_pose_pose_pose_position->x = req_9_pose_pose_pose_position_x;

double req_9_pose_pose_pose_position_y;
symros_make_symbolic_with_prefix(&req_9_pose_pose_pose_position_y, sizeof(double), fd, "req_9_pose_pose_pose_position_y", 1, "double");
req_9_pose_pose_pose_position->y = req_9_pose_pose_pose_position_y;

double req_9_pose_pose_pose_position_z;
symros_make_symbolic_with_prefix(&req_9_pose_pose_pose_position_z, sizeof(double), fd, "req_9_pose_pose_pose_position_z", 1, "double");
req_9_pose_pose_pose_position->z = req_9_pose_pose_pose_position_z;

req_9_pose_pose_pose->position = *req_9_pose_pose_pose_position;

auto req_9_pose_pose_pose_orientation = std::make_shared<geometry_msgs::msg::Quaternion>();
double req_9_pose_pose_pose_orientation_x;
symros_make_symbolic_with_prefix(&req_9_pose_pose_pose_orientation_x, sizeof(double), fd, "req_9_pose_pose_pose_orientation_x", 1, "double");
req_9_pose_pose_pose_orientation->x = req_9_pose_pose_pose_orientation_x;

double req_9_pose_pose_pose_orientation_y;
symros_make_symbolic_with_prefix(&req_9_pose_pose_pose_orientation_y, sizeof(double), fd, "req_9_pose_pose_pose_orientation_y", 1, "double");
req_9_pose_pose_pose_orientation->y = req_9_pose_pose_pose_orientation_y;

double req_9_pose_pose_pose_orientation_z;
symros_make_symbolic_with_prefix(&req_9_pose_pose_pose_orientation_z, sizeof(double), fd, "req_9_pose_pose_pose_orientation_z", 1, "double");
req_9_pose_pose_pose_orientation->z = req_9_pose_pose_pose_orientation_z;

double req_9_pose_pose_pose_orientation_w;
symros_make_symbolic_with_prefix(&req_9_pose_pose_pose_orientation_w, sizeof(double), fd, "req_9_pose_pose_pose_orientation_w", 1, "double");
req_9_pose_pose_pose_orientation->w = req_9_pose_pose_pose_orientation_w;

req_9_pose_pose_pose->orientation = *req_9_pose_pose_pose_orientation;

req_9_pose_pose->pose = *req_9_pose_pose_pose;

double req_9_pose_pose_covariance_0;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_0, sizeof(double), fd, "req_9_pose_pose_covariance_0", 1, "double");
double req_9_pose_pose_covariance_1;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_1, sizeof(double), fd, "req_9_pose_pose_covariance_1", 1, "double");
double req_9_pose_pose_covariance_2;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_2, sizeof(double), fd, "req_9_pose_pose_covariance_2", 1, "double");
double req_9_pose_pose_covariance_3;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_3, sizeof(double), fd, "req_9_pose_pose_covariance_3", 1, "double");
double req_9_pose_pose_covariance_4;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_4, sizeof(double), fd, "req_9_pose_pose_covariance_4", 1, "double");
double req_9_pose_pose_covariance_5;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_5, sizeof(double), fd, "req_9_pose_pose_covariance_5", 1, "double");
double req_9_pose_pose_covariance_6;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_6, sizeof(double), fd, "req_9_pose_pose_covariance_6", 1, "double");
double req_9_pose_pose_covariance_7;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_7, sizeof(double), fd, "req_9_pose_pose_covariance_7", 1, "double");
double req_9_pose_pose_covariance_8;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_8, sizeof(double), fd, "req_9_pose_pose_covariance_8", 1, "double");
double req_9_pose_pose_covariance_9;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_9, sizeof(double), fd, "req_9_pose_pose_covariance_9", 1, "double");
double req_9_pose_pose_covariance_10;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_10, sizeof(double), fd, "req_9_pose_pose_covariance_10", 1, "double");
double req_9_pose_pose_covariance_11;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_11, sizeof(double), fd, "req_9_pose_pose_covariance_11", 1, "double");
double req_9_pose_pose_covariance_12;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_12, sizeof(double), fd, "req_9_pose_pose_covariance_12", 1, "double");
double req_9_pose_pose_covariance_13;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_13, sizeof(double), fd, "req_9_pose_pose_covariance_13", 1, "double");
double req_9_pose_pose_covariance_14;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_14, sizeof(double), fd, "req_9_pose_pose_covariance_14", 1, "double");
double req_9_pose_pose_covariance_15;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_15, sizeof(double), fd, "req_9_pose_pose_covariance_15", 1, "double");
double req_9_pose_pose_covariance_16;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_16, sizeof(double), fd, "req_9_pose_pose_covariance_16", 1, "double");
double req_9_pose_pose_covariance_17;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_17, sizeof(double), fd, "req_9_pose_pose_covariance_17", 1, "double");
double req_9_pose_pose_covariance_18;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_18, sizeof(double), fd, "req_9_pose_pose_covariance_18", 1, "double");
double req_9_pose_pose_covariance_19;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_19, sizeof(double), fd, "req_9_pose_pose_covariance_19", 1, "double");
double req_9_pose_pose_covariance_20;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_20, sizeof(double), fd, "req_9_pose_pose_covariance_20", 1, "double");
double req_9_pose_pose_covariance_21;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_21, sizeof(double), fd, "req_9_pose_pose_covariance_21", 1, "double");
double req_9_pose_pose_covariance_22;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_22, sizeof(double), fd, "req_9_pose_pose_covariance_22", 1, "double");
double req_9_pose_pose_covariance_23;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_23, sizeof(double), fd, "req_9_pose_pose_covariance_23", 1, "double");
double req_9_pose_pose_covariance_24;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_24, sizeof(double), fd, "req_9_pose_pose_covariance_24", 1, "double");
double req_9_pose_pose_covariance_25;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_25, sizeof(double), fd, "req_9_pose_pose_covariance_25", 1, "double");
double req_9_pose_pose_covariance_26;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_26, sizeof(double), fd, "req_9_pose_pose_covariance_26", 1, "double");
double req_9_pose_pose_covariance_27;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_27, sizeof(double), fd, "req_9_pose_pose_covariance_27", 1, "double");
double req_9_pose_pose_covariance_28;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_28, sizeof(double), fd, "req_9_pose_pose_covariance_28", 1, "double");
double req_9_pose_pose_covariance_29;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_29, sizeof(double), fd, "req_9_pose_pose_covariance_29", 1, "double");
double req_9_pose_pose_covariance_30;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_30, sizeof(double), fd, "req_9_pose_pose_covariance_30", 1, "double");
double req_9_pose_pose_covariance_31;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_31, sizeof(double), fd, "req_9_pose_pose_covariance_31", 1, "double");
double req_9_pose_pose_covariance_32;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_32, sizeof(double), fd, "req_9_pose_pose_covariance_32", 1, "double");
double req_9_pose_pose_covariance_33;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_33, sizeof(double), fd, "req_9_pose_pose_covariance_33", 1, "double");
double req_9_pose_pose_covariance_34;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_34, sizeof(double), fd, "req_9_pose_pose_covariance_34", 1, "double");
double req_9_pose_pose_covariance_35;
symros_make_symbolic_with_prefix(&req_9_pose_pose_covariance_35, sizeof(double), fd, "req_9_pose_pose_covariance_35", 1, "double");
req_9_pose_pose->covariance[0] = req_9_pose_pose_covariance_0;

req_9_pose_pose->covariance[1] = req_9_pose_pose_covariance_1;

req_9_pose_pose->covariance[2] = req_9_pose_pose_covariance_2;

req_9_pose_pose->covariance[3] = req_9_pose_pose_covariance_3;

req_9_pose_pose->covariance[4] = req_9_pose_pose_covariance_4;

req_9_pose_pose->covariance[5] = req_9_pose_pose_covariance_5;

req_9_pose_pose->covariance[6] = req_9_pose_pose_covariance_6;

req_9_pose_pose->covariance[7] = req_9_pose_pose_covariance_7;

req_9_pose_pose->covariance[8] = req_9_pose_pose_covariance_8;

req_9_pose_pose->covariance[9] = req_9_pose_pose_covariance_9;

req_9_pose_pose->covariance[10] = req_9_pose_pose_covariance_10;

req_9_pose_pose->covariance[11] = req_9_pose_pose_covariance_11;

req_9_pose_pose->covariance[12] = req_9_pose_pose_covariance_12;

req_9_pose_pose->covariance[13] = req_9_pose_pose_covariance_13;

req_9_pose_pose->covariance[14] = req_9_pose_pose_covariance_14;

req_9_pose_pose->covariance[15] = req_9_pose_pose_covariance_15;

req_9_pose_pose->covariance[16] = req_9_pose_pose_covariance_16;

req_9_pose_pose->covariance[17] = req_9_pose_pose_covariance_17;

req_9_pose_pose->covariance[18] = req_9_pose_pose_covariance_18;

req_9_pose_pose->covariance[19] = req_9_pose_pose_covariance_19;

req_9_pose_pose->covariance[20] = req_9_pose_pose_covariance_20;

req_9_pose_pose->covariance[21] = req_9_pose_pose_covariance_21;

req_9_pose_pose->covariance[22] = req_9_pose_pose_covariance_22;

req_9_pose_pose->covariance[23] = req_9_pose_pose_covariance_23;

req_9_pose_pose->covariance[24] = req_9_pose_pose_covariance_24;

req_9_pose_pose->covariance[25] = req_9_pose_pose_covariance_25;

req_9_pose_pose->covariance[26] = req_9_pose_pose_covariance_26;

req_9_pose_pose->covariance[27] = req_9_pose_pose_covariance_27;

req_9_pose_pose->covariance[28] = req_9_pose_pose_covariance_28;

req_9_pose_pose->covariance[29] = req_9_pose_pose_covariance_29;

req_9_pose_pose->covariance[30] = req_9_pose_pose_covariance_30;

req_9_pose_pose->covariance[31] = req_9_pose_pose_covariance_31;

req_9_pose_pose->covariance[32] = req_9_pose_pose_covariance_32;

req_9_pose_pose->covariance[33] = req_9_pose_pose_covariance_33;

req_9_pose_pose->covariance[34] = req_9_pose_pose_covariance_34;

req_9_pose_pose->covariance[35] = req_9_pose_pose_covariance_35;

req_9_pose->pose = *req_9_pose_pose;

req_9->pose = *req_9_pose;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




symros_make_symbolic_with_prefix(&call_option_4, sizeof(bool), fd, "call_option_4", 2, "bool");
if (call_option_4) {

SYMROS__CALL_SERVICE_CALLBACK(srv_9, req_9);
}



printf("[symros] callback test finish\n");

close(fd);

// SYMROS executor

  exit(0);
}