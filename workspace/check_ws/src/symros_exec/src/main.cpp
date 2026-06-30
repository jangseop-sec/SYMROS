#include <memory>
#include <cstring>
#include <cmath>

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
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_behaviors/behavior_server.hpp"
#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_behaviors/plugins/wait.hpp"
#include "nav2_behaviors/plugins/back_up.hpp"
#include "nav2_behaviors/plugins/assisted_teleop.hpp"
#include "nav2_behaviors/plugins/spin.hpp"
// SYMROS target include headers

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_action/rcl_action.h"
#include "rmw/types.h"

#include "symros/symros.hpp"
#include "symros_string.hpp"
#include "symros_math.hpp"


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
  symcc_reset_constraints();   \
  subscription_name_->get_any_callback().dispatch(message_name_, message_info_);

#define SYMROS__DECLARE_SERVICE(service_namespace_, service_name_, index_) \
  auto service_name_ = SymROSManager::get_instance().get_service<service_namespace_>(index_);

#define SYMROS__CALL_SERVICE_CALLBACK(service_, request_ptr_)                             \
  do {                                                                                    \
    auto __req_info = std::make_shared<rmw_request_id_t>();                               \
    symcc_reset_constraints();                                                            \
    (service_)->get_any_callback().dispatch((service_), __req_info, (request_ptr_));      \
  } while (0)

#define SYMROS__DECLARE_ACTION_SERVER(service_namespace_, service_name_, index_) \
  auto service_name_ = SymROSManager::get_instance().get_action_server(index_);

template<typename ActionT>
void SYMROS_CallActionServerCallback(
    std::shared_ptr<rclcpp_action::ServerBase> action_server,
    std::shared_ptr<typename ActionT::Goal> goal_ptr)
{
    using RequestT = typename ActionT::Impl::SendGoalService::Request;

    rclcpp_action::GoalUUID uuid{};
    for (size_t i = 0; i < uuid.size(); i++) {
        uuid[i] = static_cast<uint8_t>(i + 1);
    }

    auto request = std::make_shared<RequestT>();
    request->goal = *goal_ptr;
    request->goal_id.uuid = uuid;

    std::shared_ptr<void> msg = request;

    rcl_action_goal_info_t goal_info{};
    for (size_t i = 0; i < uuid.size(); i++) {
        goal_info.goal_id.uuid[i] = uuid[i];
    }

    rmw_request_id_t header{};
    memset(&header, 0, sizeof(header));

    auto gdata = std::make_tuple(
        RCL_RET_OK,
        goal_info,
        header,
        msg
    );

    // 6. ServerBaseData wrapping
    auto data = std::make_shared<rclcpp_action::ServerBaseData>(
        std::move(gdata)
    );

    action_server->execute(reinterpret_cast<std::shared_ptr<void>&>(data));
}

extern "C" void symcc_make_symbolic_with_prefix(const void *start, size_t byte_length, const char * prefix);
extern "C" void symcc_reset_constraints();

bool STATE_MODE = false;

void symros_print_field(const std::string& name, const void* data, const std::string& type, size_t size) {}

void symros_make_symbolic_with_prefix(void * data, size_t size, int fd, std::string name, int is_target, std::string type) {

  int n = read(fd, data, size);
  if (n == -1) {
    perror("read failed");
    exit(1);
  }

  // STATE_MODE -> symbolize state + pre callback's message
  // !STATE_MODE -> symbolize target message
  if ((!STATE_MODE && is_target != 1) || (STATE_MODE && is_target != 2)) {
    void * tmp = malloc(size);
    symcc_make_symbolic_with_prefix(tmp, size, "tmp");
    free(tmp);
  } else {
    symcc_make_symbolic_with_prefix(data, size, name.c_str());
  }
  
  symros_print_field(name, data, type, size);
}

// SYMROS setup
  // SYMROS setup

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // SYMROS executor
int fd = open(argv[1], O_RDONLY);

STATE_MODE = argv[2][0] == '1';


SYMROS__INIT(false);


printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
bool param__use_sim_time;
symros_make_symbolic_with_prefix(&param__use_sim_time, sizeof(bool), fd, "param__use_sim_time", 2, "bool");

bool param___bond_disable_heartbeat_timeout;
symros_make_symbolic_with_prefix(&param___bond_disable_heartbeat_timeout, sizeof(bool), fd, "param___bond_disable_heartbeat_timeout", 2, "bool");

uint8_t param__costmap_topic_string_size;
symros_make_symbolic_with_prefix(&param__costmap_topic_string_size, sizeof(uint8_t), fd, "param__costmap_topic_string_size", 2, "uint8_t");
char param__costmap_topic_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__costmap_topic_string_data, MAX_LEN, fd, "param__costmap_topic_string_data", 2, "char *");
std::string param__costmap_topic(param__costmap_topic_string_data, param__costmap_topic_string_size);

uint8_t param__footprint_topic_string_size;
symros_make_symbolic_with_prefix(&param__footprint_topic_string_size, sizeof(uint8_t), fd, "param__footprint_topic_string_size", 2, "uint8_t");
char param__footprint_topic_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__footprint_topic_string_data, MAX_LEN, fd, "param__footprint_topic_string_data", 2, "char *");
std::string param__footprint_topic(param__footprint_topic_string_data, param__footprint_topic_string_size);

double param__cycle_frequency;
symros_make_symbolic_with_prefix(&param__cycle_frequency, sizeof(double), fd, "param__cycle_frequency", 2, "double");

std::vector<std::string> param__behavior_plugins;
uint8_t param__behavior_plugins_0_string_size;
symros_make_symbolic_with_prefix(&param__behavior_plugins_0_string_size, sizeof(uint8_t), fd, "param__behavior_plugins_0_string_size", 2, "uint8_t");
char param__behavior_plugins_0_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__behavior_plugins_0_string_data, MAX_LEN, fd, "param__behavior_plugins_0_string_data", 2, "char *");
param__behavior_plugins.push_back(std::string(param__behavior_plugins_0_string_data, param__behavior_plugins_0_string_size));

uint8_t param__behavior_plugins_1_string_size;
symros_make_symbolic_with_prefix(&param__behavior_plugins_1_string_size, sizeof(uint8_t), fd, "param__behavior_plugins_1_string_size", 2, "uint8_t");
char param__behavior_plugins_1_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__behavior_plugins_1_string_data, MAX_LEN, fd, "param__behavior_plugins_1_string_data", 2, "char *");
param__behavior_plugins.push_back(std::string(param__behavior_plugins_1_string_data, param__behavior_plugins_1_string_size));

uint8_t param__behavior_plugins_2_string_size;
symros_make_symbolic_with_prefix(&param__behavior_plugins_2_string_size, sizeof(uint8_t), fd, "param__behavior_plugins_2_string_size", 2, "uint8_t");
char param__behavior_plugins_2_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__behavior_plugins_2_string_data, MAX_LEN, fd, "param__behavior_plugins_2_string_data", 2, "char *");
param__behavior_plugins.push_back(std::string(param__behavior_plugins_2_string_data, param__behavior_plugins_2_string_size));

uint8_t param__behavior_plugins_3_string_size;
symros_make_symbolic_with_prefix(&param__behavior_plugins_3_string_size, sizeof(uint8_t), fd, "param__behavior_plugins_3_string_size", 2, "uint8_t");
char param__behavior_plugins_3_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__behavior_plugins_3_string_data, MAX_LEN, fd, "param__behavior_plugins_3_string_data", 2, "char *");
param__behavior_plugins.push_back(std::string(param__behavior_plugins_3_string_data, param__behavior_plugins_3_string_size));

uint8_t param__spin__plugin_string_size;
symros_make_symbolic_with_prefix(&param__spin__plugin_string_size, sizeof(uint8_t), fd, "param__spin__plugin_string_size", 2, "uint8_t");
char param__spin__plugin_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__spin__plugin_string_data, MAX_LEN, fd, "param__spin__plugin_string_data", 2, "char *");
std::string param__spin__plugin(param__spin__plugin_string_data, param__spin__plugin_string_size);

uint8_t param__backup__plugin_string_size;
symros_make_symbolic_with_prefix(&param__backup__plugin_string_size, sizeof(uint8_t), fd, "param__backup__plugin_string_size", 2, "uint8_t");
char param__backup__plugin_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__backup__plugin_string_data, MAX_LEN, fd, "param__backup__plugin_string_data", 2, "char *");
std::string param__backup__plugin(param__backup__plugin_string_data, param__backup__plugin_string_size);

uint8_t param__drive_on_heading__plugin_string_size;
symros_make_symbolic_with_prefix(&param__drive_on_heading__plugin_string_size, sizeof(uint8_t), fd, "param__drive_on_heading__plugin_string_size", 2, "uint8_t");
char param__drive_on_heading__plugin_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__drive_on_heading__plugin_string_data, MAX_LEN, fd, "param__drive_on_heading__plugin_string_data", 2, "char *");
std::string param__drive_on_heading__plugin(param__drive_on_heading__plugin_string_data, param__drive_on_heading__plugin_string_size);

uint8_t param__wait__plugin_string_size;
symros_make_symbolic_with_prefix(&param__wait__plugin_string_size, sizeof(uint8_t), fd, "param__wait__plugin_string_size", 2, "uint8_t");
char param__wait__plugin_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__wait__plugin_string_data, MAX_LEN, fd, "param__wait__plugin_string_data", 2, "char *");
std::string param__wait__plugin(param__wait__plugin_string_data, param__wait__plugin_string_size);

uint8_t param__global_frame_string_size;
symros_make_symbolic_with_prefix(&param__global_frame_string_size, sizeof(uint8_t), fd, "param__global_frame_string_size", 2, "uint8_t");
char param__global_frame_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__global_frame_string_data, MAX_LEN, fd, "param__global_frame_string_data", 2, "char *");
std::string param__global_frame(param__global_frame_string_data, param__global_frame_string_size);

uint8_t param__robot_base_frame_string_size;
symros_make_symbolic_with_prefix(&param__robot_base_frame_string_size, sizeof(uint8_t), fd, "param__robot_base_frame_string_size", 2, "uint8_t");
char param__robot_base_frame_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&param__robot_base_frame_string_data, MAX_LEN, fd, "param__robot_base_frame_string_data", 2, "char *");
std::string param__robot_base_frame(param__robot_base_frame_string_data, param__robot_base_frame_string_size);

double param__transform_tolerance;
symros_make_symbolic_with_prefix(&param__transform_tolerance, sizeof(double), fd, "param__transform_tolerance", 2, "double");

double param__simulate_ahead_time;
symros_make_symbolic_with_prefix(&param__simulate_ahead_time, sizeof(double), fd, "param__simulate_ahead_time", 2, "double");

double param__max_rotational_vel;
symros_make_symbolic_with_prefix(&param__max_rotational_vel, sizeof(double), fd, "param__max_rotational_vel", 2, "double");

double param__min_rotational_vel;
symros_make_symbolic_with_prefix(&param__min_rotational_vel, sizeof(double), fd, "param__min_rotational_vel", 2, "double");

double param__rotational_acc_lim;
symros_make_symbolic_with_prefix(&param__rotational_acc_lim, sizeof(double), fd, "param__rotational_acc_lim", 2, "double");

double param__backup__acceleration_limit;
symros_make_symbolic_with_prefix(&param__backup__acceleration_limit, sizeof(double), fd, "param__backup__acceleration_limit", 2, "double");

double param__backup__deceleration_limit;
symros_make_symbolic_with_prefix(&param__backup__deceleration_limit, sizeof(double), fd, "param__backup__deceleration_limit", 2, "double");

double param__backup__minimum_speed;
symros_make_symbolic_with_prefix(&param__backup__minimum_speed, sizeof(double), fd, "param__backup__minimum_speed", 2, "double");

double param__drive_on_heading__acceleration_limit;
symros_make_symbolic_with_prefix(&param__drive_on_heading__acceleration_limit, sizeof(double), fd, "param__drive_on_heading__acceleration_limit", 2, "double");

double param__drive_on_heading__deceleration_limit;
symros_make_symbolic_with_prefix(&param__drive_on_heading__deceleration_limit, sizeof(double), fd, "param__drive_on_heading__deceleration_limit", 2, "double");

double param__drive_on_heading__minimum_speed;
symros_make_symbolic_with_prefix(&param__drive_on_heading__minimum_speed, sizeof(double), fd, "param__drive_on_heading__minimum_speed", 2, "double");



symros::SymROSManager::get_instance().symros_add_param("use_sim_time", param__use_sim_time);
symros::SymROSManager::get_instance().symros_add_param("/bond_disable_heartbeat_timeout", param___bond_disable_heartbeat_timeout);
symros::SymROSManager::get_instance().symros_add_param("costmap_topic", param__costmap_topic);
symros::SymROSManager::get_instance().symros_add_param("footprint_topic", param__footprint_topic);
symros::SymROSManager::get_instance().symros_add_param("cycle_frequency", param__cycle_frequency);
symros::SymROSManager::get_instance().symros_add_param("behavior_plugins", param__behavior_plugins);
symros::SymROSManager::get_instance().symros_add_param("spin.plugin", param__spin__plugin);
symros::SymROSManager::get_instance().symros_add_param("backup.plugin", param__backup__plugin);
symros::SymROSManager::get_instance().symros_add_param("drive_on_heading.plugin", param__drive_on_heading__plugin);
symros::SymROSManager::get_instance().symros_add_param("wait.plugin", param__wait__plugin);
symros::SymROSManager::get_instance().symros_add_param("global_frame", param__global_frame);
symros::SymROSManager::get_instance().symros_add_param("robot_base_frame", param__robot_base_frame);
symros::SymROSManager::get_instance().symros_add_param("transform_tolerance", param__transform_tolerance);
symros::SymROSManager::get_instance().symros_add_param("simulate_ahead_time", param__simulate_ahead_time);
symros::SymROSManager::get_instance().symros_add_param("max_rotational_vel", param__max_rotational_vel);
symros::SymROSManager::get_instance().symros_add_param("min_rotational_vel", param__min_rotational_vel);
symros::SymROSManager::get_instance().symros_add_param("rotational_acc_lim", param__rotational_acc_lim);
symros::SymROSManager::get_instance().symros_add_param("backup.acceleration_limit", param__backup__acceleration_limit);
symros::SymROSManager::get_instance().symros_add_param("backup.deceleration_limit", param__backup__deceleration_limit);
symros::SymROSManager::get_instance().symros_add_param("backup.minimum_speed", param__backup__minimum_speed);
symros::SymROSManager::get_instance().symros_add_param("drive_on_heading.acceleration_limit", param__drive_on_heading__acceleration_limit);
symros::SymROSManager::get_instance().symros_add_param("drive_on_heading.deceleration_limit", param__drive_on_heading__deceleration_limit);
symros::SymROSManager::get_instance().symros_add_param("drive_on_heading.minimum_speed", param__drive_on_heading__minimum_speed);



printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);

SYMROS__INIT(true);
rclcpp::NodeOptions options;


auto node = std::make_shared<behavior_server::BehaviorServer>(options);

// lifecycle node transition: configuration
node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

SYMROS__INIT(true);

SYMROS__DECLARE_SUBSCRIPTION(tf2_msgs::msg::TFMessage, sub_0, 3);
SYMROS__DECLARE_SUBSCRIPTION(nav2_msgs::msg::Costmap, sub_2, 4);
SYMROS__DECLARE_SUBSCRIPTION(geometry_msgs::msg::PolygonStamped, sub_3, 5);
SYMROS__DECLARE_ACTION_SERVER(nav2_msgs::action::DriveOnHeading, act_srv_4, 2);
SYMROS__DECLARE_ACTION_SERVER(nav2_msgs::action::BackUp, act_srv_5, 1);
SYMROS__DECLARE_ACTION_SERVER(nav2_msgs::action::Spin, act_srv_6, 0);
SYMROS__DECLARE_ACTION_SERVER(nav2_msgs::action::Wait, act_srv_7, 3);

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

auto msg_0_transforms_1 = std::make_shared<geometry_msgs::msg::TransformStamped>();
auto msg_0_transforms_1_header = std::make_shared<std_msgs::msg::Header>();
auto msg_0_transforms_1_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_0_transforms_1_header_stamp_sec;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_header_stamp_sec, sizeof(int32_t), fd, "msg_0_transforms_1_header_stamp_sec", 0, "int32_t");
msg_0_transforms_1_header_stamp->sec = msg_0_transforms_1_header_stamp_sec;

uint32_t msg_0_transforms_1_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_header_stamp_nanosec, sizeof(uint32_t), fd, "msg_0_transforms_1_header_stamp_nanosec", 0, "uint32_t");
msg_0_transforms_1_header_stamp->nanosec = msg_0_transforms_1_header_stamp_nanosec;

msg_0_transforms_1_header->stamp = *msg_0_transforms_1_header_stamp;

uint8_t msg_0_transforms_1_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_header_frame_id_string_size, sizeof(uint8_t), fd, "msg_0_transforms_1_header_frame_id_string_size", 0, "uint8_t");
char msg_0_transforms_1_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_0_transforms_1_header_frame_id_string_data, MAX_LEN, fd, "msg_0_transforms_1_header_frame_id_string_data", 0, "char *");
std::string msg_0_transforms_1_header_frame_id(msg_0_transforms_1_header_frame_id_string_data, msg_0_transforms_1_header_frame_id_string_size);
msg_0_transforms_1_header->frame_id = msg_0_transforms_1_header_frame_id;

msg_0_transforms_1->header = *msg_0_transforms_1_header;

uint8_t msg_0_transforms_1_child_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_child_frame_id_string_size, sizeof(uint8_t), fd, "msg_0_transforms_1_child_frame_id_string_size", 0, "uint8_t");
char msg_0_transforms_1_child_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_0_transforms_1_child_frame_id_string_data, MAX_LEN, fd, "msg_0_transforms_1_child_frame_id_string_data", 0, "char *");
std::string msg_0_transforms_1_child_frame_id(msg_0_transforms_1_child_frame_id_string_data, msg_0_transforms_1_child_frame_id_string_size);
msg_0_transforms_1->child_frame_id = msg_0_transforms_1_child_frame_id;

auto msg_0_transforms_1_transform = std::make_shared<geometry_msgs::msg::Transform>();
auto msg_0_transforms_1_transform_translation = std::make_shared<geometry_msgs::msg::Vector3>();
double msg_0_transforms_1_transform_translation_x;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_transform_translation_x, sizeof(double), fd, "msg_0_transforms_1_transform_translation_x", 0, "double");
msg_0_transforms_1_transform_translation->x = msg_0_transforms_1_transform_translation_x;

double msg_0_transforms_1_transform_translation_y;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_transform_translation_y, sizeof(double), fd, "msg_0_transforms_1_transform_translation_y", 0, "double");
msg_0_transforms_1_transform_translation->y = msg_0_transforms_1_transform_translation_y;

double msg_0_transforms_1_transform_translation_z;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_transform_translation_z, sizeof(double), fd, "msg_0_transforms_1_transform_translation_z", 0, "double");
msg_0_transforms_1_transform_translation->z = msg_0_transforms_1_transform_translation_z;

msg_0_transforms_1_transform->translation = *msg_0_transforms_1_transform_translation;

auto msg_0_transforms_1_transform_rotation = std::make_shared<geometry_msgs::msg::Quaternion>();
double msg_0_transforms_1_transform_rotation_x;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_transform_rotation_x, sizeof(double), fd, "msg_0_transforms_1_transform_rotation_x", 0, "double");
msg_0_transforms_1_transform_rotation->x = msg_0_transforms_1_transform_rotation_x;

double msg_0_transforms_1_transform_rotation_y;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_transform_rotation_y, sizeof(double), fd, "msg_0_transforms_1_transform_rotation_y", 0, "double");
msg_0_transforms_1_transform_rotation->y = msg_0_transforms_1_transform_rotation_y;

double msg_0_transforms_1_transform_rotation_z;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_transform_rotation_z, sizeof(double), fd, "msg_0_transforms_1_transform_rotation_z", 0, "double");
msg_0_transforms_1_transform_rotation->z = msg_0_transforms_1_transform_rotation_z;

double msg_0_transforms_1_transform_rotation_w;
symros_make_symbolic_with_prefix(&msg_0_transforms_1_transform_rotation_w, sizeof(double), fd, "msg_0_transforms_1_transform_rotation_w", 0, "double");
msg_0_transforms_1_transform_rotation->w = msg_0_transforms_1_transform_rotation_w;

msg_0_transforms_1_transform->rotation = *msg_0_transforms_1_transform_rotation;

msg_0_transforms_1->transform = *msg_0_transforms_1_transform;

bool empty_msg_0_transforms;
symros_make_symbolic_with_prefix(&empty_msg_0_transforms, sizeof(bool), fd, "empty_msg_0_transforms", 2, "bool");

if (!empty_msg_0_transforms) {

msg_0->transforms.push_back(*msg_0_transforms_0);

msg_0->transforms.push_back(*msg_0_transforms_1);


}


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);



if (empty_msg_0_transforms) {}


SYMROS__CALL_SUBSCRIPTION_CALLBACK(sub_0, msg_0, msg_0_info_);



printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);

printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);



if (empty_msg_0_transforms) {}


node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);




printf("\n===== START CALLBACK TEST =====\n");fflush(stdout);


printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto msg_2 = std::make_shared<nav2_msgs::msg::Costmap>();
auto msg_2_header = std::make_shared<std_msgs::msg::Header>();
auto msg_2_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_2_header_stamp_sec;
symros_make_symbolic_with_prefix(&msg_2_header_stamp_sec, sizeof(int32_t), fd, "msg_2_header_stamp_sec", 1, "int32_t");
msg_2_header_stamp->sec = msg_2_header_stamp_sec;

uint32_t msg_2_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&msg_2_header_stamp_nanosec, sizeof(uint32_t), fd, "msg_2_header_stamp_nanosec", 1, "uint32_t");
msg_2_header_stamp->nanosec = msg_2_header_stamp_nanosec;

msg_2_header->stamp = *msg_2_header_stamp;

uint8_t msg_2_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_2_header_frame_id_string_size, sizeof(uint8_t), fd, "msg_2_header_frame_id_string_size", 1, "uint8_t");
char msg_2_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_2_header_frame_id_string_data, MAX_LEN, fd, "msg_2_header_frame_id_string_data", 1, "char *");
std::string msg_2_header_frame_id(msg_2_header_frame_id_string_data, msg_2_header_frame_id_string_size);
msg_2_header->frame_id = msg_2_header_frame_id;

msg_2->header = *msg_2_header;

auto msg_2_metadata = std::make_shared<nav2_msgs::msg::CostmapMetaData>();
auto msg_2_metadata_map_load_time = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_2_metadata_map_load_time_sec;
symros_make_symbolic_with_prefix(&msg_2_metadata_map_load_time_sec, sizeof(int32_t), fd, "msg_2_metadata_map_load_time_sec", 1, "int32_t");
msg_2_metadata_map_load_time->sec = msg_2_metadata_map_load_time_sec;

uint32_t msg_2_metadata_map_load_time_nanosec;
symros_make_symbolic_with_prefix(&msg_2_metadata_map_load_time_nanosec, sizeof(uint32_t), fd, "msg_2_metadata_map_load_time_nanosec", 1, "uint32_t");
msg_2_metadata_map_load_time->nanosec = msg_2_metadata_map_load_time_nanosec;

msg_2_metadata->map_load_time = *msg_2_metadata_map_load_time;

auto msg_2_metadata_update_time = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_2_metadata_update_time_sec;
symros_make_symbolic_with_prefix(&msg_2_metadata_update_time_sec, sizeof(int32_t), fd, "msg_2_metadata_update_time_sec", 1, "int32_t");
msg_2_metadata_update_time->sec = msg_2_metadata_update_time_sec;

uint32_t msg_2_metadata_update_time_nanosec;
symros_make_symbolic_with_prefix(&msg_2_metadata_update_time_nanosec, sizeof(uint32_t), fd, "msg_2_metadata_update_time_nanosec", 1, "uint32_t");
msg_2_metadata_update_time->nanosec = msg_2_metadata_update_time_nanosec;

msg_2_metadata->update_time = *msg_2_metadata_update_time;

uint8_t msg_2_metadata_layer_string_size;
symros_make_symbolic_with_prefix(&msg_2_metadata_layer_string_size, sizeof(uint8_t), fd, "msg_2_metadata_layer_string_size", 1, "uint8_t");
char msg_2_metadata_layer_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_2_metadata_layer_string_data, MAX_LEN, fd, "msg_2_metadata_layer_string_data", 1, "char *");
std::string msg_2_metadata_layer(msg_2_metadata_layer_string_data, msg_2_metadata_layer_string_size);
msg_2_metadata->layer = msg_2_metadata_layer;

float msg_2_metadata_resolution;
symros_make_symbolic_with_prefix(&msg_2_metadata_resolution, sizeof(float), fd, "msg_2_metadata_resolution", 1, "float");
msg_2_metadata->resolution = msg_2_metadata_resolution;

uint32_t msg_2_metadata_size_x;
symros_make_symbolic_with_prefix(&msg_2_metadata_size_x, sizeof(uint32_t), fd, "msg_2_metadata_size_x", 1, "uint32_t");
msg_2_metadata->size_x = msg_2_metadata_size_x;

uint32_t msg_2_metadata_size_y;
symros_make_symbolic_with_prefix(&msg_2_metadata_size_y, sizeof(uint32_t), fd, "msg_2_metadata_size_y", 1, "uint32_t");
msg_2_metadata->size_y = msg_2_metadata_size_y;

auto msg_2_metadata_origin = std::make_shared<geometry_msgs::msg::Pose>();
auto msg_2_metadata_origin_position = std::make_shared<geometry_msgs::msg::Point>();
double msg_2_metadata_origin_position_x;
symros_make_symbolic_with_prefix(&msg_2_metadata_origin_position_x, sizeof(double), fd, "msg_2_metadata_origin_position_x", 1, "double");
msg_2_metadata_origin_position->x = msg_2_metadata_origin_position_x;

double msg_2_metadata_origin_position_y;
symros_make_symbolic_with_prefix(&msg_2_metadata_origin_position_y, sizeof(double), fd, "msg_2_metadata_origin_position_y", 1, "double");
msg_2_metadata_origin_position->y = msg_2_metadata_origin_position_y;

double msg_2_metadata_origin_position_z;
symros_make_symbolic_with_prefix(&msg_2_metadata_origin_position_z, sizeof(double), fd, "msg_2_metadata_origin_position_z", 1, "double");
msg_2_metadata_origin_position->z = msg_2_metadata_origin_position_z;

msg_2_metadata_origin->position = *msg_2_metadata_origin_position;

auto msg_2_metadata_origin_orientation = std::make_shared<geometry_msgs::msg::Quaternion>();
double msg_2_metadata_origin_orientation_x;
symros_make_symbolic_with_prefix(&msg_2_metadata_origin_orientation_x, sizeof(double), fd, "msg_2_metadata_origin_orientation_x", 1, "double");
msg_2_metadata_origin_orientation->x = msg_2_metadata_origin_orientation_x;

double msg_2_metadata_origin_orientation_y;
symros_make_symbolic_with_prefix(&msg_2_metadata_origin_orientation_y, sizeof(double), fd, "msg_2_metadata_origin_orientation_y", 1, "double");
msg_2_metadata_origin_orientation->y = msg_2_metadata_origin_orientation_y;

double msg_2_metadata_origin_orientation_z;
symros_make_symbolic_with_prefix(&msg_2_metadata_origin_orientation_z, sizeof(double), fd, "msg_2_metadata_origin_orientation_z", 1, "double");
msg_2_metadata_origin_orientation->z = msg_2_metadata_origin_orientation_z;

double msg_2_metadata_origin_orientation_w;
symros_make_symbolic_with_prefix(&msg_2_metadata_origin_orientation_w, sizeof(double), fd, "msg_2_metadata_origin_orientation_w", 1, "double");
msg_2_metadata_origin_orientation->w = msg_2_metadata_origin_orientation_w;

msg_2_metadata_origin->orientation = *msg_2_metadata_origin_orientation;

msg_2_metadata->origin = *msg_2_metadata_origin;

msg_2->metadata = *msg_2_metadata;

uint8_t msg_2_data_0;
symros_make_symbolic_with_prefix(&msg_2_data_0, sizeof(uint8_t), fd, "msg_2_data_0", 1, "uint8_t");
uint8_t msg_2_data_1;
symros_make_symbolic_with_prefix(&msg_2_data_1, sizeof(uint8_t), fd, "msg_2_data_1", 1, "uint8_t");
uint8_t msg_2_data_2;
symros_make_symbolic_with_prefix(&msg_2_data_2, sizeof(uint8_t), fd, "msg_2_data_2", 1, "uint8_t");
uint8_t msg_2_data_3;
symros_make_symbolic_with_prefix(&msg_2_data_3, sizeof(uint8_t), fd, "msg_2_data_3", 1, "uint8_t");
uint8_t msg_2_data_4;
symros_make_symbolic_with_prefix(&msg_2_data_4, sizeof(uint8_t), fd, "msg_2_data_4", 1, "uint8_t");
uint8_t msg_2_data_5;
symros_make_symbolic_with_prefix(&msg_2_data_5, sizeof(uint8_t), fd, "msg_2_data_5", 1, "uint8_t");
uint8_t msg_2_data_6;
symros_make_symbolic_with_prefix(&msg_2_data_6, sizeof(uint8_t), fd, "msg_2_data_6", 1, "uint8_t");
uint8_t msg_2_data_7;
symros_make_symbolic_with_prefix(&msg_2_data_7, sizeof(uint8_t), fd, "msg_2_data_7", 1, "uint8_t");
uint8_t msg_2_data_8;
symros_make_symbolic_with_prefix(&msg_2_data_8, sizeof(uint8_t), fd, "msg_2_data_8", 1, "uint8_t");
uint8_t msg_2_data_9;
symros_make_symbolic_with_prefix(&msg_2_data_9, sizeof(uint8_t), fd, "msg_2_data_9", 1, "uint8_t");
uint8_t msg_2_data_10;
symros_make_symbolic_with_prefix(&msg_2_data_10, sizeof(uint8_t), fd, "msg_2_data_10", 1, "uint8_t");
uint8_t msg_2_data_11;
symros_make_symbolic_with_prefix(&msg_2_data_11, sizeof(uint8_t), fd, "msg_2_data_11", 1, "uint8_t");
uint8_t msg_2_data_12;
symros_make_symbolic_with_prefix(&msg_2_data_12, sizeof(uint8_t), fd, "msg_2_data_12", 1, "uint8_t");
uint8_t msg_2_data_13;
symros_make_symbolic_with_prefix(&msg_2_data_13, sizeof(uint8_t), fd, "msg_2_data_13", 1, "uint8_t");
uint8_t msg_2_data_14;
symros_make_symbolic_with_prefix(&msg_2_data_14, sizeof(uint8_t), fd, "msg_2_data_14", 1, "uint8_t");
uint8_t msg_2_data_15;
symros_make_symbolic_with_prefix(&msg_2_data_15, sizeof(uint8_t), fd, "msg_2_data_15", 1, "uint8_t");
uint8_t msg_2_data_16;
symros_make_symbolic_with_prefix(&msg_2_data_16, sizeof(uint8_t), fd, "msg_2_data_16", 1, "uint8_t");
uint8_t msg_2_data_17;
symros_make_symbolic_with_prefix(&msg_2_data_17, sizeof(uint8_t), fd, "msg_2_data_17", 1, "uint8_t");
uint8_t msg_2_data_18;
symros_make_symbolic_with_prefix(&msg_2_data_18, sizeof(uint8_t), fd, "msg_2_data_18", 1, "uint8_t");
uint8_t msg_2_data_19;
symros_make_symbolic_with_prefix(&msg_2_data_19, sizeof(uint8_t), fd, "msg_2_data_19", 1, "uint8_t");
uint8_t msg_2_data_20;
symros_make_symbolic_with_prefix(&msg_2_data_20, sizeof(uint8_t), fd, "msg_2_data_20", 1, "uint8_t");
uint8_t msg_2_data_21;
symros_make_symbolic_with_prefix(&msg_2_data_21, sizeof(uint8_t), fd, "msg_2_data_21", 1, "uint8_t");
uint8_t msg_2_data_22;
symros_make_symbolic_with_prefix(&msg_2_data_22, sizeof(uint8_t), fd, "msg_2_data_22", 1, "uint8_t");
uint8_t msg_2_data_23;
symros_make_symbolic_with_prefix(&msg_2_data_23, sizeof(uint8_t), fd, "msg_2_data_23", 1, "uint8_t");
uint8_t msg_2_data_24;
symros_make_symbolic_with_prefix(&msg_2_data_24, sizeof(uint8_t), fd, "msg_2_data_24", 1, "uint8_t");
uint8_t msg_2_data_25;
symros_make_symbolic_with_prefix(&msg_2_data_25, sizeof(uint8_t), fd, "msg_2_data_25", 1, "uint8_t");
uint8_t msg_2_data_26;
symros_make_symbolic_with_prefix(&msg_2_data_26, sizeof(uint8_t), fd, "msg_2_data_26", 1, "uint8_t");
uint8_t msg_2_data_27;
symros_make_symbolic_with_prefix(&msg_2_data_27, sizeof(uint8_t), fd, "msg_2_data_27", 1, "uint8_t");
uint8_t msg_2_data_28;
symros_make_symbolic_with_prefix(&msg_2_data_28, sizeof(uint8_t), fd, "msg_2_data_28", 1, "uint8_t");
uint8_t msg_2_data_29;
symros_make_symbolic_with_prefix(&msg_2_data_29, sizeof(uint8_t), fd, "msg_2_data_29", 1, "uint8_t");
uint8_t msg_2_data_30;
symros_make_symbolic_with_prefix(&msg_2_data_30, sizeof(uint8_t), fd, "msg_2_data_30", 1, "uint8_t");
uint8_t msg_2_data_31;
symros_make_symbolic_with_prefix(&msg_2_data_31, sizeof(uint8_t), fd, "msg_2_data_31", 1, "uint8_t");
uint8_t msg_2_data_32;
symros_make_symbolic_with_prefix(&msg_2_data_32, sizeof(uint8_t), fd, "msg_2_data_32", 1, "uint8_t");
uint8_t msg_2_data_33;
symros_make_symbolic_with_prefix(&msg_2_data_33, sizeof(uint8_t), fd, "msg_2_data_33", 1, "uint8_t");
uint8_t msg_2_data_34;
symros_make_symbolic_with_prefix(&msg_2_data_34, sizeof(uint8_t), fd, "msg_2_data_34", 1, "uint8_t");
uint8_t msg_2_data_35;
symros_make_symbolic_with_prefix(&msg_2_data_35, sizeof(uint8_t), fd, "msg_2_data_35", 1, "uint8_t");
uint8_t msg_2_data_36;
symros_make_symbolic_with_prefix(&msg_2_data_36, sizeof(uint8_t), fd, "msg_2_data_36", 1, "uint8_t");
uint8_t msg_2_data_37;
symros_make_symbolic_with_prefix(&msg_2_data_37, sizeof(uint8_t), fd, "msg_2_data_37", 1, "uint8_t");
uint8_t msg_2_data_38;
symros_make_symbolic_with_prefix(&msg_2_data_38, sizeof(uint8_t), fd, "msg_2_data_38", 1, "uint8_t");
uint8_t msg_2_data_39;
symros_make_symbolic_with_prefix(&msg_2_data_39, sizeof(uint8_t), fd, "msg_2_data_39", 1, "uint8_t");
uint8_t msg_2_data_40;
symros_make_symbolic_with_prefix(&msg_2_data_40, sizeof(uint8_t), fd, "msg_2_data_40", 1, "uint8_t");
uint8_t msg_2_data_41;
symros_make_symbolic_with_prefix(&msg_2_data_41, sizeof(uint8_t), fd, "msg_2_data_41", 1, "uint8_t");
uint8_t msg_2_data_42;
symros_make_symbolic_with_prefix(&msg_2_data_42, sizeof(uint8_t), fd, "msg_2_data_42", 1, "uint8_t");
uint8_t msg_2_data_43;
symros_make_symbolic_with_prefix(&msg_2_data_43, sizeof(uint8_t), fd, "msg_2_data_43", 1, "uint8_t");
uint8_t msg_2_data_44;
symros_make_symbolic_with_prefix(&msg_2_data_44, sizeof(uint8_t), fd, "msg_2_data_44", 1, "uint8_t");
uint8_t msg_2_data_45;
symros_make_symbolic_with_prefix(&msg_2_data_45, sizeof(uint8_t), fd, "msg_2_data_45", 1, "uint8_t");
uint8_t msg_2_data_46;
symros_make_symbolic_with_prefix(&msg_2_data_46, sizeof(uint8_t), fd, "msg_2_data_46", 1, "uint8_t");
uint8_t msg_2_data_47;
symros_make_symbolic_with_prefix(&msg_2_data_47, sizeof(uint8_t), fd, "msg_2_data_47", 1, "uint8_t");
uint8_t msg_2_data_48;
symros_make_symbolic_with_prefix(&msg_2_data_48, sizeof(uint8_t), fd, "msg_2_data_48", 1, "uint8_t");
uint8_t msg_2_data_49;
symros_make_symbolic_with_prefix(&msg_2_data_49, sizeof(uint8_t), fd, "msg_2_data_49", 1, "uint8_t");
uint8_t msg_2_data_50;
symros_make_symbolic_with_prefix(&msg_2_data_50, sizeof(uint8_t), fd, "msg_2_data_50", 1, "uint8_t");
uint8_t msg_2_data_51;
symros_make_symbolic_with_prefix(&msg_2_data_51, sizeof(uint8_t), fd, "msg_2_data_51", 1, "uint8_t");
uint8_t msg_2_data_52;
symros_make_symbolic_with_prefix(&msg_2_data_52, sizeof(uint8_t), fd, "msg_2_data_52", 1, "uint8_t");
uint8_t msg_2_data_53;
symros_make_symbolic_with_prefix(&msg_2_data_53, sizeof(uint8_t), fd, "msg_2_data_53", 1, "uint8_t");
uint8_t msg_2_data_54;
symros_make_symbolic_with_prefix(&msg_2_data_54, sizeof(uint8_t), fd, "msg_2_data_54", 1, "uint8_t");
uint8_t msg_2_data_55;
symros_make_symbolic_with_prefix(&msg_2_data_55, sizeof(uint8_t), fd, "msg_2_data_55", 1, "uint8_t");
uint8_t msg_2_data_56;
symros_make_symbolic_with_prefix(&msg_2_data_56, sizeof(uint8_t), fd, "msg_2_data_56", 1, "uint8_t");
uint8_t msg_2_data_57;
symros_make_symbolic_with_prefix(&msg_2_data_57, sizeof(uint8_t), fd, "msg_2_data_57", 1, "uint8_t");
uint8_t msg_2_data_58;
symros_make_symbolic_with_prefix(&msg_2_data_58, sizeof(uint8_t), fd, "msg_2_data_58", 1, "uint8_t");
uint8_t msg_2_data_59;
symros_make_symbolic_with_prefix(&msg_2_data_59, sizeof(uint8_t), fd, "msg_2_data_59", 1, "uint8_t");
uint8_t msg_2_data_60;
symros_make_symbolic_with_prefix(&msg_2_data_60, sizeof(uint8_t), fd, "msg_2_data_60", 1, "uint8_t");
uint8_t msg_2_data_61;
symros_make_symbolic_with_prefix(&msg_2_data_61, sizeof(uint8_t), fd, "msg_2_data_61", 1, "uint8_t");
uint8_t msg_2_data_62;
symros_make_symbolic_with_prefix(&msg_2_data_62, sizeof(uint8_t), fd, "msg_2_data_62", 1, "uint8_t");
uint8_t msg_2_data_63;
symros_make_symbolic_with_prefix(&msg_2_data_63, sizeof(uint8_t), fd, "msg_2_data_63", 1, "uint8_t");
uint8_t msg_2_data_64;
symros_make_symbolic_with_prefix(&msg_2_data_64, sizeof(uint8_t), fd, "msg_2_data_64", 1, "uint8_t");
uint8_t msg_2_data_65;
symros_make_symbolic_with_prefix(&msg_2_data_65, sizeof(uint8_t), fd, "msg_2_data_65", 1, "uint8_t");
uint8_t msg_2_data_66;
symros_make_symbolic_with_prefix(&msg_2_data_66, sizeof(uint8_t), fd, "msg_2_data_66", 1, "uint8_t");
uint8_t msg_2_data_67;
symros_make_symbolic_with_prefix(&msg_2_data_67, sizeof(uint8_t), fd, "msg_2_data_67", 1, "uint8_t");
uint8_t msg_2_data_68;
symros_make_symbolic_with_prefix(&msg_2_data_68, sizeof(uint8_t), fd, "msg_2_data_68", 1, "uint8_t");
uint8_t msg_2_data_69;
symros_make_symbolic_with_prefix(&msg_2_data_69, sizeof(uint8_t), fd, "msg_2_data_69", 1, "uint8_t");
uint8_t msg_2_data_70;
symros_make_symbolic_with_prefix(&msg_2_data_70, sizeof(uint8_t), fd, "msg_2_data_70", 1, "uint8_t");
uint8_t msg_2_data_71;
symros_make_symbolic_with_prefix(&msg_2_data_71, sizeof(uint8_t), fd, "msg_2_data_71", 1, "uint8_t");
uint8_t msg_2_data_72;
symros_make_symbolic_with_prefix(&msg_2_data_72, sizeof(uint8_t), fd, "msg_2_data_72", 1, "uint8_t");
uint8_t msg_2_data_73;
symros_make_symbolic_with_prefix(&msg_2_data_73, sizeof(uint8_t), fd, "msg_2_data_73", 1, "uint8_t");
uint8_t msg_2_data_74;
symros_make_symbolic_with_prefix(&msg_2_data_74, sizeof(uint8_t), fd, "msg_2_data_74", 1, "uint8_t");
uint8_t msg_2_data_75;
symros_make_symbolic_with_prefix(&msg_2_data_75, sizeof(uint8_t), fd, "msg_2_data_75", 1, "uint8_t");
uint8_t msg_2_data_76;
symros_make_symbolic_with_prefix(&msg_2_data_76, sizeof(uint8_t), fd, "msg_2_data_76", 1, "uint8_t");
uint8_t msg_2_data_77;
symros_make_symbolic_with_prefix(&msg_2_data_77, sizeof(uint8_t), fd, "msg_2_data_77", 1, "uint8_t");
uint8_t msg_2_data_78;
symros_make_symbolic_with_prefix(&msg_2_data_78, sizeof(uint8_t), fd, "msg_2_data_78", 1, "uint8_t");
uint8_t msg_2_data_79;
symros_make_symbolic_with_prefix(&msg_2_data_79, sizeof(uint8_t), fd, "msg_2_data_79", 1, "uint8_t");
uint8_t msg_2_data_80;
symros_make_symbolic_with_prefix(&msg_2_data_80, sizeof(uint8_t), fd, "msg_2_data_80", 1, "uint8_t");
uint8_t msg_2_data_81;
symros_make_symbolic_with_prefix(&msg_2_data_81, sizeof(uint8_t), fd, "msg_2_data_81", 1, "uint8_t");
uint8_t msg_2_data_82;
symros_make_symbolic_with_prefix(&msg_2_data_82, sizeof(uint8_t), fd, "msg_2_data_82", 1, "uint8_t");
uint8_t msg_2_data_83;
symros_make_symbolic_with_prefix(&msg_2_data_83, sizeof(uint8_t), fd, "msg_2_data_83", 1, "uint8_t");
uint8_t msg_2_data_84;
symros_make_symbolic_with_prefix(&msg_2_data_84, sizeof(uint8_t), fd, "msg_2_data_84", 1, "uint8_t");
uint8_t msg_2_data_85;
symros_make_symbolic_with_prefix(&msg_2_data_85, sizeof(uint8_t), fd, "msg_2_data_85", 1, "uint8_t");
uint8_t msg_2_data_86;
symros_make_symbolic_with_prefix(&msg_2_data_86, sizeof(uint8_t), fd, "msg_2_data_86", 1, "uint8_t");
uint8_t msg_2_data_87;
symros_make_symbolic_with_prefix(&msg_2_data_87, sizeof(uint8_t), fd, "msg_2_data_87", 1, "uint8_t");
uint8_t msg_2_data_88;
symros_make_symbolic_with_prefix(&msg_2_data_88, sizeof(uint8_t), fd, "msg_2_data_88", 1, "uint8_t");
uint8_t msg_2_data_89;
symros_make_symbolic_with_prefix(&msg_2_data_89, sizeof(uint8_t), fd, "msg_2_data_89", 1, "uint8_t");
uint8_t msg_2_data_90;
symros_make_symbolic_with_prefix(&msg_2_data_90, sizeof(uint8_t), fd, "msg_2_data_90", 1, "uint8_t");
uint8_t msg_2_data_91;
symros_make_symbolic_with_prefix(&msg_2_data_91, sizeof(uint8_t), fd, "msg_2_data_91", 1, "uint8_t");
uint8_t msg_2_data_92;
symros_make_symbolic_with_prefix(&msg_2_data_92, sizeof(uint8_t), fd, "msg_2_data_92", 1, "uint8_t");
uint8_t msg_2_data_93;
symros_make_symbolic_with_prefix(&msg_2_data_93, sizeof(uint8_t), fd, "msg_2_data_93", 1, "uint8_t");
uint8_t msg_2_data_94;
symros_make_symbolic_with_prefix(&msg_2_data_94, sizeof(uint8_t), fd, "msg_2_data_94", 1, "uint8_t");
uint8_t msg_2_data_95;
symros_make_symbolic_with_prefix(&msg_2_data_95, sizeof(uint8_t), fd, "msg_2_data_95", 1, "uint8_t");
uint8_t msg_2_data_96;
symros_make_symbolic_with_prefix(&msg_2_data_96, sizeof(uint8_t), fd, "msg_2_data_96", 1, "uint8_t");
uint8_t msg_2_data_97;
symros_make_symbolic_with_prefix(&msg_2_data_97, sizeof(uint8_t), fd, "msg_2_data_97", 1, "uint8_t");
uint8_t msg_2_data_98;
symros_make_symbolic_with_prefix(&msg_2_data_98, sizeof(uint8_t), fd, "msg_2_data_98", 1, "uint8_t");
uint8_t msg_2_data_99;
symros_make_symbolic_with_prefix(&msg_2_data_99, sizeof(uint8_t), fd, "msg_2_data_99", 1, "uint8_t");
bool empty_msg_2_data;
symros_make_symbolic_with_prefix(&empty_msg_2_data, sizeof(bool), fd, "empty_msg_2_data", 2, "bool");

if (!empty_msg_2_data) {

msg_2->data.push_back(msg_2_data_0);

msg_2->data.push_back(msg_2_data_1);

msg_2->data.push_back(msg_2_data_2);

msg_2->data.push_back(msg_2_data_3);

msg_2->data.push_back(msg_2_data_4);

msg_2->data.push_back(msg_2_data_5);

msg_2->data.push_back(msg_2_data_6);

msg_2->data.push_back(msg_2_data_7);

msg_2->data.push_back(msg_2_data_8);

msg_2->data.push_back(msg_2_data_9);

msg_2->data.push_back(msg_2_data_10);

msg_2->data.push_back(msg_2_data_11);

msg_2->data.push_back(msg_2_data_12);

msg_2->data.push_back(msg_2_data_13);

msg_2->data.push_back(msg_2_data_14);

msg_2->data.push_back(msg_2_data_15);

msg_2->data.push_back(msg_2_data_16);

msg_2->data.push_back(msg_2_data_17);

msg_2->data.push_back(msg_2_data_18);

msg_2->data.push_back(msg_2_data_19);

msg_2->data.push_back(msg_2_data_20);

msg_2->data.push_back(msg_2_data_21);

msg_2->data.push_back(msg_2_data_22);

msg_2->data.push_back(msg_2_data_23);

msg_2->data.push_back(msg_2_data_24);

msg_2->data.push_back(msg_2_data_25);

msg_2->data.push_back(msg_2_data_26);

msg_2->data.push_back(msg_2_data_27);

msg_2->data.push_back(msg_2_data_28);

msg_2->data.push_back(msg_2_data_29);

msg_2->data.push_back(msg_2_data_30);

msg_2->data.push_back(msg_2_data_31);

msg_2->data.push_back(msg_2_data_32);

msg_2->data.push_back(msg_2_data_33);

msg_2->data.push_back(msg_2_data_34);

msg_2->data.push_back(msg_2_data_35);

msg_2->data.push_back(msg_2_data_36);

msg_2->data.push_back(msg_2_data_37);

msg_2->data.push_back(msg_2_data_38);

msg_2->data.push_back(msg_2_data_39);

msg_2->data.push_back(msg_2_data_40);

msg_2->data.push_back(msg_2_data_41);

msg_2->data.push_back(msg_2_data_42);

msg_2->data.push_back(msg_2_data_43);

msg_2->data.push_back(msg_2_data_44);

msg_2->data.push_back(msg_2_data_45);

msg_2->data.push_back(msg_2_data_46);

msg_2->data.push_back(msg_2_data_47);

msg_2->data.push_back(msg_2_data_48);

msg_2->data.push_back(msg_2_data_49);

msg_2->data.push_back(msg_2_data_50);

msg_2->data.push_back(msg_2_data_51);

msg_2->data.push_back(msg_2_data_52);

msg_2->data.push_back(msg_2_data_53);

msg_2->data.push_back(msg_2_data_54);

msg_2->data.push_back(msg_2_data_55);

msg_2->data.push_back(msg_2_data_56);

msg_2->data.push_back(msg_2_data_57);

msg_2->data.push_back(msg_2_data_58);

msg_2->data.push_back(msg_2_data_59);

msg_2->data.push_back(msg_2_data_60);

msg_2->data.push_back(msg_2_data_61);

msg_2->data.push_back(msg_2_data_62);

msg_2->data.push_back(msg_2_data_63);

msg_2->data.push_back(msg_2_data_64);

msg_2->data.push_back(msg_2_data_65);

msg_2->data.push_back(msg_2_data_66);

msg_2->data.push_back(msg_2_data_67);

msg_2->data.push_back(msg_2_data_68);

msg_2->data.push_back(msg_2_data_69);

msg_2->data.push_back(msg_2_data_70);

msg_2->data.push_back(msg_2_data_71);

msg_2->data.push_back(msg_2_data_72);

msg_2->data.push_back(msg_2_data_73);

msg_2->data.push_back(msg_2_data_74);

msg_2->data.push_back(msg_2_data_75);

msg_2->data.push_back(msg_2_data_76);

msg_2->data.push_back(msg_2_data_77);

msg_2->data.push_back(msg_2_data_78);

msg_2->data.push_back(msg_2_data_79);

msg_2->data.push_back(msg_2_data_80);

msg_2->data.push_back(msg_2_data_81);

msg_2->data.push_back(msg_2_data_82);

msg_2->data.push_back(msg_2_data_83);

msg_2->data.push_back(msg_2_data_84);

msg_2->data.push_back(msg_2_data_85);

msg_2->data.push_back(msg_2_data_86);

msg_2->data.push_back(msg_2_data_87);

msg_2->data.push_back(msg_2_data_88);

msg_2->data.push_back(msg_2_data_89);

msg_2->data.push_back(msg_2_data_90);

msg_2->data.push_back(msg_2_data_91);

msg_2->data.push_back(msg_2_data_92);

msg_2->data.push_back(msg_2_data_93);

msg_2->data.push_back(msg_2_data_94);

msg_2->data.push_back(msg_2_data_95);

msg_2->data.push_back(msg_2_data_96);

msg_2->data.push_back(msg_2_data_97);

msg_2->data.push_back(msg_2_data_98);

msg_2->data.push_back(msg_2_data_99);


}


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);



if (empty_msg_2_data) {}


SYMROS__CALL_SUBSCRIPTION_CALLBACK(sub_2, msg_2, msg_2_info_);



printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto msg_3 = std::make_shared<geometry_msgs::msg::PolygonStamped>();
auto msg_3_header = std::make_shared<std_msgs::msg::Header>();
auto msg_3_header_stamp = std::make_shared<builtin_interfaces::msg::Time>();
int32_t msg_3_header_stamp_sec;
symros_make_symbolic_with_prefix(&msg_3_header_stamp_sec, sizeof(int32_t), fd, "msg_3_header_stamp_sec", 1, "int32_t");
msg_3_header_stamp->sec = msg_3_header_stamp_sec;

uint32_t msg_3_header_stamp_nanosec;
symros_make_symbolic_with_prefix(&msg_3_header_stamp_nanosec, sizeof(uint32_t), fd, "msg_3_header_stamp_nanosec", 1, "uint32_t");
msg_3_header_stamp->nanosec = msg_3_header_stamp_nanosec;

msg_3_header->stamp = *msg_3_header_stamp;

uint8_t msg_3_header_frame_id_string_size;
symros_make_symbolic_with_prefix(&msg_3_header_frame_id_string_size, sizeof(uint8_t), fd, "msg_3_header_frame_id_string_size", 1, "uint8_t");
char msg_3_header_frame_id_string_data[MAX_LEN];
symros_make_symbolic_with_prefix(&msg_3_header_frame_id_string_data, MAX_LEN, fd, "msg_3_header_frame_id_string_data", 1, "char *");
std::string msg_3_header_frame_id(msg_3_header_frame_id_string_data, msg_3_header_frame_id_string_size);
msg_3_header->frame_id = msg_3_header_frame_id;

msg_3->header = *msg_3_header;

auto msg_3_polygon = std::make_shared<geometry_msgs::msg::Polygon>();
auto msg_3_polygon_points_0 = std::make_shared<geometry_msgs::msg::Point32>();
float msg_3_polygon_points_0_x;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_0_x, sizeof(float), fd, "msg_3_polygon_points_0_x", 1, "float");
msg_3_polygon_points_0->x = msg_3_polygon_points_0_x;

float msg_3_polygon_points_0_y;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_0_y, sizeof(float), fd, "msg_3_polygon_points_0_y", 1, "float");
msg_3_polygon_points_0->y = msg_3_polygon_points_0_y;

float msg_3_polygon_points_0_z;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_0_z, sizeof(float), fd, "msg_3_polygon_points_0_z", 1, "float");
msg_3_polygon_points_0->z = msg_3_polygon_points_0_z;

auto msg_3_polygon_points_1 = std::make_shared<geometry_msgs::msg::Point32>();
float msg_3_polygon_points_1_x;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_1_x, sizeof(float), fd, "msg_3_polygon_points_1_x", 1, "float");
msg_3_polygon_points_1->x = msg_3_polygon_points_1_x;

float msg_3_polygon_points_1_y;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_1_y, sizeof(float), fd, "msg_3_polygon_points_1_y", 1, "float");
msg_3_polygon_points_1->y = msg_3_polygon_points_1_y;

float msg_3_polygon_points_1_z;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_1_z, sizeof(float), fd, "msg_3_polygon_points_1_z", 1, "float");
msg_3_polygon_points_1->z = msg_3_polygon_points_1_z;

auto msg_3_polygon_points_2 = std::make_shared<geometry_msgs::msg::Point32>();
float msg_3_polygon_points_2_x;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_2_x, sizeof(float), fd, "msg_3_polygon_points_2_x", 1, "float");
msg_3_polygon_points_2->x = msg_3_polygon_points_2_x;

float msg_3_polygon_points_2_y;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_2_y, sizeof(float), fd, "msg_3_polygon_points_2_y", 1, "float");
msg_3_polygon_points_2->y = msg_3_polygon_points_2_y;

float msg_3_polygon_points_2_z;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_2_z, sizeof(float), fd, "msg_3_polygon_points_2_z", 1, "float");
msg_3_polygon_points_2->z = msg_3_polygon_points_2_z;

auto msg_3_polygon_points_3 = std::make_shared<geometry_msgs::msg::Point32>();
float msg_3_polygon_points_3_x;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_3_x, sizeof(float), fd, "msg_3_polygon_points_3_x", 1, "float");
msg_3_polygon_points_3->x = msg_3_polygon_points_3_x;

float msg_3_polygon_points_3_y;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_3_y, sizeof(float), fd, "msg_3_polygon_points_3_y", 1, "float");
msg_3_polygon_points_3->y = msg_3_polygon_points_3_y;

float msg_3_polygon_points_3_z;
symros_make_symbolic_with_prefix(&msg_3_polygon_points_3_z, sizeof(float), fd, "msg_3_polygon_points_3_z", 1, "float");
msg_3_polygon_points_3->z = msg_3_polygon_points_3_z;

bool empty_msg_3_polygon_points;
symros_make_symbolic_with_prefix(&empty_msg_3_polygon_points, sizeof(bool), fd, "empty_msg_3_polygon_points", 2, "bool");

if (!empty_msg_3_polygon_points) {

msg_3_polygon->points.push_back(*msg_3_polygon_points_0);

msg_3_polygon->points.push_back(*msg_3_polygon_points_1);

msg_3_polygon->points.push_back(*msg_3_polygon_points_2);

msg_3_polygon->points.push_back(*msg_3_polygon_points_3);


}

msg_3->polygon = *msg_3_polygon;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);



if (empty_msg_3_polygon_points) {}


SYMROS__CALL_SUBSCRIPTION_CALLBACK(sub_3, msg_3, msg_3_info_);



printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto act_req_4 = std::make_shared<nav2_msgs::action::DriveOnHeading::Goal>();
auto act_req_4_target = std::make_shared<geometry_msgs::msg::Point>();
double act_req_4_target_x;
symros_make_symbolic_with_prefix(&act_req_4_target_x, sizeof(double), fd, "act_req_4_target_x", 1, "double");
act_req_4_target->x = act_req_4_target_x;

double act_req_4_target_y;
symros_make_symbolic_with_prefix(&act_req_4_target_y, sizeof(double), fd, "act_req_4_target_y", 1, "double");
act_req_4_target->y = act_req_4_target_y;

double act_req_4_target_z;
symros_make_symbolic_with_prefix(&act_req_4_target_z, sizeof(double), fd, "act_req_4_target_z", 1, "double");
act_req_4_target->z = act_req_4_target_z;

act_req_4->target = *act_req_4_target;

float act_req_4_speed;
symros_make_symbolic_with_prefix(&act_req_4_speed, sizeof(float), fd, "act_req_4_speed", 1, "float");
act_req_4->speed = act_req_4_speed;

auto act_req_4_time_allowance = std::make_shared<builtin_interfaces::msg::Duration>();
int32_t act_req_4_time_allowance_sec;
symros_make_symbolic_with_prefix(&act_req_4_time_allowance_sec, sizeof(int32_t), fd, "act_req_4_time_allowance_sec", 1, "int32_t");
act_req_4_time_allowance->sec = act_req_4_time_allowance_sec;

uint32_t act_req_4_time_allowance_nanosec;
symros_make_symbolic_with_prefix(&act_req_4_time_allowance_nanosec, sizeof(uint32_t), fd, "act_req_4_time_allowance_nanosec", 1, "uint32_t");
act_req_4_time_allowance->nanosec = act_req_4_time_allowance_nanosec;

act_req_4->time_allowance = *act_req_4_time_allowance;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




SYMROS_CallActionServerCallback<nav2_msgs::action::DriveOnHeading>(act_srv_4, act_req_4);



printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto act_req_5 = std::make_shared<nav2_msgs::action::BackUp::Goal>();
auto act_req_5_target = std::make_shared<geometry_msgs::msg::Point>();
double act_req_5_target_x;
symros_make_symbolic_with_prefix(&act_req_5_target_x, sizeof(double), fd, "act_req_5_target_x", 1, "double");
act_req_5_target->x = act_req_5_target_x;

double act_req_5_target_y;
symros_make_symbolic_with_prefix(&act_req_5_target_y, sizeof(double), fd, "act_req_5_target_y", 1, "double");
act_req_5_target->y = act_req_5_target_y;

double act_req_5_target_z;
symros_make_symbolic_with_prefix(&act_req_5_target_z, sizeof(double), fd, "act_req_5_target_z", 1, "double");
act_req_5_target->z = act_req_5_target_z;

act_req_5->target = *act_req_5_target;

float act_req_5_speed;
symros_make_symbolic_with_prefix(&act_req_5_speed, sizeof(float), fd, "act_req_5_speed", 1, "float");
act_req_5->speed = act_req_5_speed;

auto act_req_5_time_allowance = std::make_shared<builtin_interfaces::msg::Duration>();
int32_t act_req_5_time_allowance_sec;
symros_make_symbolic_with_prefix(&act_req_5_time_allowance_sec, sizeof(int32_t), fd, "act_req_5_time_allowance_sec", 1, "int32_t");
act_req_5_time_allowance->sec = act_req_5_time_allowance_sec;

uint32_t act_req_5_time_allowance_nanosec;
symros_make_symbolic_with_prefix(&act_req_5_time_allowance_nanosec, sizeof(uint32_t), fd, "act_req_5_time_allowance_nanosec", 1, "uint32_t");
act_req_5_time_allowance->nanosec = act_req_5_time_allowance_nanosec;

act_req_5->time_allowance = *act_req_5_time_allowance;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




SYMROS_CallActionServerCallback<nav2_msgs::action::BackUp>(act_srv_5, act_req_5);



printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto act_req_6 = std::make_shared<nav2_msgs::action::Spin::Goal>();
float act_req_6_target_yaw;
symros_make_symbolic_with_prefix(&act_req_6_target_yaw, sizeof(float), fd, "act_req_6_target_yaw", 1, "float");
act_req_6->target_yaw = act_req_6_target_yaw;

auto act_req_6_time_allowance = std::make_shared<builtin_interfaces::msg::Duration>();
int32_t act_req_6_time_allowance_sec;
symros_make_symbolic_with_prefix(&act_req_6_time_allowance_sec, sizeof(int32_t), fd, "act_req_6_time_allowance_sec", 1, "int32_t");
act_req_6_time_allowance->sec = act_req_6_time_allowance_sec;

uint32_t act_req_6_time_allowance_nanosec;
symros_make_symbolic_with_prefix(&act_req_6_time_allowance_nanosec, sizeof(uint32_t), fd, "act_req_6_time_allowance_nanosec", 1, "uint32_t");
act_req_6_time_allowance->nanosec = act_req_6_time_allowance_nanosec;

act_req_6->time_allowance = *act_req_6_time_allowance;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




SYMROS_CallActionServerCallback<nav2_msgs::action::Spin>(act_srv_6, act_req_6);



printf("\n==== SYMBOLIC BARRIER START ====\n");fflush(stdout);
auto act_req_7 = std::make_shared<nav2_msgs::action::Wait::Goal>();
auto act_req_7_time = std::make_shared<builtin_interfaces::msg::Duration>();
int32_t act_req_7_time_sec;
symros_make_symbolic_with_prefix(&act_req_7_time_sec, sizeof(int32_t), fd, "act_req_7_time_sec", 1, "int32_t");
act_req_7_time->sec = act_req_7_time_sec;

uint32_t act_req_7_time_nanosec;
symros_make_symbolic_with_prefix(&act_req_7_time_nanosec, sizeof(uint32_t), fd, "act_req_7_time_nanosec", 1, "uint32_t");
act_req_7_time->nanosec = act_req_7_time_nanosec;

act_req_7->time = *act_req_7_time;


printf("\n==== SYMBOLIC BARRIER STOP ====\n");fflush(stdout);




SYMROS_CallActionServerCallback<nav2_msgs::action::Wait>(act_srv_7, act_req_7);


printf("[symros] callback test finish\n");

close(fd);


node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DESTROY);
// SYMROS executor

  exit(0);
}