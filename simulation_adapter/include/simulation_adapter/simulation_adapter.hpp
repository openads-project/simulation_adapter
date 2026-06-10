#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

// definitions
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_planning_msgs/msg/trajectory.hpp>

// access functions
#include <perception_msgs_utils/object_access.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>
#include <tf2_trajectory_planning_msgs/tf2_trajectory_planning_msgs.hpp>

// namespaces
namespace gm = geometry_msgs::msg;
namespace pm = perception_msgs::msg;
namespace sm = std_msgs::msg;
namespace tp = trajectory_planning_msgs::msg;

namespace simulation_adapter {

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

/**
 * @brief SimulationAdapter class
 */
class SimulationAdapter : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   *
   */
  SimulationAdapter();

  /**
   * @brief Number of threads for MultiThreadedExecutor
   */
  int num_threads_ = 1;

 private:
  /**
   * @brief Declares and loads a ROS parameter
   *
   * @param name name
   * @param param parameter variable to load into
   * @param description description
   * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
   * @param is_required whether failure to load parameter will stop node
   * @param read_only set parameter to read-only
   * @param from_value parameter range minimum
   * @param to_value parameter range maximum
   * @param step_value parameter range step
   * @param additional_constraints additional constraints description
   */
  template <typename T>
  void declareAndLoadParameter(const std::string& name,
                               T& param,
                               const std::string& description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double>& from_value = std::nullopt,
                               const std::optional<double>& to_value = std::nullopt,
                               const std::optional<double>& step_value = std::nullopt,
                               const std::string& additional_constraints = "");

  /**
   * @brief Handles reconfiguration when a parameter value is changed
   *
   * @param parameters parameters
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  /**
   * @brief Sets map for the lanelet2 map server based on the current simulation map name
   *
   * @param msg map name message
   */
  void mapInfoCallback(const sm::String::ConstSharedPtr& msg);

  /**
   * @brief Converts incoming ego data to fixed_frame_id and vehicle_frame_id frames
   *
   * @param msg ego data message
   */
  void egoDataCallback(const pm::EgoData::ConstSharedPtr& msg);

  /**
   * @brief Converts incoming object list to fixed_frame_id and vehicle_frame_id frames
   *
   * @param msg object list message
   */
  void objectListCallback(const pm::ObjectList::ConstSharedPtr& msg);

  /**
   * @brief Stores and transforms the latest planned trajectory to fixed_frame_id
   *
   * @param msg trajectory message
   */
  void trajectoryCallback(const tp::Trajectory::ConstSharedPtr& msg);

  /**
   * @brief Timer callback that initializes the static TF link between simulation and driving-stack frames
   */
  void initializeVehicleFrameTransform();

 private:
  /**
   * @brief Auto-reconfigurable parameters for dynamic reconfiguration
   */
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter&)>>> auto_reconfigurable_params_;

  /**
   * @brief Callback handle for dynamic parameter reconfiguration
   */
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  /**
   * @brief Callback group for callbacks that may run concurrently
   */
  rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;

  /**
   * @brief Parameters client used to configure the lanelet2 map server
   */
  std::shared_ptr<rclcpp::AsyncParametersClient> map_server_parameters_client_;

  /**
   * @brief Subscriber for simulation map info (latching QoS)
   */
  rclcpp::Subscription<sm::String>::SharedPtr sub_map_info_;

  /**
   * @brief Subscriber for incoming ego data
   */
  rclcpp::Subscription<pm::EgoData>::SharedPtr sub_ego_data_;

  /**
   * @brief Subscriber for incoming object list
   */
  rclcpp::Subscription<pm::ObjectList>::SharedPtr sub_object_list_;

  /**
   * @brief Subscriber for incoming planned trajectory
   */
  rclcpp::Subscription<tp::Trajectory>::SharedPtr sub_trajectory_;

  /**
   * @brief Publisher for ego data in vehicle/fixed frame
   */
  rclcpp::Publisher<pm::EgoData>::SharedPtr pub_ego_data_;

  /**
   * @brief Publisher for ego odometry in map/vehicle_frame_id frames
   */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ego_odometry_;

  /**
   * @brief Publisher for ego object state containing the steering acknowledgement values
   */
  rclcpp::Publisher<pm::ObjectState>::SharedPtr pub_ego_vehicle_state_;

  /**
   * @brief Publisher for ego IMU containing linear acceleration only
   */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_ego_imu_;

  /**
   * @brief Publisher for object list in vehicle frame
   */
  rclcpp::Publisher<pm::ObjectList>::SharedPtr pub_object_list_;

  /**
   * @brief Publisher for object list in fixed frame
   */
  rclcpp::Publisher<pm::ObjectList>::SharedPtr pub_object_list_fixed_;

  /**
   * @brief Timer for repeatedly attempting to initialize the vehicle frame transform
   */
  rclcpp::TimerBase::SharedPtr tf_init_timer_;

  /**
   * @brief tf2 buffer for transform lookups
   */
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;

  /**
   * @brief tf2 transform listener
   */
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  /**
   * @brief Static transform broadcaster for publishing the vehicle frame link
   */
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_br_tf_;

  // input topic names
  const std::string kInputMapInfoTopic = "~/input_map_info";
  const std::string kInputEgoDataTopic = "~/input_ego_data";
  const std::string kInputObjectListTopic = "~/input_object_list";
  const std::string kInputTrajectoryTopic = "~/input_trajectory";

  // output topic names
  const std::string kEgoDataTopic = "~/ego_data";
  const std::string kEgoOdometryTopic = "~/ego_odometry";
  const std::string kEgoVehicleStateTopic = "~/ego_vehicle_state";
  const std::string kEgoImuTopic = "~/ego_imu";
  const std::string kObjectListTopic = "~/object_list";
  const std::string kObjectListFixedTopic = "~/object_list_fixed";

  /**
   * @brief Name of the lanelet2 map server node
   */
  std::string map_server_name_ = "/ll2_map_server";

  /**
   * @brief Whether to automatically set the lanelet2 map based on the simulation map info
   */
  bool set_ll2_map_ = true;

  /**
   * @brief Name of the fixed frame id in simulation
   */
  std::string simulation_fixed_frame_id_ = "simulation_map";

  /**
   * @brief Name of the fixed frame id in the driving stack
   */
  std::string fixed_frame_id_ = "map";

  /**
   * @brief Name of the vehicle frame id in simulation
   */
  std::string simulation_vehicle_frame_id_ = "ego_vehicle";

  /**
   * @brief Name of the vehicle frame id in the driving stack
   */
  std::string vehicle_frame_id_ = "geo_center";

  /**
   * @brief Whether to publish the static TF from simulation_vehicle_frame_id to vehicle_frame_id
   */
  bool publish_vehicle_frame_tf_ = true;

  /**
   * @brief Whether to publish ego odometry
   */
  bool publish_ego_odometry_ = false;

  /**
   * @brief Whether to publish the ego vehicle state
   */
  bool publish_ego_vehicle_state_ = false;

  /**
   * @brief Whether to publish the ego IMU
   */
  bool publish_ego_imu_ = false;

  /**
   * @brief Longitudinal offset from simulation_vehicle_frame_id to vehicle_frame_id
   */
  double simulation_vehicle_frame_id_to_vehicle_frame_id_ = 0.0;

  /**
   * @brief List of supported simulation map names (parallel to lanelet_files_)
   */
  std::vector<std::string> simulation_maps_;

  /**
   * @brief List of lanelet2 map file paths (parallel to simulation_maps_)
   */
  std::vector<std::string> lanelet_files_;

  /**
   * @brief Latest planned trajectory, transformed to fixed_frame_id
   */
  tp::Trajectory trajectory_planned_;

  /**
   * @brief Protects planned trajectory access across concurrent callbacks
   */
  mutable std::mutex trajectory_planned_mutex_;

  /**
   * @brief Last received simulation map info string
   */
  std::string map_info_;
};

}  // namespace simulation_adapter
