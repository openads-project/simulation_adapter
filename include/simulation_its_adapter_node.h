#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

// definitions
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_planning_msgs/msg/trajectory.hpp>

// access functions
#include <perception_msgs_utils/object_access.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>
#include <tf2_trajectory_planning_msgs/tf2_trajectory_planning_msgs.hpp>

// namespaces
namespace gm = geometry_msgs::msg;
namespace pm = perception_msgs::msg;
namespace sm = std_msgs::msg;
namespace tp = trajectory_planning_msgs::msg;

namespace simulation_its_adapter {

template <typename T>
using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;
template <typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

class SimulationItsAdapterNode : public rclcpp::Node {
 public:
  explicit SimulationItsAdapterNode(const rclcpp::NodeOptions &options);

  ~SimulationItsAdapterNode();

 private:
  // input topics
  const std::string kInputMapInfoTopic = "~/input_map_info";
  const std::string kInputEgoDataTopic = "~/input_ego_data";
  const std::string kInputObjectListTopic = "~/input_object_list";
  const std::string kInputTrajectoryTopic = "~/input_trajectory";

  // output topics
  const std::string kEgoDataTopic = "~/ego_data";
  const std::string kObjectListTopic = "~/object_list";
  const std::string kObjectListFixedTopic = "~/object_list_fixed";

  template <typename T>
  void declareAndLoadParameter(const std::string &name,
                               T &param,
                               const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  void setup();

  void mapInfoCallback(const sm::String::ConstSharedPtr msg);
  void egoDataCallback(const pm::EgoData::ConstSharedPtr msg);
  void objectListCallback(const pm::ObjectList::ConstSharedPtr msg);
  void initializeVehicleFrameTransform();
  void trajectoryCallback(const tp::Trajectory::ConstSharedPtr msg);

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;
  std::shared_ptr<rclcpp::AsyncParametersClient> map_server_parameters_client_;

  // subscriber and publisher
  Subscriber<sm::String> sub_map_info_;
  Subscriber<pm::EgoData> sub_ego_data_;
  Subscriber<pm::ObjectList> sub_object_list_;
  Subscriber<tp::Trajectory> sub_trajectory_;

  Publisher<pm::EgoData> pub_ego_data_;
  Publisher<pm::ObjectList> pub_object_list_;
  Publisher<pm::ObjectList> pub_object_list_fixed_;

  // tf2 variables
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_br_tf_;
  rclcpp::TimerBase::SharedPtr tf_init_timer_;

  // input parameters
  std::string map_server_name_ = "/ll2_map_server";
  bool set_ll2_map_ = true;
  std::string simulation_fixed_frame_id_ = "simulation_map";
  std::string fixed_frame_id_ = "map";
  std::string simulation_vehicle_frame_id_ = "ego_vehicle";
  std::string vehicle_frame_id_ = "geo_center";
  double simulation_vehicle_frame_id_to_vehicle_frame_id_ = 0.0;

  std::vector<std::string> simulation_maps_;
  std::vector<std::string> lanelet_files_;

  tp::Trajectory trajectory_planned_;
  std::string map_info_;

  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>> auto_reconfigurable_params_;
};

}  // end of namespace simulation_its_adapter
