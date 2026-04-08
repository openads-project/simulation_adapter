#include <simulation_adapter/simulation_adapter.hpp>

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simulation_adapter::SimulationAdapter)


namespace simulation_adapter {

SimulationAdapter::SimulationAdapter(const rclcpp::NodeOptions& options) : Node("simulation_adapter", options) {
  this->declareAndLoadParameter("map_server_name", map_server_name_, "Name of the map server.");
  this->declareAndLoadParameter("set_ll2_map", set_ll2_map_,
                                "Automatically set the ll2 map based on the simulation map.");
  this->declareAndLoadParameter("simulation_fixed_frame_id", simulation_fixed_frame_id_, "Name of the fixed frame id in simulation.");
  this->declareAndLoadParameter("fixed_frame_id", fixed_frame_id_, "Name of the fixed frame id over time.");
  this->declareAndLoadParameter("simulation_vehicle_frame_id", simulation_vehicle_frame_id_,
                                "Name of the vehicle frame id in simulation.");
  this->declareAndLoadParameter("vehicle_frame_id", vehicle_frame_id_, "Name of the vehicle frame id.");
  this->declareAndLoadParameter("publish_vehicle_frame_tf", publish_vehicle_frame_tf_,
                                "Whether to publish the static TF from simulation_vehicle_frame_id to vehicle_frame_id.");
  this->declareAndLoadParameter("publish_ego_odometry", publish_ego_odometry_,
                                "Whether to publish ego odometry.");
  this->declareAndLoadParameter("publish_ego_vehicle_state", publish_ego_vehicle_state_,
                                "Whether to publish the ego vehicle state.");

  this->declareAndLoadParameter("simulation_vehicle_frame_id_to_vehicle_frame_id", simulation_vehicle_frame_id_to_vehicle_frame_id_,
                                "Longitudinal offset from simulation_vehicle_frame_id to vehicle_frame_id.");

  this->declareAndLoadParameter("maps.simulation_maps", simulation_maps_, "List of supported simulation maps.");
  this->declareAndLoadParameter("maps.lanelet_files", lanelet_files_, "Lanelet files for all supported simulation maps");

  this->setup();
}

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
void SimulationAdapter::declareAndLoadParameter(const std::string& name,
                                                  T& param,
                                                  const std::string& description,
                                                  const bool add_to_auto_reconfigurable_params,
                                                  const bool is_required,
                                                  const bool read_only,
                                                  const std::optional<double>& from_value,
                                                  const std::optional<double>& to_value,
                                                  const std::optional<double>& step_value,
                                                  const std::string& additional_constraints) {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr(std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.integer_range = {range};
    } else if constexpr(std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      T step = static_cast<T>(step_value.has_value() ? step_value.value() : 1.0);
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value())).set__step(step);
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr(is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "]");
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr(is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "]");
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult SimulationAdapter::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s' to: %s", param.get_name().c_str(), param.value_to_string().c_str());
        break;
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void SimulationAdapter::setup() {
  // initialize tf2 buffer, listener and static broadcaster
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  static_br_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  map_info_ = "";

  // parameters client to map server for setting map server's parameters
  map_server_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, map_server_name_);
  using namespace std::chrono_literals;
  while (!map_server_parameters_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_FATAL(this->get_logger(),
                   "Interrupted while waiting for the map server ('%s') parameter service, shutting down",
                   map_server_name_.c_str());
      rclcpp::shutdown();
    }
    RCLCPP_WARN(this->get_logger(), "Waiting for map server ('%s') parameter service ...", map_server_name_.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Connected to map server ('%s') parameter service", map_server_name_.c_str());

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&SimulationAdapter::parametersCallback, this, std::placeholders::_1));

  // setup subscriber for input topics
  // Use transient_local (latching) QoS so the node receives the map info even
  // when it starts after the publisher has already sent the single startup message.
  rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
  qosLatching.transient_local();
  qosLatching.reliable();
  sub_map_info_ = this->create_subscription<sm::String>(
      kInputMapInfoTopic, qosLatching,
      std::bind(&SimulationAdapter::mapInfoCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_map_info_->get_topic_name());

  sub_ego_data_ = this->create_subscription<pm::EgoData>(
      kInputEgoDataTopic, 1, std::bind(&SimulationAdapter::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_ego_data_->get_topic_name());

  sub_object_list_ = this->create_subscription<pm::ObjectList>(
      kInputObjectListTopic, 1, std::bind(&SimulationAdapter::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_object_list_->get_topic_name());

  sub_trajectory_ = this->create_subscription<tp::Trajectory>(
      kInputTrajectoryTopic, 1, std::bind(&SimulationAdapter::trajectoryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_trajectory_->get_topic_name());

  if (publish_vehicle_frame_tf_) {
    tf_init_timer_ = this->create_wall_timer(
        500ms, std::bind(&SimulationAdapter::initializeVehicleFrameTransform, this));
    RCLCPP_INFO(this->get_logger(), "Started timer to initialize transformation from '%s' to '%s'",
                fixed_frame_id_.c_str(), vehicle_frame_id_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Static TF publication from '%s' to '%s' is disabled",
                simulation_vehicle_frame_id_.c_str(), vehicle_frame_id_.c_str());
  }

  // set up publisher for output topics
  pub_ego_data_ = this->create_publisher<pm::EgoData>(kEgoDataTopic, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_ego_data_->get_topic_name());

  if (publish_ego_odometry_) {
    pub_ego_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(kEgoOdometryTopic, 1);
    RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_ego_odometry_->get_topic_name());
  }

  if (publish_ego_vehicle_state_) {
    pub_ego_vehicle_state_ = this->create_publisher<pm::ObjectState>(kEgoVehicleStateTopic, 1);
    RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_ego_vehicle_state_->get_topic_name());
  }

  pub_object_list_ = this->create_publisher<pm::ObjectList>(kObjectListTopic, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_object_list_->get_topic_name());

  pub_object_list_fixed_ = this->create_publisher<pm::ObjectList>(kObjectListFixedTopic, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_object_list_fixed_->get_topic_name());

  // logging info
  RCLCPP_INFO(this->get_logger(), "simulation_adapter is running...");
}

/**
 * @brief Set map for the lanelet2 map server based on the current simulation map name.
 *
 * @param msg Map name message.
 */
void SimulationAdapter::mapInfoCallback(const sm::String::ConstSharedPtr& msg) {
  if (msg->data == map_info_) return;
  map_info_ = msg->data;

  std::string lanelet_map_name;
  if (set_ll2_map_) {
    // convert simulation name to lanelet map name
    std::string simulation_map_name = map_info_;

    // find simulation_map_name in simulation_maps_
    auto it = std::find(simulation_maps_.begin(), simulation_maps_.end(), simulation_map_name);
    if (it == simulation_maps_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Simulation map name '%s' not found in the list of supported maps", simulation_map_name.c_str());
      return;
    }

    // Get lanelet map for simulation map name
    size_t index = std::distance(simulation_maps_.begin(), it);
    if (index >= lanelet_files_.size()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No lanelet file at index %zu for map '%s'. Check that 'maps.simulation_maps' and "
                   "'maps.lanelet_files' have the same length in params.yml.",
                   index, simulation_map_name.c_str());
      return;
    }
    lanelet_map_name = lanelet_files_[index];

    // change map by setting map server parameters
    map_server_parameters_client_->set_parameters(
      {rclcpp::Parameter("map_filepath", lanelet_map_name),
       rclcpp::Parameter("map_frame_id", fixed_frame_id_)},
      [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        auto results = future.get();
        for (const auto& result : results) {
          if (!result.successful)
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Finished setting map server parameters");
      });
  }
}


void SimulationAdapter::egoDataCallback(const pm::EgoData::ConstSharedPtr& msg) {
  auto timeout = rclcpp::Duration::from_seconds(1.0);
  gm::TransformStamped vehicle_frame_position_in_map_tf;

  // transform ego_data (input header is simulation_fixed_frame_id, output header is fixed_frame_id)
  pm::EgoData ego_data;

  gm::TransformStamped to_map_tf;
  try {
    to_map_tf = tf2_buffer_->lookupTransform(fixed_frame_id_, msg->header.frame_id, msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transformation from '%s' to '%s' is not available.", msg->header.frame_id.c_str(),
                fixed_frame_id_.c_str());
    return;
  }
  tf2::doTransform(*msg, ego_data, to_map_tf);

  tf2::Quaternion vehicle_orientation;
  tf2::fromMsg(perception_msgs::object_access::getOrientation(ego_data.state), vehicle_orientation);
  const tf2::Vector3 vehicle_offset(simulation_vehicle_frame_id_to_vehicle_frame_id_, 0.0, 0.0);
  const tf2::Vector3 offset_in_map = tf2::quatRotate(vehicle_orientation, vehicle_offset);

  perception_msgs::object_access::setX(ego_data, perception_msgs::object_access::getX(ego_data.state) + offset_in_map.x());
  perception_msgs::object_access::setY(ego_data, perception_msgs::object_access::getY(ego_data.state) + offset_in_map.y());
  perception_msgs::object_access::setZ(ego_data, perception_msgs::object_access::getZ(ego_data.state) + offset_in_map.z());

  ego_data.state.reference_point.value = pm::ObjectReferencePoint::REAR_AXLE_GROUND;
  ego_data.state.reference_point.translation_to_geometric_center.x = -simulation_vehicle_frame_id_to_vehicle_frame_id_;
  ego_data.state.reference_point.translation_to_geometric_center.z = perception_msgs::object_access::getHeight(ego_data.state) / 2.0;
  perception_msgs::object_access::setZ(ego_data, perception_msgs::object_access::getZ(ego_data.state) - perception_msgs::object_access::getHeight(ego_data.state) / 2.0);

  // add planned trajectory to ego_data if exists
  int n = trajectory_planning_msgs::trajectory_access::getSamplePointSize(trajectory_planned_);
  if (n > 0) {
    // clear current trajectory
    ego_data.trajectory_planned.clear();

    // initialize state
    pm::ObjectState object_state;
    perception_msgs::object_access::initializeState(object_state, 1);
    object_state.reference_point = ego_data.state.reference_point;

    // update trajectory state
    perception_msgs::object_access::setStandstill(
        object_state, trajectory_planning_msgs::trajectory_access::getStandstill(trajectory_planned_));

    for (int i = 0; i < n; i++) {
      // update header stamp
      object_state.header = trajectory_planned_.header;
      float time = trajectory_planning_msgs::trajectory_access::getT(trajectory_planned_, i);
      object_state.header.stamp.sec += (int)time;
      object_state.header.stamp.nanosec += (time - (int)time) * 1e9;

      perception_msgs::object_access::setX(object_state,
                                           trajectory_planning_msgs::trajectory_access::getX(trajectory_planned_, i));
      perception_msgs::object_access::setY(object_state,
                                           trajectory_planning_msgs::trajectory_access::getY(trajectory_planned_, i));
      perception_msgs::object_access::setVelLon(
          object_state, trajectory_planning_msgs::trajectory_access::getV(trajectory_planned_, i));
      perception_msgs::object_access::setAccLon(
          object_state, trajectory_planning_msgs::trajectory_access::getA(trajectory_planned_, i));
      perception_msgs::object_access::setYaw(
          object_state, trajectory_planning_msgs::trajectory_access::getTheta(trajectory_planned_, i));
      ego_data.trajectory_planned.push_back(object_state);
    }
  }

  // publish ego data in fixed_frame_id
  pub_ego_data_->publish(ego_data);

  if (publish_ego_odometry_) {
    // Odometry is published in map/<vehicle_frame_id> convention: pose in map, twist in the body frame.
    nav_msgs::msg::Odometry ego_odometry;
    ego_odometry.header.frame_id = fixed_frame_id_;
    ego_odometry.header.stamp = ego_data.state.header.stamp;
    ego_odometry.child_frame_id = vehicle_frame_id_;
    ego_odometry.pose.pose.position.x = perception_msgs::object_access::getX(ego_data.state);
    ego_odometry.pose.pose.position.y = perception_msgs::object_access::getY(ego_data.state);
    ego_odometry.pose.pose.position.z = perception_msgs::object_access::getZ(ego_data.state);
    ego_odometry.pose.pose.orientation = perception_msgs::object_access::getOrientation(ego_data.state);
    ego_odometry.pose.covariance = perception_msgs::object_access::getPoseWithCovariance(ego_data.state).covariance;
    ego_odometry.twist.twist.linear = perception_msgs::object_access::getVelocity(ego_data.state);
    ego_odometry.twist.twist.angular.z = perception_msgs::object_access::getYawRate(ego_data.state);
    pub_ego_odometry_->publish(ego_odometry);
  }

  if (publish_ego_vehicle_state_) {
    pm::ObjectState ego_vehicle_state;
    perception_msgs::object_access::initializeState(ego_vehicle_state, pm::EGO::MODEL_ID);
    ego_vehicle_state.header = ego_data.state.header;
    if (perception_msgs::object_access::hasSteeringAngleAck(ego_data.state.model_id)) {
      perception_msgs::object_access::setSteeringAngleAck(
          ego_vehicle_state, perception_msgs::object_access::getSteeringAngleAck(ego_data.state));
    }
    if (perception_msgs::object_access::hasSteeringAngleRateAck(ego_data.state.model_id)) {
      perception_msgs::object_access::setSteeringAngleRateAck(
          ego_vehicle_state, perception_msgs::object_access::getSteeringAngleRateAck(ego_data.state));
    }
    pub_ego_vehicle_state_->publish(ego_vehicle_state);
  }
}

void SimulationAdapter::objectListCallback(const pm::ObjectList::ConstSharedPtr& msg) {
  auto timeout = rclcpp::Duration::from_seconds(1.0);

  // Option A: transform object_list to fixed_frame_id
  pm::ObjectList msg_object_list_fixed;

  gm::TransformStamped to_map_tf;
  try {
    to_map_tf = tf2_buffer_->lookupTransform(fixed_frame_id_, msg->header.frame_id, msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transformation from '%s' to '%s' is not available.", msg->header.frame_id.c_str(),
                fixed_frame_id_.c_str());
    return;
  }
  tf2::doTransform(*msg, msg_object_list_fixed, to_map_tf);

  // publish object list in fixed_frame_id
  pub_object_list_fixed_->publish(msg_object_list_fixed);

  if (!publish_vehicle_frame_tf_) {
    return;
  }

  // Option B: transform object list to vehicle_frame_id_
  pm::ObjectList msg_object_list;

  gm::TransformStamped to_vehicle_frame_tf;
  try {
    to_vehicle_frame_tf =
        tf2_buffer_->lookupTransform(vehicle_frame_id_, msg->header.frame_id, msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transformation from '%s' to '%s' is not available", msg->header.frame_id.c_str(),
                vehicle_frame_id_.c_str());
    return;
  }
  tf2::doTransform(*msg, msg_object_list, to_vehicle_frame_tf);

  // publish object_list in vehicle_frame_id
  pub_object_list_->publish(msg_object_list);
}

void SimulationAdapter::initializeVehicleFrameTransform() {
  if (!publish_vehicle_frame_tf_) {
    return;
  }

  /* set up a transformation link between fixed_frame_id and vehicle_frame_id

           /        utm_<zone>     \
          /                         \
         / static                    \ static (published by lanelet2_map_server)
        /  (published by              \
       /     rspecific simulation      \
    simulation_fixed_frame_id         fixed_frame_id
      |
      dynamic (published by specific simulation)
      |
      v
    simulation_vehicle_frame_id ---static---> vehicle_frame_id
  */

  auto timezero = tf2::TimePointZero;

  try {
    // check if desired transformation is already defined
    tf2_buffer_->lookupTransform(vehicle_frame_id_, fixed_frame_id_, timezero);
    if (tf_init_timer_ && !tf_init_timer_->is_canceled()) {
      tf_init_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Static transformation from '%s' to '%s' is now available",
                  vehicle_frame_id_.c_str(), fixed_frame_id_.c_str());
    }
    return;
  } catch (const tf2::TransformException&) {
    RCLCPP_DEBUG(this->get_logger(), "Transformation from '%s' to '%s' not yet available, retrying ...",
                 fixed_frame_id_.c_str(), vehicle_frame_id_.c_str());

    // step 1: simulation_fixed_frame_id -> fixed_frame_id
    try {
      tf2_buffer_->lookupTransform(fixed_frame_id_, simulation_fixed_frame_id_, timezero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(this->get_logger(),
                  "Transformation from '%s' to '%s' is not available. Should be provided using a shared parent "
                  "utm frame.",
                  simulation_fixed_frame_id_.c_str(), fixed_frame_id_.c_str());
      RCLCPP_WARN(this->get_logger(), "\tSkipped ...");
      return;
    }

    // step 2: simulation_fixed_frame_id -> simulation_vehicle_frame_id
    try {
      tf2_buffer_->lookupTransform(simulation_vehicle_frame_id_, simulation_fixed_frame_id_, timezero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(this->get_logger(), "\tTransformation from '%s' to '%s' not available", simulation_fixed_frame_id_.c_str(),
                  simulation_vehicle_frame_id_.c_str());
      RCLCPP_WARN(this->get_logger(), "\tSkipped ...");
      return;
    }

    // step 3: simulation_vehicle_frame_id -> vehicle_frame_id
    try {
      tf2_buffer_->lookupTransform(vehicle_frame_id_, simulation_vehicle_frame_id_, timezero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(this->get_logger(), "\tTransformation from '%s' to '%s' is not available",
                  simulation_vehicle_frame_id_.c_str(), vehicle_frame_id_.c_str());

      // publish static transformation from simulation_vehicle_frame_id to vehicle_frame_id
      gm::TransformStamped ego_vehicle_to_vehicle_frame;
      ego_vehicle_to_vehicle_frame.header.stamp = this->get_clock()->now();
      ego_vehicle_to_vehicle_frame.header.frame_id = simulation_vehicle_frame_id_;
      ego_vehicle_to_vehicle_frame.child_frame_id = vehicle_frame_id_;

      ego_vehicle_to_vehicle_frame.transform.translation.x = simulation_vehicle_frame_id_to_vehicle_frame_id_;
      ego_vehicle_to_vehicle_frame.transform.translation.y = 0.0;
      ego_vehicle_to_vehicle_frame.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      ego_vehicle_to_vehicle_frame.transform.rotation.x = q.x();
      ego_vehicle_to_vehicle_frame.transform.rotation.y = q.y();
      ego_vehicle_to_vehicle_frame.transform.rotation.z = q.z();
      ego_vehicle_to_vehicle_frame.transform.rotation.w = q.w();

      static_br_tf_->sendTransform(ego_vehicle_to_vehicle_frame);
      RCLCPP_INFO(this->get_logger(), "\tTransformation from '%s' to '%s' was published",
                  simulation_vehicle_frame_id_.c_str(), vehicle_frame_id_.c_str());
    }
  }
}

void SimulationAdapter::trajectoryCallback(const tp::Trajectory::ConstSharedPtr& msg) {
  if (msg->type_id != trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID) {
    RCLCPP_WARN(this->get_logger(),
                "Invalid trajectory type, planned trajectory states are only filled for trajectories of type DRIVABLE");
    return;
  }

  // transform trajectory to fixed_frame_id frame
  try {
    trajectory_planned_ = tf2_buffer_->transform(*msg, fixed_frame_id_, tf2::durationFromSec(0.01));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Trajectory could not be transformed from '%s' to '%s'",
                msg->header.frame_id.c_str(), fixed_frame_id_.c_str());
    return;
  }
}


}  // namespace simulation_adapter
