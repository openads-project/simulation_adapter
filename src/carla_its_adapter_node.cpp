#include <carla_its_adapter_node.h>

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(carla_its_adapter::CarlaItsAdapterNode)

/**
 * @brief Namespace for carla_its_adapter package
 *
 */

namespace carla_its_adapter {

/**
 * @brief Creates a CarlaItsAdapterNode node
 *
 */
CarlaItsAdapterNode::CarlaItsAdapterNode(const rclcpp::NodeOptions& options) : Node("carla_its_adapter_node", options) {
  this->declareAndLoadParameter("map_server_name", map_server_name_, "Name of the map server.");
  this->declareAndLoadParameter("set_ll2_map_from_carla", set_ll2_map_from_carla_,
                                "Automatically set the ll2 map based on the CARLA map.");
  this->declareAndLoadParameter("carla_fixed_frame_id", carla_fixed_frame_id_, "Name of the fixed frame id in CARLA.");
  this->declareAndLoadParameter("fixed_frame_id", fixed_frame_id_, "Name of the fixed frame id over time.");
  this->declareAndLoadParameter("carla_vehicle_frame_id", carla_vehicle_frame_id_,
                                "Name of the vehicle frame id in CARLA.");
  this->declareAndLoadParameter("vehicle_frame_id", vehicle_frame_id_, "Name of the vehicle frame id.");

  this->declareAndLoadParameter("carla_vehicle_frame_id_to_vehicle_frame_id", carla_vehicle_frame_id_to_vehicle_frame_id_,
                                "Longitudinal offset from carla_vehicle_frame_id to vehicle_frame_id.");

  this->setup();
}

/**
 * @brief Destroys a CarlaItsAdapterNode node
 *
 */
CarlaItsAdapterNode::~CarlaItsAdapterNode() {}

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
void CarlaItsAdapterNode::declareAndLoadParameter(const std::string& name,
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
rcl_interfaces::msg::SetParametersResult CarlaItsAdapterNode::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s'", param.get_name().c_str());
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
void CarlaItsAdapterNode::setup() {
  // initialize tf2 buffer and listener
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  current_map_name_ = "";

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
      std::bind(&CarlaItsAdapterNode::parametersCallback, this, std::placeholders::_1));

  // define custom QoS for world info topic
  rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
  qosLatching.transient_local();
  qosLatching.reliable();

  // setup subscriber for input topics
  sub_world_info_ = this->create_subscription<cm::CarlaWorldInfo>(
      kInputWorldInfoTopic, qosLatching,
      std::bind(&CarlaItsAdapterNode::worldInfoCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_world_info_->get_topic_name());

  sub_ego_data_ = this->create_subscription<pm::EgoData>(
      kInputEgoDataTopic, 1, std::bind(&CarlaItsAdapterNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_ego_data_->get_topic_name());

  sub_object_list_ = this->create_subscription<pm::ObjectList>(
      kInputObjectListTopic, 1, std::bind(&CarlaItsAdapterNode::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_object_list_->get_topic_name());

  sub_odometry_ = this->create_subscription<nm::Odometry>(
      kInputOdometryTopic, 1, std::bind(&CarlaItsAdapterNode::odometryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_odometry_->get_topic_name());

  sub_trajectory_ = this->create_subscription<tp::Trajectory>(
      kInputTrajectoryTopic, 1, std::bind(&CarlaItsAdapterNode::trajectoryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", sub_trajectory_->get_topic_name());

  // set up publisher for output topics
  pub_ego_data_ = this->create_publisher<pm::EgoData>(kEgoDataTopic, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_ego_data_->get_topic_name());

  pub_object_list_ = this->create_publisher<pm::ObjectList>(kObjectListTopic, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_object_list_->get_topic_name());

  pub_object_list_fixed_ = this->create_publisher<pm::ObjectList>(kObjectListFixedTopic, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", pub_object_list_fixed_->get_topic_name());

  // logging info
  RCLCPP_INFO(this->get_logger(), "carla_its_adapter_node is running...");
}

/**
 * @brief Set map for the lanelet2 map server based on the CarlaWorldInfo message.
 *
 * @param msg CarlaWorldInfo message.
 */

void CarlaItsAdapterNode::worldInfoCallback(const cm::CarlaWorldInfo::ConstSharedPtr msg) {

  std::map<std::string, std::string> map_files = {
    {"Town10HD", "/data/maps/locations/synthetic-carla/lanelet2/town10hd/Town10HD.osm"},
    {"aldenhoven", "/data/maps/locations/germany-aldenhoven-atc/lanelet2/unicaragil-atlatec/ATC_demo_2024-05-24.osm"},
    {"ika-test-track", "/data/maps/locations/germany-aachen-campusmelaten/lanelet2/ika-testtrack/ika-testtrack-autoshuttle.osm"},
  };

  if (msg->map_name == current_map_name_) return;
  current_map_name_ = msg->map_name;

  std::string lanelet_map_name;
  if (set_ll2_map_from_carla_) {
    // derive latitude and longitude from OpenDRIVE file
    std::string opendrive_string = msg->opendrive;

    std::string lat;
    size_t latPos = opendrive_string.find("+lat_0=");
    if (latPos != std::string::npos) {
      size_t latValueStart = latPos + 7;  // length of "+lat_0="
      size_t latValueEnd = opendrive_string.find(" ", latValueStart);
      lat = opendrive_string.substr(latValueStart, latValueEnd - latValueStart);
    } else {
      RCLCPP_ERROR(this->get_logger(), "OpenDRIVE-Header is invalid. Latitude is required.");
      return;
    }

    std::string lon;
    size_t lonPos = opendrive_string.find("+lon_0=");
    if (lonPos != std::string::npos) {
      size_t lonValueStart = lonPos + 7;  // length of "+lon_0="
      size_t lonValueEnd = opendrive_string.find(" ", lonValueStart);
      lon = opendrive_string.substr(lonValueStart, lonValueEnd - lonValueStart);
    } else {
      RCLCPP_ERROR(this->get_logger(), "OpenDRIVE-Header is invalid. Longitude is required.");
      return;
    }

    // convert carla map name to lanelet map name
    std::smatch match;
    std::string carla_map_name;

    // check if the string matches the default pattern
    std::regex pattern_default_map(R"(Carla/Maps/([^/]+))");
    if (std::regex_match(current_map_name_, match, pattern_default_map)) {
      carla_map_name = match[1];
    }

    // check if the string matches the custom pattern
    std::regex pattern_custom_map(R"((.+)/Maps/([^/]+)/\2)");
    if (std::regex_match(current_map_name_, match, pattern_custom_map)) {
      carla_map_name = match[2];
    }

    if (carla_map_name.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Wrong format of CARLA map name");
      return;
    }

    // get lanelet2 map name from dict
    lanelet_map_name = map_files[carla_map_name];

    // change map by setting map server parameters
    auto set_parameters_results = map_server_parameters_client_->set_parameters(
      {rclcpp::Parameter("map_filepath", lanelet_map_name),
       rclcpp::Parameter("map_frame_id", fixed_frame_id_), 
       rclcpp::Parameter("origin_lat", std::stod(lat)),
       rclcpp::Parameter("origin_lon", std::stod(lon))},
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

void CarlaItsAdapterNode::egoDataCallback(const pm::EgoData::ConstSharedPtr msg) {
  auto timeout = rclcpp::Duration::from_seconds(1.0);

  gm::TransformStamped vehicle_frame_position_in_carla_map_tf, vehicle_frame_position_in_map_tf, carla_map_to_map_tf;

  // transform ego_data (input header is carla_fixed_frame_id, output header is fixed_frame_id)
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

  // update reference point only if vehicle_frame_id is base_link
  if (vehicle_frame_id_ == "base_link") {
    try {
      vehicle_frame_position_in_map_tf =
          tf2_buffer_->lookupTransform(ego_data.header.frame_id, vehicle_frame_id_, msg->header.stamp, timeout);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Transformation from '%s' to '%s' is not available. No transformed ego-data could be published.",
                  vehicle_frame_id_.c_str(), ego_data.header.frame_id.c_str());
      return;
    }

    // set transformed ego_data header frames
    ego_data.header.frame_id = vehicle_frame_position_in_map_tf.header.frame_id;
    ego_data.state.header.frame_id = vehicle_frame_position_in_map_tf.header.frame_id;

    perception_msgs::object_access::setX(ego_data, vehicle_frame_position_in_map_tf.transform.translation.x);
    perception_msgs::object_access::setY(ego_data, vehicle_frame_position_in_map_tf.transform.translation.y);
    perception_msgs::object_access::setZ(ego_data, vehicle_frame_position_in_map_tf.transform.translation.z);
    perception_msgs::object_access::setOrientation(ego_data, vehicle_frame_position_in_map_tf.transform.rotation);
    ego_data.state.reference_point.value = pm::ObjectReferencePoint::REAR_AXLE_GROUND;
    ego_data.state.reference_point.translation_to_geometric_center.x = -carla_vehicle_frame_id_to_vehicle_frame_id_;
    ego_data.state.reference_point.translation_to_geometric_center.z = msg->height / 2.0;
  }

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
}

void CarlaItsAdapterNode::objectListCallback(const pm::ObjectList::ConstSharedPtr msg) {
  auto timeout = rclcpp::Duration::from_seconds(1.0);

  // Option A: transform object_list to fixed_frame_id
  pm::ObjectList msg_object_list_map;

  gm::TransformStamped to_map_tf;
  try {
    to_map_tf = tf2_buffer_->lookupTransform(fixed_frame_id_, msg->header.frame_id, msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transformation from '%s' to '%s' is not available.", msg->header.frame_id.c_str(),
                fixed_frame_id_.c_str());
    return;
  }
  tf2::doTransform(*msg, msg_object_list_map, to_map_tf);

  // publish object list in fixed_frame_id
  pub_object_list_fixed_->publish(msg_object_list_map);

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

void CarlaItsAdapterNode::odometryCallback(const nm::Odometry::ConstSharedPtr msg) {
  /* set up a transformation link between fixed_frame_id and vehicle_frame_id

           /        utm_<zone>     \
          /                         \
         / static                    \ static (published by lanelet2_map_server)
        /  (published by              \
       /     ros-bridge)               \
    carla_fixed_frame_id               fixed_frame_id
      |
      dynamic (published by carla-ros-bridge)
      |
      v
    carla_vehicle_frame_id ---static---> vehicle_frame_id
  */

  auto timezero = tf2::TimePointZero;

  try {
    // check if desired transformation is already defined
    gm::TransformStamped transform;
    transform = tf2_buffer_->lookupTransform(vehicle_frame_id_, fixed_frame_id_, timezero);
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN(this->get_logger(), "Transformation from '%s' to '%s' is not available", fixed_frame_id_.c_str(),
                vehicle_frame_id_.c_str());
    static tf2_ros::StaticTransformBroadcaster static_br_tf_(this);

    // step 1: carla_fixed_frame_id -> fixed_frame_id
    try {
      tf2_buffer_->lookupTransform(fixed_frame_id_, carla_fixed_frame_id_, timezero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(this->get_logger(),
                  "Transformation from '%s' to '%s' is not available. Should be provided using a shared parent "
                  "utm frame.",
                  carla_fixed_frame_id_.c_str(), fixed_frame_id_.c_str());
      RCLCPP_WARN(this->get_logger(), "\tSkipped ...");
      return;
    }

    // step 2: carla_fixed_frame_id -> carla_vehicle_frame_id
    try {
      tf2_buffer_->lookupTransform(carla_vehicle_frame_id_, carla_fixed_frame_id_, timezero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(this->get_logger(), "\tTransformation from '%s' to '%s' not available", carla_fixed_frame_id_.c_str(),
                  carla_vehicle_frame_id_.c_str());
      RCLCPP_WARN(this->get_logger(), "\tSkipped ...");
      return;
    }

    // step 3: carla_vehicle_frame_id -> vehicle_frame_id
    try {
      tf2_buffer_->lookupTransform(vehicle_frame_id_, carla_vehicle_frame_id_, timezero);
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN(this->get_logger(), "\tTransformation from '%s' to '%s' is not available",
                  carla_vehicle_frame_id_.c_str(), vehicle_frame_id_.c_str());

      // publish static transformation from carla_vehicle_frame_id to vehicle_frame_id
      gm::TransformStamped ego_vehicle_to_vehicle_frame;
      ego_vehicle_to_vehicle_frame.header.stamp = this->get_clock()->now();
      ego_vehicle_to_vehicle_frame.header.frame_id = carla_vehicle_frame_id_;
      ego_vehicle_to_vehicle_frame.child_frame_id = vehicle_frame_id_;

      ego_vehicle_to_vehicle_frame.transform.translation.x = carla_vehicle_frame_id_to_vehicle_frame_id_;
      ego_vehicle_to_vehicle_frame.transform.translation.y = 0.0;
      ego_vehicle_to_vehicle_frame.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      ego_vehicle_to_vehicle_frame.transform.rotation.x = q.x();
      ego_vehicle_to_vehicle_frame.transform.rotation.y = q.y();
      ego_vehicle_to_vehicle_frame.transform.rotation.z = q.z();
      ego_vehicle_to_vehicle_frame.transform.rotation.w = q.w();

      static_br_tf_.sendTransform(ego_vehicle_to_vehicle_frame);
      RCLCPP_INFO(this->get_logger(), "\tTransformation from '%s' to '%s' was published",
                  carla_vehicle_frame_id_.c_str(), vehicle_frame_id_.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Static transformation from '%s' to '%s' is now available",
                vehicle_frame_id_.c_str(), fixed_frame_id_.c_str());
  }
}

void CarlaItsAdapterNode::trajectoryCallback(const tp::Trajectory::ConstSharedPtr msg) {
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

}  // namespace carla_its_adapter
