#include <CarlaItsAdapterNode.h>


namespace carla {

// Constants
const std::string ItsAdapter::kInputTopicTrajectory{"~/trajectory_topic"};

ItsAdapter::ItsAdapter() : Node("CarlaItsAdapter") {

  // load Parameters and if not successful, return
  if(!loadParameters()) return;
  

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

  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
  qosLatching.transient_local();
  qosLatching.reliable();

  // setup subscriber
  sub_world_info_ = this->create_subscription<cm::CarlaWorldInfo>("/carla/world_info", qosLatching, std::bind(&ItsAdapter::worldInfoCallback, this, std::placeholders::_1));
  sub_its_converter_objects_ = this->create_subscription<pi::ObjectList>("/carla_its_converter/ego_vehicle/objects", 1, std::bind(&ItsAdapter::itsConverterObjectsCallback, this, std::placeholders::_1));
  sub_its_converter_egoData_ = this->create_subscription<pi::EgoData>("/carla_its_converter/ego_vehicle/ego_data", 1, std::bind(&ItsAdapter::itsConverterEgoCallback, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<nm::Odometry>("/carla/ego_vehicle/odometry", 1, std::bind(&ItsAdapter::odometryCallback, this, std::placeholders::_1));
  sub_trajectory_ = this->create_subscription<tp::Trajectory>(kInputTopicTrajectory, 1, std::bind(&ItsAdapter::trajectoryCallback, this, std::placeholders::_1));

  // setup publisher
  pub_objects_map_ = this->create_publisher<pi::ObjectList>("~/object_list/map", 1);
  pub_objects_base_link_ = this->create_publisher<pi::ObjectList>("~/object_list/base_link", 1);
  pub_ego_data_ = this->create_publisher<pi::EgoData>("~/ego_data", 1);

  ROS_LOG_STREAM(INFO, "carla_its_adapter running...");
}

bool ItsAdapter::loadParameters() {
  // load value parameters
  try {
    this->declare_parameter("fov_range", rclcpp::ParameterType::PARAMETER_DOUBLE);
    fov_range_ = this->get_parameter("fov_range").as_double();
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    ROS_LOG_STREAM(INFO, "Parameter \'fov_range\' is not set correctly");
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    ROS_LOG_STREAM(INFO, "Parameter \'fov_range\' is not set");
  }
  this->declare_parameter("center_to_baselink", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    center_to_baselink_ = this->get_parameter("center_to_baselink").as_double();
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    ROS_LOG_STREAM(INFO, "Parameter \'center_to_baselink\' is not set correctly");
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    ROS_LOG_STREAM(ERROR, "Parameter \'center_to_baselink\' is required");
    return false;
  }

  this->declare_parameter("ego_veh_filter_thr_x", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    ego_veh_filter_thr_x_ = this->get_parameter("ego_veh_filter_thr_x").as_double();
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    ROS_LOG_STREAM(WARN, "Parameter \'ego_veh_filter_thr_x\' is not set correctly, using default value: "+std::to_string(ego_veh_filter_thr_x_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    ROS_LOG_STREAM(WARN, "Parameter \'ego_veh_filter_thr_x\' is not set, using default value: "+std::to_string(ego_veh_filter_thr_x_));
  }

  this->declare_parameter("ego_veh_filter_thr_y", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    ego_veh_filter_thr_y_ = this->get_parameter("ego_veh_filter_thr_y").as_double();
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    ROS_LOG_STREAM(WARN, "Parameter \'ego_veh_filter_thr_y\' is not set correctly, using default value: "+std::to_string(ego_veh_filter_thr_y_));
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    ROS_LOG_STREAM(WARN, "Parameter \'ego_veh_filter_thr_y\' is not set, using default value: "+std::to_string(ego_veh_filter_thr_y_));
  }

  this->declare_parameter("map_server_name", map_server_name_);
  map_server_name_ = this->get_parameter("map_server_name").as_string();

  return true;
}

void ItsAdapter::worldInfoCallback(const cm::CarlaWorldInfo::ConstPtr &msg){
  // get CARLA world info to set the correct map for the lanelet2 map server so that the maps and the origin of maps are equal

  // derive latitude and longitude from OpenDRIVE file
  std::string opendrive_string = msg->opendrive;

  std::string latValue;
  size_t latPos = opendrive_string.find("+lat_0=");
  if (latPos != std::string::npos) {
      size_t latValueStart = latPos + 7;  // length of "+lat_0="
      size_t latValueEnd = opendrive_string.find(" ", latValueStart);
      latValue = opendrive_string.substr(latValueStart, latValueEnd - latValueStart);
  } else {
    ROS_LOG_STREAM(ERROR, "OpenDRIVE-Header is invalid. Latitude is required.");
    return;
  }

  std::string lonValue;
  size_t lonPos = opendrive_string.find("+lon_0=");
  if (lonPos != std::string::npos) {
      size_t lonValueStart = lonPos + 7;  // length of "+lon_0="
      size_t lonValueEnd = opendrive_string.find(" ", lonValueStart);
      lonValue = opendrive_string.substr(lonValueStart, lonValueEnd - lonValueStart);
  } else {
    ROS_LOG_STREAM(ERROR, "OpenDRIVE-Header is invalid. Longitude is required.");
    return;
  }

  // convert CARLA map name to lanelet map name
  std::string map_filepath = msg->map_name;
  size_t pos = map_filepath.find("Carla/Maps");
  if (pos != std::string::npos)
    map_filepath.replace(pos, 10, "/data/maps");
  else {
    ROS_LOG_STREAM(ERROR, "Wrong format of CARLA map name");
    return;
  }
  map_filepath += ".osm";
  std::string map_frame_id = "map";

  // change map by setting map server parameters
  auto set_parameters_results = map_server_parameters_client_->set_parameters(
    {
      rclcpp::Parameter("map_filepath", map_filepath),
      rclcpp::Parameter("map_frame_id", map_frame_id),
      rclcpp::Parameter("origin_lat", std::stod(latValue)),
      rclcpp::Parameter("origin_lon", std::stod(lonValue))
    },
    [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      auto results = future.get();
      for (const auto& result : results) {
        if (!result.successful)
          RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
      }
      RCLCPP_INFO(this->get_logger(), "Finished setting map server parameters");
    });
}

void ItsAdapter::itsConverterEgoCallback(const pi::EgoData::ConstPtr &msg){
  auto timeout = rclcpp::Duration::from_seconds(1.0);
  gm::TransformStamped rear_axle_ground_position_in_carla_map_tf, rear_axle_ground_position_in_map_tf, carla_map_to_map_tf;

  // transform ego_data (input header is carla_map, output header is map)
  pi::EgoData ego_data = *msg;

  // get rear_axle_ground position in carla_map
  try {
    rear_axle_ground_position_in_carla_map_tf = tf2_buffer_->lookupTransform(msg->header.frame_id, "base_link", msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    ROS_LOG_STREAM(WARN, "Transformation from 'base_link' to '" + msg->header.frame_id + "' is not available. No transformed ego-data could be published.");
    return;
  }

  // get transform from carla_map to map
  try {
    carla_map_to_map_tf = tf2_buffer_->lookupTransform("map", msg->header.frame_id, msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    ROS_LOG_STREAM(WARN, "Transformation from '" + msg->header.frame_id + "' to 'map' is not available. No transformed ego-data could be published.");
    return;
  }

  // convert rear_axle_ground position from carla_map to map
  tf2::doTransform(rear_axle_ground_position_in_carla_map_tf, rear_axle_ground_position_in_map_tf, carla_map_to_map_tf);

  // set transformed ego_data header frames
  ego_data.header.frame_id = rear_axle_ground_position_in_map_tf.header.frame_id;
  ego_data.state.header.frame_id = rear_axle_ground_position_in_map_tf.header.frame_id;

  oa::setX(ego_data, rear_axle_ground_position_in_map_tf.transform.translation.x);
  oa::setY(ego_data, rear_axle_ground_position_in_map_tf.transform.translation.y);
  oa::setZ(ego_data, rear_axle_ground_position_in_map_tf.transform.translation.z);
  oa::setOrientation(ego_data, rear_axle_ground_position_in_map_tf.transform.rotation);
  ego_data.state.reference_point.value = pi::ObjectReferencePoint::REAR_AXLE_GROUND;
  ego_data.state.reference_point.translation_to_geometric_center.x = -center_to_baselink_;
  ego_data.state.reference_point.translation_to_geometric_center.z = msg->height/2.0;

  // add planned trajectory to ego_data
  int nSamplePoints = trajectory_planning_msgs::trajectory_access::getSamplePointSize(planned_trajectory_);
  if (planned_trajectory_.type_id == trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID and nSamplePoints > 0){
    ego_data.trajectory_planned.clear();
    pi::ObjectState object_state;

    // initialize state
    object_state.model_id = 1;
    std::vector<double> zeros(13, 0.0);
    std::vector<long int> zero(1, 0);
    perception_msgs::object_access::setContinuousState(object_state, zeros);
    perception_msgs::object_access::setDiscreteState(object_state, zero);
    perception_msgs::object_access::setStandstill(object_state, trajectory_planning_msgs::trajectory_access::getStandstill(planned_trajectory_));
    object_state.reference_point = ego_data.state.reference_point;

    float time;

    for (int i=0; i<nSamplePoints; i++){
      // update header stamp
      object_state.header = planned_trajectory_.header;
      time = trajectory_planning_msgs::trajectory_access::getT(planned_trajectory_, i);
      if (time >= 1){
        object_state.header.stamp.sec += (int) time;
        object_state.header.stamp.nanosec += (time - (int) time) * 1e9;
      } else {
        object_state.header.stamp.nanosec += time * 1e9;
      }

      // update trajectory state
      zeros[0] = trajectory_planning_msgs::trajectory_access::getX(planned_trajectory_, i);
      zeros[1] = trajectory_planning_msgs::trajectory_access::getY(planned_trajectory_, i);
      zeros[3] = trajectory_planning_msgs::trajectory_access::getV(planned_trajectory_, i);
      zeros[5] = trajectory_planning_msgs::trajectory_access::getA(planned_trajectory_, i);
      zeros[9] = trajectory_planning_msgs::trajectory_access::getTheta(planned_trajectory_, i);
      perception_msgs::object_access::setContinuousState(object_state, zeros);
      ego_data.trajectory_planned.push_back(object_state);    
    }
  }

  // publish object list in map frame
  pub_ego_data_->publish(ego_data);
}

void ItsAdapter::itsConverterObjectsCallback(const pi::ObjectList::ConstPtr &msg){
  auto timeout = rclcpp::Duration::from_seconds(1.0);

  // transform the object list to map frame
  if(!tf2_buffer_->_frameExists(msg->header.frame_id)){
    ROS_LOG_STREAM(WARN, "Frame '"+msg->header.frame_id+"' does not exist");
    return;
  }

  pi::ObjectList msg_object_list_map;

  gm::TransformStamped carla_map_to_map_tf;
  try {
    carla_map_to_map_tf = tf2_buffer_->lookupTransform("map", msg->header.frame_id, msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    ROS_LOG_STREAM(WARN, "Tranformation from 'carla_map' to 'map' is not available. No transformed object list could be published.");
    return;
  }
  tf2::doTransform(*msg, msg_object_list_map, carla_map_to_map_tf);

  // publish object list in map frame
  pub_objects_map_->publish(msg_object_list_map);

  // transform the object list to base_link frame
  if(!tf2_buffer_->_frameExists("base_link")){
    ROS_LOG_STREAM(WARN, "Frame 'base_link' does not exist");
    return;
  }

  pi::ObjectList msg_object_list_base_link;
  gm::TransformStamped carla_map_to_base_link_tf;
  try {
    carla_map_to_base_link_tf = tf2_buffer_->lookupTransform("base_link", msg->header.frame_id, msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    ROS_LOG_STREAM(WARN, "Tranformation from '"+msg->header.frame_id+"' to 'base_link' is not available");
    return;
  }
  tf2::doTransform(*msg, msg_object_list_base_link, carla_map_to_base_link_tf);

  pi::ObjectList msg_object_list_base_link_filtered;
  msg_object_list_base_link_filtered.header = msg_object_list_base_link.header;
  if(fov_range_){
    // only consider objects that are within the fov_range
    for (size_t i = 0; i < msg_object_list_base_link.objects.size(); i++) {
      double x = oa::getX(msg_object_list_base_link.objects[i]);
      double y = oa::getY(msg_object_list_base_link.objects[i]);
      if (sqrt(x*x + y*y) <= fov_range_) {
        // Filter Ego-Object from List
        if (std::abs(std::abs(x)-std::abs(center_to_baselink_)) > ego_veh_filter_thr_x_ || std::abs(y) > ego_veh_filter_thr_y_) {
          msg_object_list_base_link_filtered.objects.push_back(msg_object_list_base_link.objects[i]);
        }
      }
    }
    // publish filtered objectList in base_link frame
    pub_objects_base_link_->publish(msg_object_list_base_link_filtered);
  } else {
    // Filter Ego-Object from List
    for (size_t i = 0; i < msg_object_list_base_link.objects.size(); i++) {
      double x = oa::getX(msg_object_list_base_link.objects[i]);
      double y = oa::getY(msg_object_list_base_link.objects[i]);
      if (std::abs(std::abs(x)-std::abs(center_to_baselink_)) < ego_veh_filter_thr_x_ && std::abs(y) < ego_veh_filter_thr_y_) {
        msg_object_list_base_link.objects.erase(msg_object_list_base_link.objects.begin() + i);
        break;
      }
    }
    // publish objectList in base_link frame
    pub_objects_base_link_->publish(msg_object_list_base_link);
  }

}

void ItsAdapter::odometryCallback(const nm::Odometry::ConstPtr &msg)
{
  // set up a transformation link between map and base_link

  //       /     utm_<zone>  \
  //      /                   \
  //     / static              \ static (published by lanelet2_map_server)
  //    /  (published by        \
  //   /     ros-bridge)         \
  // carla_map                   map
  //   |
  //   dynamic (published by ros-bridge)
  //   |
  //   v
  // ego_vehicle ---static---> base_link

  auto timezero = tf2::TimePointZero;

  try
  {
    // check if desired transformation is already defined
    gm::TransformStamped transform;
    transform = tf2_buffer_->lookupTransform("base_link", "map", timezero);
  }
  catch(const tf2::TransformException& e)
  {
    ROS_LOG_STREAM(WARN, "Tranformation from 'map' to 'base_link' is not available");
    static tf2_ros::StaticTransformBroadcaster static_br_tf_(this);

    // step 1: carla_map -> map
    try
    {
      tf2_buffer_->lookupTransform("map", "carla_map", timezero);
    }
    catch(const tf2::TransformException& e)
    {
      ROS_LOG_STREAM(WARN, "Tranformation from 'carla_map' to 'map' is not available. Should be provided using a shared parent utm frame.");
      ROS_LOG_STREAM(WARN, "\tSkipped ...");
      return;
    }

    // step 2: carla_map -> ego_vehicle
    try
    {
      tf2_buffer_->lookupTransform("ego_vehicle", "carla_map", timezero);
    }
    catch(const tf2::TransformException& e)
    {
      ROS_LOG_STREAM(WARN, "\tTranformation from 'carla_map' to 'ego_vehicle' not available");
      ROS_LOG_STREAM(WARN, "\tSkipped ...");
      return;
    }

    // step 3: ego_vehicle -> base_link
    try {
      tf2_buffer_->lookupTransform("base_link", "ego_vehicle", timezero);
    }
    catch (const tf2::TransformException& e)
    {
      ROS_LOG_STREAM(WARN, "\tTranformation from 'ego_vehicle' to 'base_link' is not available");

      // transformation between map and carla_map is always 0
      gm::TransformStamped ego_vehicle_base_link;
      ego_vehicle_base_link.header.stamp = this->get_clock()->now();
      ego_vehicle_base_link.header.frame_id = "ego_vehicle";
      ego_vehicle_base_link.child_frame_id = "base_link";

      ego_vehicle_base_link.transform.translation.x = center_to_baselink_;
      ego_vehicle_base_link.transform.translation.y = 0.0;
      ego_vehicle_base_link.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      ego_vehicle_base_link.transform.rotation.x = q.x();
      ego_vehicle_base_link.transform.rotation.y = q.y();
      ego_vehicle_base_link.transform.rotation.z = q.z();
      ego_vehicle_base_link.transform.rotation.w = q.w();

      static_br_tf_.sendTransform(ego_vehicle_base_link);
      ROS_LOG_STREAM(INFO, "\tTranformation from 'ego_vehicle' to 'base_link' was published");
    }

    ROS_LOG_STREAM(INFO, "Static transformation from 'base_link' to 'map' is now available");
  }

}

void ItsAdapter::trajectoryCallback(const tp::Trajectory &msg)
{
  planned_trajectory_ = msg;
}


}  // end of namespace


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<carla::ItsAdapter>();
  // node->initializeMapInterface();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

