#include <CarlaItsAdapterNode.h>


namespace carla {

ItsAdapter::ItsAdapter() : Node("CarlaItsAdapter") {  
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // create client for lanelet2 map change
  client_ = this->create_client<lanelet2_map_server_interfaces::srv::ChangeMapParams>("/ll2_map_server/change_map_parameters");

  rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
  qosLatching.transient_local();
  qosLatching.reliable();

  // setup subscriber
  sub_world_info_ = this->create_subscription<cm::CarlaWorldInfo>("/carla/world_info", qosLatching, std::bind(&ItsAdapter::worldInfoCallback, this, std::placeholders::_1));
  sub_its_converter_ = this->create_subscription<pi::ObjectList>("/carla_its_converter/object_list/carla_map", 1, std::bind(&ItsAdapter::itsConverterCallback, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<nm::Odometry>("/carla/ego_vehicle/odometry", 1, std::bind(&ItsAdapter::odometryCallback, this, std::placeholders::_1));

  // setup publisher
  pub_objects_map_ = this->create_publisher<pi::ObjectList>("~/object_list/map", 1);
  pub_objects_base_link_ = this->create_publisher<pi::ObjectList>("~/object_list/base_link", 1);

  // load Parameters and if not successful, return
  if(!loadParameters()) return;

  ROS_LOG_STREAM(INFO, "CarlaItsAdapter running...");  
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
    ROS_LOG_STREAM(ERROR, "OpenDRIVE-Map is not valid. Longitude is needed.");
    return;
  }

  // convert CARLA map name to lanelet map name
  std::string map_filenpath = msg->map_name;
  size_t pos = map_filenpath.find("Carla/Maps");
  if (pos != std::string::npos)
    map_filenpath.replace(pos, 10, "/data/maps");
  else {
    ROS_LOG_STREAM(ERROR, "Wrong format of CARLA map name");
    return;
  }
  map_filenpath += ".osm";

  std::string map_frame_id = "map";
  // TODO set rosparams instead of service call
  // this->set_parameter(rclcpp::Parameter("map_filepath", map_filenpath));
  // this->set_parameter(rclcpp::Parameter("map_frame_id", map_frame_id));
  // this->set_parameter(rclcpp::Parameter("origin_lat", latValue));
  // this->set_parameter(rclcpp::Parameter("origin_lon", lonValue));

  // lanelet service call to change map of lanelet2 map server
  auto request = std::make_shared<lanelet2_map_server_interfaces::srv::ChangeMapParams::Request>();
  request->map_filename = map_filenpath;
  request->map_frame_id = map_frame_id;
  request->origin_lat = std::stod(latValue);
  request->origin_lon = std::stod(lonValue);

  // check if service is available and send request
  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to call service ChangeMapParams");
  } else {
    auto result = client_->async_send_request(request);
  }
}

void ItsAdapter::itsConverterCallback(const pi::ObjectList::ConstPtr &msg){
  auto timeout = rclcpp::Duration::from_seconds(1.0);

  // transform the object list from carla_map to map frame
  if(!tf2_buffer_->_frameExists("carla_map")){
    ROS_LOG_STREAM(WARN, "Frame 'carla_map' does not exist");
    return;
  }

  pi::ObjectList msg_object_list_map;
  gm::TransformStamped carla_map_to_map_tf;
  try {
    carla_map_to_map_tf = tf2_buffer_->lookupTransform("map", "carla_map", msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    ROS_LOG_STREAM(WARN, "Tranformation from 'carla_map' to 'map' is not available. No transformed object list could be published.");
    return;
  }
  tf2::doTransform(*msg, msg_object_list_map, carla_map_to_map_tf);

  // publish object list in map frame
  pub_objects_map_->publish(msg_object_list_map);
  
  // transform the object list from carla_map to base_link frame
  if(!tf2_buffer_->_frameExists("base_link")){
    ROS_LOG_STREAM(WARN, "Frame 'base_link' does not exist");
    return;
  }

  pi::ObjectList msg_object_list_base_link;
  gm::TransformStamped carla_map_to_base_link_tf;
  try {
    carla_map_to_base_link_tf = tf2_buffer_->lookupTransform("base_link", "carla_map", msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    ROS_LOG_STREAM(WARN, "Tranformation from 'carla_map' to 'base_link' is not available");
    return;
  }
  tf2::doTransform(*msg, msg_object_list_base_link, carla_map_to_base_link_tf);

  if(fov_range_){
    // Only consider objects that are within the fov_range
    pi::ObjectList msg_object_list_base_link_filtered;
    msg_object_list_base_link_filtered.header = msg_object_list_base_link.header;
    for (size_t i = 0; i < msg_object_list_base_link.objects.size(); i++) {
      double x = oa::getX(msg_object_list_base_link.objects[i]);
      double y = oa::getY(msg_object_list_base_link.objects[i]);
      if (sqrt(x*x + y*y) <= fov_range_) {
        msg_object_list_base_link_filtered.objects.push_back(msg_object_list_base_link.objects[i]);
      }
    }
    // publish objectList in base_link frame within fov_range
    pub_objects_base_link_->publish(msg_object_list_base_link_filtered);
  } else {
    // publish objectList in base_link frame
    pub_objects_base_link_->publish(msg_object_list_base_link);
  }
}

void ItsAdapter::odometryCallback(const nm::Odometry::ConstPtr &msg) 
{
  // set up a transformation link between map and base_link
  // map -> carla_map -> ego_vehicle -> base_link

  auto timezero = tf2::TimePointZero;

  try
  {
    // check if transformation is already defined
    gm::TransformStamped transform;
    transform = tf2_buffer_->lookupTransform("base_link", "map", timezero);
  }
  catch(const tf2::TransformException& e)
  {
    ROS_LOG_STREAM(WARN, "Tranformation from 'map' to 'base_link' not available");
    static tf2_ros::StaticTransformBroadcaster static_br_tf_(this);

    // broadcast transformation between map and carla_map (always 0)
    tf2::Transform map_carla_map_tf;
    map_carla_map_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    map_carla_map_tf.setRotation(tf2::Quaternion(1.0, 0.0, 0.0, 0.0));

    gm::TransformStamped map_carla_map;
    tf2::convert(map_carla_map.transform, map_carla_map_tf);
    map_carla_map.header.stamp = this->get_clock()->now();
    map_carla_map.header.frame_id = "carla_map";
    map_carla_map.child_frame_id = "map";

    static_br_tf_.sendTransform(map_carla_map);

    // check if transformation between carla_map and ego_vehicle is available
    if(!tf2_buffer_->_frameExists("carla_map")){
      ROS_LOG_STREAM(WARN, "Frame 'carla_map' does not exist");
      return;
    }

    try
    {
      gm::TransformStamped carla_ego_vehicle;
      carla_ego_vehicle = tf2_buffer_->lookupTransform("ego_vehicle", "carla_map", timezero);
    }
    catch(const tf2::TransformException& e)
    {
      ROS_LOG_STREAM(WARN, "Tranformation from 'carla_map' to 'ego_vehicle' not available");
      return;
    }

    // broadcast transformation between ego_vehicle and base_link if it does not exist
    try {
      gm::TransformStamped base_link_ego_vehicle_transform;
      base_link_ego_vehicle_transform = tf2_buffer_->lookupTransform("ego_vehicle", "base_link", timezero);   
    } catch (const tf2::TransformException& e) {
      tf2::Transform ego_vehicle_base_link_tf;
      ego_vehicle_base_link_tf.setOrigin(tf2::Vector3(center_to_baselink_, 0.0, 0.0));
      ego_vehicle_base_link_tf.setRotation(tf2::Quaternion(1.0, 0.0, 0.0, 0.0));

      gm::TransformStamped ego_vehicle_base_link;
      tf2::convert(ego_vehicle_base_link.transform, ego_vehicle_base_link_tf);
      ego_vehicle_base_link.header.stamp = this->get_clock()->now();
      ego_vehicle_base_link.header.frame_id = "ego_vehicle";
      ego_vehicle_base_link.child_frame_id = "base_link";

      static_br_tf_.sendTransform(ego_vehicle_base_link);
    }

    ROS_LOG_STREAM(INFO, "Published static transform between map and base_link successfully");
  }

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

