#include <CarlaItsAdapterNode.h>


namespace carla {

ItsAdapter::ItsAdapter() : Node("CarlaItsAdapter") {
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  
  client_ = this->create_client<lanelet2_map_server_interfaces::srv::ChangeMapParams>("/ll2_map_server/change_map_parameters");

  rclcpp::QoS qosLatching = rclcpp::QoS(rclcpp::KeepLast(1));
  qosLatching.transient_local();
  qosLatching.reliable();

  sub_world_info_ = this->create_subscription<carla_msgs::msg::CarlaWorldInfo>("/carla/world_info", qosLatching, std::bind(&ItsAdapter::worldInfoCallback, this, std::placeholders::_1));
  sub_its_converter_ = this->create_subscription<perception_interfaces::msg::ObjectList>("/carla_its_converter/objectList/carla_map", 1, std::bind(&ItsAdapter::itsConverterCallback, this, std::placeholders::_1));
  sub_odometry_ = this->create_subscription<nam::Odometry>("/carla/ego_vehicle/odometry", 1, std::bind(&ItsAdapter::odometryCallback, this, std::placeholders::_1));

  pub_objects_map_ = this->create_publisher<pin::ObjectList>("/carla_its_adapter/objectList/map", 1);
  pub_objects_base_link_ = this->create_publisher<pin::ObjectList>("/carla_its_adapter/objectList/base_link", 1);

  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // Load Parameters and if not successful, return
  if(!loadParameters()) return;

  ROS_LOG_STREAM(INFO, "CarlaItsAdapter running...");  
}

bool ItsAdapter::loadParameters() {
  // Load value parameters
  try {
    this->declare_parameter("fov_range", rclcpp::ParameterType::PARAMETER_DOUBLE);
    fov_range_ = this->get_parameter("fov_range").as_double();
  } catch (rclcpp::exceptions::InvalidParameterTypeException&) {
    ROS_LOG_STREAM(INFO, "Parameter \'fov_range\' is not set");
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    ROS_LOG_STREAM(INFO, "Parameter \'fov_range\' is not set");
  }
  this->declare_parameter("center_to_baselink", rclcpp::ParameterType::PARAMETER_DOUBLE);
  try {
    center_to_baselink_ = this->get_parameter("center_to_baselink").as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    ROS_LOG_STREAM(ERROR, "Parameter \'center_to_baselink\' is required");
    return false;
  }

  return true;
}

void ItsAdapter::worldInfoCallback(const carla_msgs::msg::CarlaWorldInfo::ConstPtr &msg){
  std::string opendrive_string = msg->opendrive;
  
  // size_t first_delim_pos = opendrive_string.find("<geoReference>");
  // size_t end_pos_of_first_delim = first_delim_pos;
  // size_t last_delim_pos = opendrive_string.find("</geoReference>");

  // std::string geoReference = opendrive_string.substr(end_pos_of_first_delim, last_delim_pos - end_pos_of_first_delim);

  std::string latValue;
  size_t latPos = opendrive_string.find("+lat_0=");
  if (latPos != std::string::npos) {
      size_t latValueStart = latPos + 7;  // Length of "+lat_0="
      size_t latValueEnd = opendrive_string.find(" ", latValueStart);
      latValue = opendrive_string.substr(latValueStart, latValueEnd - latValueStart);
      std::cout << "Latitude: " << latValue << std::endl;
  }

  std::string lonValue;
  size_t lonPos = opendrive_string.find("+lon_0=");
  if (lonPos != std::string::npos) {
      size_t lonValueStart = lonPos + 7;  // Length of "+lon_0="
      size_t lonValueEnd = opendrive_string.find(" ", lonValueStart);
      lonValue = opendrive_string.substr(lonValueStart, lonValueEnd - lonValueStart);
      std::cout << "Longitude: " << lonValue << std::endl;
  }

  // TODO Fehlermeldung, wenn nicht gesetzt oder gefunden


  // TODO get map and lat/long
  std::cout << msg->map_name << std::endl;
  std::cout << ll2if_->map_loaded_ << std::endl;

  // There shall be no more than one definition of the projection. 
  // If the definition is missing, a local Cartesian coordinate system is assumed.

  if(ll2if_->map_loaded_){
    lanelet::LaneletMapConstPtr llmap = ll2if_->getMapPtr();
    std::cout << "llmap" << std::endl;
    std::cout << llmap << std::endl;
    ROS_LOG_STREAM(INFO, llmap);
    lanelet::LaneletMapPtr nonconst_llmap = ll2if_->getNonConstMapPtr();
    std::shared_ptr<lanelet::Projector> proj = ll2if_->getProjectorPtr();
    // std::string ll2_map_frame_id = ll2if_->map_frame_id_;
    if(ll2if_->update_pending_){
      // The map provided by the server has changed!
      // Update the local map and projector variables
      llmap = ll2if_->getMapPtr();
      nonconst_llmap = ll2if_->getNonConstMapPtr();
      proj = ll2if_->getProjectorPtr();
      ll2if_->update_pending_ = false;
    }
  }

  std::string map_filenpath = "";
  std::string map_frame_id = "";
  // TODO set rosparam namespace lanelet
  // auto node = rclcpp::Node::make_shared("my_node");
  this->set_parameter(rclcpp::Parameter("/ll2_map_server/map_filepath", map_filenpath));
  // ros::param::set("/ll2_map_server/map_filepath", map_filenpath);
  // ros::param::set("/ll2_map_server/map_frame_id", map_frame_id);
  // ros::param::set("/ll2_map_server/origin_lat", origin_lat);
  // ros::param::set("/ll2_map_server/origin_lon", origin_lon);

  // TODO lanelet service aufrufen
  auto request = std::make_shared<lanelet2_map_server_interfaces::srv::ChangeMapParams::Request>();
  request->map_filename = ""; // TODO dict to map carlaMapName to laneletMapName
  request->map_frame_id = "map";
  request->origin_lat = std::stod(latValue);
  request->origin_lon = std::stod(lonValue);

  // Check if service is available
  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to call service ChangeMapParams");
  }
  auto result = client_->async_send_request(request);
}

void ItsAdapter::itsConverterCallback(const perception_interfaces::msg::ObjectList::ConstPtr &msg){
  auto timeout = rclcpp::Duration::from_seconds(1.0);

  // transform the object list from carla_map to map frame
  perception_interfaces::msg::ObjectList msg_object_list_map;
  gm::TransformStamped carla_map_to_map_tf;
  try {
    carla_map_to_map_tf = tf2_buffer_->lookupTransform("map", "carla_map", msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    ROS_LOG_STREAM(ERROR, "\"Exception caught: \" << ex.what()");
    return;
  }
  tf2::doTransform(*msg, msg_object_list_map, carla_map_to_map_tf);

  // publish object list in map frame
  pub_objects_map_->publish(msg_object_list_map);
  
  // transform the object list from carla_map to base_link frame
  perception_interfaces::msg::ObjectList msg_object_list_base_link;
  gm::TransformStamped carla_map_to_base_link_tf;
  try {
    carla_map_to_base_link_tf = tf2_buffer_->lookupTransform("base_link", "carla_map", msg->header.stamp, timeout);
  } catch (tf2::TransformException& ex) {
    ROS_LOG_STREAM(ERROR, "\"Exception caught: \" << ex.what()");
    return;
  }
  tf2::doTransform(*msg, msg_object_list_base_link, carla_map_to_base_link_tf);

  if(fov_range_){
  // Only consider objects that are within the fov_range
  perception_interfaces::msg::ObjectList msg_object_list_base_link_filtered;
  msg_object_list_base_link_filtered.header = msg_object_list_base_link.header;
  for (size_t i = 0; i < msg_object_list_base_link.objects.size(); i++) {
    double x = perception_interfaces::object_access::getX(msg_object_list_base_link.objects[i]);
    double y = perception_interfaces::object_access::getY(msg_object_list_base_link.objects[i]);
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

void ItsAdapter::odometryCallback(const nam::Odometry::ConstPtr &msg) 
{
  // Set up a transformation link between CARLA map and map
  auto timezero = tf2::TimePointZero;

  try
  {
    // check if transformation is already defined
    gm::TransformStamped transform;
    transform = tf2_buffer_->lookupTransform("base_link", "map", timezero);
  }
  catch(const tf2::TransformException& e)
  {
    static tf2_ros::StaticTransformBroadcaster static_br_tf_(this);

    // broadcast transformation between carla_map and map (always 0)
    tf2::Transform map_carla_map_tf;
    map_carla_map_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    map_carla_map_tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    geometry_msgs::msg::TransformStamped map_carla_map;
    tf2::convert(map_carla_map.transform, map_carla_map_tf);
    map_carla_map.header.stamp = this->get_clock()->now();
    map_carla_map.header.frame_id = "carla_map";
    map_carla_map.child_frame_id = "map";

    static_br_tf_.sendTransform(map_carla_map);


    // broadcast transformation between ego_vehicle and base_link
    tf2::Transform ego_vehicle_base_link_tf;
    ego_vehicle_base_link_tf.setOrigin(tf2::Vector3(1.0, 0.0, 0.0)); //TODO center_to_baselink_
    ego_vehicle_base_link_tf.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    geometry_msgs::msg::TransformStamped ego_vehicle_base_link;
    tf2::convert(ego_vehicle_base_link.transform, ego_vehicle_base_link_tf);
    ego_vehicle_base_link.header.stamp = this->get_clock()->now();
    ego_vehicle_base_link.header.frame_id = "ego_vehicle";
    ego_vehicle_base_link.child_frame_id = "base_link";

    static_br_tf_.sendTransform(ego_vehicle_base_link);


    // CARLA map to ego_vehicle transform
    tf2::Transform carla_ego_vehicle_tf;
    try
    {
      geometry_msgs::msg::TransformStamped carla_ego_vehicle;
      carla_ego_vehicle = tf2_buffer_->lookupTransform("ego_vehicle", "carla_map", timezero);
      tf2::convert(carla_ego_vehicle.transform, carla_ego_vehicle_tf);
    }
    catch(const tf2::TransformException& e)
    {
      return;
    }

    // gm::TransformStamped base_link_map_tf;
    // combine transformations
    // carla_map -> ego_vehicle -> base_link -> map
    // tf2::Transform base_link_map;
    // tf2::Transform carla_ego_map;
    // tf2::convert(base_link_map_tf.transform, base_link_map);
    // tf2::convert(carla_ego_vehicle_tf.transform, carla_ego_map);
    // tf2::Transform br_tf = base_link_map * ego_vehicle_base_link_tf * carla_ego_map;

    // map -> carla_map -> ego_vehicle -> base_link
  
    tf2::Transform map_base_link_tf = ego_vehicle_base_link_tf * carla_ego_vehicle_tf * map_carla_map_tf;
    map_base_link_tf = map_base_link_tf.inverse();

    // broadcast transformation between map and base_link
    geometry_msgs::msg::TransformStamped map_base_link;
    map_base_link.header.stamp = this->get_clock()->now();
    map_base_link.header.frame_id = "map";
    map_base_link.child_frame_id = "base_link";

    tf2::convert(map_base_link.transform, map_base_link_tf);

    // map_base_link.transform.translation = map_base_link_tf.getOrigin();
    // map_base_link.transform.rotation = map_base_link_tf.getRotation();

    static_br_tf_.sendTransform(map_base_link);

    ROS_LOG_STREAM(INFO, "Published static transform between map and base_link");

  }

}


}  // end of namespace


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<carla::ItsAdapter>();
  node->initializeMapInterface();
  rclcpp::spin(node);
  // rclcpp::spin(std::make_shared<carla::ItsAdapter>());
  rclcpp::shutdown();
  return 0;
}

