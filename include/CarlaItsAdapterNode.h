#pragma once

#include <memory>
#include <string>
#include <map>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#include <derived_object_msgs/msg/object_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <perception_interfaces/object_access.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <carla_msgs/msg/carla_world_info.hpp>
#include <lanelet2_map_server_interfaces/srv/change_map_params.hpp>
#include <lanelet2_map_interface/lanelet2_map_interface.hpp>


#define ROS_LOG_STREAM(level, ...) RCLCPP_##level##_STREAM(this->get_logger(), __VA_ARGS__)

namespace nm = nav_msgs::msg;
namespace pi = perception_interfaces::msg;
namespace gm = geometry_msgs::msg;
namespace cm = carla_msgs::msg;
namespace oa = perception_interfaces::object_access;

template<typename T>
using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;
template<typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

namespace carla {

class ItsAdapter : public rclcpp::Node {

  public:
    ItsAdapter();

    // Initialization of Map-Interface
    void initializeMapInterface()
    {
      std::string map_server_name = "ll2_map_server";
      // Important: shared_from_this() can not be called from within the constructor
      ll2if_ = new LL2MapInterface(shared_from_this(), map_server_name);
    }

  private:
    void worldInfoCallback(const cm::CarlaWorldInfo::ConstPtr &msg);
    void itsConverterCallback(const pi::ObjectList::ConstPtr &msg);
    void odometryCallback(const nm::Odometry::ConstPtr &msg);
    bool loadParameters();

    rclcpp::Client<lanelet2_map_server_interfaces::srv::ChangeMapParams>::SharedPtr client_;
    LL2MapInterface *ll2if_;

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;

    Subscriber<nm::Odometry> sub_odometry_;
    Subscriber<cm::CarlaWorldInfo> sub_world_info_;
    Subscriber<pi::ObjectList> sub_its_converter_;

    Publisher<pi::ObjectList> pub_objects_map_;
    Publisher<pi::ObjectList> pub_objects_base_link_;

    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    double fov_range_;
    double center_to_baselink_;
};

}  // end of namespace carla
