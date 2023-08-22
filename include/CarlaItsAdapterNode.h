#pragma once

#include <memory>
#include <string>
#include <map>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#include <derived_object_msgs/msg/object_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <carla_msgs/msg/carla_world_info.hpp>
#include <lanelet2_map_server_interfaces/srv/change_map_params.hpp>


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

  private:
    bool loadParameters();
    void itsConverterObjectsCallback(const pi::ObjectList::ConstPtr &msg);
    void itsConverterEgoCallback(const pi::EgoData::ConstPtr &msg);
    void odometryCallback(const nm::Odometry::ConstPtr &msg);
    void worldInfoCallback(const cm::CarlaWorldInfo::ConstPtr &msg);

    rclcpp::Client<lanelet2_map_server_interfaces::srv::ChangeMapParams>::SharedPtr client_;

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;

    Subscriber<pi::ObjectList> sub_its_converter_objects_;
    Subscriber<pi::EgoData> sub_its_converter_egoData_;
    Subscriber<nm::Odometry> sub_odometry_;
    Subscriber<cm::CarlaWorldInfo> sub_world_info_;

    Publisher<pi::ObjectList> pub_objects_base_link_;
    Publisher<pi::ObjectList> pub_objects_map_;
    Publisher<pi::EgoData> pub_ego_data_base_link_;

    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    double center_to_baselink_;
    double fov_range_;

    double ego_veh_filter_thr_x_=0.5;
    double ego_veh_filter_thr_y_=0.5;
};

}  // end of namespace carla
