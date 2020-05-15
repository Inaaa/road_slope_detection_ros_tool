#pragma once
#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "road_slope_detection_ros_tool/RingKeeperInterface.h"

namespace road_slope_detection_ros_tool {

class RingKeeper {
public:
    using Interface = RingKeeperInterface;
    using Msg = std_msgs::Header;

    explicit RingKeeper(const ros::NodeHandle& nhPrivate);

private:
    void messageCallback(const Msg::ConstPtr& msg);
    void reconfigureCallback(const Interface::Config& config, uint32_t /*level*/);

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace road_slope_detection_ros_tool
