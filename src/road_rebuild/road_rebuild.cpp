#include "road_rebuild.hpp"
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>// 移动立方体算法
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/visualization/pcl_visualizer.h>//可视化
#include <boost/thread/thread.hpp>//多线程
#include <string.h>
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/don.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>




namespace road_slope_detection_ros_tool {

/**
 * Initialization
 */
RoadRebuild::RoadRebuild(const ros::NodeHandle& nhPrivate) : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    interface_.fromParamServer();

    /*
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/RoadRebuild.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&RoadRebuild::reconfigureCallback, this, _1, _2));
    interface_.dummy_subscriber->registerCallback(&RoadRebuild::messageCallback, this);

    rosinterface_handler::showNodeInfo();

    /*
     * The preferred way of logging is to call the logging functions of the interface object.
     * These also work for nodelets and are better than using ROS_DEBUG (etc.) macros.
     */
    interface_.logDebug("Node initialized.");
}

/**
 * This callback is called whenever a new message arrives on the subscriber's topic.
 * To make this work, the callback has to be registered in the constructor above.
 */
void RoadRebuild::messageCallback(const Msg::ConstPtr& msg) {
    // Transform the message to the reference frame
    Msg msgTransformed;
    try {
        // Msg is just a std_msgs::Header so the transformation currently makes no sense:
        // msgTransformed = tfBuffer_.transform(*msg, interface_.reference_frame);
    } catch (tf2::TransformException& e) {
        interface_.logDebug("Failed to transform the message to " + interface_.reference_frame);
        return;
    }
    // TODO: do stuff to process the message
    Msg::Ptr newMsg = boost::make_shared<Msg>(*msg);
    interface_.dummy_publisher.publish(newMsg);
}

/**
 * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window.
 * The parameter "level" is unused here. It is set to the number of changes in the config.
 * At startup, the callback is automatically called with level == std::numeric_limits<uint32_t>::max().
 */
void RoadRebuild::reconfigureCallback(const Interface::Config& config, uint32_t /*level*/) {
    interface_.fromConfig(config);
}

} // namespace road_slope_detection_ros_tool

