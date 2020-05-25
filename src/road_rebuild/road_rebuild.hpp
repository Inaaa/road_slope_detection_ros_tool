#ifndef ROAD_REBUILD_HPP
#define ROAD_REBUILD_HPP
#pragma once
#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "road_slope_detection_ros_tool/RoadRebuildInterface.h"

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
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <boost/sort/spreadsort/integer_sort.hpp>

namespace road_slope_detection_ros_tool {

class RoadRebuild {
public:
    using Interface = RoadRebuildInterface;
    using Msg = std_msgs::Header;

    explicit RoadRebuild(const ros::NodeHandle& nhPrivate);

private:
    void messageCallback(const Msg::ConstPtr& msg);
    void reconfigureCallback(const Interface::Config& config, uint32_t /*level*/);

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};


using Array4size_t = Eigen::Array<std::size_t, 4, 1>;

class CustomVoxelGrid: public pcl::VoxelGrid<pcl::PCLPointCloud2> {

    using PCLPointCloud2 = pcl::PCLPointCloud2;
    using PCLPointCloud2Ptr = PCLPointCloud2::Ptr;
    using PCLPointCloud2ConstPtr = PCLPointCloud2::ConstPtr;

    void applyFilter (pcl::PCLPointCloud2 &output) override;
};


void voxel_grid (pcl::PCLPointCloud2Ptr cloud,
                 pcl::PCLPointCloud2Ptr cloud_filtered);

} // namespace road_slope_detection_ros_tool
#endif

