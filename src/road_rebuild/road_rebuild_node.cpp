#include "road_rebuild.hpp"
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
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


ros::Publisher pub;

void radius_filter(pcl::PCLPointCloud2Ptr cloud,
                   pcl::PCLPointCloud2Ptr cloud_filtered);

void voxel_grid (pcl::PCLPointCloud2Ptr cloudPtr,
                 pcl::PCLPointCloud2Ptr cloud_filteredPtr);
void passthrough(pcl::PCLPointCloud2Ptr cloud,
                 pcl::PCLPointCloud2Ptr cloud_filtered);

void
cloud_cb (const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudPtr(cloud);
    pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloud_filteredPtr(cloud2);
    pcl::PCLPointCloud2Ptr cloud_filteredPtr2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2Ptr cloud_filteredPtr3(new pcl::PCLPointCloud2());

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloudPtr);

    // filter pointcloud
    radius_filter(cloudPtr, cloud_filteredPtr);
    passthrough(cloud_filteredPtr, cloud_filteredPtr2);
    voxel_grid(cloud_filteredPtr2, cloud_filteredPtr3);


    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filteredPtr3, output);


    // Publish the data
    pub.publish (output);
    std::cout<<"publish successful"<<std::endl;
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "road_rebuild_node");
    //road_slope_detection_ros_tool::RoadRebuild road_rebuild(ros::NodeHandle("~"));
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/road_pcl", 1, cloud_cb);


    pub = nh.advertise<sensor_msgs::PointCloud2> ("/filter_points", 1);


    std::cout<<"####################"<<std::endl;

    ros::spin();
    std::cout << "7" << std::endl;

    return EXIT_SUCCESS;
    std::cout << "8" << std::endl;

}


void radius_filter(pcl::PCLPointCloud2Ptr cloud,
                   pcl::PCLPointCloud2Ptr cloud_filtered)
{

    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius (5);
    // apply filter
    outrem.filter (*cloud_filtered);

}



void voxel_grid (pcl::PCLPointCloud2Ptr cloudPtr,
                 pcl::PCLPointCloud2Ptr cloud_filteredPtr)
{

    //std::cerr << "voxel_PointCloud before filtering: " << cloud->width * cloud->height << std::endl;
    std::cout<< "!!!!!" <<std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.getMinBoxCoordinates();
    sor.filter (*cloud_filteredPtr);

    //std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
    //          << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<< std::endl;

    //pcl::PCDWriter writer;
    //writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered,
    //              Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

}

void passthrough(pcl::PCLPointCloud2Ptr cloud,
                 pcl::PCLPointCloud2Ptr cloud_filtered)
{
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.3, 0.3);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

}


