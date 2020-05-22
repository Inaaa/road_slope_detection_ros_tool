#include "ring_keeper.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <isotream>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);


    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);



    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    cloud1->header=point_cloud->header;



    pcl::fromPCLPointCloud2( *cloud, *point_cloud);
    //std::cout<<"poincloud"<<point_cloud->points.size()<< std::endl;



    for (int i = 0; i < point_cloud->points.size (); ++i)
    {
        if (point_cloud->points[i].x > 0 && point_cloud->points[i].x <50)
        {
            if (point_cloud->points[i].y > -20 && point_cloud->points[i].y <20)
            cloud1->push_back(point_cloud->points[i]);
        }

    }



    sensor_msgs::PointCloud2 lidarl_new;

    pcl::toROSMsg(*cloud1,lidarl_new);
    std::cout<<point_cloud->header << std::endl;
    lidarl_new.header.frame_id = point_cloud->header.frame_id;
    //lidarl_new.header.stamp.now() ;
    //std::cout<<lidarl_new.header << std::endl;

    if (lidarl_new.width >20){
    std::cout <<"width"<< lidarl_new.width<<std::endl;
    // Publish the data.
    pub.publish (lidarl_new);}

}


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "ring_keeper_node");

    //road_slope_detection_ros_tool::RingKeeper ring_keeper(ros::NodeHandle("~"));

    ros::NodeHandle nh;
    std::cout<<"!!!!!!!!!!!!!!!!"<<std::endl;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/preproc/lidar/velodyne/fl/cartesian/points", 1, cloud_cb);
    std::cout<<"***********"<<std::endl;

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("lidarl_new", 1);

    ros::spin();
    return EXIT_SUCCESS;
}
