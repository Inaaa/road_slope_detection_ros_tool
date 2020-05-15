#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "road_slope_detection_ros_tool/SurfaceInterface.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>


namespace road_slope_detection_ros_tool {

    class Surface {
    public:
        using Interface = SurfaceInterface;
        using Msg = std_msgs::Header;

        explicit Surface(const ros::NodeHandle &nhPrivate);

    private:
        void messageCallback(const Msg::ConstPtr &msg);

        void reconfigureCallback(const Interface::Config &config, uint32_t /*level*/);

        Interface interface_;
        dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_{tfBuffer_};
        tf2_ros::TransformBroadcaster tfBroadcaster_;
    };

    void resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points);
    void greedy_triangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,
                              pcl::PolygonMesh::Ptr triangles);
    void
    PointCloud2Vector3d (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);

    void bspline_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh::Ptr mesh );

} // namespace road_slope_detection_ros_tool


