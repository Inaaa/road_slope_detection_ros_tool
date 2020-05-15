#include "surface.hpp"
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
#include <pcl/surface/gp3.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl/PolygonMesh.h>
#include <shape_msgs/Mesh.h>
#include <visualization_msgs/Marker.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/gp3.h>


#include <pcl/visualization/pcl_visualizer.h>//可视化


ros::Publisher pub;
void greedy_triangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,
                          pcl::PolygonMesh::Ptr triangles);
float slope(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

static bool meshToShapeMsg(const pcl::PolygonMesh& in, shape_msgs::Mesh& mesh);

static bool meshToMarkerMsg(const pcl::PolygonMesh& in, visualization_msgs::Marker& marker);


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

    pcl::fromPCLPointCloud2( *cloud, *point_cloud);
    std::cout<<"poincloud"<<point_cloud->points.size()<< std::endl;
    //float gradient = slope(point_cloud);
    //std::cout << "gradient" << gradient << std::endl;

    //pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>);
    //road_slope_detection_ros_tool::resampling(point_cloud,mls_points);



    //bspline
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    road_slope_detection_ros_tool::bspline_fitting(point_cloud, mesh);

    //greedy triangularation
    //pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    //road_slope_detection_ros_tool::greedy_triangulation(mls_points, triangles);


    visualization_msgs::Marker marker;
    meshToMarkerMsg(*mesh, marker);
    std::cout<<"+~~~~~~~~~~~"<<std::endl;


    // Publish the data
    pub.publish (marker);
    std::cout<<"publish successful"<<std::endl;

}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "surface_node");

    //road_slope_detection_ros_tool::Surface surface(ros::NodeHandle("~"));
    ros::NodeHandle nh;
    std::cout<<"!!!!!!!!!!!!!!!!"<<std::endl;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("filter_points", 1, cloud_cb);

    std::cout<<"***********"<<std::endl;

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<visualization_msgs::Marker> ("lidar_surface", 1);

    ros::spin();
    return EXIT_SUCCESS;
}



float slope(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.3, 0.3);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setLeafSize (3.0f, 2.0f, 0.1f);
    sor.getMinBoxCoordinates();
    sor.filter (*cloud_filtered2);

    int size = cloud_filtered2->size();
    float slope=0;
    if (size >=2)
    {


    std::vector <float> x;
    std::vector <float> z;
    for( int i = 0; i < size;i++)
    {
        std::cout<< cloud_filtered2->points[i] << std::endl;
        x.push_back(cloud_filtered2->points[i].x);
        z.push_back(cloud_filtered2->points[i].z);
    }
    std::vector<float>::iterator biggest = std::max_element(std::begin(z), std::end(z));
    auto h = std::distance(std::begin(z), biggest);
    float biggest_x = x.at(h);
    auto smallest = std::min_element(std::begin(z), std::end(z));
    float smallest_x = x.at(std::distance(std::begin(z), smallest));

    std::cout << *biggest<< " and big_x " << biggest_x << std::endl;
    std::cout << *smallest<< " and small_x " << smallest_x << std::endl;

    slope = (*biggest-*smallest)/(biggest_x-smallest_x);
    std::cout << slope << std::endl;
    }
    return slope;

}

static bool meshToShapeMsg(const pcl::PolygonMesh& in, shape_msgs::Mesh& mesh)
{
    pcl_msgs::PolygonMesh pcl_msg_mesh;

    pcl_conversions::fromPCL(in, pcl_msg_mesh);


    sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

    size_t size = pcd_modifier.size();

    mesh.vertices.resize(size);

    std::cout << "polys: " << pcl_msg_mesh.polygons.size() << " vertices: " << pcd_modifier.size() << "\n";

    sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

    for(size_t i = 0; i < size ; i++, ++pt_iter){
        mesh.vertices[i].x = pt_iter[0];
        mesh.vertices[i].y = pt_iter[1];
        mesh.vertices[i].z = pt_iter[2];
    }

    //ROS_INFO("Found %ld polygons", triangles.size());

    std::cout << "Updated vertices" << "\n";

    //BOOST_FOREACH(const Vertices polygon, triangles)

    mesh.triangles.resize(in.polygons.size());

    for (size_t i = 0; i < in.polygons.size(); ++i)
    {
        if(in.polygons[i].vertices.size() < 3)
        {
            ROS_WARN("Not enough points in polygon. Ignoring it.");
            continue;
        }

        //shape_msgs::MeshTriangle triangle = shape_msgs::MeshTriangle();
        //boost::array<uint32_t, 3> xyz = {{in.polygons[i].vertices[0], in.polygons[i].vertices[1], in.polygons[i].vertices[2]}};
        //triangle.vertex_indices = xyz;

        //mesh.triangles.push_back(shape_msgs::MeshTriangle());
        //mesh.triangles[i].vertex_indices.resize(3);

        for (int j = 0; j < 3; ++j)
            mesh.triangles[i].vertex_indices[j] = in.polygons[i].vertices[j];
    }
    return true;
}
static bool meshToMarkerMsg(const pcl::PolygonMesh& in, visualization_msgs::Marker& marker) {
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    //marker.header.frame_id = in.cloud.header.frame_id;
    marker.header.frame_id = "sensor/lidar/velodyne/fl";

    std::cout<< "id" << marker.header.frame_id << std::endl;
    marker.header.stamp = ros::Time::now();
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.id = 1;
    marker.action = visualization_msgs::Marker::ADD;


    shape_msgs::Mesh shape_msg_mesh;

    meshToShapeMsg(in, shape_msg_mesh);

    size_t size_triangles = shape_msg_mesh.triangles.size();

    marker.points.resize(size_triangles * 3);

    std::cout << "polys: " << size_triangles << " vertices: " << shape_msg_mesh.vertices.size() << "\n";

    size_t i = 0;

    for (size_t tri_index = 0; tri_index < size_triangles; ++tri_index) {

        /*
        std::cout << shape_msg_mesh.triangles[tri_index].vertex_indices[0] <<  " " <<
                     shape_msg_mesh.triangles[tri_index].vertex_indices[1] <<  " " <<
                     shape_msg_mesh.triangles[tri_index].vertex_indices[2] << "\n";
        */

        marker.points[i] = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[0]];
        marker.points[i + 1] = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[1]];
        marker.points[i + 2] = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[2]];
        i = i + 3;

    }
    return true;
}







