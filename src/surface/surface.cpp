#include "surface.hpp"
#include <pcl/point_cloud.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace road_slope_detection_ros_tool {

/**
 * Initialization
 */
Surface::Surface(const ros::NodeHandle& nhPrivate) : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    interface_.fromParamServer();

    /*
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/Surface.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&Surface::reconfigureCallback, this, _1, _2));
    interface_.dummy_subscriber->registerCallback(&Surface::messageCallback, this);

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
void Surface::messageCallback(const Msg::ConstPtr& msg) {
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
void Surface::reconfigureCallback(const Interface::Config& config, uint32_t /*level*/) {
    interface_.fromConfig(config);
}



void resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                pcl::PointCloud<pcl::PointNormal>::Ptr mls_points)
{
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList (*cloud) << ").";
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.3);

    // Reconstruct
    mls.process (*mls_points);
    std::cerr << "PointCloud after filtering: " << mls_points->width * mls_points->height
              << " data points (" << pcl::getFieldsList (*mls_points) << ")."<<std::endl;
}

void greedy_triangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,
                          pcl::PolygonMesh::Ptr triangles)
{
    // Create search tree*

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);

    tree2->setInputCloud (cloud_with_normals);
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;




    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (500);
    std::cout<< "4"<<std::endl;

    // Set typical values for the parameters
    gp3.setMu (100);
    gp3.setMaximumNearestNeighbors (1570);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    std::cout<< "6"<<std::endl;

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (*triangles);
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    std::cout<< "get triangle"<<std::endl;

    /*
    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0); //设置背景
    viewer->addPolygonMesh(*triangles,"my"); //设置显示的网格
    //viewer->addCoordinateSystem (1.0); //设置坐标系
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    */

}

void
PointCloud2Vector3d (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
    for (unsigned i = 0; i < cloud->size (); i++)
    {
        pcl::PointXYZ &p = cloud->at (i);
        if (!std::isnan (p.x) && !std::isnan (p.y) && !std::isnan (p.z))
            data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
    }
}
void  bspline_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh::Ptr mesh) {
    const clock_t begin_time = clock();
    //pcl::visualization::PCLVisualizer viewer("B-spline surface fitting");
    pcl::on_nurbs::NurbsDataSurface data;
    PointCloud2Vector3d(cloud, data.interior);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cloud, 0, 255, 0);
    //viewer.addPointCloud<pcl::PointXYZ>(cloud, handler, "cloud_cylinder");
    //printf("  %lu points in data set\n", cloud->size());

    // ############################################################################
    // fit B-spline surface

    // parameters

    unsigned order(2);
    unsigned refinement(3);
    //unsigned iterations (10);
    unsigned mesh_resolution(100);

    pcl::on_nurbs::FittingSurface::Parameter params;
    params.interior_smoothness = 0.2;
    params.interior_weight = 1.0;
    params.boundary_smoothness = 0.2;
    params.boundary_weight = 0.0;

    // initialize
    printf("  surface fitting ...\n");
    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
    //std::cout << nurbs.m << std::endl;
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);
    //  fit.setQuiet (false); // enable/disable debug output

    // mesh for visualization
    //pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> mesh_vertices;
    std::string mesh_id = "mesh_nurbs";
    //pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
    //viewer.addPolygonMesh(mesh, mesh_id);
    //viewer.spin ();

    // surface refinement
    for (unsigned i = 0; i < refinement; i++) {
        fit.refine(0);
        fit.refine(1);
        fit.assemble(params);
        fit.solve();
        //pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices,
         //                                                     mesh_resolution);
        //viewer.updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
        //viewer.spinOnce();
        std::cout << "time" << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;
    }
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, *mesh, mesh_resolution);
    std::cout << "time" << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;

    ON_3dPoint point;
    ON_3dVector normal;
    ON_3dVector ds;
    ON_3dVector dt;

    double max0 = fit.m_nurbs.Domain(0).Max();
    double max1 = fit.m_nurbs.Domain(1).Max();
    double min0 = fit.m_nurbs.Domain(0).Min();
    double min1 = fit.m_nurbs.Domain(1).Min();

    int x = 0;
    const int RESOLUTION_U = 20;
    const int RESOLUTION_V = RESOLUTION_U;

    for (int u = 0; u < RESOLUTION_U; u++) {
        for (int v = 0; v < RESOLUTION_V; v++) {
            fit.m_nurbs.EvNormal(min0 + (u / (double) RESOLUTION_U) * max0,
                                 min1 + (v / (double) RESOLUTION_V) * max1,
                                 point, ds, dt, normal);

            ON_3dPoint end = point + normal;

            pcl::PointXYZ p1(point.x, point.y, point.z);
            pcl::PointXYZ p2(end.x, end.y, end.z);


            //viewer.addLine(p1, p2, "line" + std::to_string(x));
            x++;
        }
    }
    std::cout << "time" << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;

    printf("  ... done.\n");

    //viewer.spin();

}

} // namespace road_slope_detection_ros_tool