#include "tf_tramsform.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <cstdio>
#include <vector>
#include <fstream>
#include <automated_driving_msgs/MotionState.h>

void cloud_cb(const automated_driving_msgs::MotionState& motionState)
{
    std::cout<< motionState.pose.pose.position.x << std::endl;

    std::cout<< motionState.pose.pose.position.y << std::endl;


    using namespace lanelet;

    std::string exampleMapPath = "./ludwigsburg_large.osm";

    lanelet::Origin origin{{49.01439, 8.41722, 0.}}; //地图起点

    //lanelet::projection::UtmProjector::Ptr projector = std::make_shared<lanelet::projection::UtmProjector>(origin);
    projection::UtmProjector projector(origin);
    LaneletMapPtr map = load(exampleMapPath, projector);
    // 加载出错时会抛出error
    ErrorMessages errors;
    map = load(exampleMapPath, projector, &errors);
    assert(errors.empty());
    std::string str;
    int i=0;
    //std::cout << iss  << std::endl;
    std::vector <std::pair<double, Lanelet>> actuallyNearestLanelets =
            geometry::findNearest(map->laneletLayer, BasicPoint2d(motionState.pose.pose.position.x, motionState.pose.pose.position.y), 5);  // 应该给入真实的地址
    assert(!actuallyNearestLanelets.empty());

    for (int j =0; j<5; j++)
    {
        // 将找出的lanelet的左边界存入vector points_left中
        int n = actuallyNearestLanelets[j].second.leftBound3d().size();
        std::vector<lanelet::BasicPoint3d> points_left;
        for (int i=0;i<n;i++)
        {
            auto point_l = actuallyNearestLanelets[j].second.leftBound3d()[i];

            std::cout << i <<"HIER!!!!!"<<std::endl;
            points_left.push_back(point_l);
            std::cout <<  points_left[i] << std::endl;
            //outFile << points_left[i].x()<<" "<< points_left[i].y() <<" " << points_left[i].z()<<"\n";
        }
        int n1 = actuallyNearestLanelets[j].second.rightBound3d().size();
        std::vector<lanelet::BasicPoint3d> points_right;

        for (int i=0;i<n1;i++)
        {
            auto point_r = actuallyNearestLanelets[j].second.rightBound3d()[i];
            //std::cout << i <<std::endl;
            points_right.push_back(point_r);
            std::cout <<  points_right[i] << std::endl;
            //outFile_r << points_right[i].x()<<" "<< points_right[i].y() <<" " << points_right[i].z()<<"\n";
        }
    }


}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "tf_tramsform_node");

    //road_slope_detection_ros_tool::TfTramsform tf_tramsform(ros::NodeHandle("~"));

    ros::NodeHandle nh;
    std::cout<<"!!!!!!!!!!!!!!!!"<<std::endl;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/fusion/motion_state", 1, cloud_cb);

    std::cout<<"***********"<<std::endl;

    // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<visualization_msgs::Marker> ("lidar_surface", 1)


    ros::spin();
    return EXIT_SUCCESS;
}


