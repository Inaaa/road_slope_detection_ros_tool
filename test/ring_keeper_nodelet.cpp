// google test docs:
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples

#include <ros/ros.h>
#include "gtest/gtest.h"

// Because node and nodelet usually offer the same functionality, the nodelet test is only here to test
// whether the nodelet starts and stops correcly. All the unittests can go to the node test.
TEST(road_slope_detection_ros_tool, testRingKeeperNodeletInitialization) {
    auto initDelay = ros::NodeHandle("~").param("init_delay", 3.);
    ros::Duration(initDelay).sleep();
    ASSERT_TRUE(ros::ok()) << "Ros crashed or the nodelet failed to initialize!";
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ring_keeper_nodelet_test");
    // The async spinner lets you publish and receive messages during the tests, no need to call spinOnce()
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
