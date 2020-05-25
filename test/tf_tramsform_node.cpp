// Google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples

#include <ros/ros.h>
#include "gtest/gtest.h"

// This module test should only test the ros functionality: does it run, does it respond correctly to messages.
// The actual functionality should be tested in the library that implements it.
// Any test acts like a node and can receive and send messages to your nodes/nodelets to test their behaviour
// You can even control time by setting "use_sim_time" in the .test-file and advertising the /clock topic.
// Keep in mind that:
// - your node needs some time to initialize and subscribe to your topics
// - if you control /clock, nothing might happen until you send the first clock message
// - it takes time until a node responds to your messages
// - utils_testing_ros provides functionality to make your testing life easier

// TEST(road_slope_detection_ros_tool, testTfTramsform){
//    your test goes here
// }


// This test just ensures that the node can be launched sucessfully. You can remove this if you have a better test.
TEST(road_slope_detection_ros_tool, testTfTramsformNodeInitialization) {
    auto initDelay = ros::NodeHandle("~").param("init_delay", 3.);
    ros::Duration(initDelay).sleep();
    ASSERT_TRUE(ros::ok()) << "Ros crashed or the node failed to initialize!";
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tf_tramsform_test");
    // The async spinner lets you publish and receive messages during the tests, no need to call spinOnce()
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    ros::shutdown();
    return ret;
}
