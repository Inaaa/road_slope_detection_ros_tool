#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "ring_keeper.hpp"

namespace road_slope_detection_ros_tool {

class RingKeeperNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<RingKeeper>(getPrivateNodeHandle());
    }
    std::unique_ptr<RingKeeper> impl_;
};
} // namespace road_slope_detection_ros_tool

PLUGINLIB_EXPORT_CLASS(road_slope_detection_ros_tool::RingKeeperNodelet, nodelet::Nodelet);
