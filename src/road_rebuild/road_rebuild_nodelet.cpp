#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "road_rebuild.hpp"

namespace road_slope_detection_ros_tool {

class RoadRebuildNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<RoadRebuild>(getPrivateNodeHandle());
    }
    std::unique_ptr<RoadRebuild> impl_;
};
} // namespace road_slope_detection_ros_tool

PLUGINLIB_EXPORT_CLASS(road_slope_detection_ros_tool::RoadRebuildNodelet, nodelet::Nodelet);
