#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "surface.hpp"

namespace road_slope_detection_ros_tool {

class SurfaceNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<Surface>(getPrivateNodeHandle());
    }
    std::unique_ptr<Surface> impl_;
};
} // namespace road_slope_detection_ros_tool

PLUGINLIB_EXPORT_CLASS(road_slope_detection_ros_tool::SurfaceNodelet, nodelet::Nodelet);
