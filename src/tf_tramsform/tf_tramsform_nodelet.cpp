#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "tf_tramsform.hpp"

namespace road_slope_detection_ros_tool {

class TfTramsformNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<TfTramsform>(getPrivateNodeHandle());
    }
    std::unique_ptr<TfTramsform> impl_;
};
} // namespace road_slope_detection_ros_tool

PLUGINLIB_EXPORT_CLASS(road_slope_detection_ros_tool::TfTramsformNodelet, nodelet::Nodelet);
