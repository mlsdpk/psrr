#include <psrr_planner/visualizer.h>

namespace psrr_planner {
namespace visualizer {

Visualizer::Visualizer(ros::NodeHandle &mt_nh, ros::NodeHandle &prv_nh) {
  // visualization related parameters

  // register publishers
  markers_pub_ = mt_nh.advertise<visualization_msgs::MarkerArray>(
      "/psrr_planner/markers", 10);
  path_footprints_pub_ = mt_nh.advertise<jsk_recognition_msgs::PolygonArray>(
      "/psrr_planner/path_footprints", 1);
}

Visualizer::~Visualizer() {}

}  // namespace visualizer
}  // namespace psrr_planner