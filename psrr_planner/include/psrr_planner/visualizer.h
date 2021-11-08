#pragma once

#include <jsk_recognition_msgs/PolygonArray.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// ompl related
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>

namespace psrr_planner {
namespace visualizer {

inline std_msgs::ColorRGBA getColorRed() {
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 1.0;
  return color;
};

inline std_msgs::ColorRGBA getColorGreen() {
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 1.0;
  return color;
};

inline std_msgs::ColorRGBA getColorBlue() {
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 1.0;
  return color;
};

class Visualizer {
 public:
  /**
   * @brief Constructor
   */
  Visualizer(ros::NodeHandle &mt_nh, ros::NodeHandle &prv_nh);

  /**
   * @brief Destructor
   */
  ~Visualizer();

  void renderPathFootprints(
      std::vector<geometry_msgs::PolygonStamped> &polygons);

 private:
  ros::Publisher markers_pub_;
  ros::Publisher path_footprints_pub_;
};

};  // namespace visualizer
};  // namespace psrr_planner