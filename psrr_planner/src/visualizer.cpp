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

void Visualizer::renderPathFootprints(
    std::vector<geometry_msgs::PolygonStamped> &polygons) {
  for (auto &p : polygons) {
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
  }

  jsk_recognition_msgs::PolygonArray footprint_arr;
  footprint_arr.header.frame_id = "map";
  footprint_arr.header.stamp = ros::Time::now();
  footprint_arr.polygons.swap(polygons);

  path_footprints_pub_.publish(footprint_arr);
}

}  // namespace visualizer
}  // namespace psrr_planner

// publish markers
// if (planner_markers_pub_.getNumSubscribers()) {
//   visualization_msgs::MarkerArray markers;

//   // edge markers
//   visualization_msgs::Marker edges_marker;
//   edges_marker.header.frame_id = "map";
//   edges_marker.header.stamp = ros::Time::now();
//   edges_marker.ns = "edges";
//   edges_marker.id = 0;
//   edges_marker.action = visualization_msgs::Marker::ADD;
//   edges_marker.type = visualization_msgs::Marker::LINE_LIST;
//   edges_marker.pose.orientation.w = 1.0;
//   edges_marker.scale.x = 0.01;

//   std_msgs::ColorRGBA color;
//   color.r = 1.0;
//   color.g = 0.0;
//   color.b = 0.0;
//   color.a = 1.0;

//   for (const auto edge : *(planner_->getEdges())) {
//     geometry_msgs::Point p1;
//     p1.x = static_cast<double>(edge.first->state.x);
//     p1.y = static_cast<double>(edge.first->state.y);
//     p1.z = 0.0;
//     edges_marker.points.push_back(p1);
//     edges_marker.colors.push_back(color);
//     geometry_msgs::Point p2;
//     p2.x = static_cast<double>(edge.second->state.x);
//     p2.y = static_cast<double>(edge.second->state.y);
//     p2.z = 0.0;
//     edges_marker.points.push_back(p2);
//     edges_marker.colors.push_back(color);
//   }

//   markers.markers.emplace_back(std::move(edges_marker));

//   // if we are using informed rrt*
//   // we have an option to visualize the informed set and greedy informed
//   set
//   // only for 2D planar surface (future work will focus for general 3D
//   // robots)
//   if (planner_type_ == "informed_rrt_star" && planner_->hasSolution()) {
//     visualization_msgs::Marker informed_set_marker;
//     informed_set_marker.header.frame_id = "map";
//     informed_set_marker.header.stamp = ros::Time::now();
//     informed_set_marker.ns = "informed_set";
//     informed_set_marker.id = 1;
//     informed_set_marker.action = visualization_msgs::Marker::ADD;
//     informed_set_marker.type = visualization_msgs::Marker::CYLINDER;
//     informed_set_marker.scale.z = 0.01;

//     visualization_msgs::Marker greedy_informed_set_marker;
//     greedy_informed_set_marker.header.frame_id = "map";
//     greedy_informed_set_marker.header.stamp = ros::Time::now();
//     greedy_informed_set_marker.ns = "greedy_informed_set";
//     greedy_informed_set_marker.id = 2;
//     greedy_informed_set_marker.action = visualization_msgs::Marker::ADD;
//     greedy_informed_set_marker.type =
//     visualization_msgs::Marker::CYLINDER;
//     greedy_informed_set_marker.scale.z = 0.01;

//     // find transverse and conjugate diameters for scale.x and scale.y
//     auto transverse_diameter =
//         std::static_pointer_cast<InformedRRTStar>(planner_)
//             ->getTransverseDiameter();
//     auto conjugate_diameter =
//         std::static_pointer_cast<InformedRRTStar>(planner_)
//             ->getConjugateDiameter();

//     informed_set_marker.scale.x = transverse_diameter[0];
//     informed_set_marker.scale.y = conjugate_diameter[0];
//     greedy_informed_set_marker.scale.x = transverse_diameter[1];
//     greedy_informed_set_marker.scale.y = conjugate_diameter[1];

//     // find ellipse center
//     auto center = std::static_pointer_cast<InformedRRTStar>(planner_)
//                       ->getEllipseCenter();
//     informed_set_marker.pose.position.x = center[0];
//     informed_set_marker.pose.position.y = center[1];
//     greedy_informed_set_marker.pose.position.x = center[0];
//     greedy_informed_set_marker.pose.position.y = center[1];

//     // find ellipse orientation
//     const geometry_msgs::Quaternion orientation(
//         tf::createQuaternionMsgFromYaw(
//             std::static_pointer_cast<InformedRRTStar>(planner_)
//                 ->getEllipseOrientation()));
//     informed_set_marker.pose.orientation = orientation;
//     greedy_informed_set_marker.pose.orientation = orientation;

//     // update colors
//     std_msgs::ColorRGBA color;
//     color.r = 1.0;
//     color.g = 0.0;
//     color.b = 0.0;
//     color.a = 0.3;
//     informed_set_marker.color = color;

//     color.r = 0.0;
//     color.g = 0.0;
//     color.b = 1.0;
//     color.a = 0.3;
//     greedy_informed_set_marker.color = color;

//     markers.markers.emplace_back(std::move(informed_set_marker));
//     markers.markers.emplace_back(std::move(greedy_informed_set_marker));
//   }

//   planner_markers_pub_.publish(markers);
// }