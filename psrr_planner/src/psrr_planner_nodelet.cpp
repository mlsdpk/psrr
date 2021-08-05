/******************************************************************************
Copyright (c) 2021, Phone Thiha Kyaw. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <nav_msgs/Path.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <psrr_msgs/FootPrint.h>
#include <psrr_msgs/Path.h>
#include <psrr_planner/collision_checker.h>
#include <psrr_planner/rrt.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <atomic>
#include <memory>
#include <mutex>

namespace psrr_planner {
class PsrrPlannerNodelet : public nodelet::Nodelet {
 public:
  PsrrPlannerNodelet() {}
  virtual ~PsrrPlannerNodelet() {}

  void onInit() override {
    NODELET_DEBUG("initializing psrr_planner_nodelet...");

    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // parameters
    private_nh_.param<double>("planner_update_interval",
                              planner_update_interval_, 0.002);
    private_nh_.param<double>("path_update_interval", path_update_interval_,
                              1.0);
    private_nh_.param<int>("max_vertices", max_vertices_, 1000);
    private_nh_.param<bool>("publish_path_footprints", publish_path_footprints_,
                            false);

    // subscribers

    // subscriber for receiving goal pose
    // we just simply subscribe to /move_base_simple/goal
    // (geometry_msgs/PoseStamped) for now
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1,
                              &PsrrPlannerNodelet::goalCB, this);

    // publishers
    tree_markers_pub_ = mt_nh_.advertise<visualization_msgs::MarkerArray>(
        "/psrr_planner/markers", 10);
    path_pub_ = mt_nh_.advertise<psrr_msgs::Path>("/psrr_planner/path", 1);
    path_2d_pub_ = mt_nh_.advertise<nav_msgs::Path>("/psrr_planner/path_2d", 1);
    footprint_arr_pub_ = mt_nh_.advertise<jsk_recognition_msgs::PolygonArray>(
        "/psrr_planner/path_footprints", 1);

    ////////////////////////////////////////////////////////////////////////////
    // Footprint of self-reconfigurable robots cannot be defined as static like
    // normal robot. As for now, user needs to provide the dynamically changing
    // footprint of the robot through ROS services. Why? Currently this
    // footprint is used during the SE(2) collison detection in a 2D occupancy
    // grid map. This planner will request the footprint of the robot as a
    // service client. Not sure this is the best way, but for now we'll stick
    // with this approach

    footprint_client_ = std::make_shared<ros::ServiceClient>(
        mt_nh_.serviceClient<psrr_msgs::FootPrint>("footprint"));

    // we wait here until we have a footprint server available
    ROS_INFO("Waiting footprint service...");
    footprint_client_->waitForExistence();
    ROS_INFO("Success: Connected to footprint service.");
    ////////////////////////////////////////////////////////////////////////////

    // TODO: User needs to provide initial footprint with ROS params which is
    // used for creating costmap

    // fixed hardcoded footprint for now
    auto p0 = geometry_msgs::Point();
    p0.x = 1.025;
    p0.y = -0.45;
    auto p1 = geometry_msgs::Point();
    p1.x = 1.025;
    p1.y = 0.45;
    auto p2 = geometry_msgs::Point();
    p2.x = -1.025;
    p2.y = 0.45;
    auto p3 = geometry_msgs::Point();
    p3.x = -1.025;
    p3.y = -0.45;
    footprint_.push_back(p0);
    footprint_.push_back(p1);
    footprint_.push_back(p2);
    footprint_.push_back(p3);

    tf_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    planner_costmap_ros_.reset(
        new costmap_2d::Costmap2DROS("global_costmap", *tf_));
    planner_costmap_ros_->setUnpaddedRobotFootprint(footprint_);

    // create collision checker object
    // and passes costmap pointer to it
    collision_checker_.reset(new GridCollisionChecker(
        planner_costmap_ros_->getCostmap(), footprint_client_));

    planner_costmap_ros_->start();

    // find the min and max bounds of the costmap in world coordinate
    unsigned int costmap_size_x, costmap_size_y;
    costmap_size_x = planner_costmap_ros_->getCostmap()->getSizeInCellsX();
    costmap_size_y = planner_costmap_ros_->getCostmap()->getSizeInCellsY();

    double w0_x, w0_y, w1_x, w1_y;
    planner_costmap_ros_->getCostmap()->mapToWorld(0, 0, w0_x, w0_y);
    planner_costmap_ros_->getCostmap()->mapToWorld(
        costmap_size_x - 1, costmap_size_y - 1, w1_x, w1_y);

    // now we can create state space of our robot including min and max limits
    StateLimits state_limits;

    // x and y of state
    state_limits.min_x = static_cast<float>(w0_x);
    state_limits.max_x = static_cast<float>(w1_x);
    state_limits.min_y = static_cast<float>(w0_y);
    state_limits.max_y = static_cast<float>(w1_y);

    // orientation of state
    state_limits.min_theta = -M_PI;
    state_limits.max_theta = M_PI;

    // joint positions of state
    // TODO: create state limits of joint positions based on user defined ros
    // params

    // create rrt object
    // we just directly create this for now
    // later improve it to make it modular planner algorithm object for
    // any sampling-based algorithm

    rrt_.reset(new RRT(state_limits, max_vertices_, collision_checker_));

    ROS_INFO("Planner created with %d maximum vertices.", max_vertices_);

    // TODO: we manually provide start and goal pose for now
    // later get goal pose from rviz 2D nav goal or ros param
    start_vertex_ = std::make_shared<Vertex>();
    start_vertex_->state.x = 0.0;
    start_vertex_->state.y = 0.0;
    start_vertex_->state.theta = 0.0;
    goal_vertex_ = std::make_shared<Vertex>();
    goal_vertex_->state.x = 17.0;
    goal_vertex_->state.y = -1.0;
    goal_vertex_->state.theta = 0.0;
    rrt_->init(start_vertex_, goal_vertex_);

    // timers
    planner_timer_ =
        mt_nh_.createWallTimer(ros::WallDuration(planner_update_interval_),
                               &PsrrPlannerNodelet::plannerTimerCB, this);
    path_pub_timer_ =
        mt_nh_.createWallTimer(ros::WallDuration(path_update_interval_),
                               &PsrrPlannerNodelet::pathPublisherTimerCB, this);
  }

 private:
  void goalCB(const geometry_msgs::PoseStamped& msg) {
    // get the desired goal pose

    ///////////////////////////////////////
    // Testing code for collison checking
    ///////////////////////////////////////

    // we'll test our collision checker based on RVIZ 2D Nav Goal

    // auto init_time = std::chrono::system_clock::now();

    // bool is_collision = collision_checker_->isCollision(
    //     msg.pose.position.x, msg.pose.position.y,
    //     tf2::getYaw(msg.pose.orientation),
    //     planner_costmap_ros_->getRobotFootprint());

    // auto execution_time =
    // std::chrono::duration_cast<std::chrono::microseconds>(
    //                           std::chrono::system_clock::now() - init_time)
    //                           .count();

    // std::cout << "Collision detection finished in " << execution_time << "
    // us"
    //           << std::endl;
    // std::cout << "Collision status: " << is_collision << std::endl;
    ///////////////////////////////////////
  }

  void plannerTimerCB(const ros::WallTimerEvent& event) {
    // update rrt tree

    // auto init_time = std::chrono::system_clock::now();
    rrt_->update();
    // rrt_->update(getRobotFootprint());
    // auto execution_time =
    // std::chrono::duration_cast<std::chrono::microseconds>(
    //                           std::chrono::system_clock::now() - init_time)
    //                           .count();
    // std::cout << "RRT update step takes " << execution_time << " us"
    //           << std::endl;

    // publish tree markers
    if (tree_markers_pub_.getNumSubscribers()) {
      visualization_msgs::MarkerArray markers;
      markers.markers.resize(1);

      // edge markers
      visualization_msgs::Marker& vertices_marker = markers.markers[0];
      vertices_marker.header.frame_id = "map";
      vertices_marker.header.stamp = ros::Time::now();
      vertices_marker.ns = "edges";
      vertices_marker.id = 0;
      vertices_marker.type = visualization_msgs::Marker::LINE_LIST;
      vertices_marker.pose.orientation.w = 1.0;
      vertices_marker.scale.x = 0.01;

      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;

      for (const auto edge : *(rrt_->getEdges())) {
        geometry_msgs::Point p1;
        p1.x = static_cast<double>(edge.first->state.x);
        p1.y = static_cast<double>(edge.first->state.y);
        p1.z = 0.0;
        vertices_marker.points.push_back(p1);
        vertices_marker.colors.push_back(color);
        geometry_msgs::Point p2;
        p2.x = static_cast<double>(edge.second->state.x);
        p2.y = static_cast<double>(edge.second->state.y);
        p2.z = 0.0;
        vertices_marker.points.push_back(p2);
        vertices_marker.colors.push_back(color);
      }

      tree_markers_pub_.publish(markers);
    }
  }

  void pathPublisherTimerCB(const ros::WallTimerEvent& event) {
    // check if rrt has solution or not
    if (rrt_->hasSolution()) {
      psrr_msgs::Path path;
      path.header.frame_id = "map";
      path.header.stamp = ros::Time::now();

      nav_msgs::Path path_2d;
      path_2d.header.frame_id = "map";
      path_2d.header.stamp = ros::Time::now();

      jsk_recognition_msgs::PolygonArray footprint_arr;
      footprint_arr.header.frame_id = "map";
      footprint_arr.header.stamp = ros::Time::now();

      std::shared_ptr<Vertex> current = goal_vertex_;
      while (current->parent && current != start_vertex_) {
        geometry_msgs::Pose path_pose;
        geometry_msgs::PoseStamped path_pose_stamped;

        const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(current->state.theta));

        path_pose.position.x = current->state.x;
        path_pose.position.y = current->state.y;
        path_pose.orientation = orientation;

        path_pose_stamped.pose.position.x = current->state.x;
        path_pose_stamped.pose.position.y = current->state.y;
        path_pose_stamped.pose.orientation = orientation;

        if (publish_path_footprints_) {
          geometry_msgs::PolygonStamped polygon_stamped;
          polygon_stamped.header.frame_id = "map";
          polygon_stamped.header.stamp = ros::Time::now();
          std::vector<geometry_msgs::Point> transformed_footprint;
          costmap_2d::transformFootprint(
              current->state.x, current->state.y, current->state.theta,
              planner_costmap_ros_->getRobotFootprint(), transformed_footprint);

          for (auto& p : transformed_footprint) {
            geometry_msgs::Point32 p32;
            p32.x = p.x;
            p32.y = p.y;
            p32.z = p.z;
            polygon_stamped.polygon.points.push_back(p32);
          }
          footprint_arr.polygons.push_back(polygon_stamped);
        }

        path.poses.push_back(path_pose);
        path_2d.poses.push_back(path_pose_stamped);

        current = current->parent;
      }

      path_pub_.publish(path);
      path_2d_pub_.publish(path_2d);

      if (publish_path_footprints_) {
        footprint_arr_pub_.publish(footprint_arr);
      }
    }
  }

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;

  // subscribers
  ros::Subscriber goal_sub_;

  // publishers
  ros::Publisher path_pub_;
  ros::Publisher path_2d_pub_;
  ros::Publisher tree_markers_pub_;
  ros::Publisher footprint_arr_pub_;

  // timers
  ros::WallTimer planner_timer_;
  ros::WallTimer path_pub_timer_;

  // service clients
  std::shared_ptr<ros::ServiceClient> footprint_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<costmap_2d::Costmap2DROS> planner_costmap_ros_;
  std::shared_ptr<GridCollisionChecker> collision_checker_;

  std::shared_ptr<RRT> rrt_;
  std::shared_ptr<Vertex> start_vertex_;
  std::shared_ptr<Vertex> goal_vertex_;

  double planner_update_interval_;
  double path_update_interval_;
  int max_vertices_;
  bool publish_path_footprints_;

  std::vector<geometry_msgs::Point> footprint_;
};
}  // namespace psrr_planner

PLUGINLIB_EXPORT_CLASS(psrr_planner::PsrrPlannerNodelet, nodelet::Nodelet)