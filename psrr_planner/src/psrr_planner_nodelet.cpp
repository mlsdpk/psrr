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
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <psrr_msgs/Path.h>
#include <psrr_planner/collision_checker.h>
#include <psrr_planner/rrt.h>
#include <ros/ros.h>
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

    double planner_update_interval =
        private_nh_.param<double>("planner_update_interval", 0.001);
    double path_update_interval =
        private_nh_.param<double>("path_update_interval", 10.0);

    ////////////////////////////////////////////////////////////////////////////
    // Footprint of self-reconfigurable robots cannot be defined as static like
    // normal robot. As for now, user needs to provide the dynamically changing
    // footprint of the robot through ROS services. This planner will request
    // the footprint of the robot as a service client.

    // footprint_client_ =
    // mt_nh_.serviceClient<psrr_srvs::FootPrint>("footprint");
    ////////////////////////////////////////////////////////////////////////////

    // fixed hardcoded footprint for now
    auto p0 = geometry_msgs::Point();
    p0.x = 1.025;
    p0.y = -0.5;
    auto p1 = geometry_msgs::Point();
    p1.x = 1.025;
    p1.y = 0.5;
    auto p2 = geometry_msgs::Point();
    p2.x = -1.025;
    p2.y = 0.5;
    auto p3 = geometry_msgs::Point();
    p3.x = -1.025;
    p3.y = -0.5;
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
    collision_checker_.reset(
        new GridCollisionChecker(planner_costmap_ros_->getCostmap()));

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
    // later this depends on user configurable with ROS param
    // for now just assume using [0-2pi]
    state_limits.min_theta = -M_PI;
    state_limits.max_theta = M_PI;

    // joint positions of state
    // we don't use reconfigurability for now

    // create rrt object
    // we just directly create this for now
    // later improve it to make it modular planner algorithm object for
    // any sampling-based algorithm

    rrt_.reset(new RRT(state_limits, 10000, collision_checker_));

    // timers
    planner_timer_ =
        mt_nh_.createWallTimer(ros::WallDuration(planner_update_interval),
                               &PsrrPlannerNodelet::plannerTimerCB, this);
    path_pub_timer_ =
        mt_nh_.createWallTimer(ros::WallDuration(path_update_interval),
                               &PsrrPlannerNodelet::pathPublisherTimerCB, this);
  }

 private:
  void goalCB(const geometry_msgs::PoseStamped& msg) {
    // get the desired goal pose

    ///////////////////////////////////////
    // Testing code for collison checking
    ///////////////////////////////////////

    // we'll test our collision checker based on RVIZ 2D Nav Goal

    auto init_time = std::chrono::system_clock::now();

    bool is_collision = collision_checker_->isCollision(
        msg.pose.position.x, msg.pose.position.y,
        tf2::getYaw(msg.pose.orientation),
        planner_costmap_ros_->getRobotFootprint());

    auto execution_time = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::system_clock::now() - init_time)
                              .count();

    std::cout << "Collision detection finished in " << execution_time << " us"
              << std::endl;
    std::cout << "Collision status: " << is_collision << std::endl;
    ///////////////////////////////////////
  }

  void plannerTimerCB(const ros::WallTimerEvent& event) {
    // update rrt tree

    rrt_->update(planner_costmap_ros_->getRobotFootprint());

    // publish tree markers
    if (tree_markers_pub_.getNumSubscribers()) {
      visualization_msgs::MarkerArray markers;
      markers.markers.resize(1);

      // vertex markers
      visualization_msgs::Marker& vertices_marker = markers.markers[0];
      vertices_marker.header.frame_id = "map";
      vertices_marker.header.stamp = ros::Time::now();
      vertices_marker.ns = "vertices";
      vertices_marker.id = 0;
      vertices_marker.type = visualization_msgs::Marker::SPHERE_LIST;
      vertices_marker.pose.orientation.w = 1.0;
      vertices_marker.scale.x = vertices_marker.scale.y =
          vertices_marker.scale.z = 0.1;

      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;

      for (const auto v : *(rrt_->getVertices())) {
        geometry_msgs::Point p;
        p.x = static_cast<double>(v->state.x);
        p.y = static_cast<double>(v->state.y);
        p.z = 0.01;
        vertices_marker.points.push_back(p);
        vertices_marker.colors.push_back(color);
      }

      tree_markers_pub_.publish(markers);
    }
  }

  void pathPublisherTimerCB(const ros::WallTimerEvent& event) {}

  // ROS related
  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber goal_sub_;

  ros::Publisher path_pub_;
  ros::Publisher tree_markers_pub_;

  ros::WallTimer planner_timer_;
  ros::WallTimer path_pub_timer_;

  ros::ServiceClient footprint_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<costmap_2d::Costmap2DROS> planner_costmap_ros_;
  std::shared_ptr<GridCollisionChecker> collision_checker_;

  std::shared_ptr<RRT> rrt_;

  std::vector<geometry_msgs::Point> footprint_;
};
}  // namespace psrr_planner

PLUGINLIB_EXPORT_CLASS(psrr_planner::PsrrPlannerNodelet, nodelet::Nodelet)