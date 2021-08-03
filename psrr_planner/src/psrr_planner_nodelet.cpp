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
        private_nh_.param<double>("planner_update_interval", 3.0);
    double path_update_interval =
        private_nh_.param<double>("path_update_interval", 10.0);

    // timers
    planner_timer_ =
        mt_nh_.createWallTimer(ros::WallDuration(planner_update_interval),
                               &PsrrPlannerNodelet::plannerTimerCB, this);
    path_pub_timer_ =
        mt_nh_.createWallTimer(ros::WallDuration(path_update_interval),
                               &PsrrPlannerNodelet::pathPublisherTimerCB, this);

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
  }

 private:
  void goalCB(const geometry_msgs::PoseStamped& msg) {
    // get the desired goal pose

    ///////////////////////////////////////
    // Testing code for collison checking
    ///////////////////////////////////////

    // we'll test our collision checker based on RVIZ 2D Nav Goal

    std::lock_guard<std::mutex> lock(planner_costmap_mutex_);

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

  void plannerTimerCB(const ros::WallTimerEvent& event) {}

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

  std::mutex planner_costmap_mutex_;
  std::unique_ptr<costmap_2d::Costmap2DROS> planner_costmap_ros_;

  std::unique_ptr<GridCollisionChecker> collision_checker_;

  std::vector<geometry_msgs::Point> footprint_;
};
}  // namespace psrr_planner

PLUGINLIB_EXPORT_CLASS(psrr_planner::PsrrPlannerNodelet, nodelet::Nodelet)