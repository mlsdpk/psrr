#include <geometry_msgs/PoseStamped.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <psrr_msgs/Path.h>
#include <ros/ros.h>

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
  }

 private:
  void goalCB(const geometry_msgs::PoseStamped& msg) {}

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
};
}  // namespace psrr_planner

PLUGINLIB_EXPORT_CLASS(psrr_planner::PsrrPlannerNodelet, nodelet::Nodelet)