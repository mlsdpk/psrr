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
#include <psrr_planner/planners/rrt.h>
#include <psrr_planner/planners/rrt_star.h>
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
    private_nh_.param<bool>("publish_path_footprints", publish_path_footprints_,
                            false);

    // get initial footprint from ros param
    std::vector<double> initial_footprint_x, initial_footprint_y;
    if (private_nh_.hasParam("initial_footprint/x") &&
        private_nh_.hasParam("initial_footprint/y")) {
      private_nh_.param("initial_footprint/x", initial_footprint_x,
                        initial_footprint_x);
      private_nh_.param("initial_footprint/y", initial_footprint_y,
                        initial_footprint_y);

      // check they have the same size or not
      if (!(initial_footprint_x.size() == initial_footprint_y.size())) {
        ROS_ERROR("Footprint x and y have different size.");
        return;
      }
      // or empty
      if (initial_footprint_x.size() == 0 || initial_footprint_y.size() == 0) {
        ROS_ERROR("Footprint size is empty.");
        return;
      }
    }
    // create initial footprint
    footprint_.resize(initial_footprint_x.size());
    for (std::size_t i = 0; i < initial_footprint_x.size(); ++i) {
      footprint_[i].x = initial_footprint_x[i];
      footprint_[i].y = initial_footprint_y[i];
    }

    tf_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    planner_costmap_ros_.reset(
        new costmap_2d::Costmap2DROS("global_costmap", *tf_));
    planner_costmap_ros_->setUnpaddedRobotFootprint(footprint_);
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
    bool use_orientation;
    private_nh_.param<bool>("use_orientation", use_orientation, false);
    if (use_orientation) {
      state_limits.min_theta = -M_PI;
      state_limits.max_theta = M_PI;
    }

    use_static_collision_checking_ = true;

    // joint positions of state
    if (private_nh_.hasParam("joint_pos_limits_min") &&
        private_nh_.hasParam("joint_pos_limits_max")) {
      std::vector<double> joint_pos_limits_min, joint_pos_limits_max;
      private_nh_.param("joint_pos_limits_min", joint_pos_limits_min,
                        joint_pos_limits_min);
      private_nh_.param("joint_pos_limits_max", joint_pos_limits_max,
                        joint_pos_limits_max);

      // check to make sure both have the same size
      if (joint_pos_limits_min.size() != joint_pos_limits_max.size()) {
        ROS_ERROR("Joint position limits do not have the same size.");
        return;
      }
      // can be empty, but if they are empty, we neglect these joints and don't
      // use in the planning
      if (joint_pos_limits_min.size() > 0 && joint_pos_limits_max.size() > 0) {
        state_limits.min_joint_pos.resize(joint_pos_limits_min.size());
        state_limits.max_joint_pos.resize(joint_pos_limits_max.size());

        for (std::size_t i = 0; i < joint_pos_limits_min.size(); ++i) {
          state_limits.min_joint_pos[i] = joint_pos_limits_min[i];
          state_limits.max_joint_pos[i] = joint_pos_limits_max[i];
        }
        use_static_collision_checking_ = false;
      }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Footprint of self-reconfigurable robots cannot be defined as static like
    // normal robot. As for now, user needs to provide the dynamically changing
    // footprint of the robot through ROS services. Why? Currently this
    // footprint is used during the SE(2) collison detection in a 2D occupancy
    // grid map. This planner will request the footprint of the robot as a
    // service client. Not sure this is the best way, but for now we'll stick
    // with this approach

    if (!use_static_collision_checking_) {
      // we wait here until we have a footprint server available
      ROS_INFO("Waiting footprint service...");
      ros::service::waitForService("footprint_server");
      ROS_INFO("Success: Connected to footprint service.");

      // make persistent ros service for faster client calls
      footprint_client_ = std::make_shared<ros::ServiceClient>(
          mt_nh_.serviceClient<psrr_msgs::FootPrint>("footprint_server", true));
    }

    ////////////////////////////////////////////////////////////////////////////

    // create collision checker object
    // and passes costmap pointer to it
    if (use_static_collision_checking_) {
      ROS_INFO("Static collision checking is used.");
      collision_checker_.reset(new GridCollisionChecker(
          planner_costmap_ros_->getCostmap(), footprint_));
    } else {
      ROS_INFO("Dynamic collision checking is used.");
      collision_checker_.reset(new GridCollisionChecker(
          planner_costmap_ros_->getCostmap(), footprint_client_));
    }

    std::string planner_type;
    if (private_nh_.hasParam("planner_type")) {
      private_nh_.param<std::string>("planner_type", planner_type,
                                     planner_type);
      // RRT
      if (planner_type == "rrt") {
        ROS_INFO("Planner Type: rrt");
        // we need to check planner specific parameters are given
        if (private_nh_.hasParam("rrt/max_vertices") &&
            private_nh_.hasParam("rrt/delta_q") &&
            private_nh_.hasParam("rrt/interpolation_dist") &&
            private_nh_.hasParam("rrt/goal_radius")) {
          int max_vertices;
          double delta_q, interpolation_dist, goal_radius;
          private_nh_.param<int>("rrt/max_vertices", max_vertices,
                                 max_vertices);
          private_nh_.param<double>("rrt/delta_q", delta_q, delta_q);
          private_nh_.param<double>("rrt/interpolation_dist",
                                    interpolation_dist, interpolation_dist);
          private_nh_.param<double>("rrt/goal_radius", goal_radius,
                                    goal_radius);

          // now check seeding is used or not
          bool use_seed = false;
          int seed_number = 0;

          if (private_nh_.hasParam("rrt/use_seed")) {
            private_nh_.param<bool>("rrt/use_seed", use_seed, use_seed);
            if (use_seed) {
              if (private_nh_.hasParam("rrt/seed_number")) {
                private_nh_.param<int>("rrt/seed_number", seed_number,
                                       seed_number);
                ROS_INFO("Random seed %d provided.", seed_number);
              } else {
                ROS_INFO("Default Random seed 0 is used.");
              }
            } else {
              ROS_INFO("Random seeding is used.");
            }
          } else {
            ROS_WARN(
                "RRT use_seed parameter not found. Using random seeding...");
          }

          // create rrt planner
          planner_.reset(new RRT(state_limits, collision_checker_, max_vertices,
                                 delta_q, interpolation_dist, goal_radius,
                                 use_seed, seed_number));
        } else {
          ROS_ERROR("RRT specific parameters not found.");
          return;
        }
      }
      // RRT*
      else if (planner_type == "rrt_star") {
        ROS_INFO("Planner Type: rrt_star");
        // TODO: Add rrt*
        // we need to check planner specific parameters are given
        if (private_nh_.hasParam("rrt_star/max_vertices") &&
            private_nh_.hasParam("rrt_star/goal_parent_size_interval") &&
            private_nh_.hasParam("rrt_star/max_distance") &&
            private_nh_.hasParam("rrt_star/r_rrt") &&
            private_nh_.hasParam("rrt_star/interpolation_dist") &&
            private_nh_.hasParam("rrt_star/goal_radius")) {
          int max_vertices, goal_parent_size_interval;
          double max_distance, r_rrt, interpolation_dist, goal_radius;
          private_nh_.param<int>("rrt_star/max_vertices", max_vertices,
                                 max_vertices);
          private_nh_.param<int>("rrt_star/goal_parent_size_interval",
                                 goal_parent_size_interval,
                                 goal_parent_size_interval);
          private_nh_.param<double>("rrt_star/max_distance", max_distance,
                                    max_distance);
          private_nh_.param<double>("rrt_star/r_rrt", r_rrt, r_rrt);
          private_nh_.param<double>("rrt_star/interpolation_dist",
                                    interpolation_dist, interpolation_dist);
          private_nh_.param<double>("rrt_star/goal_radius", goal_radius,
                                    goal_radius);

          // now check seeding is used or not
          bool use_seed = false;
          int seed_number = 0;

          if (private_nh_.hasParam("rrt_star/use_seed")) {
            private_nh_.param<bool>("rrt_star/use_seed", use_seed, use_seed);
            if (use_seed) {
              if (private_nh_.hasParam("rrt_star/seed_number")) {
                private_nh_.param<int>("rrt_star/seed_number", seed_number,
                                       seed_number);
                ROS_INFO("Random seed %d provided.", seed_number);
              } else {
                ROS_INFO("Default Random seed 0 is used.");
              }
            } else {
              ROS_INFO("Random seeding is used.");
            }
          } else {
            ROS_WARN(
                "RRT-STAR use_seed parameter not found. Using random "
                "seeding...");
          }

          // create rrt-star planner
          planner_.reset(new RRTStar(state_limits, collision_checker_,
                                     max_vertices, goal_parent_size_interval,
                                     max_distance, r_rrt, interpolation_dist,
                                     goal_radius, use_seed, seed_number));
        } else {
          ROS_ERROR("RRT specific parameters not found.");
          return;
        }
      } else {
        ROS_ERROR("Invalid planner type.");
        return;
      }
    } else {
      ROS_ERROR("No planner type found. Make sure planner_type is set.");
      return;
    }

    // initial pose can be set from ros param
    // (In real planning case, this needs to get it from tf [map->base_link])
    // if user does not provide inital pose parameter
    // we assume robot is at (0 m, 0 m, 0 rad) pose

    if (private_nh_.hasParam("initial_pose")) {
      std::vector<double> initial_pose;
      private_nh_.param("initial_pose", initial_pose, initial_pose);
      // this pose exactly needs to have size of 3
      if (initial_pose.size() == 3) {
        ROS_INFO(
            "Robot initial pose starts at given configuration x: %f, y: "
            "%f, "
            "theta: %f.",
            initial_pose[0], initial_pose[1], initial_pose[2]);
        start_vertex_.state.x = initial_pose[0];
        start_vertex_.state.y = initial_pose[1];
        start_vertex_.state.theta = initial_pose[2];
      } else {
        // otherwise, we just use default initial pose
        ROS_WARN(
            "Initial pose does not have size of 3. Using default configuration "
            "x: 0.0, y: 0.0, "
            "theta: 0.0.");
        start_vertex_.state.x = 0.0;
        start_vertex_.state.y = 0.0;
        start_vertex_.state.theta = 0.0;
      }
    } else {
      ROS_INFO(
          "Robot initial pose starts at default configuration x: 0.0, y: 0.0, "
          "theta: 0.0.");
      start_vertex_.state.x = 0.0;
      start_vertex_.state.y = 0.0;
      start_vertex_.state.theta = 0.0;
    }

    if (!use_static_collision_checking_) {
      std::vector<double> initial_joint_pos;
      if (private_nh_.hasParam("initial_joint_pos")) {
        private_nh_.param("initial_joint_pos", initial_joint_pos,
                          initial_joint_pos);

        // check to make sure the same size with joint_pos_limits_min &
        // joint_pos_limits_max
        if (initial_joint_pos.size() != state_limits.min_joint_pos.size()) {
          ROS_ERROR(
              "Initial joint pos must be the same size with joint pos limits.");
          return;
        }

        // check to make sure they are between limits
        for (std::size_t i = 0; i < initial_joint_pos.size(); ++i) {
          if (!(initial_joint_pos[i] >= state_limits.min_joint_pos[i] &&
                initial_joint_pos[i] <= state_limits.max_joint_pos[i])) {
            ROS_ERROR(
                "Initial joint pos are not in the given joint pos limits.");
            return;
          }
        }

        // now ok, add those in initial vertex
        start_vertex_.state.joint_pos.resize(initial_joint_pos.size());
        for (std::size_t i = 0; i < initial_joint_pos.size(); ++i) {
          start_vertex_.state.joint_pos[i] = initial_joint_pos[i];
        }
      } else {
        ROS_ERROR("Initial joint pos not provided.");
        return;
      }
    }

    has_goal_pose_ = false;
    planning_finished_ = false;
    planner_initialized_ = false;

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

    goal_vertex_.state.x = static_cast<float>(msg.pose.position.x);
    goal_vertex_.state.y = static_cast<float>(msg.pose.position.y);
    goal_vertex_.state.theta =
        static_cast<float>(tf2::getYaw(msg.pose.orientation));

    // here we use the same joint pos as the initial pose
    // TODO: Make this goal joint pos sendable from rviz
    goal_vertex_.state.joint_pos.clear();
    goal_vertex_.state.joint_pos.resize(start_vertex_.state.joint_pos.size());
    for (std::size_t i = 0; i < goal_vertex_.state.joint_pos.size(); ++i) {
      goal_vertex_.state.joint_pos[i] = start_vertex_.state.joint_pos[i];
    }

    ROS_INFO("Goal pose received at x: %f, y: %f, theta: %f.",
             goal_vertex_.state.x, goal_vertex_.state.y,
             goal_vertex_.state.theta);

    has_goal_pose_ = true;
    planning_finished_ = false;
    planner_initialized_ = false;
  }

  void plannerTimerCB(const ros::WallTimerEvent& event) {
    // update rrt tree

    if (has_goal_pose_) {
      if (!planner_initialized_) {
        planner_->init(start_vertex_, goal_vertex_);
        planner_init_time_ = std::chrono::system_clock::now();
        planner_initialized_ = true;
      }

      if (planner_->isPlanningFinished()) {
        if (!planning_finished_) {
          auto execution_time =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now() - planner_init_time_)
                  .count();
          ROS_INFO("Solution found. Planning finished in %ld ms.",
                   execution_time);

          double solution_cost = planner_->getSolutionCost();
          ROS_INFO("Solution cost: %f", solution_cost);
          planning_finished_ = true;
        }
      } else {
        // auto init_time = std::chrono::system_clock::now();
        planner_->update();
        // auto execution_time =
        //     std::chrono::duration_cast<std::chrono::microseconds>(
        //         std::chrono::system_clock::now() - init_time)
        //         .count();
        // std::cout << "RRT update step takes " << execution_time << " us"
        //           << std::endl;
      }
    }

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
      vertices_marker.action = visualization_msgs::Marker::ADD;
      vertices_marker.type = visualization_msgs::Marker::LINE_LIST;
      vertices_marker.pose.orientation.w = 1.0;
      vertices_marker.scale.x = 0.01;

      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;

      for (const auto edge : *(planner_->getEdges())) {
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
    if (planner_->hasSolution()) {
      psrr_msgs::Path path;
      path.header.frame_id = "map";
      path.header.stamp = ros::Time::now();

      nav_msgs::Path path_2d;
      path_2d.header.frame_id = "map";
      path_2d.header.stamp = ros::Time::now();

      jsk_recognition_msgs::PolygonArray footprint_arr;
      footprint_arr.header.frame_id = "map";
      footprint_arr.header.stamp = ros::Time::now();

      std::shared_ptr<Vertex> start_vertex = planner_->getStartVertex();
      std::shared_ptr<Vertex> current = planner_->getGoalVertex();
      while (current->parent && current != start_vertex) {
        geometry_msgs::Pose path_pose;
        geometry_msgs::PoseStamped path_pose_stamped;
        sensor_msgs::JointState joint_state;

        const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(current->state.theta));

        path_pose.position.x = current->state.x;
        path_pose.position.y = current->state.y;
        path_pose.orientation = orientation;

        path_pose_stamped.pose.position.x = current->state.x;
        path_pose_stamped.pose.position.y = current->state.y;
        path_pose_stamped.pose.orientation = orientation;

        joint_state.position.resize(current->state.joint_pos.size());
        for (std::size_t i = 0; i < joint_state.position.size(); ++i) {
          joint_state.position[i] =
              static_cast<double>(current->state.joint_pos[i]);
        }

        if (publish_path_footprints_) {
          geometry_msgs::PolygonStamped polygon_stamped;
          polygon_stamped.header.frame_id = "map";
          polygon_stamped.header.stamp = ros::Time::now();

          std::vector<geometry_msgs::Point> footprint;

          if (use_static_collision_checking_) {
            footprint = planner_costmap_ros_->getUnpaddedRobotFootprint();
          } else {
            // here we use our shared ros service to get the footprint
            psrr_msgs::FootPrint footprint_req_msg;
            for (const auto pos : current->state.joint_pos) {
              footprint_req_msg.request.position.push_back(
                  static_cast<double>(pos));
            }
            if (footprint_client_->call(footprint_req_msg)) {
              footprint = footprint_req_msg.response.points;
            } else {
              ROS_ERROR("Failed to call footprint service.");
              return;
            }
          }

          std::vector<geometry_msgs::Point> transformed_footprint;
          costmap_2d::transformFootprint(current->state.x, current->state.y,
                                         current->state.theta, footprint,
                                         transformed_footprint);

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
        path.joint_state.push_back(joint_state);
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

  std::shared_ptr<BasePlanner> planner_;
  Vertex start_vertex_;
  Vertex goal_vertex_;

  std::chrono::system_clock::time_point planner_init_time_;

  double planner_update_interval_;
  double path_update_interval_;
  int max_vertices_;
  bool publish_path_footprints_;
  std::atomic_bool planning_finished_;
  std::atomic_bool has_goal_pose_;
  std::atomic_bool use_static_collision_checking_;
  std::atomic_bool planner_initialized_;

  std::vector<geometry_msgs::Point> footprint_;
};
}  // namespace psrr_planner

PLUGINLIB_EXPORT_CLASS(psrr_planner::PsrrPlannerNodelet, nodelet::Nodelet)