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
#include <nav_msgs/Path.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <psrr_msgs/FootPrintSrv.h>
#include <psrr_msgs/Path.h>
#include <psrr_planner/collision_checker.h>
#include <psrr_planner/utilities.h>
#include <psrr_planner/visualizer.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// ompl
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <atomic>
#include <memory>
#include <mutex>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::time;

namespace psrr_planner {

using utils::State;
using utils::StateLimits;

enum PLANNING_MODE { DURATION, ITERATIONS };
enum PLANNERS_IDS { RRT_CONNECT, RRT_STAR };

const std::vector<std::string> PLANNER_NAMES{"rrt_connect", "rrt_star"};

class PsrrPlannerNodelet : public nodelet::Nodelet {
 public:
  PsrrPlannerNodelet() {}
  virtual ~PsrrPlannerNodelet() {}

  void onInit() override {
    NODELET_DEBUG("initializing psrr_planner_nodelet...");

    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    private_nh_.param<double>("planner_update_interval",
                              planner_update_interval_, 0.002);
    private_nh_.param<double>("path_publish_frequency", path_publish_frequency_,
                              10.0);
    private_nh_.param<bool>("publish_path_footprints", publish_path_footprints_,
                            false);

    // initialize visualizer
    visualizer_ = std::make_shared<visualizer::Visualizer>(mt_nh_, private_nh_);

    // here we decide to use either static footprint or dynamic one based on
    // joint_pos_limits_min and joint_pos_limits_max parameters
    use_static_collision_checking_ = true;

    // create state limits of the robot state space
    StateLimits state_limits;

    // joint positions of state space
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
        exit(1);
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
    // if these parameters are not provided, we assume we are using static
    // footprint and planning in only R2 or SE2 state space and warn the user
    // about this
    else {
      ROS_WARN(
          "Joint position limits are not found in parameter server. Assuming "
          "planning in R2 or SE2 space with static footprint.");
    }

    // create footprint object
    footprint_ = std::make_shared<std::vector<geometry_msgs::Point>>();

    // get initial footprint from ros param if using static footprint
    if (use_static_collision_checking_) {
      std::vector<double> initial_footprint_x, initial_footprint_y;
      if (private_nh_.hasParam("initial_footprint/x") &&
          private_nh_.hasParam("initial_footprint/y")) {
        private_nh_.param("initial_footprint/x", initial_footprint_x,
                          initial_footprint_x);
        private_nh_.param("initial_footprint/y", initial_footprint_y,
                          initial_footprint_y);

        // check they have the same size or not
        if (!(initial_footprint_x.size() == initial_footprint_y.size())) {
          ROS_ERROR("Initial footprint x and y have different size.");
          exit(1);
        }
        // or empty
        if (initial_footprint_x.size() == 0 ||
            initial_footprint_y.size() == 0) {
          ROS_ERROR("Initial footprint size is empty.");
          exit(1);
        }

        footprint_->resize(initial_footprint_x.size());
        for (std::size_t i = 0; i < initial_footprint_x.size(); ++i) {
          (*footprint_)[i].x = initial_footprint_x[i];
          (*footprint_)[i].y = initial_footprint_y[i];
        }
      }
      // if not provided, raise error
      else {
        ROS_ERROR("Initial footprint not found in the parameter server.");
        exit(1);
      }
    }

    tf_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    planner_costmap_ros_.reset(
        new costmap_2d::Costmap2DROS("global_costmap", *tf_));
    planner_costmap_ros_->setUnpaddedRobotFootprint(*footprint_);
    planner_costmap_ros_->start();

    // find the min and max bounds of the costmap in world coordinate
    unsigned int costmap_size_x, costmap_size_y;
    costmap_size_x = planner_costmap_ros_->getCostmap()->getSizeInCellsX();
    costmap_size_y = planner_costmap_ros_->getCostmap()->getSizeInCellsY();

    double w0_x, w0_y, w1_x, w1_y;
    planner_costmap_ros_->getCostmap()->mapToWorld(0, 0, w0_x, w0_y);
    planner_costmap_ros_->getCostmap()->mapToWorld(
        costmap_size_x - 1, costmap_size_y - 1, w1_x, w1_y);

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
          mt_nh_.serviceClient<psrr_msgs::FootPrintSrv>("footprint_server",
                                                        true));
    }

    // get number of joints
    const auto n_joints = state_limits.min_joint_pos.size();
    ndim_joint_ = static_cast<unsigned>(n_joints);
    // number of real vector states
    const auto rn = 2u + static_cast<unsigned>(n_joints);

    // ompl real state space bounds
    ob::RealVectorBounds bounds(rn);
    bounds.setLow(0, state_limits.min_x);
    bounds.setLow(1, state_limits.min_y);
    bounds.setHigh(0, state_limits.max_x);
    bounds.setHigh(1, state_limits.max_y);
    for (std::size_t i = 0; i < n_joints; ++i) {
      bounds.setLow(i + 2, state_limits.min_joint_pos[i]);
      bounds.setHigh(i + 2, state_limits.max_joint_pos[i]);
    }

    // create ompl compound state space
    space_ = std::make_shared<ob::CompoundStateSpace>();
    // add real vector states (order are in x, y, j1, j2, ...)
    space_->as<ob::CompoundStateSpace>()->addSubspace(
        std::make_shared<ob::RealVectorStateSpace>(rn), 1.0);
    // heading
    space_->as<ob::CompoundStateSpace>()->addSubspace(
        std::make_shared<ob::SO2StateSpace>(), 0.5);

    // set bounds for real vector states
    space_->as<ob::CompoundStateSpace>()
        ->as<ob::RealVectorStateSpace>(0)
        ->setBounds(bounds);

    space_->setup();
    ss_ = std::make_shared<og::SimpleSetup>(space_);
    si_ = ss_->getSpaceInformation();

    ompl_start_state_ = std::make_shared<ob::ScopedState<>>(space_);
    ompl_goal_state_ = std::make_shared<ob::ScopedState<>>(space_);

    // create collision checker object
    // and passes costmap pointer to it
    if (use_static_collision_checking_) {
      ROS_INFO("Static collision checking is used.");
      collision_checker_.reset(new GridCollisionChecker(
          planner_costmap_ros_->getCostmap(), *footprint_));
    } else {
      ROS_INFO("Dynamic collision checking is used.");
      collision_checker_.reset(new GridCollisionChecker(
          planner_costmap_ros_->getCostmap(), footprint_client_, space_));
    }

    ss_->setStateValidityChecker([this](const ob::State* state) {
      return !collision_checker_->isCollision(state);
    });

    // planning objective
    ss_->setOptimizationObjective(
        std::make_shared<ob::PathLengthOptimizationObjective>(si_));

    if (private_nh_.hasParam("planner_type")) {
      private_nh_.param<std::string>("planner_type", planner_type_,
                                     planner_type_);

      // now check seeding is used or not
      bool use_seed = false;
      int seed_number = 0;

      if (private_nh_.hasParam("use_seed")) {
        private_nh_.param<bool>("use_seed", use_seed, use_seed);
        if (use_seed) {
          if (private_nh_.hasParam("seed_number")) {
            private_nh_.param<int>("seed_number", seed_number, seed_number);
            ROS_INFO("Random seed %d provided.", seed_number);
          } else {
            ROS_INFO("Default Random seed 0 is used.");
          }
        } else {
          ROS_INFO("Random seeding is used.");
        }
      } else {
        ROS_WARN("use_seed parameter not found. Using random seeding...");
      }

      // check are we planning in time or iterations
      bool use_planning_time = false;
      int max_iterations;
      if (private_nh_.hasParam("use_planning_time")) {
        private_nh_.param<bool>("use_planning_time", use_planning_time,
                                use_planning_time);
        // if use planning, make sure planning time is given
        if (use_planning_time) {
          if (private_nh_.hasParam("planning_time")) {
            private_nh_.param<double>("planning_time", planning_time_, 0.0);
            planning_mode_ = PLANNING_MODE::DURATION;
          } else {
            ROS_ERROR(
                "use_planning_time is true but planning_time parameter not "
                "found.");
            exit(1);
          }
        } else {
          // otherwise use iterations instead
          if (private_nh_.hasParam("max_iterations")) {
            private_nh_.param<int>("max_iterations", max_iterations, 1000);
            planning_mode_ = PLANNING_MODE::ITERATIONS;
          } else {
            ROS_ERROR(
                "use_planning_time is false but max_iterations parameter not "
                "found.");
            exit(1);
          }
        }
      } else {
        ROS_ERROR(
            "use_planning_time parameter not found. Make sure you set it to "
            "either true or false.");
        exit(1);
      }
      planning_iterations_ = static_cast<unsigned>(max_iterations);

      // default to rrt_connect
      planner_id_ = PLANNERS_IDS::RRT_CONNECT;
      for (std::size_t i = 0; i < PLANNER_NAMES.size(); ++i) {
        if (planner_type_ == PLANNER_NAMES[i]) {
          planner_id_ = static_cast<int>(i);
          ROS_INFO("Planner Type: %s", PLANNER_NAMES[i].c_str());
          break;
        }
      }

      // choose the planner
      switch (planner_id_) {
        case RRT_CONNECT:
          ss_->setPlanner(
              ob::PlannerPtr(std::make_shared<og::RRTConnect>(si_)));
          break;
        case RRT_STAR:
          ss_->setPlanner(ob::PlannerPtr(std::make_shared<og::RRTstar>(si_)));
          break;
        default:
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
    start_state_ = std::make_shared<State>();
    goal_state_ = std::make_shared<State>();

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
        start_state_->x = initial_pose[0];
        start_state_->y = initial_pose[1];
        start_state_->theta = initial_pose[2];
      } else {
        // otherwise, we just use default initial pose
        ROS_WARN(
            "Initial pose does not have size of 3. Using default configuration "
            "x: 0.0, y: 0.0, "
            "theta: 0.0.");
        start_state_->x = 0.0;
        start_state_->y = 0.0;
        start_state_->theta = 0.0;
      }
    } else {
      ROS_INFO(
          "Robot initial pose starts at default configuration x: 0.0, y: 0.0, "
          "theta: 0.0.");
      start_state_->x = 0.0;
      start_state_->y = 0.0;
      start_state_->theta = 0.0;
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
        start_state_->joint_pos.resize(initial_joint_pos.size());
        for (std::size_t i = 0; i < initial_joint_pos.size(); ++i) {
          start_state_->joint_pos[i] = initial_joint_pos[i];
        }
      } else {
        ROS_ERROR("Initial joint pos is not provided.");
        return;
      }
    }

    // convert from psrr_planner::State to ompl::base::ScopedStatePtr
    updateOMPLState(ompl_start_state_, start_state_);

    has_goal_pose_ = false;
    planning_finished_ = false;
    planner_initialized_ = false;
    solution_path_exits_ = false;

    // subscribers

    // subscriber for receiving goal pose
    // we just simply subscribe to /move_base_simple/goal
    // (geometry_msgs/PoseStamped) for now
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1,
                              &PsrrPlannerNodelet::goalCB, this);

    // publishers
    path_se2_pub_ =
        mt_nh_.advertise<nav_msgs::Path>("/psrr_planner/path_se2", 1);
    path_nd_pub_ =
        mt_nh_.advertise<psrr_msgs::Path>("/psrr_planner/path_nd", 1);

    // timers
    planner_timer_ =
        mt_nh_.createWallTimer(ros::WallDuration(planner_update_interval_),
                               &PsrrPlannerNodelet::plannerTimerCB, this);
    path_pub_timer_ =
        mt_nh_.createWallTimer(ros::WallDuration(1.0 / path_publish_frequency_),
                               &PsrrPlannerNodelet::pathPublisherTimerCB, this);
  }

 private:
  void goalCB(const geometry_msgs::PoseStamped& msg) {
    const auto goal_theta = tf2::getYaw(msg.pose.orientation);
    // check goal pose is in collision or not
    if (collision_checker_->isCollision(msg.pose.position.x,
                                        msg.pose.position.y, goal_theta,
                                        start_state_->joint_pos)) {
      ROS_INFO("Goal pose at x: %f, y: %f, theta: %f is in collision.",
               msg.pose.position.x, msg.pose.position.y, goal_theta);
      return;
    }

    // update the goal pose
    goal_state_->x = msg.pose.position.x;
    goal_state_->y = msg.pose.position.y;
    goal_state_->theta = goal_theta;

    // here we use the same joint pos as the initial pose
    // TODO: Make this goal joint pos sendable from rviz
    goal_state_->joint_pos.clear();
    goal_state_->joint_pos.resize(start_state_->joint_pos.size());
    for (std::size_t i = 0; i < goal_state_->joint_pos.size(); ++i) {
      goal_state_->joint_pos[i] = start_state_->joint_pos[i];
    }

    ROS_INFO("Goal pose received at x: %f, y: %f, theta: %f.", goal_state_->x,
             goal_state_->y, goal_theta);

    has_goal_pose_ = true;
    planning_finished_ = false;
    planner_initialized_ = false;
  }

  void initPlanning() {
    // get planner specific parameters from the parameter server
    std::vector<std::string> param_names;
    ss_->getPlanner()->params().getParamNames(param_names);
    std::map<std::string, std::string> updated_param_names_values;
    for (const auto& n : param_names) {
      std::string param_name =
          "ompl_planner_parameters/" + PLANNER_NAMES[planner_id_] + "/" + n;
      XmlRpc::XmlRpcValue param;
      if (private_nh_.hasParam(param_name)) {
        private_nh_.getParam(param_name, param);

        if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          updated_param_names_values[n] =
              std::to_string(static_cast<double>(param));
        } else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt) {
          updated_param_names_values[n] =
              std::to_string(static_cast<int>(param));
        } else if (param.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
          updated_param_names_values[n] =
              std::to_string(static_cast<bool>(param));
        }
      }
    }
    ss_->getPlanner()->params().setParams(updated_param_names_values);
    ROS_INFO("Planner parameters updated.");

    // convert from psrr_planner::State to ompl::base::ScopedStatePtr
    updateOMPLState(ompl_goal_state_, goal_state_);

    ss_->clear();
    ss_->clearStartStates();

    // set the start and goal states
    ss_->setStartAndGoalStates(*ompl_start_state_, *ompl_goal_state_);
    ss_->setup();
    solution_path_exits_ = false;
    ROS_INFO("Planning initialization setup finished.");
  }

  void plannerTimerCB(const ros::WallTimerEvent& event) {
    if (has_goal_pose_) {
      if (!planner_initialized_) {
        initPlanning();
        planner_initialized_ = true;
      }

      if (!planning_finished_) {
        // we need to have two options for the planning
        // 1. DEFAULT mode (planning with time or iterations)
        // 2. ANIMATION mode (animate the planning with only iterations, we will
        // only put this mode in experimental branch. This mode can animate the
        // planning progress; useful for debugging and publications)

        // Create the termination condition
        std::shared_ptr<ob::PlannerTerminationCondition> ptc;
        if (planning_mode_ == PLANNING_MODE::DURATION)
          ptc = std::make_shared<ob::PlannerTerminationCondition>(
              ob::timedPlannerTerminationCondition(planning_time_, 0.01));
        else
          ptc = std::make_shared<ob::PlannerTerminationCondition>(
              ob::PlannerTerminationCondition(
                  ob::IterationTerminationCondition(planning_iterations_)));

        ROS_INFO("planning_mode: %d", planning_mode_);
        ROS_INFO("planning_time: %.2f", planning_time_);

        // solve the planning problem
        ob::PlannerStatus solved;
        solved = ss_->solve(*ptc);

        if (solved) {
          ss_->getSolutionPath().interpolate();
          convertToPath(ss_->getSolutionPath());
          solution_path_exits_ = true;

          // render visualization stuffs
          // only one time
          if (publish_path_footprints_) {
            std::vector<geometry_msgs::PolygonStamped> polygons;
            convertToPolygons(polygons, ss_->getSolutionPath());
            visualizer_->renderPathFootprints(polygons);
          }
        }

        planning_finished_ = true;
      }
    }
  }

  void pathPublisherTimerCB(const ros::WallTimerEvent& event) {
    if (solution_path_exits_) {
      path_se2_.header.frame_id = "map";
      path_se2_.header.stamp = ros::Time::now();
      path_se2_pub_.publish(path_se2_);

      if (!use_static_collision_checking_) {
        path_nd_.header.frame_id = "map";
        path_nd_.header.stamp = ros::Time::now();
        path_nd_pub_.publish(path_nd_);
      }
    }
  }

  void convertToPath(const ompl::geometric::PathGeometric& path) {
    if (path.getStateCount() <= 0) {
      ROS_WARN("No states found in path");
      return;
    }
    // convert to SE2 path
    path_se2_.poses.clear();
    path_se2_.poses.resize(path.getStateCount());
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      path_se2_.poses[i].pose = utils::omplStateToSE2Pose(path.getState(i));
    }

    // only convert to SE2 + R^n if using dynamic footprint
    if (!use_static_collision_checking_) {
      path_nd_.poses.clear();
      path_nd_.joint_state.clear();
      path_nd_.poses.resize(path.getStateCount());
      path_nd_.joint_state.resize(path.getStateCount());
      for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        path_nd_.poses[i] = utils::omplStateToSE2Pose(path.getState(i));
        path_nd_.joint_state[i] =
            utils::omplStateToJointState(path.getState(i), ndim_joint_);
      }
    }
  }

  void convertToPolygons(std::vector<geometry_msgs::PolygonStamped>& polygons,
                         const ompl::geometric::PathGeometric& path) {
    if (path.getStateCount() <= 0) {
      ROS_WARN("No states found in path");
      return;
    }

    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      const auto* state = path.getState(i);
      const auto* rn_state = state->as<ob::CompoundState>()
                                 ->as<ob::RealVectorStateSpace::StateType>(0);
      const auto* so2_state =
          state->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);

      // extract joint positions from left-over portion of R^n state space
      std::vector<double> joint_pos;
      for (std::size_t i = 0; i < static_cast<std::size_t>(ndim_joint_); ++i) {
        joint_pos.push_back(rn_state->values[i + 2]);
      }

      std::vector<geometry_msgs::Point> footprint;
      if (use_static_collision_checking_) {
        footprint = planner_costmap_ros_->getUnpaddedRobotFootprint();
      } else {
        // here we use our shared ros service to get the footprint
        psrr_msgs::FootPrintSrv footprint_req_msg;
        for (const auto& pos : joint_pos) {
          footprint_req_msg.request.position.push_back(pos);
        }
        if (footprint_client_->call(footprint_req_msg)) {
          footprint = footprint_req_msg.response.points;
        } else {
          ROS_ERROR("Failed to call footprint service.");
          return;
        }
      }

      std::vector<geometry_msgs::Point> transformed_footprint;
      costmap_2d::transformFootprint(rn_state->values[0], rn_state->values[1],
                                     so2_state->value, footprint,
                                     transformed_footprint);

      geometry_msgs::PolygonStamped polygon_stamped;
      for (auto& p : transformed_footprint) {
        geometry_msgs::Point32 p32;
        p32.x = static_cast<float>(p.x);
        p32.y = static_cast<float>(p.y);
        p32.z = static_cast<float>(p.z);
        polygon_stamped.polygon.points.push_back(p32);
      }

      polygons.emplace_back(polygon_stamped);
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
  ros::Publisher path_se2_pub_;
  ros::Publisher path_nd_pub_;
  // ros::Publisher planner_markers_pub_;
  // ros::Publisher footprint_arr_pub_;

  // timers
  ros::WallTimer planner_timer_;
  ros::WallTimer path_pub_timer_;

  // service clients
  std::shared_ptr<ros::ServiceClient> footprint_client_;

  // tf related
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<visualizer::Visualizer> visualizer_;

  std::unique_ptr<costmap_2d::Costmap2DROS> planner_costmap_ros_;
  std::shared_ptr<GridCollisionChecker> collision_checker_;

  std::string planner_type_;
  std::shared_ptr<State> start_state_;
  std::shared_ptr<State> goal_state_;

  unsigned int ndim_joint_;  // number of joint dimensions

  // 2D and n-D paths
  nav_msgs::Path path_se2_;
  psrr_msgs::Path path_nd_;
  std::atomic_bool solution_path_exits_;

  int planning_mode_;
  int planner_id_;
  double planning_time_;
  unsigned int planning_iterations_;

  double planner_update_interval_;
  double path_publish_frequency_;
  int max_vertices_;
  bool publish_path_footprints_;
  std::atomic_bool planning_finished_;
  std::atomic_bool has_goal_pose_;
  std::atomic_bool use_static_collision_checking_;
  std::atomic_bool planner_initialized_;

  std::shared_ptr<std::vector<geometry_msgs::Point>> footprint_;

  // ompl related
  ob::StateSpacePtr space_;
  og::SimpleSetupPtr ss_;
  ob::SpaceInformationPtr si_;
  ob::ScopedStatePtr ompl_start_state_;
  ob::ScopedStatePtr ompl_goal_state_;
};
}  // namespace psrr_planner

PLUGINLIB_EXPORT_CLASS(psrr_planner::PsrrPlannerNodelet, nodelet::Nodelet)