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

#pragma once

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Polygon.h>
#include <psrr_msgs/FootPrintSrv.h>
#include <psrr_planner/line_iterator.h>

// ompl related
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;

namespace psrr_planner {

static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;

class GridCollisionChecker {
 public:
  /**
   * @brief A constructor for psrr_planner::GridCollisionChecker
   * for static footprint collision checking
   * @param costmap The costmap to collision check against
   * @param footprint Static footprint
   */
  GridCollisionChecker(costmap_2d::Costmap2D* costmap,
                       const std::vector<geometry_msgs::Point>& footprint)
      : costmap_{costmap},
        footprint_{footprint},
        rn_dim_{2u},
        use_static_footprint_{true} {}

  /**
   * @brief A constructor for psrr_planner::GridCollisionChecker
   * for dynamic footprint collision checking
   * @param costmap The costmap to collision check against
   * @param footprint_client shared_ptr ROS service client for dynamic footprint
   * @param space shared_ptr ompl state space
   */
  GridCollisionChecker(costmap_2d::Costmap2D* costmap,
                       std::shared_ptr<ros::ServiceClient> footprint_client,
                       ob::StateSpacePtr space)
      : costmap_{costmap},
        footprint_client_{footprint_client},
        use_static_footprint_{false} {
    rn_dim_ = space->as<ob::CompoundStateSpace>()
                  ->getSubspace(0)
                  ->as<ob::RealVectorStateSpace>()
                  ->getDimension();
    ROS_INFO("Dynamic collision checker is using %d R^n states.", rn_dim_);
  }

  /**
   * @brief Check whether ompl compound state is in collision or not
   * @param state OMPL state
   * @return true if is in collision, otherwise false
   */
  bool isCollision(const ob::State* state) {
    if (!state) {
      throw std::runtime_error("No state found for vertex");
    }

    const auto* rn_state =
        state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(
            0);

    const auto* so2_state =
        state->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);

    // extract joint positions from left-over portion of R^n state space
    std::vector<double> joint_pos;
    for (std::size_t i = 0; i < static_cast<std::size_t>(rn_dim_ - 2); ++i) {
      joint_pos.push_back(rn_state->values[i + 2]);
    }

    return isCollision(rn_state->values[0], rn_state->values[1],
                       so2_state->value, joint_pos);
  }

  /**
   * @brief Check if the footprint is in collision with the current shared
   * costmap (footprint here is not transformed to pose yet)
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle of pose to check against
   * @param joint_pos n-dimensional joint positions of pose to check against
   * @return true if is in collision, otherwise false
   */
  bool isCollision(const double x, const double y, const double theta,
                   const std::vector<double>& joint_pos) {
    // always check the cell corrdinate of the center of the robot
    unsigned int cell_x, cell_y;
    if (!costmap_->worldToMap(x, y, cell_x, cell_y)) return true;

    unsigned char cost = costmap_->getCost(cell_x, cell_y);
    if (cost == NO_INFORMATION || cost == LETHAL_OBSTACLE ||
        cost == INSCRIBED_INFLATED_OBSTACLE)
      return true;

    // now we need to check the full polygon footprint of the robot

    // create a new footprint by transforming the current one into desired
    // pose
    std::vector<geometry_msgs::Point> transformed_footprint;
    if (use_static_footprint_) {
      costmap_2d::transformFootprint(x, y, theta, footprint_,
                                     transformed_footprint);
    } else {
      // here we use our shared ros service to get the footprint
      psrr_msgs::FootPrintSrv footprint_req_msg;
      for (const auto pos : joint_pos) {
        footprint_req_msg.request.position.push_back(pos);
      }
      std::vector<geometry_msgs::Point> footprint;
      if (footprint_client_->call(footprint_req_msg)) {
        footprint = footprint_req_msg.response.points;
      } else {
        ROS_ERROR("Failed to call footprint service.");
        exit(1);
      }
      costmap_2d::transformFootprint(x, y, theta, footprint,
                                     transformed_footprint);
    }
    // now use this transformed footprint to check collision in the costmap
    unsigned int x0, x1, y0, y1;
    double footprint_cost = 0.0;

    // rasterize each line in the footprint
    for (unsigned int i = 0; i < transformed_footprint.size() - 1; ++i) {
      // get the cell coord of the first point
      if (!costmap_->worldToMap(transformed_footprint[i].x,
                                transformed_footprint[i].y, x0, y0))
        return true;

      // get the cell coord of the second point
      if (!costmap_->worldToMap(transformed_footprint[i + 1].x,
                                transformed_footprint[i + 1].y, x1, y1))
        return true;

      footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);

      // if in collision, no need to continue
      if (footprint_cost == static_cast<double>(NO_INFORMATION) ||
          footprint_cost == static_cast<double>(LETHAL_OBSTACLE) ||
          footprint_cost == static_cast<double>(INSCRIBED_INFLATED_OBSTACLE)) {
        return true;
      }
    }

    // connect the first point in the footprint to the last point
    // get the cell coord of the last point
    if (!costmap_->worldToMap(transformed_footprint.back().x,
                              transformed_footprint.back().y, x0, y0))
      return true;

    // get the cell coord of the first point
    if (!costmap_->worldToMap(transformed_footprint.front().x,
                              transformed_footprint.front().y, x1, y1))
      return true;

    footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);
    if (footprint_cost == static_cast<double>(NO_INFORMATION) ||
        footprint_cost == static_cast<double>(LETHAL_OBSTACLE) ||
        footprint_cost == static_cast<double>(INSCRIBED_INFLATED_OBSTACLE)) {
      return true;
    }

    return false;
  }

  double lineCost(int x0, int x1, int y0, int y1) const {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
      point_cost =
          pointCost(line.getX(), line.getY());  // Score the current point

      if (point_cost < 0) return point_cost;

      if (line_cost < point_cost) line_cost = point_cost;
    }

    return line_cost;
  }

  double pointCost(int x, int y) const {
    return static_cast<double>(costmap_->getCost(x, y));
  }

 private:
  costmap_2d::Costmap2D* costmap_;
  std::vector<geometry_msgs::Point> footprint_;
  std::shared_ptr<ros::ServiceClient> footprint_client_;
  unsigned int rn_dim_;
  bool use_static_footprint_;
};
}  // namespace psrr_planner