#pragma once

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Polygon.h>

namespace psrr_planner {
class GridCollisionChecker {
 public:
  /**
   * @brief A constructor for psrr_planner::GridCollisionChecker
   * @param costmap The costmap to collision check against
   */
  GridCollisionChecker(costmap_2d::Costmap2D* costmap) {
    //
  }

  /**
   * @brief Check if the footprint is in collision with the current shared
   * costmap (footprint here is not transformed to pose yet)
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle of pose to check against
   * @param footprint Untransformed footprint
   * @return boolean if in collision or not.
   */
  bool isCollision(const double x, const double y, const double theta,
                   const geometry_msgs::Polygon& footprint) {
    //
  }

 private:
};
}  // namespace psrr_planner