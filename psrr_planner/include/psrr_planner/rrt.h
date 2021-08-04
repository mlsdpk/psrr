#pragma once

#include <psrr_planner/collision_checker.h>
#include <psrr_planner/utilities.h>

#include <random>

namespace psrr_planner {

class RRT {
 public:
  /**
   * @brief A constructor for psrr_planner::RRT
   * @param state_limits The state space of the robot including limits
   * @param max_vertices Maximum number of vertices in a tree
   * @param collision_checker Grid Collision Checker
   */
  RRT(const StateLimits& state_limits, unsigned int max_vertices,
      std::shared_ptr<GridCollisionChecker> collision_checker);

  ~RRT();

  void init();

  /**
   * @brief Randomly sample a n-dimensional state limited by min and max of each
   * state variables
   * @param v Sampled vertex
   */
  void sampleFree(Vertex& v);
  void nearest(const Vertex& x_rand, std::shared_ptr<Vertex>& x_near);
  void steer(const Vertex& x_rand, const std::shared_ptr<Vertex>& x_near,
             std::shared_ptr<Vertex>& x_new);
  double distance(const Vertex& v1, const Vertex& v2);
  void interpolate(const Vertex& from_v, const Vertex& to_v, const double t,
                   std::shared_ptr<Vertex>& v);
  bool isCollision(const Vertex& from_v, const Vertex& to_v);

  void update(const std::vector<geometry_msgs::Point>& footprint);

  const std::vector<std::shared_ptr<Vertex>>* getVertices() const;

 private:
  StateLimits state_limits_;
  unsigned int max_vertices_;

  std::shared_ptr<GridCollisionChecker> collision_checker_;

  std::vector<std::shared_ptr<Vertex>> vertices_;

  std::uniform_real_distribution<float> x_dis_;
  std::uniform_real_distribution<float> y_dis_;
  std::uniform_real_distribution<float> theta_dis_;

  double delta_q_;

  bool stopped_;
};

}  // namespace psrr_planner