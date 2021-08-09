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

#include <psrr_planner/collision_checker.h>
#include <psrr_planner/planners/base_planner.h>
#include <psrr_planner/utilities.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <boost/random/uniform_on_sphere.hpp>
#include <boost/random/variate_generator.hpp>

namespace psrr_planner {

class InformedRRTStar : public BasePlanner {
 public:
  /**
   * @brief A constructor for psrr_planner::RRT
   * @param state_limits The state space of the robot including limits
   * @param collision_checker Grid Collision Checker
   * @param max_iterations Maximum number of iterations to run the algorithm
   * @param goal_parent_size_interval Only find parent of goal vertex for fix
   * amount of vertices interval
   * @param max_distance Maximum distance allowed between two vertices
   * @param r_rrt Rewiring factor
   * @param interpolation_dist Interpolation distance during collsion checking
   * @param goal_radius Distance between vertex and goal to stop planning
   * @param use_seed Either use seeding or not (default: false)
   * @param seed_number Seed number to be used if use_seed is true. (default: 0)
   * @param print_every Print solution info at every n iteration (default: 0)
   */
  InformedRRTStar(const StateLimits& state_limits,
                  std::shared_ptr<GridCollisionChecker> collision_checker,
                  unsigned int max_iterations,
                  unsigned int goal_parent_size_interval, double max_distance,
                  double r_rrt, double interpolation_dist, double goal_radius,
                  bool use_seed = false, unsigned int seed_number = 0,
                  unsigned int print_every = 0);

  /**
   * @brief A destructor for psrr_planner::RRT
   */
  virtual ~InformedRRTStar();

  /**
   * @brief Initialize rrt with start and goal vertices
   * @param start Initial configuration of the robot in world frame
   * @param goal Final configuration of the robot in world frame
   */
  void init(const Vertex& start, const Vertex& goal) override;

  /**
   * @brief Main Update function of the algorithm
   * Simulataneously calling this function will grow/improve the tree (not
   * start from scratch)
   */
  void update() override;

  /**
   * @brief Function for finding the solution and calculating the path cost
   * @return Solution cost
   */
  double getSolutionCost() override;

 private:
  /**
   * @brief Randomly sample a n-dimensional state limited by min and max of each
   * state variables
   * @param v Sampled vertex
   */
  void sample(const std::shared_ptr<Vertex>& v);

  /**
   * @brief Uniform random sampling of a unit-length vector, i.e., the surface
   * of an n-ball.
   */
  void uniformNormalVector(std::vector<double>& v);

  /**
   * @brief Uniform random sampling of the content of an n-ball, with a radius
   * appropriately distributed between [0,r) so that the distribution is uniform
   * in a Cartesian coordinate system.
   */
  void uniformInBall(double r, std::vector<double>& v);

  /**
   * @brief Find the nearest neighbour in a tree
   * @param v Nearest vertex
   */
  void nearest(const std::shared_ptr<const Vertex>& x_rand,
               std::shared_ptr<Vertex>& x_near);

  /**
   * @brief Find all the nearest neighbours inside the radius of particular
   * vertex provided
   * @param x_new Target vertex
   * @param X_near Vector of nearest neighbours
   */
  void near(const std::shared_ptr<const Vertex>& x_new,
            std::vector<std::shared_ptr<Vertex>>& X_near);

  /**
   * @brief The cost to come of a vertex considering all the state spaces
   */
  double cost(std::shared_ptr<Vertex> v);

  /**
   * @brief The cost to come of a vertex considering only the informed state
   * spaces (without SO(n) components)
   */
  double euclideanCost(std::shared_ptr<Vertex> v);

  /**
   * @brief The euclidean distance between two vertices (this function does not
   * use SO(n) components since they are not informed)
   */
  double euclideanDistance(const std::shared_ptr<const Vertex>& v1,
                           const std::shared_ptr<const Vertex>& v2);

  /**
   * @brief Convert vertex object into eigen vector. SO(2) component of
   * vertex is not added into vector since it is not informed.
   * @param v State vertex
   * @param vec Informed eigen vector
   */
  void convertVertexToVectorXd(const std::shared_ptr<const Vertex>& v,
                               Eigen::VectorXd& vec);

  /**
   * @brief Calculate a rotation matrix from hyperellipsoid frame to world frame
   * via singular value decomposition
   * Ref: https://ompl.kavrakilab.org/ProlateHyperspheroid_8cpp_source.html
   */
  void updateRotationMatrix();

  /**
   * @brief Update a transformation matrix from hyperellipsoid frame to world
   * frame
   * Ref: https://ompl.kavrakilab.org/ProlateHyperspheroid_8cpp_source.html
   */
  void updateTransformationMatrix();

  /**
   * @brief Calculate r_rrt_ based on current measure
   */
  void updateRewiringLowerBounds();

  /**
   * @brief Check whether a vertex lies within goal radius or not
   */
  bool inGoalRegion(const std::shared_ptr<const Vertex>& v);

  /**
   * @brief Maximum number of iterations to run the algorithm
   */
  unsigned int max_iterations_;

  /**
   * @brief Iteration number to keep track
   */
  unsigned int iteration_number_;

  /**
   * @brief Print solution info at every n iteration
   */
  unsigned int print_every_;

  unsigned int goal_parent_size_interval_;
  double r_rrt_;
  double rewire_factor_;
  double max_distance_;
  double goal_radius_;
  std::vector<std::shared_ptr<Vertex>> x_soln_;

  unsigned int state_dims_;
  unsigned int informed_dims_;
  unsigned int max_sampling_tries_;

  Eigen::VectorXd x_start_focus_;  // start focal point
  Eigen::VectorXd x_goal_focus_;   // goal focal point
  Eigen::VectorXd x_center_;       // ellipse center point
  double c_i_;    // current best solution cost (transverse diameter)
  double c_min_;  // theoritical minimum cost between two focii (minimum
                  // transverse diameter)

  double current_measure_;

  Eigen::MatrixXd rotation_world_from_ellipse_;
  Eigen::MatrixXd transformation_world_from_ellipse_;

 public:
  using gen_type = std::mt19937;
};

}  // namespace psrr_planner