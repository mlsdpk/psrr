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
#include <psrr_planner/planners/rrt_star.h>
#include <psrr_planner/utilities.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <boost/random/uniform_on_sphere.hpp>
#include <boost/random/variate_generator.hpp>

namespace psrr_planner {

class InformedRRTStar : public RRTStar {
 public:
  /**
   * @brief A constructor for psrr_planner::RRT
   * @param state_limits The state space of the robot including limits
   * @param collision_checker Grid Collision Checker
   * @param max_iterations Maximum number of iterations to run the algorithm
   * @param max_sampling_tries Maximum number of tries to sample on prolate
   * hyperellipsoid
   * @param max_distance Maximum distance allowed between two vertices
   * @param rewire_factor Rewiring factor
   * @param interpolation_dist Interpolation distance during collsion checking
   * @param goal_radius Distance between vertex and goal to stop planning
   * @param goal_bias Goal biased sampling percentage
   * @param update_goal_every Find best goal parent at every n iteration
   * @param use_greedy_informed_set Whether use greedy informed set or not
   * (default: false)
   * @param use_seed Either use seeding or not (default: false)
   * @param seed_number Seed number to be used if use_seed is true. (default: 0)
   * @param print_every Print solution info at every n iteration (default: 0)
   */
  InformedRRTStar(const StateLimits& state_limits,
                  std::shared_ptr<GridCollisionChecker> collision_checker,
                  unsigned int max_iterations, unsigned int max_sampling_tries,
                  double max_distance, double rewire_factor,
                  double interpolation_dist, double goal_radius,
                  double goal_bias, unsigned int update_goal_every,
                  bool use_greedy_informed_set = false, bool use_seed = false,
                  unsigned int seed_number = 0, unsigned int print_every = 0);

  /**
   * @brief A destructor for psrr_planner::RRT
   */
  virtual ~InformedRRTStar();

  /**
   * @brief Initialize rrt with start and goal vertices
   * @param start Initial configuration of the robot in world frame
   * @param goal Final configuration of the robot in world frame
   */
  void init(const Vertex& start, const Vertex& goal,
            unsigned int planning_time = 0) override;

  /**
   * @brief Main Update function of the algorithm
   * Simulataneously calling this function will grow/improve the tree (not
   * start from scratch)
   */
  void update() override;

  /**
   * @brief Getter for 2d ellipse transverse diameter
   */
  const std::vector<double>& getTransverseDiameter() const noexcept {
    return transverse_dia_2d_;
  }

  /**
   * @brief Getter for 2d ellipse conjugate diameter
   */
  const std::vector<double>& getConjugateDiameter() const noexcept {
    return conjugate_dia_2d_;
  }

  /**
   * @brief Getter for 2d ellipse orientation
   */
  const double getEllipseOrientation() const noexcept {
    return ellipse_orien_2d_;
  }

  /**
   * @brief Getter for 2d ellipse center point
   */
  const std::vector<double> getEllipseCenter() const noexcept {
    return ellipse_center_2d_;
  }

 protected:
  /**
   * @brief Randomly sample a n-dimensional state limited by min and max of each
   * state variables
   * @param v Sampled vertex
   */
  void sample(const std::shared_ptr<Vertex>& v) override;

  /**
   * @brief Admissible estimate of a vertex
   */
  double heuristicCost(const std::shared_ptr<const Vertex>& v) const;

  /**
   * @brief Admissible estimate of a vertex in 2D (x and y)
   */
  double heuristicCost2D(const std::shared_ptr<const Vertex>& v) const;

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
   * @brief Convert vertex object into eigen vector. SO(2) component of
   * vertex is not added into vector since it is not informed.
   * @param v State vertex
   * @param vec Informed eigen vector
   */
  void convertVertexToVectorXd(const std::shared_ptr<const Vertex>& v,
                               Eigen::VectorXd& vec);

  /**
   * @brief Update the conjugate diameter of the 2d ellipse. This function will
   * throw error if either transverse diameter or minimum transverse diameter
   * does not exist.
   */
  void updateConjugateDiameter2D();

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
   * @brief Number of dimensions of informed state spaces
   */
  unsigned int informed_dims_;

  /**
   * @brief Maximum number of tries to sample on prolate hyperellipsoid
   */
  unsigned int max_sampling_tries_;

  /**
   * @brief Whether use greedy informed set or not
   */
  bool use_greedy_informed_set_;

  Eigen::VectorXd x_start_focus_;  // start focal point
  Eigen::VectorXd x_goal_focus_;   // goal focal point
  Eigen::VectorXd x_center_;       // ellipse center point
  double c_i_;    // current best solution cost (transverse diameter)
  double c_min_;  // theoritical minimum cost between two focii (minimum
                  // transverse diameter)

  // 2d ellipse properties
  // transverse and conjugate diameters for both informed and greedy informed
  // sets
  std::vector<double> transverse_dia_2d_;  // transverse diameter
  std::vector<double> conjugate_dia_2d_;   // conjugate diameter
  double d_focii_2d_;                      // distance between two focii
  double ellipse_orien_2d_;                // orientation of the ellipse
  std::vector<double> ellipse_center_2d_;  // center point of the ellipse

  Eigen::MatrixXd rotation_world_from_ellipse_;        // rotation matrix
  Eigen::MatrixXd transformation_world_from_ellipse_;  // transformation matrix

 public:
  using gen_type = std::mt19937;
};

}  // namespace psrr_planner