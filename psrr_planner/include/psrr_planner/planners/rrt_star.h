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

namespace psrr_planner {

class RRTStar : public BasePlanner {
 public:
  /**
   * @brief A constructor for psrr_planner::RRT
   * @param state_limits The state space of the robot including limits
   * @param collision_checker Grid Collision Checker
   * @param max_iterations Maximum number of iterations to run the algorithm
   * @param update_goal_every Find best goal parent at every n iteration
   * @param max_distance Maximum distance allowed between two vertices
   * @param rewire_factor Rewiring factor
   * @param interpolation_dist Interpolation distance during collsion checking
   * @param goal_radius Distance between vertex and goal to stop planning
   * @param use_seed Either use seeding or not (default: false)
   * @param seed_number Seed number to be used if use_seed is true. (default: 0)
   * @param print_every Print solution info at every n iteration (default: 0)
   */
  RRTStar(const StateLimits& state_limits,
          std::shared_ptr<GridCollisionChecker> collision_checker,
          unsigned int max_iterations, unsigned int update_goal_every,
          double max_distance, double rewire_factor, double interpolation_dist,
          double goal_radius, bool use_seed = false,
          unsigned int seed_number = 0, unsigned int print_every = 0);

  /**
   * @brief A destructor for psrr_planner::RRT
   */
  virtual ~RRTStar();

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

 protected:
  /**
   * @brief Find all the nearest neighbours inside the radius of particular
   * vertex provided
   * @param x_new Target vertex
   * @param X_near Vector of nearest neighbours
   */
  void near(const std::shared_ptr<const Vertex>& x_new,
            std::vector<std::shared_ptr<Vertex>>& X_near);

  /**
   * @brief Calculate r_rrt_ based on current measure
   */
  void updateRewiringLowerBounds();

  /**
   * @brief Print solution info at every n iteration
   */
  unsigned int print_every_;

  /**
   * @brief Find best goal parent at every n iteration
   */
  unsigned int update_goal_every_;

  /**
   * @brief Rewiring lower bound constant
   */
  double r_rrt_;

  /**
   * @brief Rewiring factor
   */
  double rewire_factor_;

  /**
   * @brief Maximum distance allowed between two vertices
   */
  double max_distance_;

  /**
   * @brief Measure (i.e., n-dimensional volume) of the current state space
   */
  double current_measure_;

  /**
   * @brief Vertices that lie within the goal radius
   */
  std::vector<std::shared_ptr<Vertex>> x_soln_;
};

}  // namespace psrr_planner