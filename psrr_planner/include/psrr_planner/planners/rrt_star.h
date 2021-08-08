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
  RRTStar(const StateLimits& state_limits,
          std::shared_ptr<GridCollisionChecker> collision_checker,
          unsigned int max_iterations, unsigned int goal_parent_size_interval,
          double max_distance, double r_rrt, double interpolation_dist,
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
  void sampleFree(const std::shared_ptr<Vertex>& v);

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

  double cost(std::shared_ptr<Vertex> v);

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
  double max_distance_;
  double goal_radius_;
};

}  // namespace psrr_planner