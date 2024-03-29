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

class RRT : public BasePlanner {
 public:
  /**
   * @brief A constructor for psrr_planner::RRT
   * @param state_limits The state space of the robot including limits
   * @param collision_checker Grid Collision Checker
   * @param max_iterations Maximum number of iterations to run the algorithm
   * @param max_distance Maximum distance allowed between two vertices
   * @param interpolation_dist Interpolation distance during collsion checking
   * @param goal_radius Distance between vertex and goal to stop planning
   * @param goal_bias Goal biased sampling percentage
   * @param use_seed Either use seeding or not (default: false)
   * @param seed_number Seed number to be used if use_seed is true. (default: 0)
   */
  RRT(const StateLimits& state_limits,
      std::shared_ptr<GridCollisionChecker> collision_checker,
      unsigned int max_iterations, double max_distance,
      double interpolation_dist, double goal_radius, double goal_bias,
      bool use_seed = false, unsigned int seed_number = 0);

  /**
   * @brief A destructor for psrr_planner::RRT
   */
  virtual ~RRT();

  /**
   * @brief Initialize rrt with start and goal vertices
   * @param start Initial configuration of the robot in world frame
   * @param goal Final configuration of the robot in world frame
   * @param planning_time Time to plan the path (if time is 0,
   * max_iterations is used instead)
   */
  void init(const Vertex& start, const Vertex& goal,
            unsigned int planning_time = 0) override;

  /**
   * @brief Main Update function of the algorithm
   * Simulataneously calling this function will grow/improve the tree (not
   * start from scratch)
   */
  void update() override;

 private:
  /**
   * @brief Maximum distance allowed between two vertices
   */
  double max_distance_;
};

}  // namespace psrr_planner