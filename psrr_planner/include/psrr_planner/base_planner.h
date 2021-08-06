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
#include <psrr_planner/utilities.h>

#include <memory>
#include <vector>

namespace psrr_planner {
class BasePlanner {
 public:
  /**
   * @brief A constructor for psrr_planner::BasePlanner abstract class
   * @param state_limits The state space of the robot including limits
   * @param collision_checker Grid Collision Checker
   */
  BasePlanner(const StateLimits& state_limits,
              std::shared_ptr<GridCollisionChecker> collision_checker)
      : state_limits_{state_limits},
        collision_checker_{collision_checker},
        solution_found_{false} {}

  /**
   * @brief A destructor for psrr_planner::BasePlanner abstract class
   */
  virtual ~BasePlanner() {}

  /**
   * @brief Getter for start vertex
   */
  std::shared_ptr<Vertex> getStartVertex() const { return start_vertex_; }

  /**
   * @brief Getter for goal vertex
   */
  std::shared_ptr<Vertex> getGoalVertex() const { return goal_vertex_; }

  /**
   * @brief Getter for vertices in a current tree
   */
  const std::vector<std::shared_ptr<Vertex>>* getVertices() const {
    return &vertices_;
  }

  /**
   * @brief Getter for edges in a current tree
   */
  const std::vector<
      std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Vertex>>>*
  getEdges() const {
    return &edges_;
  }

  /**
   * @brief Check whether the solution is available or not
   */
  bool hasSolution() const { return solution_found_; }

  /**
   * @brief Virtual function for returning the current solution cost
   */
  virtual double getSolutionCost() = 0;

  /**
   * @brief Virtual initialization function
   * @param start Initial configuration of the robot in world frame
   * @param goal Final configuration of the robot in world frame
   */
  virtual void init(const Vertex& start, const Vertex& goal) = 0;

  /**
   * @brief Main Update function of the algorithm
   * Simulataneously calling this function will grow/improve the tree (not
   * start from scratch)
   */
  virtual void update() = 0;

 protected:
  StateLimits state_limits_;
  std::shared_ptr<GridCollisionChecker> collision_checker_;
  std::shared_ptr<Vertex> start_vertex_;
  std::shared_ptr<Vertex> goal_vertex_;
  std::vector<std::shared_ptr<Vertex>> vertices_;
  std::vector<std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Vertex>>>
      edges_;

  bool solution_found_;
};
}  // namespace psrr_planner
