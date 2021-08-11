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
#include <random>
#include <vector>

namespace psrr_planner {
class BasePlanner {
 public:
  /**
   * @brief A constructor for psrr_planner::BasePlanner abstract class
   * @param state_limits The state space of the robot including limits
   * @param collision_checker Grid Collision Checker
   * @param interpolation_dist Interpolation distance during collsion checking
   * @param goal_radius Distance between vertex and goal to stop planning
   * @param goal_bias Goal biased sampling percentage
   * @param max_iterations Maximum number of iterations to run the algorithm
   * @param use_seed Either use seeding or not (default: false)
   * @param seed_number Seed number to be used if use_seed is true. (default: 0)
   */
  BasePlanner(const StateLimits& state_limits,
              std::shared_ptr<GridCollisionChecker> collision_checker,
              double interpolation_dist, double goal_radius, double goal_bias,
              unsigned int max_iterations, bool use_seed = false,
              unsigned int seed_number = 0);

  /**
   * @brief A destructor for psrr_planner::BasePlanner abstract class
   */
  virtual ~BasePlanner();

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

  /**
   * @brief Function for finding the solution and calculating the path cost
   * @return Solution cost
   */
  virtual double getSolutionCost();

  /**
   * @brief Calculate distance between two vertices
   * Distance function is separated into two parts for R^n and SO(2) state
   * spaces
   * @return distance between two vertices
   */
  virtual double distance(const std::shared_ptr<const Vertex>& v1,
                          const std::shared_ptr<const Vertex>& v2);

  /**
   * @brief Find the new interpolated vertex from from_v vertex to to_v
   * vertex
   * @param from_v Starting vertex
   * @param to_v Ending vertex
   * @param t Interpolation distance
   * @param v New vertex
   */
  virtual void interpolate(const std::shared_ptr<const Vertex>& from_v,
                           const std::shared_ptr<const Vertex>& to_v,
                           const double t, const std::shared_ptr<Vertex>& v);

  /**
   * @brief Check whether collision or not between two vertices
   * This function assumes from_v vertex is collision-free
   * @param from_v Starting vertex
   * @param to_v Ending vertex
   * @return true if there is a collision otherwise false
   */
  virtual bool isCollision(const std::shared_ptr<const Vertex>& from_v,
                           const std::shared_ptr<const Vertex>& to_v);

  /**
   * @brief Randomly sample a n-dimensional state limited by min and max of each
   * state variables
   * @param v Sampled vertex
   */
  virtual void sample(const std::shared_ptr<Vertex>& v);

  /**
   * @brief Find the nearest neighbour in a tree
   * @param v Nearest vertex
   */
  virtual void nearest(const std::shared_ptr<const Vertex>& x_rand,
                       std::shared_ptr<Vertex>& x_near);

  /**
   * @brief The cost to come of a vertex considering all the state spaces
   */
  virtual double cost(std::shared_ptr<Vertex> v);

  /**
   * @brief The cost to come of a vertex considering only the informed state
   * spaces (without SO(n) components)
   */
  virtual double euclideanCost(std::shared_ptr<Vertex> v);

  /**
   * @brief The cost to come of a vertex considering only x and y state spaces
   */
  virtual double euclideanCost2D(std::shared_ptr<Vertex> v);

  /**
   * @brief The euclidean distance between two vertices (this function does not
   * use SO(n) components since they are not informed)
   */
  virtual double euclideanDistance(
      const std::shared_ptr<const Vertex>& v1,
      const std::shared_ptr<const Vertex>& v2) const;

  /**
   * @brief The 2D euclidean distance between two vertices (this function only
   * uses x and y state spaces)
   */
  virtual double euclideanDistance2D(
      const std::shared_ptr<const Vertex>& v1,
      const std::shared_ptr<const Vertex>& v2) const;

  /**
   * @brief Check whether a vertex lies within goal radius or not
   */
  bool inGoalRegion(const std::shared_ptr<const Vertex>& v);

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
   * @brief Check whether planning is finsihed or not
   */
  bool isPlanningFinished() const { return planning_finished_; }

 protected:
  /**
   * @brief The state space of the robot including limits
   */
  StateLimits state_limits_;

  /**
   * @brief Grid Collision Checker
   */
  std::shared_ptr<GridCollisionChecker> collision_checker_;

  // start & goal vertices
  std::shared_ptr<Vertex> start_vertex_;
  std::shared_ptr<Vertex> goal_vertex_;

  // vertices & edges vector
  std::vector<std::shared_ptr<Vertex>> vertices_;
  std::vector<std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Vertex>>>
      edges_;

  // uniform distribution for state spaces
  std::uniform_real_distribution<float> x_dis_;
  std::uniform_real_distribution<float> y_dis_;
  std::uniform_real_distribution<float> theta_dis_;
  std::vector<std::uniform_real_distribution<float>> joint_pos_dis_;

  /**
   * @brief Random number generator
   */
  std::mt19937 rn_gen_;

  /**
   * @brief Flag to check whether seeding is used or not
   */
  bool use_seed_;

  /**
   * @brief Seed number to be used
   */
  unsigned int seed_number_;

  /**
   * @brief Maximum number of iterations to run the algorithm
   */
  unsigned int max_iterations_;

  /**
   * @brief Iteration number to keep track
   */
  unsigned int iteration_number_;

  /**
   * @brief Interpolation distance during collsion checking
   */
  double interpolation_dist_;

  /**
   * @brief Distance between vertex and goal to stop planning
   */
  double goal_radius_;

  /**
   * @brief Goal biased sampling percentage
   */
  double goal_bias_;

  /**
   * @brief Total number of dimensions of problem state space
   */
  int state_dimensions_;

  /**
   * @brief Flag to check whether solution is found or not
   */
  bool solution_found_;

  /**
   * @brief Flag to check whether planning is finished or not
   */
  bool planning_finished_;
};
}  // namespace psrr_planner
