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

#include <psrr_planner/planners/rrt.h>

namespace psrr_planner {
RRT::RRT(const StateLimits& state_limits,
         std::shared_ptr<GridCollisionChecker> collision_checker,
         unsigned int max_iterations, double max_distance,
         double interpolation_dist, double goal_radius, double goal_bias,
         bool use_seed, unsigned int seed_number)
    : BasePlanner(state_limits, collision_checker, interpolation_dist,
                  goal_radius, goal_bias, max_iterations, use_seed,
                  seed_number),

      max_distance_{max_distance} {}

RRT::~RRT(){};

void RRT::init(const Vertex& start, const Vertex& goal,
               unsigned int planning_time) {
  planning_finished_ = false;
  solution_found_ = false;
  iteration_number_ = 1;

  start_vertex_ = std::make_shared<Vertex>();
  goal_vertex_ = std::make_shared<Vertex>();
  start_vertex_->state = start.state;
  goal_vertex_->state = goal.state;

  vertices_.clear();
  edges_.clear();

  vertices_.emplace_back(start_vertex_);

  use_planning_time_ = false;
  if (planning_time >= 1u) {
    // convert planning time seconds to milliseconds
    planning_time_ = planning_time * 1000u;
    use_planning_time_ = true;
  }

  init_time_ = std::chrono::system_clock::now();
}

void RRT::update() {
  if (planning_finished_) return;

  std::shared_ptr<Vertex> x_rand = std::make_shared<Vertex>();
  std::shared_ptr<Vertex> x_nearest = std::make_shared<Vertex>();
  std::shared_ptr<Vertex> x_new = std::make_shared<Vertex>();

  sample(x_rand);
  nearest(x_rand, x_nearest);

  // find the distance between x_rand and x_nearest
  double d = distance(x_rand, x_nearest);

  // if this distance d > delta_q, we need to find nearest state in the
  // direction of x_rand
  if (d > max_distance_) {
    interpolate(x_nearest, x_rand, max_distance_ / d, x_new);
  } else {
    x_new->state = x_rand->state;
  }

  if (!isCollision(x_nearest, x_new)) {
    x_new->parent = x_nearest;
    vertices_.emplace_back(x_new);
    edges_.emplace_back(x_nearest, x_new);

    if (inGoalRegion(x_new)) {
      goal_vertex_->parent = x_new;
      solution_found_ = true;
    }
  }

  iteration_number_++;
  auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now() - init_time_)
                            .count();

  // are we using time to plan?
  if (use_planning_time_) {
    if (execution_time >= planning_time_) {
      planning_finished_ = true;

      std::cout << "Iter no: " << iteration_number_ - 1
                << " | Soluion cost: " << getSolutionCost() << std::endl;

      if (execution_time < 1000) {
        std::cout << "Total planning time is " << execution_time << " ms."
                  << std::endl;
      } else {
        std::cout << "Total planning time is " << execution_time * 0.001
                  << " s." << std::endl;
      }
    }
  } else {
    if (iteration_number_ > max_iterations_) {
      planning_finished_ = true;
      std::cout << "Iterations number reach max limit. Algorithm stopped."
                << '\n';

      std::cout << "Iter no: " << iteration_number_ - 1
                << " | Soluion cost: " << getSolutionCost() << std::endl;

      if (execution_time < 1000) {
        std::cout << "Total planning time is " << execution_time << " ms."
                  << std::endl;
      } else {
        std::cout << "Total planning time is " << execution_time * 0.001
                  << " s." << std::endl;
      }
    }
  }
}

}  // namespace psrr_planner