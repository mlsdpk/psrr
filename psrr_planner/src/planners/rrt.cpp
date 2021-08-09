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
         unsigned int max_iterations, double delta_q, double interpolation_dist,
         double goal_radius, bool use_seed, unsigned int seed_number)
    : BasePlanner(state_limits, collision_checker, interpolation_dist, use_seed,
                  seed_number),
      max_iterations_{max_iterations},
      delta_q_{delta_q},
      goal_radius_{goal_radius} {}

RRT::~RRT(){};

void RRT::init(const Vertex& start, const Vertex& goal) {
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
}

void RRT::sampleFree(const std::shared_ptr<Vertex>& v) {
  if (!use_seed_) {
    std::random_device rd;
    rn_gen_ = std::mt19937(rd());
  }
  v->state.x = x_dis_(rn_gen_);
  v->state.y = y_dis_(rn_gen_);
  v->state.theta = theta_dis_(rn_gen_);

  v->state.joint_pos.resize(joint_pos_dis_.size());
  for (std::size_t i = 0; i < joint_pos_dis_.size(); ++i) {
    v->state.joint_pos[i] = joint_pos_dis_[i](rn_gen_);
  }
}

void RRT::nearest(const std::shared_ptr<const Vertex>& x_rand,
                  std::shared_ptr<Vertex>& x_near) {
  double minDist = std::numeric_limits<double>::infinity();

  for (const auto& v : vertices_) {
    double dist = distance(v, x_rand);
    if (dist < minDist) {
      minDist = dist;
      x_near = v;
    }
  }
}

void RRT::update() {
  if (planning_finished_) return;

  if (iteration_number_ <= max_iterations_) {
    std::shared_ptr<Vertex> x_rand = std::make_shared<Vertex>();
    std::shared_ptr<Vertex> x_nearest = std::make_shared<Vertex>();
    std::shared_ptr<Vertex> x_new = std::make_shared<Vertex>();

    sampleFree(x_rand);
    nearest(x_rand, x_nearest);

    // find the distance between x_rand and x_nearest
    double d = distance(x_rand, x_nearest);

    // if this distance d > delta_q, we need to find nearest state in the
    // direction of x_rand
    if (d > delta_q_) {
      interpolate(x_nearest, x_rand, delta_q_ / d, x_new);
    } else {
      x_new->state = x_rand->state;
    }

    if (!isCollision(x_nearest, x_new)) {
      x_new->parent = x_nearest;
      vertices_.emplace_back(x_new);
      edges_.emplace_back(x_nearest, x_new);

      double goal_dist = distance(x_new, goal_vertex_);

      if (goal_dist < goal_radius_) {
        goal_vertex_->parent = x_new;
        planning_finished_ = true;
        solution_found_ = true;
      }
    }

    iteration_number_++;
  } else {
    planning_finished_ = true;
    std::cout << "Iterations number reach max limit. Algorithm stopped."
              << '\n';
  }
}

double RRT::getSolutionCost() {
  double total_cost = 0.0;
  std::shared_ptr<Vertex> current = goal_vertex_;
  while (current->parent && current != start_vertex_) {
    total_cost += distance(current, current->parent);
    current = current->parent;
  }
  return total_cost;
}
}  // namespace psrr_planner