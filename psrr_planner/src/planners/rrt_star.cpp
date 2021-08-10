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

#include <psrr_planner/planners/rrt_star.h>

namespace psrr_planner {
RRTStar::RRTStar(const StateLimits& state_limits,
                 std::shared_ptr<GridCollisionChecker> collision_checker,
                 unsigned int max_iterations, unsigned int update_goal_every,
                 double max_distance, double rewire_factor,
                 double interpolation_dist, double goal_radius, bool use_seed,
                 unsigned int seed_number, unsigned int print_every)
    : BasePlanner(state_limits, collision_checker, interpolation_dist,
                  goal_radius, max_iterations, use_seed, seed_number),
      update_goal_every_{update_goal_every},
      max_distance_{max_distance},
      rewire_factor_{rewire_factor},
      print_every_{print_every} {}

RRTStar::~RRTStar(){};

void RRTStar::init(const Vertex& start, const Vertex& goal) {
  planning_finished_ = false;
  solution_found_ = false;
  iteration_number_ = 1;

  start_vertex_ = std::make_shared<Vertex>();
  goal_vertex_ = std::make_shared<Vertex>();
  start_vertex_->state = start.state;
  goal_vertex_->state = goal.state;

  // update the current measure to problem space
  current_measure_ = problemMeasure(state_limits_);
  // now update rewiring lower bounds with current updated measure
  updateRewiringLowerBounds();

  vertices_.clear();
  edges_.clear();
  x_soln_.clear();
  vertices_.emplace_back(start_vertex_);
}

void RRTStar::near(const std::shared_ptr<const Vertex>& x_new,
                   std::vector<std::shared_ptr<Vertex>>& X_near) {
  double r = std::min(
      r_rrt_ * std::pow(std::log(static_cast<double>(vertices_.size())) /
                            (static_cast<double>(vertices_.size())),
                        1.0 / static_cast<double>(state_dimensions_)),
      max_distance_);

  for (const auto& v : vertices_) {
    if (distance(v, x_new) < r) {
      X_near.emplace_back(v);
    }
  }
}

void RRTStar::updateRewiringLowerBounds() {
  // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
  r_rrt_ =
      rewire_factor_ *
      std::pow(2 * (1.0 + 1.0 / static_cast<double>(state_dimensions_)) *
                   (current_measure_ / unitNBallMeasure(state_dimensions_)),
               1.0 / static_cast<double>(state_dimensions_));
}

void RRTStar::update() {
  if (planning_finished_) return;

  if (iteration_number_ <= max_iterations_) {
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
      // find all the nearest neighbours inside radius
      std::vector<std::shared_ptr<Vertex>> X_near;
      near(x_new, X_near);

      vertices_.emplace_back(x_new);

      // choose parent
      std::shared_ptr<Vertex> x_min = x_nearest;
      for (const auto& x_near : X_near) {
        double c_new = cost(x_near) + distance(x_near, x_new);
        if (c_new < cost(x_min) + distance(x_min, x_new)) {
          if (!isCollision(x_near, x_new)) {
            x_min = x_near;
          }
        }
      }
      x_new->parent = x_min;
      edges_.emplace_back(x_new->parent, x_new);

      // rewiring
      for (const auto& x_near : X_near) {
        double c_near = cost(x_new) + distance(x_new, x_near);
        if (c_near < cost(x_near)) {
          if (!isCollision(x_near, x_new)) {
            edges_.erase(std::remove(edges_.begin(), edges_.end(),
                                     std::make_pair(x_near->parent, x_near)),
                         edges_.end());
            x_near->parent = x_new;
            edges_.emplace_back(x_new, x_near);
          }
        }
      }

      // add into x_soln if the vertex is within the goal radius
      if (inGoalRegion(x_new)) {
        x_soln_.emplace_back(x_new);
      }
    }

    // update the best parent for the goal vertex every n iterations
    if (iteration_number_ % update_goal_every_ == 0) {
      if (x_soln_.size() > 0) {
        std::shared_ptr<Vertex> best_goal_parent;
        double min_goal_parent_cost = std::numeric_limits<double>::infinity();

        for (const auto& v : x_soln_) {
          double c = cost(v);
          if (c < min_goal_parent_cost) {
            min_goal_parent_cost = c;
            best_goal_parent = v;
          }
        }
        goal_vertex_->parent = best_goal_parent;
        solution_found_ = true;
      }
    }

    if (iteration_number_ % print_every_ == 0) {
      if (solution_found_) {
        std::cout << "Iter no. " << iteration_number_
                  << " | Solution cost: " << getSolutionCost() << std::endl;
      } else {
        std::cout << "Iter no. " << iteration_number_
                  << " | Solution is not found yet." << std::endl;
      }
    }

    iteration_number_++;
  } else {
    planning_finished_ = true;
    std::cout << "Iterations number reach max limit. Algorithm stopped."
              << '\n';
  }
}

}  // namespace psrr_planner