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
                 unsigned int max_vertices,
                 unsigned int goal_parent_size_interval, double max_distance,
                 double r_rrt, double interpolation_dist, double goal_radius,
                 bool use_seed, unsigned int seed_number)
    : BasePlanner(state_limits, collision_checker, interpolation_dist, use_seed,
                  seed_number),
      max_vertices_{max_vertices},
      goal_parent_size_interval_{goal_parent_size_interval},
      max_distance_{max_distance},
      r_rrt_{r_rrt},
      goal_radius_{goal_radius} {}

RRTStar::~RRTStar(){};

void RRTStar::init(const Vertex& start, const Vertex& goal) {
  planning_finished_ = false;
  solution_found_ = false;

  start_vertex_ = std::make_shared<Vertex>();
  goal_vertex_ = std::make_shared<Vertex>();
  start_vertex_->state = start.state;
  goal_vertex_->state = goal.state;

  vertices_.clear();
  edges_.clear();

  vertices_.emplace_back(start_vertex_);
}

void RRTStar::sampleFree(const std::shared_ptr<Vertex>& v) {
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

void RRTStar::nearest(const std::shared_ptr<const Vertex>& x_rand,
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

double RRTStar::cost(std::shared_ptr<Vertex> v) {
  std::shared_ptr<Vertex> curr_p = std::move(v);
  double cost = 0.0;
  while (curr_p->parent) {
    cost += distance(curr_p, curr_p->parent);
    curr_p = curr_p->parent;
  }
  return cost;
}

void RRTStar::update() {
  if (planning_finished_) return;

  std::shared_ptr<Vertex> x_rand = std::make_shared<Vertex>();
  std::shared_ptr<Vertex> x_nearest = std::make_shared<Vertex>();
  std::shared_ptr<Vertex> x_new = std::make_shared<Vertex>();

  sampleFree(x_rand);
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
  }

  // only find best parent of goal vertex for fix amount of time
  if (vertices_.size() % goal_parent_size_interval_ == 0) {
    // find nearest vertices
    std::vector<std::shared_ptr<Vertex>> nearest_vertices;
    for (const auto& v : vertices_) {
      double dist = distance(v, goal_vertex_);
      if (dist < goal_radius_) {
        nearest_vertices.emplace_back(v);
      }
    }

    if (nearest_vertices.size() > 0) {
      std::shared_ptr<Vertex> bestVertex;
      double bestCost = std::numeric_limits<double>::infinity();
      for (const auto& v : nearest_vertices) {
        double c = cost(v);
        if (c < bestCost) {
          bestCost = c;
          bestVertex = v;
        }
      }
      goal_vertex_->parent = bestVertex;
      solution_found_ = true;
    }
  }

  if (vertices_.size() > max_vertices_ - 1) {
    std::cout << "Vertices reach maximum limit. Algorithm stopped." << '\n';
    planning_finished_ = true;
  }
}

double RRTStar::getSolutionCost() {
  double total_cost = 0.0;
  std::shared_ptr<Vertex> current = goal_vertex_;
  while (current->parent && current != start_vertex_) {
    total_cost += distance(current, current->parent);
    current = current->parent;
  }
  return total_cost;
}
}  // namespace psrr_planner