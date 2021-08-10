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

#include <psrr_planner/planners/base_planner.h>

namespace psrr_planner {
BasePlanner::BasePlanner(
    const StateLimits& state_limits,
    std::shared_ptr<GridCollisionChecker> collision_checker,
    double interpolation_dist, double goal_radius, unsigned int max_iterations,
    bool use_seed, unsigned int seed_number)
    : state_limits_{state_limits},
      collision_checker_{collision_checker},
      interpolation_dist_{interpolation_dist},
      goal_radius_{goal_radius},
      max_iterations_{max_iterations},
      use_seed_{use_seed},
      seed_number_{seed_number},
      solution_found_{false} {
  // TODO: make sure min and max are actual
  // minimum and maximum limits
  x_dis_ = std::uniform_real_distribution<float>(state_limits_.min_x,
                                                 state_limits_.max_x);
  y_dis_ = std::uniform_real_distribution<float>(state_limits_.min_y,
                                                 state_limits_.max_y);
  theta_dis_ = std::uniform_real_distribution<float>(state_limits_.min_theta,
                                                     state_limits_.max_theta);

  // currently we assume all x,y,theta state space is used
  // use_orientation ros param is not working here yet
  // will fix it later
  state_dimensions_ = 3;

  // assume state limits min and max joint pos have the same size
  // this must be taken care by the outside of the planner
  joint_pos_dis_.resize(state_limits_.min_joint_pos.size());
  for (std::size_t i = 0; i < state_limits_.min_joint_pos.size(); ++i) {
    joint_pos_dis_[i] = std::uniform_real_distribution<float>(
        state_limits_.min_joint_pos[i], state_limits_.max_joint_pos[i]);
  }

  state_dimensions_ += static_cast<int>(joint_pos_dis_.size());

  if (use_seed_) {
    rn_gen_ = std::mt19937(seed_number_);
  }
}

BasePlanner::~BasePlanner() {}

double BasePlanner::getSolutionCost() {
  double total_cost = 0.0;
  std::shared_ptr<Vertex> current = goal_vertex_;
  while (current->parent && current != start_vertex_) {
    total_cost += distance(current, current->parent);
    current = current->parent;
  }
  return total_cost;
}

double BasePlanner::distance(const std::shared_ptr<const Vertex>& v1,
                             const std::shared_ptr<const Vertex>& v2) {
  // since we are operating in SE(2) + R^n state space
  // we'll separate calculating distance functions into two
  // one for R^n and another for SO(2)

  double total_dist = 0.0;

  // calculate distance in R^n state space
  total_dist += (v1->state.x - v2->state.x) * (v1->state.x - v2->state.x) +
                (v1->state.y - v2->state.y) * (v1->state.y - v2->state.y);

  for (std::size_t i = 0; i < v1->state.joint_pos.size(); ++i) {
    total_dist += (v1->state.joint_pos[i] - v2->state.joint_pos[i]) *
                  (v1->state.joint_pos[i] - v2->state.joint_pos[i]);
  }

  total_dist = std::sqrt(total_dist);

  // calculate distance in SO(2) state space
  double rad_dist = fabs(v1->state.theta - v2->state.theta);
  if (rad_dist > M_PI) {
    rad_dist = 2.0 * M_PI - rad_dist;
  }
  total_dist += rad_dist;

  return total_dist;
}

void BasePlanner::interpolate(const std::shared_ptr<const Vertex>& from_v,
                              const std::shared_ptr<const Vertex>& to_v,
                              const double t,
                              const std::shared_ptr<Vertex>& v) {
  // this interpolation also needs to be separated into two parts for different
  // state spaces
  float x_new, y_new, theta_new;
  std::vector<float> joint_pos_new;
  joint_pos_new.resize(from_v->state.joint_pos.size());

  //////////////////////
  // R^n interpolation
  //////////////////////
  x_new = from_v->state.x +
          (to_v->state.x - from_v->state.x) * static_cast<float>(t);
  y_new = from_v->state.y +
          (to_v->state.y - from_v->state.y) * static_cast<float>(t);

  // for each of n joints, find new joint pos
  for (std::size_t i = 0; i < from_v->state.joint_pos.size(); ++i) {
    joint_pos_new[i] = from_v->state.joint_pos[i] +
                       (to_v->state.joint_pos[i] - from_v->state.joint_pos[i]) *
                           static_cast<float>(t);
  }

  ////////////////////////
  // SO(2) interpolation
  ////////////////////////
  float diff = to_v->state.theta - from_v->state.theta;
  if (fabs(diff) <= M_PI) {
    theta_new = from_v->state.theta + diff * static_cast<float>(t);
  } else {
    if (diff > 0.0)
      diff = 2.0 * M_PI - diff;
    else
      diff = -2.0 * M_PI - diff;
    theta_new = from_v->state.theta - diff * static_cast<float>(t);

    if (theta_new > M_PI)
      theta_new -= 2.0 * M_PI;
    else if (theta_new < -M_PI)
      theta_new += 2.0 * M_PI;
  }

  v->state.x = x_new;
  v->state.y = y_new;
  v->state.theta = theta_new;
  v->state.joint_pos = joint_pos_new;
}

bool BasePlanner::isCollision(const std::shared_ptr<const Vertex>& from_v,
                              const std::shared_ptr<const Vertex>& to_v) {
  // check collison from from_v to to_v
  // interpolate vertices between from_v and to_v
  // assume from_v is collision free

  const double max_dist = distance(from_v, to_v);

  double d = interpolation_dist_;
  while (d < max_dist) {
    std::shared_ptr<Vertex> temp_v = std::make_shared<Vertex>();
    interpolate(from_v, to_v, d / max_dist, temp_v);

    if (collision_checker_->isCollision(
            static_cast<double>(temp_v->state.x),
            static_cast<double>(temp_v->state.y),
            static_cast<double>(temp_v->state.theta),
            temp_v->state.joint_pos)) {
      return true;
    }

    d += interpolation_dist_;
  }

  // now we check the destination vertex to_v
  if (collision_checker_->isCollision(static_cast<double>(to_v->state.x),
                                      static_cast<double>(to_v->state.y),
                                      static_cast<double>(to_v->state.theta),
                                      to_v->state.joint_pos)) {
    return true;
  }

  return false;
}

void BasePlanner::sample(const std::shared_ptr<Vertex>& v) {
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

void BasePlanner::nearest(const std::shared_ptr<const Vertex>& x_rand,
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

bool BasePlanner::inGoalRegion(const std::shared_ptr<const Vertex>& v) {
  if (distance(v, goal_vertex_) <= goal_radius_) return true;
  return false;
}

double BasePlanner::cost(std::shared_ptr<Vertex> v) {
  std::shared_ptr<Vertex> curr_p = std::move(v);
  double cost = 0.0;
  while (curr_p->parent) {
    cost += distance(curr_p, curr_p->parent);
    curr_p = curr_p->parent;
  }
  return cost;
}

double BasePlanner::euclideanCost(std::shared_ptr<Vertex> v) {
  std::shared_ptr<Vertex> curr_p = std::move(v);
  double cost = 0.0;
  while (curr_p->parent) {
    cost += euclideanDistance(curr_p, curr_p->parent);
    curr_p = curr_p->parent;
  }
  return cost;
}

double BasePlanner::euclideanCost2D(std::shared_ptr<Vertex> v) {
  std::shared_ptr<Vertex> curr_p = std::move(v);
  double cost = 0.0;
  while (curr_p->parent) {
    cost += euclideanDistance2D(curr_p, curr_p->parent);
    curr_p = curr_p->parent;
  }
  return cost;
}

double BasePlanner::euclideanDistance(
    const std::shared_ptr<const Vertex>& v1,
    const std::shared_ptr<const Vertex>& v2) const {
  double total_dist = 0.0;

  // calculate distance in R^n state space
  total_dist += (v1->state.x - v2->state.x) * (v1->state.x - v2->state.x) +
                (v1->state.y - v2->state.y) * (v1->state.y - v2->state.y);

  for (std::size_t i = 0; i < v1->state.joint_pos.size(); ++i) {
    total_dist += (v1->state.joint_pos[i] - v2->state.joint_pos[i]) *
                  (v1->state.joint_pos[i] - v2->state.joint_pos[i]);
  }

  total_dist = std::sqrt(total_dist);
  return total_dist;
}

double BasePlanner::euclideanDistance2D(
    const std::shared_ptr<const Vertex>& v1,
    const std::shared_ptr<const Vertex>& v2) const {
  double total_dist = 0.0;

  // calculate distance in R^2 state space
  total_dist += (v1->state.x - v2->state.x) * (v1->state.x - v2->state.x) +
                (v1->state.y - v2->state.y) * (v1->state.y - v2->state.y);

  total_dist = std::sqrt(total_dist);
  return total_dist;
}
}  // namespace psrr_planner