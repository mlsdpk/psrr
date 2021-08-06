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

#include <psrr_planner/rrt.h>

namespace psrr_planner {
RRT::RRT(const StateLimits& state_limits, unsigned int max_vertices,
         std::shared_ptr<GridCollisionChecker> collision_checker)
    : state_limits_{state_limits},
      max_vertices_{max_vertices},
      collision_checker_{collision_checker} {
  // TODO: make sure min and max are actual minimum and maximum limits
  x_dis_ = std::uniform_real_distribution<float>(state_limits_.min_x,
                                                 state_limits_.max_x);
  y_dis_ = std::uniform_real_distribution<float>(state_limits_.min_y,
                                                 state_limits_.max_y);
  theta_dis_ = std::uniform_real_distribution<float>(state_limits_.min_theta,
                                                     state_limits_.max_theta);

  // assume state limits min and max joint pos have the same size
  // this must be taken care by the outside of the planner
  joint_pos_dis_.resize(state_limits_.min_joint_pos.size());
  for (std::size_t i = 0; i < state_limits_.min_joint_pos.size(); ++i) {
    joint_pos_dis_[i] = std::uniform_real_distribution<float>(
        state_limits_.min_joint_pos[i], state_limits_.max_joint_pos[i]);
  }

  start_vertex_ = std::make_shared<Vertex>();
  goal_vertex_ = std::make_shared<Vertex>();
}

RRT::~RRT(){};

void RRT::init(std::shared_ptr<Vertex> start, std::shared_ptr<Vertex> goal) {
  // TODO: make these parameters as ROS params
  delta_q_ = 0.5;
  interpolation_dist_ = 0.01;
  goal_radius_ = 0.5;
  stopped_ = false;
  solution_found_ = false;

  start_vertex_ = start;
  goal_vertex_ = goal;

  vertices_.emplace_back(start_vertex_);
}

void RRT::sampleFree(Vertex& v) {
  std::random_device rd;
  std::mt19937 gen(rd());

  v.state.x = x_dis_(gen);
  v.state.y = y_dis_(gen);
  v.state.theta = theta_dis_(gen);

  v.state.joint_pos.resize(joint_pos_dis_.size());
  for (std::size_t i = 0; i < joint_pos_dis_.size(); ++i) {
    v.state.joint_pos[i] = joint_pos_dis_[i](gen);
  }
}

void RRT::nearest(const Vertex& x_rand, std::shared_ptr<Vertex>& x_near) {
  double minDist = std::numeric_limits<double>::infinity();

  for (const auto& v : vertices_) {
    double dist = distance(*v, x_rand);
    if (dist < minDist) {
      minDist = dist;
      x_near = v;
    }
  }
}

double RRT::distance(const Vertex& v1, const Vertex& v2) {
  // since we are operating in SE(2) + R^n state space
  // we'll separate calculating distance functions into two
  // one for R^n and another for SO(2)

  double total_dist = 0.0;

  // calculate distance in R^n state space
  total_dist += (v1.state.x - v2.state.x) * (v1.state.x - v2.state.x) +
                (v1.state.y - v2.state.y) * (v1.state.y - v2.state.y);

  for (std::size_t i = 0; i < v1.state.joint_pos.size(); ++i) {
    total_dist += (v1.state.joint_pos[i] - v2.state.joint_pos[i]) *
                  (v1.state.joint_pos[i] - v2.state.joint_pos[i]);
  }

  total_dist = std::sqrt(total_dist);

  // calculate distance in SO(2) state space
  double rad_dist = fabs(v1.state.theta - v2.state.theta);
  if (rad_dist > M_PI) {
    rad_dist = 2.0 * M_PI - rad_dist;
  }
  total_dist += rad_dist;

  return total_dist;
}

void RRT::interpolate(const Vertex& from_v, const Vertex& to_v, const double t,
                      std::shared_ptr<Vertex> v) {
  // this interpolation also needs to be separated into two parts for different
  // state spaces
  float x_new, y_new, theta_new;
  std::vector<float> joint_pos_new;
  joint_pos_new.resize(from_v.state.joint_pos.size());

  //////////////////////
  // R^n interpolation
  //////////////////////
  x_new =
      from_v.state.x + (to_v.state.x - from_v.state.x) * static_cast<float>(t);
  y_new =
      from_v.state.y + (to_v.state.y - from_v.state.y) * static_cast<float>(t);

  // for each of n joints, find new joint pos
  for (std::size_t i = 0; i < from_v.state.joint_pos.size(); ++i) {
    joint_pos_new[i] = from_v.state.joint_pos[i] +
                       (to_v.state.joint_pos[i] - from_v.state.joint_pos[i]) *
                           static_cast<float>(t);
  }

  ////////////////////////
  // SO(2) interpolation
  ////////////////////////
  float diff = to_v.state.theta - from_v.state.theta;
  if (fabs(diff) <= M_PI) {
    theta_new = from_v.state.theta + diff * static_cast<float>(t);
  } else {
    if (diff > 0.0)
      diff = 2.0 * M_PI - diff;
    else
      diff = -2.0 * M_PI - diff;
    theta_new = from_v.state.theta - diff * static_cast<float>(t);

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

bool RRT::isCollision(const Vertex& from_v, const Vertex& to_v) {
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
  if (collision_checker_->isCollision(
          static_cast<double>(to_v.state.x), static_cast<double>(to_v.state.y),
          static_cast<double>(to_v.state.theta), to_v.state.joint_pos)) {
    return true;
  }

  return false;
}

void RRT::update() {
  if (stopped_) return;

  Vertex x_rand;
  std::shared_ptr<Vertex> x_nearest = std::make_shared<Vertex>();
  std::shared_ptr<Vertex> x_new = std::make_shared<Vertex>();

  sampleFree(x_rand);
  nearest(x_rand, x_nearest);

  // find the distance between x_rand and x_nearest
  double d = distance(x_rand, *x_nearest);

  // if this distance d > delta_q, we need to find nearest state in the
  // direction of x_rand
  if (d > delta_q_) {
    interpolate(*x_nearest, x_rand, delta_q_ / d, x_new);
  } else {
    x_new->state = x_rand.state;
  }

  if (!isCollision(*x_nearest, *x_new)) {
    x_new->parent = x_nearest;
    vertices_.emplace_back(x_new);
    edges_.emplace_back(x_nearest, x_new);

    double goal_dist = distance(*x_new, *goal_vertex_);

    if (goal_dist < goal_radius_) {
      goal_vertex_->parent = x_new;
      std::cout << "Solution found. Algorithm stopped." << '\n';
      stopped_ = true;
      solution_found_ = true;
    }
  }

  if (vertices_.size() > max_vertices_ - 1) {
    std::cout << "Vertices reach maximum limit. Algorithm stopped." << '\n';
    stopped_ = true;
  }
}

const std::vector<std::shared_ptr<Vertex>>* RRT::getVertices() const {
  return &vertices_;
}

const std::vector<std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Vertex>>>*
RRT::getEdges() const {
  return &edges_;
}

bool RRT::hasSolution() const { return solution_found_; }
}  // namespace psrr_planner