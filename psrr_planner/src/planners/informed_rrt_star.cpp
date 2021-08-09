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

#include <psrr_planner/planners/informed_rrt_star.h>

namespace psrr_planner {
InformedRRTStar::InformedRRTStar(
    const StateLimits& state_limits,
    std::shared_ptr<GridCollisionChecker> collision_checker,
    unsigned int max_iterations, unsigned int goal_parent_size_interval,
    double max_distance, double rewire_factor, double interpolation_dist,
    double goal_radius, bool use_seed, unsigned int seed_number,
    unsigned int print_every)
    : BasePlanner(state_limits, collision_checker, interpolation_dist, use_seed,
                  seed_number),
      max_iterations_{max_iterations},
      goal_parent_size_interval_{goal_parent_size_interval},
      max_distance_{max_distance},
      rewire_factor_{rewire_factor},
      goal_radius_{goal_radius},
      print_every_{print_every} {}

InformedRRTStar::~InformedRRTStar(){};

void InformedRRTStar::init(const Vertex& start, const Vertex& goal) {
  planning_finished_ = false;
  solution_found_ = false;
  iteration_number_ = 1;

  start_vertex_ = std::make_shared<Vertex>();
  goal_vertex_ = std::make_shared<Vertex>();
  start_vertex_->state = start.state;
  goal_vertex_->state = goal.state;

  // first find the number of informed dimensions
  // we use [x,y + len(joints)] as no: of dimensions for informed states and
  // [informed_dims_ + SO(2)] for total state dimensions
  // TODO: here we assume start and goal has same informed dims
  // although we need to have sanity check here just in case
  informed_dims_ = 2 + start_vertex_->state.joint_pos.size();
  state_dims_ = informed_dims_ + 1;

  // create two focii with only informed state spaces
  convertVertexToVectorXd(start_vertex_, x_start_focus_);
  convertVertexToVectorXd(goal_vertex_, x_goal_focus_);

  c_i_ = std::numeric_limits<double>::infinity();
  c_min_ = (x_start_focus_ - x_goal_focus_).norm();
  x_center_ = 0.5 * (x_start_focus_ - x_goal_focus_);
  updateRotationMatrix();

  // initially no solution is found yet
  // update the current measure to problem space
  current_measure_ = problemMeasure(state_limits_);
  // now update rewiring lower bounds with current updated measure
  updateRewiringLowerBounds();

  vertices_.clear();
  edges_.clear();
  x_soln_.clear();
  vertices_.emplace_back(start_vertex_);
}

void InformedRRTStar::convertVertexToVectorXd(
    const std::shared_ptr<const Vertex>& v, Eigen::VectorXd& vec) {
  vec.resize(informed_dims_);
  vec[0] = v->state.x;
  vec[1] = v->state.y;
  for (std::size_t i = 0; i < v->state.joint_pos.size(); ++i) {
    vec[i + 2] = v->state.joint_pos[i];
  }
}

void InformedRRTStar::updateRotationMatrix() {
  // Variables
  // The transverse axis of the PHS expressed in the world frame.
  Eigen::VectorXd transverse_axis(informed_dims_);
  // The matrix representation of the Wahba problem
  Eigen::MatrixXd wahba_prob(informed_dims_, informed_dims_);
  // The middle diagonal matrix in the SVD solution to the Wahba problem
  Eigen::VectorXd middle_m(informed_dims_);

  // Calculate the major axis, storing as the first eigenvector
  transverse_axis = (x_goal_focus_ - x_start_focus_) / c_min_;

  // Calculate the rotation that will allow us to generate the remaining
  // eigenvectors Formulate as a Wahba problem, first forming the matrix
  // a_j*a_i' where a_j is the transverse axis if the ellipse in the world
  // frame, and a_i is the first basis vector of the world frame (i.e., [1 0
  // .... 0])
  wahba_prob = transverse_axis *
               Eigen::MatrixXd::Identity(informed_dims_, informed_dims_)
                   .col(0)
                   .transpose();

  // Then run it through the  SVD solver
  Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::NoQRPreconditioner> svd(
      wahba_prob, Eigen::ComputeFullV | Eigen::ComputeFullU);

  // Then calculate the rotation matrix from the U and V components of SVD
  // Calculate the middle diagonal matrix
  middle_m = Eigen::VectorXd::Ones(informed_dims_);
  // Make the last value equal to det(U)*det(V) (zero-based indexing remember)
  middle_m(informed_dims_ - 1) =
      svd.matrixU().determinant() * svd.matrixV().determinant();

  // Calculate the rotation
  rotation_world_from_ellipse_ =
      svd.matrixU() * middle_m.asDiagonal() * svd.matrixV().transpose();
}

void InformedRRTStar::updateTransformationMatrix() {
  // here we assume both rotation matrix and c_i (conjugate diameter) exists

  // Variables
  // The radii of the ellipse
  Eigen::VectorXd diag_as_vector(informed_dims_);
  // The conjugate diameters:
  double conjugate_diamater;

  // Calculate the conjugate diameter
  conjugate_diamater = std::sqrt(c_i_ * c_i_ - c_min_ * c_min_);

  // Store into the diagonal matrix
  // All the elements but one are the conjugate radius
  diag_as_vector.fill(conjugate_diamater / 2.0);

  // The first element in diagonal is the transverse radius
  diag_as_vector(0) = 0.5 * c_i_;

  // Calculate the transformation matrix
  transformation_world_from_ellipse_ =
      rotation_world_from_ellipse_ * diag_as_vector.asDiagonal();
}

void InformedRRTStar::uniformNormalVector(std::vector<double>& v) {
  boost::uniform_on_sphere<double> unif_sphere(v.size());
  boost::variate_generator<gen_type&, boost::uniform_on_sphere<double>>
      random_on_sphere(rn_gen_, unif_sphere);
  v = random_on_sphere();
}

// Ref: https://ompl.kavrakilab.org/RandomNumbers_8cpp_source.html
// See: http://math.stackexchange.com/a/87238
void InformedRRTStar::uniformInBall(double r, std::vector<double>& v) {
  // Draw a random point on the unit sphere
  uniformNormalVector(v);

  // Draw a random radius scale
  std::uniform_real_distribution<double> dis(0.0, 1.0);
  double radiusScale =
      r * std::pow(dis(rn_gen_), 1.0 / static_cast<double>(v.size()));

  // Scale the point on the unit sphere
  std::transform(v.begin(), v.end(), v.begin(),
                 [radiusScale](double x) { return radiusScale * x; });
}

void InformedRRTStar::sample(const std::shared_ptr<Vertex>& v) {
  if (!use_seed_) {
    std::random_device rd;
    rn_gen_ = std::mt19937(rd());
  }
  // sample on the hyperellipsoid
  if (c_i_ < std::numeric_limits<double>::infinity()) {
    unsigned int i = 0;
    bool found_informed_sample = false;
    while (i < max_sampling_tries_) {
      i++;
      // The spherical point as a std::vector
      std::vector<double> sphere(informed_dims_);

      // Get a random point in the sphere
      uniformInBall(1.0, sphere);

      // now we need to transform this point to the prolate hyperspheroid
      // update transformation matrix first
      updateTransformationMatrix();

      std::vector<double> informed_state(informed_dims_);
      Eigen::Map<Eigen::VectorXd>(&informed_state[0], informed_dims_) =
          transformation_world_from_ellipse_ *
          Eigen::Map<const Eigen::VectorXd>(&sphere[0], informed_dims_);
      Eigen::Map<Eigen::VectorXd>(&informed_state[0], informed_dims_) +=
          x_center_;

      // make sure sampled informed state satisfies bounds
      // check x
      if (!(informed_state[0] >= state_limits_.min_x &&
            informed_state[0] <= state_limits_.max_x))
        continue;
      // check y
      if (!(informed_state[1] >= state_limits_.min_y &&
            informed_state[1] <= state_limits_.max_y))
        continue;

      // check joint pos
      bool is_joints_satisfied = true;
      for (std::size_t i = 0; i < state_limits_.min_joint_pos.size(); ++i) {
        if (!(informed_state[i + 2] >= state_limits_.min_joint_pos[i] &&
              informed_state[i + 2] <= state_limits_.max_joint_pos[i]))
          is_joints_satisfied = false;
      }
      if (!is_joints_satisfied) continue;

      // this means informed sampled state satisfies all the limits
      // now we create sampled vertex
      // make sure SO(n) component is sampled and added into the sampled vertex
      v->state.x = informed_state[0];
      v->state.y = informed_state[1];
      v->state.theta = theta_dis_(rn_gen_);
      v->state.joint_pos.resize(joint_pos_dis_.size());
      for (std::size_t i = 0; i < joint_pos_dis_.size(); ++i) {
        v->state.joint_pos[i] = informed_state[i + 2];
      }
      found_informed_sample = true;
      break;
    }

    // if informed sample is not found with maximum number of tries
    // we just sample on the whole problem space, otherwise return
    if (found_informed_sample) return;
  }

  // sample problem with state limits
  v->state.x = x_dis_(rn_gen_);
  v->state.y = y_dis_(rn_gen_);
  v->state.theta = theta_dis_(rn_gen_);

  v->state.joint_pos.resize(joint_pos_dis_.size());
  for (std::size_t i = 0; i < joint_pos_dis_.size(); ++i) {
    v->state.joint_pos[i] = joint_pos_dis_[i](rn_gen_);
  }
}

void InformedRRTStar::nearest(const std::shared_ptr<const Vertex>& x_rand,
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

void InformedRRTStar::near(const std::shared_ptr<const Vertex>& x_new,
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

void InformedRRTStar::updateRewiringLowerBounds() {
  // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
  r_rrt_ = rewire_factor_ *
           std::pow(2 * (1.0 + 1.0 / static_cast<double>(state_dims_)) *
                        (current_measure_ / unitNBallMeasure(state_dims_)),
                    1.0 / static_cast<double>(state_dims_));
}

double InformedRRTStar::cost(std::shared_ptr<Vertex> v) {
  std::shared_ptr<Vertex> curr_p = std::move(v);
  double cost = 0.0;
  while (curr_p->parent) {
    cost += distance(curr_p, curr_p->parent);
    curr_p = curr_p->parent;
  }
  return cost;
}

double InformedRRTStar::euclideanCost(std::shared_ptr<Vertex> v) {
  std::shared_ptr<Vertex> curr_p = std::move(v);
  double cost = 0.0;
  while (curr_p->parent) {
    cost += euclideanDistance(curr_p, curr_p->parent);
    curr_p = curr_p->parent;
  }
  return cost;
}

double InformedRRTStar::euclideanDistance(
    const std::shared_ptr<const Vertex>& v1,
    const std::shared_ptr<const Vertex>& v2) {
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

bool inGoalRegion(const std::shared_ptr<const Vertex>& v) { return true; }

void InformedRRTStar::update() {
  if (planning_finished_) return;

  if (iteration_number_ <= max_iterations_) {
    // find the current best solution cost and parent vertex
    std::shared_ptr<Vertex> best_vertex;
    double min_cost = std::numeric_limits<double>::infinity();
    for (const auto& v : x_soln_) {
      double c = euclideanCost(v);
      if (c < min_cost) {
        min_cost = c;
        best_vertex = v;
      }
    }

    if (min_cost < c_i_) {
      c_i_ = min_cost;
      goal_vertex_->parent = best_vertex;

      // Update the new measure:
      // make sure we include SO(2) component
      current_measure_ =
          prolateHyperspheroidMeasure(informed_dims_, c_min_, c_i_) * 2.0 *
          M_PI;
    }

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

double InformedRRTStar::getSolutionCost() {
  double total_cost = 0.0;
  std::shared_ptr<Vertex> current = goal_vertex_;
  while (current->parent && current != start_vertex_) {
    total_cost += distance(current, current->parent);
    current = current->parent;
  }
  return total_cost;
}
}  // namespace psrr_planner