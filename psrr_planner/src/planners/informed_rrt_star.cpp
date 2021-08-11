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
    unsigned int max_iterations, unsigned int max_sampling_tries,
    double max_distance, double rewire_factor, double interpolation_dist,
    double goal_radius, double goal_bias, unsigned int update_goal_every,
    bool use_greedy_informed_set, bool use_seed, unsigned int seed_number,
    unsigned int print_every)
    : RRTStar(state_limits, collision_checker, max_iterations,
              update_goal_every, max_distance, rewire_factor,
              interpolation_dist, goal_radius, goal_bias, use_seed, seed_number,
              print_every),
      max_sampling_tries_{max_sampling_tries},
      use_greedy_informed_set_{use_greedy_informed_set} {}

InformedRRTStar::~InformedRRTStar(){};

void InformedRRTStar::init(const Vertex& start, const Vertex& goal,
                           unsigned int planning_time) {
  planning_finished_ = false;
  solution_found_ = false;
  iteration_number_ = 1;

  start_vertex_ = std::make_shared<Vertex>();
  goal_vertex_ = std::make_shared<Vertex>();
  start_vertex_->state = start.state;
  goal_vertex_->state = goal.state;

  // first find the number of informed dimensions
  // we use [x,y + len(joints)] as no: of dimensions for informed states
  // TODO: here we assume start and goal has same informed dims
  // although we need to have sanity check here just in case
  informed_dims_ = 2 + start_vertex_->state.joint_pos.size();

  // create two focii with only informed state spaces
  convertVertexToVectorXd(start_vertex_, x_start_focus_);
  convertVertexToVectorXd(goal_vertex_, x_goal_focus_);

  // 2d ellipse properties for visualization purposes
  transverse_dia_2d_.clear();
  transverse_dia_2d_.resize(2);
  conjugate_dia_2d_.clear();
  conjugate_dia_2d_.resize(2);
  d_focii_2d_ = euclideanDistance2D(start_vertex_, goal_vertex_);
  ellipse_orien_2d_ =
      std::atan2(goal_vertex_->state.y - start_vertex_->state.y,
                 goal_vertex_->state.x - start_vertex_->state.x);
  ellipse_center_2d_.resize(2);
  ellipse_center_2d_[0] =
      0.5 * (start_vertex_->state.x + goal_vertex_->state.x);
  ellipse_center_2d_[1] =
      0.5 * (start_vertex_->state.y + goal_vertex_->state.y);

  c_i_ = std::numeric_limits<double>::infinity();
  c_min_ = (x_start_focus_ - x_goal_focus_).norm();
  x_center_ = 0.5 * (x_start_focus_ + x_goal_focus_);
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

  use_planning_time_ = false;
  if (planning_time >= 1u) {
    // convert planning time seconds to milliseconds
    planning_time_ = planning_time * 1000u;
    use_planning_time_ = true;
  }

  init_time_ = std::chrono::system_clock::now();
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

void InformedRRTStar::updateConjugateDiameter2D() {
  conjugate_dia_2d_[0] =
      std::sqrt(transverse_dia_2d_[0] * transverse_dia_2d_[0] -
                d_focii_2d_ * d_focii_2d_);
  conjugate_dia_2d_[1] =
      std::sqrt(transverse_dia_2d_[1] * transverse_dia_2d_[1] -
                d_focii_2d_ * d_focii_2d_);
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

  // goal biasing distribution
  std::uniform_real_distribution<double> dis(0.0, 1.0);
  if (dis(rn_gen_) > goal_bias_) {
    // sample on the hyperellipsoid
    if (c_i_ < std::numeric_limits<double>::infinity()) {
      unsigned int iter = 0;
      bool found_informed_sample = false;
      while (iter < max_sampling_tries_) {
        iter++;

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
        // make sure SO(n) component is sampled and added into the sampled
        // vertex
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
      if (found_informed_sample) {
        return;
      }
    }

    // sample problem with state limits
    v->state.x = x_dis_(rn_gen_);
    v->state.y = y_dis_(rn_gen_);
    v->state.theta = theta_dis_(rn_gen_);

    v->state.joint_pos.resize(joint_pos_dis_.size());
    for (std::size_t i = 0; i < joint_pos_dis_.size(); ++i) {
      v->state.joint_pos[i] = joint_pos_dis_[i](rn_gen_);
    }
  } else {
    // otherwise, sampled at goal vertex
    v->state.x = goal_vertex_->state.x;
    v->state.y = goal_vertex_->state.y;
    v->state.theta = goal_vertex_->state.theta;

    v->state.joint_pos.resize(joint_pos_dis_.size());
    for (std::size_t i = 0; i < joint_pos_dis_.size(); ++i) {
      v->state.joint_pos[i] = goal_vertex_->state.joint_pos[i];
    }
  }
}

double InformedRRTStar::heuristicCost(
    const std::shared_ptr<const Vertex>& v) const {
  // heuristic cost to come + heuristic cost to go
  return euclideanDistance(start_vertex_, v) +
         euclideanDistance(v, goal_vertex_);
}

double InformedRRTStar::heuristicCost2D(
    const std::shared_ptr<const Vertex>& v) const {
  // 2D heuristic cost to come + heuristic cost to go
  return euclideanDistance2D(start_vertex_, v) +
         euclideanDistance2D(v, goal_vertex_);
}

void InformedRRTStar::update() {
  if (planning_finished_) return;

  // find the current best solution cost and parent vertex
  double best_cost = std::numeric_limits<double>::infinity();
  double greedy_best_cost = std::numeric_limits<double>::infinity();
  double best_cost_2d, greedy_best_cost_2d;

  if (x_soln_.size() > 0) {
    // find the best parent vertex for the goal
    std::shared_ptr<Vertex> best_goal_parent;
    for (const auto& v : x_soln_) {
      double c = euclideanCost(v);
      if (c < best_cost) {
        best_cost = c;
        best_goal_parent = v;
      }
    }

    // now we need to find properties for both informed set and greedy
    // informed set just for visualization purposes, however we only use one
    // during sampling based on use_greedy_informed_set_ flag

    // greedy informed set stuffs
    double greedy_max_cost = 0.0;
    std::shared_ptr<Vertex> greedy_max_vertex;
    std::shared_ptr<Vertex> curr_v = best_goal_parent;
    while (curr_v->parent) {
      double hc = heuristicCost(curr_v);
      if (hc > greedy_max_cost) {
        greedy_max_cost = hc;
        greedy_max_vertex = curr_v;
      }
      curr_v = curr_v->parent;
    }
    greedy_best_cost = greedy_max_cost;
    greedy_best_cost_2d = heuristicCost2D(greedy_max_vertex);

    // informed set stuffs
    best_cost += euclideanDistance(best_goal_parent, goal_vertex_);
    // we need to find 2D transverse diameter
    // we gonna use it for visualization purposes
    best_cost_2d = euclideanCost2D(best_goal_parent) +
                   euclideanDistance2D(best_goal_parent, goal_vertex_);
  }

  bool c_i_updated = false;
  if (use_greedy_informed_set_) {
    if (greedy_best_cost < c_i_) {
      c_i_ = greedy_best_cost;
      c_i_updated = true;
    }
  } else {
    if (best_cost < c_i_) {
      c_i_ = best_cost;
      c_i_updated = true;
    }
  }

  if (c_i_updated) {
    // update 2D transverse and conjugate diameters
    transverse_dia_2d_[0] = best_cost_2d;
    transverse_dia_2d_[1] = greedy_best_cost_2d;
    updateConjugateDiameter2D();

    // Update the new measure:
    // make sure we include SO(2) component
    current_measure_ =
        prolateHyperspheroidMeasure(informed_dims_, c_min_, c_i_) * 2.0 * M_PI;
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