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

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

// ompl related
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;

namespace psrr_planner {
namespace utils {

// n-dimensional state of the self-reconfigurable robot
struct State {
  double x;
  double y;
  double theta;
  std::vector<double> joint_pos;
};

struct StateLimits {
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float min_theta;
  float max_theta;
  std::vector<float> min_joint_pos;
  std::vector<float> max_joint_pos;
};

struct Vertex {
  State state;
  std::shared_ptr<Vertex> parent{nullptr};
};

/**
 * @brief Convenient function for converting from psrr_planner::State into
 * ompl::base::ScopedStatePtr
 */
inline void updateOMPLState(ob::ScopedStatePtr ompl_state,
                            std::shared_ptr<const State> state) {
  // number of dimensions in R^n excluding x and y
  const auto rn_size = state->joint_pos.size();

  // x and y in R^n
  (*ompl_state)[0] = state->x;
  (*ompl_state)[1] = state->y;

  // left-over portion of R^n
  for (std::size_t i = 0; i < rn_size; ++i) {
    (*ompl_state)[i + 2] = state->joint_pos[i];
  }

  // SO(2) State Space
  (*ompl_state)
      ->as<ob::CompoundState>()
      ->as<ob::SO2StateSpace::StateType>(1)
      ->value = state->theta;
}

inline void convertToQuaternion(const double& angle,
                                geometry_msgs::Quaternion& quat) {
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  quat = tf2::toMsg(q);
}

/**
 * @brief Convenient function for converting from ompl::base::State into
 * geometry_msgs::Pose SE2 state
 */
inline geometry_msgs::Pose omplStateToSE2Pose(const ob::State* state) {
  if (!state) {
    throw std::runtime_error("No state found for vertex");
  }
  const auto* rn_state =
      state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(
          0);
  const auto* so2_state =
      state->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(1);

  geometry_msgs::Pose temp_pose;
  temp_pose.position.x = rn_state->values[0];
  temp_pose.position.y = rn_state->values[1];
  convertToQuaternion(so2_state->value, temp_pose.orientation);
  return temp_pose;
}

/**
 * @brief Convenient function for converting from ompl::base::State into
 * sensor_msgs::JointState R^n states excluding x and y positions
 */
inline sensor_msgs::JointState omplStateToJointState(const ob::State* state,
                                                     unsigned int ndim_joint) {
  if (!state) {
    throw std::runtime_error("No state found for vertex");
  }
  const auto* rn_state =
      state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(
          0);
  sensor_msgs::JointState joint_state;
  joint_state.position.resize(static_cast<std::size_t>(ndim_joint));
  for (std::size_t i = 0; i < static_cast<std::size_t>(ndim_joint); ++i) {
    joint_state.position[i] = rn_state->values[i + 2];
  }
  return joint_state;
}

/**
 * @brief The Lebesgue measure (i.e., "volume") of an n-dimensional ball with a
 * unit radius.
 * Ref: https://ompl.kavrakilab.org/GeometricEquations_8cpp_source.html
 */
inline double unitNBallMeasure(unsigned int n) {
  // This is the radius version with r removed (as it is 1) for efficiency
  return std::pow(std::sqrt(boost::math::constants::pi<double>()),
                  static_cast<double>(n)) /
         std::tgamma(static_cast<double>(n) / 2.0 + 1.0);
}

/**
 * @brief The Lebesgue measure (i.e., "volume") of an n-dimensional prolate
 * hyperspheroid (a symmetric hyperellipse).
 * Ref: https://ompl.kavrakilab.org/GeometricEquations_8cpp_source.html
 * @param n number of dimensions
 * @param d_foci distance between two focii
 * @param d_transverse transverse diameter
 */
inline double prolateHyperspheroidMeasure(unsigned int n, double d_foci,
                                          double d_transverse) {
  // Sanity check input
  if (d_transverse < d_foci) {
    std::cout << "this" << std::endl;
    throw(
        "Transverse diameter cannot be less than the minimum transverse "
        "diameter.");
  }

  // Variable
  // The conjugate diameter:
  double conjugate_diameter;
  // The Lebesgue measure return value
  double lmeas;

  // Calculate the conjugate diameter:
  conjugate_diameter = std::sqrt(d_transverse * d_transverse - d_foci * d_foci);

  // Calculate the volume
  // First multiply together the radii, noting that the first radius is the
  // transverse diameter/2.0, and the other N-1 are the conjugate diameter/2.0
  lmeas = d_transverse / 2.0;
  for (unsigned int i = 1u; i < n; ++i) {
    lmeas = lmeas * conjugate_diameter / 2.0;
  }

  // Then multiply by the volume of the unit n-ball.
  lmeas = lmeas * unitNBallMeasure(n);

  // Finally return:
  return lmeas;
}

/**
 * @brief Calculate the measure (i.e., "n-dimensional volume") of a problem with
 * R^n and SO(2) state spaces
 */
inline double problemMeasure(const StateLimits& state_limits) {
  double measure = 1.0;

  // real vector components first
  // x and y
  measure *= (state_limits.max_x - state_limits.min_x) *
             (state_limits.max_y - state_limits.min_y);
  // joints
  for (std::size_t i = 0; i < state_limits.min_joint_pos.size(); ++i) {
    measure *= state_limits.max_joint_pos[i] - state_limits.min_joint_pos[i];
  }

  // SO(2)
  measure *= 2.0 * M_PI;

  return measure;
}

}  // namespace utils
}  // namespace psrr_planner