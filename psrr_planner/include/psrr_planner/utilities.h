#pragma once

#include <iostream>
#include <memory>
#include <vector>

namespace psrr_planner {

// n-dimensional state of the self-reconfigurable robot
struct State {
  float x;
  float y;
  float theta;
  std::vector<float> joint_pos;
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

}  // namespace psrr_planner