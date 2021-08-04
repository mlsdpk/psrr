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

  init();
}

RRT::~RRT(){};

void RRT::init() {
  delta_q_ = 0.1;
  stopped_ = false;
}

void RRT::sampleFree(Vertex& v) {
  std::random_device rd;
  std::mt19937 gen(rd());

  v.state.x = x_dis_(gen);
  v.state.y = y_dis_(gen);
  v.state.theta = theta_dis_(gen);
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

void RRT::steer(const Vertex& x_rand, const std::shared_ptr<Vertex>& x_near,
                std::shared_ptr<Vertex>& x_new) {
  //
}

double RRT::distance(const Vertex& v1, const Vertex& v2) {
  // since we are operating in SE(2) + R^n state space
  // we'll separate calculating distance functions into two
  // one for R^n and another for SO(2)

  double total_dist = 0.0;

  // calculate distance in R^n state space
  // leave joint states for now
  total_dist +=
      std::sqrt((v1.state.x - v2.state.x) * (v1.state.x - v2.state.x) +
                (v1.state.y - v2.state.y) * (v1.state.y - v2.state.y));

  // calculate distance in SO(2) state space
  double rad_dist = fabs(v1.state.theta - v2.state.theta);
  if (rad_dist > M_PI) {
    rad_dist = 2.0 * M_PI - rad_dist;
  }
  total_dist += rad_dist;

  return total_dist;
}

void RRT::interpolate(const Vertex& from_v, const Vertex& to_v, const double t,
                      std::shared_ptr<Vertex>& v) {
  // this interpolation also needs to be separated into two parts for different
  // state spaces
  float x_new, y_new, theta_new;

  // R^n interpolation
  // we'll leave joint states for now
  x_new =
      from_v.state.x + (to_v.state.x - from_v.state.x) * static_cast<float>(t);
  y_new =
      from_v.state.y + (to_v.state.y - from_v.state.y) * static_cast<float>(t);

  // SO(2) interpolation
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
}

bool RRT::isCollision(const Vertex& from_v, const Vertex& to_v) {}

void RRT::update(const std::vector<geometry_msgs::Point>& footprint) {
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

  if (isCollision(*x_nearest, *x_new)) {
  }

  //   steer(x_rand, x_nearest, x_new);

  if (!collision_checker_->isCollision(static_cast<double>(x_rand->state.x),
                                       static_cast<double>(x_rand->state.y),
                                       static_cast<double>(x_rand->state.theta),
                                       footprint)) {
    vertices_.emplace_back(x_rand);
  }

  if (vertices_.size() > max_vertices_ - 1) {
    std::cout << "Vertices reach maximum limit. Algorithm stopped." << '\n';
    stopped_ = true;
  }
}

const std::vector<std::shared_ptr<Vertex>>* RRT::getVertices() const {
  return &vertices_;
}
}  // namespace psrr_planner