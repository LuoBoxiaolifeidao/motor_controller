#pragma once

#include <vector>
#include <functional>
#include <random>
#include <cmath>
#include <algorithm>

namespace mppi_controller {

struct MPPIConfig {
  int horizon = 15;
  double dt = 0.05;
  int num_samples = 256;
  double lambda = 5.0;
  double sigma_pris = 0.005;
  double sigma_rev = 0.025;
  double goal_weight = 30.0;
  double control_weight = 0.05;
  double terminal_vel_weight = 10.0;
  double collision_penalty = 1000.0;
  int collision_check_stride = 3;
  int arm_count = 0;
  int joints_per_arm = 0;
  double priority_eps = 0.01;
};

class MPPIPlanner {
public:
  MPPIPlanner(MPPIConfig cfg, size_t n_joints,
              std::function<bool(size_t)> is_prismatic)
    : cfg_(cfg), n_joints_(n_joints), H_(cfg.horizon),
      is_prismatic_(std::move(is_prismatic)),
      rng_(std::random_device{}()),
      gauss_(0.0, 1.0) {
    u_warm_.resize(H_ * n_joints_, 0.0);
  }

  std::vector<double> plan_step(
      const std::vector<double>& q_curr,
      const std::vector<double>& q_goal,
      const std::vector<double>& q_min,
      const std::vector<double>& q_max,
      const std::vector<double>& u_min,
      const std::vector<double>& u_max,
      std::function<bool(const std::vector<double>&)> collision_check,
      int& coll_hits_out) {
    size_t N = n_joints_;
    size_t K = static_cast<size_t>(cfg_.num_samples);

    shift_warm(N);

    double total_err = 0.0;
    for (size_t j = 0; j < N; j++) {
      total_err += std::fabs(q_goal[j] - q_curr[j]);
    }
    bool curr_in_collision = collision_check(q_curr);
    bool far_from_target = (total_err > 0.005 * N);
    bool danger_zone = (prev_coll_ratio_ > 0.9);

    if (far_from_target && !danger_zone && !curr_in_collision) {
      for (size_t j = 0; j < N; j++) {
        double err = q_goal[j] - q_curr[j];
        double prop_u = err / (static_cast<double>(H_) * cfg_.dt);
        prop_u = std::clamp(prop_u, u_min[j], u_max[j]);
        for (size_t t = 0; t < H_; t++) {
          u_warm_[t * N + j] = 0.7 * u_warm_[t * N + j] + 0.3 * prop_u;
        }
      }
    }

    if (danger_zone) {
      bool left_moving = false, right_moving = false;
      for (size_t j = 0; j < 8; j++) {
        if (std::fabs(q_goal[j] - q_curr[j]) > 0.01) left_moving = true;
        if (std::fabs(q_goal[8 + j] - q_curr[8 + j]) > 0.01) right_moving = true;
      }
      for (size_t t = 0; t < H_; t++) {
        if (left_moving)
          u_warm_[t * N + 1] = 0.7 * u_warm_[t * N + 1] + 0.3 * u_max[1];
        if (right_moving)
          u_warm_[t * N + 9] = 0.7 * u_warm_[t * N + 9] + 0.3 * u_min[9];
      }
    }

    std::vector<double> weighted_u(N * H_, 0.0);
    double total_weight = 0.0;
    int coll_hits = 0;

    for (size_t k = 0; k < K; k++) {
      std::vector<double> u_noisy = sample_noisy_controls(u_min, u_max);

      bool has_collision = false;
      double cost = rollout(q_curr, q_goal, u_noisy, q_min, q_max, collision_check, has_collision);
      if (has_collision) coll_hits++;
      double w = std::exp(-cost / cfg_.lambda);
      total_weight += w;

      for (size_t i = 0; i < N * H_; i++) {
        weighted_u[i] += w * u_noisy[i];
      }
    }
    coll_hits_out = coll_hits;
    prev_coll_ratio_ = static_cast<double>(coll_hits) / static_cast<double>(K);

    if (total_weight < 1e-12) {
      return std::vector<double>(N, 0.0);
    }

    for (size_t i = 0; i < N * H_; i++) {
      u_warm_[i] = weighted_u[i] / total_weight;
      u_warm_[i] = std::clamp(u_warm_[i], u_min[i % N], u_max[i % N]);
    }

    return std::vector<double>(u_warm_.begin(), u_warm_.begin() + N);
  }

private:
  void shift_warm(size_t N) {
    std::rotate(u_warm_.begin(), u_warm_.begin() + N, u_warm_.end());
    for (size_t j = (H_ - 1) * N; j < H_ * N; j++) {
      u_warm_[j] = u_warm_[j - N];
    }
  }

  std::vector<double> sample_noisy_controls(
      const std::vector<double>& u_min,
      const std::vector<double>& u_max) {
    size_t N = n_joints_;
    std::vector<double> u = u_warm_;
    for (size_t i = 0; i < N * H_; i++) {
      double sigma = is_prismatic_(i % N) ? cfg_.sigma_pris : cfg_.sigma_rev;
      u[i] += sigma * gauss_(rng_);
      u[i] = std::clamp(u[i], u_min[i % N], u_max[i % N]);
    }
    return u;
  }

  double rollout(const std::vector<double>& q_curr,
                 const std::vector<double>& q_goal,
                 const std::vector<double>& u,
                 const std::vector<double>& q_min,
                 const std::vector<double>& q_max,
                 std::function<bool(const std::vector<double>&)>& collision_check,
                 bool& has_collision) {
    double cost = 0.0;
    has_collision = false;
    size_t N = n_joints_;
    std::vector<double> q = q_curr;

    for (size_t t = 0; t < H_; t++) {
      for (size_t j = 0; j < N; j++) {
        q[j] += u[t * N + j] * cfg_.dt;
        q[j] = std::clamp(q[j], q_min[j], q_max[j]);
      }

      if (static_cast<int>(t + 1) % cfg_.collision_check_stride == 0 ||
          t == H_ - 1) {
        if (collision_check(q)) {
          cost += cfg_.collision_penalty;
          has_collision = true;
        }
      }

      double pos_err = 0.0;
      if (cfg_.arm_count > 0) {
        for (int a = 0; a < cfg_.arm_count; a++) {
          double arm_dist = 0.0;
          int base = a * cfg_.joints_per_arm;
          for (int j = 0; j < cfg_.joints_per_arm; j++) {
            double e = q[base + j] - q_goal[base + j];
            arm_dist += std::fabs(e);
          }
          arm_dist = std::max(arm_dist, cfg_.priority_eps);
          double w = 1.0 / arm_dist;
          double total_w = 0.0;
          for (int a2 = 0; a2 < cfg_.arm_count; a2++) {
            double d2 = 0.0;
            int b2 = a2 * cfg_.joints_per_arm;
            for (int j = 0; j < cfg_.joints_per_arm; j++) {
              d2 += std::fabs(q[b2 + j] - q_goal[b2 + j]);
            }
            total_w += 1.0 / std::max(d2, cfg_.priority_eps);
          }
          w = (w / total_w) * cfg_.arm_count;
          for (int j = 0; j < cfg_.joints_per_arm; j++) {
            double e = q[base + j] - q_goal[base + j];
            pos_err += w * e * e;
          }
        }
      } else {
        for (size_t j = 0; j < N; j++) {
          double e = q[j] - q_goal[j];
          pos_err += e * e;
        }
      }
      cost += cfg_.goal_weight * pos_err;

      double ctrl = 0.0;
      for (size_t j = 0; j < N; j++) {
        ctrl += u[t * N + j] * u[t * N + j];
      }
      cost += cfg_.control_weight * ctrl;
    }

    {
      double term_vel = 0.0;
      for (size_t j = 0; j < N; j++) {
        term_vel += u[(H_ - 1) * N + j] * u[(H_ - 1) * N + j];
      }
      cost += cfg_.terminal_vel_weight * term_vel;
    }

    return cost;
  }

  MPPIConfig cfg_;
  size_t n_joints_;
  size_t H_;
  std::function<bool(size_t)> is_prismatic_;
  std::mt19937 rng_;
  std::normal_distribution<double> gauss_;
  std::vector<double> u_warm_;
  double prev_coll_ratio_ = 0.0;
};

}  // namespace mppi_controller
