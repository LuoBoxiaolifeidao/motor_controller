#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>

namespace s_curve_planner {

struct SCurvePoint {
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;
};

struct SCurveParams {
  double v_max = 0.0;
  double a_max = 0.0;
  double j_max = 0.0;
  double v0 = 0.0;
  double v1 = 0.0;
  double a0 = 0.0;
  double a1 = 0.0;
};

class SCurveTrajectory {
public:
  SCurveTrajectory() = default;

  bool plan(double p0, double p1, const SCurveParams &params) {
    p0_ = p0;
    p1_ = p1;
    params_ = params;

    double displacement = p1_ - p0_;
    if (std::fabs(displacement) < 1e-12) {
      durations_ = {0, 0, 0, 0, 0, 0, 0};
      compute_timestamps();
      return true;
    }

    double dir = (displacement > 0.0) ? 1.0 : -1.0;
    L_ = std::fabs(displacement);
    v_max_ = std::fabs(params_.v_max);
    a_max_ = std::fabs(params_.a_max);
    j_max_ = std::fabs(params_.j_max);

    if (j_max_ <= 0.0) {
      durations_ = {0, 0, 0, 0, 0, 0, 0};
      compute_timestamps();
      return false;
    }

    compute_7segment(dir);
    compute_timestamps();
    return true;
  }

  SCurvePoint evaluate(double t) const {
    SCurvePoint pt;
    if (durations_[0] < 0) {
      return pt;
    }

    double ct = clamp_time(t);
    double tj = durations_[0];  // T1 = T3 = T5 = T7 (assuming symmetric)
    double ta = durations_[1];  // T2 = T6 (constant accel/decel)
    double tv = durations_[3];  // T4 (constant velocity)

    // Accumulated phase boundaries
    double t_after[7];
    t_after[0] = tj;
    t_after[1] = t_after[0] + ta;
    t_after[2] = t_after[1] + tj;
    t_after[3] = t_after[2] + tv;
    t_after[4] = t_after[3] + tj;
    t_after[5] = t_after[4] + ta;
    t_after[6] = t_after[5] + tj;

    // State variables at the start of each phase
    // We compute them cumulatively as we locate the phase.

    // Phase 1: j = +j_max
    if (ct <= t_after[0]) {
      double tau = ct;
      pt.jerk = j_max_;
      pt.acceleration = j_max_ * tau;
      pt.velocity = 0.5 * j_max_ * tau * tau;
      pt.position = (1.0 / 6.0) * j_max_ * tau * tau * tau;
      pt = apply_dir(pt);
      return pt;
    }

    // State at end of phase 1
    double v1_end = 0.5 * j_max_ * tj * tj;
    double p1_end = (1.0 / 6.0) * j_max_ * tj * tj * tj;

    // Phase 2: j = 0, a = a_max
    if (ct <= t_after[1]) {
      double tau = ct - t_after[0];
      pt.jerk = 0.0;
      pt.acceleration = a_max_;
      pt.velocity = v1_end + a_max_ * tau;
      pt.position = p1_end + v1_end * tau + 0.5 * a_max_ * tau * tau;
      pt = apply_dir(pt);
      return pt;
    }

    // State at end of phase 2
    double a2 = a_max_;
    double v2 = v1_end + a_max_ * ta;
    double p2 = p1_end + v1_end * ta + 0.5 * a_max_ * ta * ta;

    // Phase 3: j = -j_max
    if (ct <= t_after[2]) {
      double tau = ct - t_after[1];
      pt.jerk = -j_max_;
      pt.acceleration = a2 - j_max_ * tau;
      pt.velocity = v2 + a2 * tau - 0.5 * j_max_ * tau * tau;
      pt.position = p2 + v2 * tau + 0.5 * a2 * tau * tau - (1.0 / 6.0) * j_max_ * tau * tau * tau;
      pt = apply_dir(pt);
      return pt;
    }

    // State at end of phase 3 (a = 0, v = v_max)
    double v3 = v2 + a_max_ * tj - 0.5 * j_max_ * tj * tj;  // = v_max
    double p3 = p2 + v2 * tj + 0.5 * a_max_ * tj * tj - (1.0 / 6.0) * j_max_ * tj * tj * tj;

    // Phase 4: j = 0, a = 0, v = v_max
    if (ct <= t_after[3]) {
      double tau = ct - t_after[2];
      pt.jerk = 0.0;
      pt.acceleration = 0.0;
      pt.velocity = v3;
      pt.position = p3 + v3 * tau;
      pt = apply_dir(pt);
      return pt;
    }

    // State at end of phase 4
    double p4 = p3 + v3 * tv;

    // Phase 5: j = -j_max
    if (ct <= t_after[4]) {
      double tau = ct - t_after[3];
      pt.jerk = -j_max_;
      pt.acceleration = -j_max_ * tau;
      pt.velocity = v3 - 0.5 * j_max_ * tau * tau;
      pt.position = p4 + v3 * tau - (1.0 / 6.0) * j_max_ * tau * tau * tau;
      pt = apply_dir(pt);
      return pt;
    }

    // State at end of phase 5 (a = -a_max)
    double v5 = v3 - 0.5 * j_max_ * tj * tj;
    double p5 = p4 + v3 * tj - (1.0 / 6.0) * j_max_ * tj * tj * tj;

    // Phase 6: j = 0, a = -a_max
    if (ct <= t_after[5]) {
      double tau = ct - t_after[4];
      pt.jerk = 0.0;
      pt.acceleration = -a_max_;
      pt.velocity = v5 - a_max_ * tau;
      pt.position = p5 + v5 * tau - 0.5 * a_max_ * tau * tau;
      pt = apply_dir(pt);
      return pt;
    }

    // State at end of phase 6
    double v6 = v5 - a_max_ * ta;
    double p6 = p5 + v5 * ta - 0.5 * a_max_ * ta * ta;

    // Phase 7: j = +j_max
    {
      double tau = ct - t_after[5];
      pt.jerk = j_max_;
      pt.acceleration = -a_max_ + j_max_ * tau;
      pt.velocity = v6 - a_max_ * tau + 0.5 * j_max_ * tau * tau;
      pt.position = p6 + v6 * tau - 0.5 * a_max_ * tau * tau + (1.0 / 6.0) * j_max_ * tau * tau * tau;
      pt = apply_dir(pt);
      return pt;
    }
  }

  std::vector<SCurvePoint> sample(double dt) const {
    std::vector<SCurvePoint> result;
    if (durations_[0] < 0 || total_time_ <= 0.0) {
      return result;
    }
    int N = static_cast<int>(std::ceil(total_time_ / dt)) + 1;
    result.reserve(N);
    for (int i = 0; i < N; ++i) {
      double t = std::min(i * dt, total_time_);
      result.push_back(evaluate(t));
    }
    return result;
  }

  double total_time() const { return total_time_; }
  const std::vector<double>& durations() const { return durations_; }
  const std::vector<double>& timestamps() const { return timestamps_; }

private:
  double p0_ = 0.0, p1_ = 0.0, L_ = 0.0;
  double v_max_ = 0.0, a_max_ = 0.0, j_max_ = 0.0;
  double total_time_ = 0.0;
  double dir_ = 1.0;
  SCurveParams params_;
  std::vector<double> durations_;
  std::vector<double> timestamps_;

  double clamp_time(double t) const {
    return std::max(0.0, std::min(t, total_time_));
  }

  SCurvePoint apply_dir(SCurvePoint pt) const {
    pt.position = p0_ + dir_ * pt.position;
    pt.velocity *= dir_;
    pt.acceleration *= dir_;
    pt.jerk *= dir_;
    return pt;
  }

  void compute_7segment(double dir) {
    dir_ = dir;
    double tj = 0.0;
    double ta = 0.0;
    double tv = 0.0;

    // v_tri = a_max^2 / j_max : velocity at end of T1+T3 without T2
    double v_tri = a_max_ * a_max_ / j_max_;

    if (v_max_ > v_tri) {
      // Can maintain original a_max; T2/T6 exist
      tj = a_max_ / j_max_;
      // Velocity change in T1+T3 without T2 = v_tri
      // Remaining velocity change to reach v_max done in T2 at constant a_max
      ta = (v_max_ - v_tri) / a_max_;

      // Distance of one accel ramp (T1+T2+T3): v_avg = v_max/2, time = 2*tj + ta
      double t_accel = 2.0 * tj + ta;
      double d_ramp = 0.5 * v_max_ * t_accel;
      double d_min = 2.0 * d_ramp;  // accel + decel

      if (L_ >= d_min) {
        tv = (L_ - d_min) / v_max_;
      } else {
        // Distance too short for v_max, reduce v_peak (no T4)
        // L = v_peak * (tj + v_peak/a_max)
        // v_peak^2 + v_tri*v_peak - a_max*L = 0
        double b = v_tri;
        double c = -a_max_ * L_;
        double v_peak = (-b + std::sqrt(b * b - 4.0 * c)) / 2.0;

        v_max_ = v_peak;
        ta = (v_max_ > v_tri) ? (v_max_ - v_tri) / a_max_ : 0.0;
        tv = 0.0;

        if (v_max_ <= v_tri) {
          // v_peak from T2 equation is invalid for T2-free profile
          // Recompute: L = 2 * v_peak * sqrt(v_peak/j_max)
          // v_peak = (L * sqrt(j_max) / 2)^(2/3)
          v_max_ = std::pow(L_ * std::sqrt(j_max_) / 2.0, 2.0 / 3.0);
          tj = std::sqrt(v_max_ / j_max_);
          a_max_ = j_max_ * tj;
          ta = 0.0;
        }
      }
    } else {
      // v_max <= v_tri: cannot maintain original a_max
      // Scale a_max down so v_tri' = v_max => tj' = sqrt(v_max/j_max)
      tj = std::sqrt(v_max_ / j_max_);
      a_max_ = j_max_ * tj;
      ta = 0.0;

      // Distance of triangular ramps: d_ramp = v_max * tj (v_avg = v_max/2, time = 2*tj)
      double d_ramps = 2.0 * v_max_ * tj;

      if (L_ >= d_ramps) {
        tv = (L_ - d_ramps) / v_max_;
      } else {
        // Distance too short even for triangular v_max, reduce peak
        // L = 2 * v_peak * sqrt(v_peak/j_max) => v_peak = (L*sqrt(j_max)/2)^(2/3)
        v_max_ = std::pow(L_ * std::sqrt(j_max_) / 2.0, 2.0 / 3.0);
        tj = std::sqrt(v_max_ / j_max_);
        a_max_ = j_max_ * tj;
        tv = 0.0;
      }
    }

    durations_ = {tj, ta, tj, tv, tj, ta, tj};
  }

  void compute_timestamps() {
    timestamps_.resize(8);
    timestamps_[0] = 0.0;
    for (int i = 0; i < 7; ++i) {
      timestamps_[i + 1] = timestamps_[i] + durations_[i];
    }
    total_time_ = timestamps_[7];
  }
};

}  // namespace s_curve_planner
