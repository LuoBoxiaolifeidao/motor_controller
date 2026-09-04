#pragma once

#include <vector>
#include "s_curve_trajectory.hpp"

namespace s_curve_planner {

class SyncedTrajectory {
public:
  SyncedTrajectory() = default;

  bool plan(const std::vector<double>& p0,
            const std::vector<double>& p1,
            const std::vector<SCurveParams>& params);

  std::vector<SCurvePoint> evaluate(double t) const;
  std::vector<double> positions(double t) const;
  std::vector<std::vector<SCurvePoint>> sample(double dt) const;

  double sync_time() const { return sync_time_; }
  size_t num_joints() const { return trajs_.size(); }
  const SCurveTrajectory& joint_trajectory(size_t i) const { return trajs_[i]; }

private:
  std::vector<SCurveTrajectory> trajs_;
  double sync_time_ = 0.0;
};

}  // namespace s_curve_planner
