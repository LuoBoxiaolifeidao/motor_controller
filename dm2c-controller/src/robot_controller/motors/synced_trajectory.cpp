#include "synced_trajectory.hpp"
#include <algorithm>
#include <cmath>

namespace s_curve_planner {

bool SyncedTrajectory::plan(const std::vector<double>& p0,
                            const std::vector<double>& p1,
                            const std::vector<SCurveParams>& params) {
  size_t N = p0.size();
  if (p1.size() != N || params.size() != N) return false;

  trajs_.resize(N);
  sync_time_ = 0.0;

  for (size_t i = 0; i < N; i++) {
    if (!trajs_[i].plan(p0[i], p1[i], params[i]))
      return false;
    double T = trajs_[i].total_time();
    if (T > sync_time_) sync_time_ = T;
  }

  if (sync_time_ <= 0.0) {
    sync_time_ = 0.0;
    return true;
  }
  return true;
}

std::vector<SCurvePoint> SyncedTrajectory::evaluate(double t) const {
  size_t N = trajs_.size();
  std::vector<SCurvePoint> result(N);
  double ct = std::max(0.0, std::min(t, sync_time_));

  for (size_t i = 0; i < N; i++) {
    double Ti = trajs_[i].total_time();
    if (sync_time_ <= 0.0 || Ti <= 0.0) {
      result[i] = trajs_[i].evaluate(Ti);
    } else {
      double scale = Ti / sync_time_;
      result[i] = trajs_[i].evaluate(ct * scale);
      result[i].velocity *= scale;
      result[i].acceleration *= scale * scale;
    }
  }
  return result;
}

std::vector<double> SyncedTrajectory::positions(double t) const {
  auto pts = evaluate(t);
  std::vector<double> pos(pts.size());
  for (size_t i = 0; i < pts.size(); i++) pos[i] = pts[i].position;
  return pos;
}

std::vector<std::vector<SCurvePoint>> SyncedTrajectory::sample(double dt) const {
  if (sync_time_ <= 0.0) {
    auto pts = evaluate(0.0);
    return {pts};
  }
  int N = static_cast<int>(std::ceil(sync_time_ / dt)) + 1;
  std::vector<std::vector<SCurvePoint>> result(N);
  for (int i = 0; i < N; i++) {
    result[i] = evaluate(std::min(i * dt, sync_time_));
  }
  return result;
}

}  // namespace s_curve_planner
