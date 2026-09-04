#pragma once

#include <array>
#include "s_curve_trajectory.hpp"
#include "synced_trajectory.hpp"

class SCurvePlanner {
public:
    struct Params {
        double v_max = 30;
        double v_min = 1;
        double a_max = 100;
        double j_max = 500;
    };

    static constexpr int kAxes = 3;

    void setParams(int idx, const Params& p);
    bool plan(const std::array<double, kAxes>& cur,
              const std::array<double, kAxes>& tgt,
              const std::array<double, kAxes>& cur_vel);

    void evaluate(int idx, double t, double& pos, double& vel, double& acc) const;

    double axisTime(int idx) const;
    double totalTime() const { return max_time_; }

private:
    s_curve_planner::SyncedTrajectory ft_sync_;
    Params params_[kAxes];
    double max_time_ = 0;
};
