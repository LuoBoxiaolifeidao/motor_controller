#include "SCurvPlan.h"
#include <algorithm>
#include <vector>

void SCurvePlanner::setParams(int idx, const Params& p) {
    params_[idx] = p;
}

bool SCurvePlanner::plan(const std::array<double, kAxes>& cur,
                          const std::array<double, kAxes>& tgt,
                          const std::array<double, kAxes>& cur_vel)
{
    std::vector<double> ft_cur{cur[0], cur[1], cur[2]};
    std::vector<double> ft_tgt{tgt[0], tgt[1], tgt[2]};
    std::vector<double> ft_vel{cur_vel[0], cur_vel[1], cur_vel[2]};
    std::vector<s_curve_planner::SCurveParams> ft_params(3);
    for (int i = 0; i < 3; ++i) {
        ft_params[i].v_max = std::abs(params_[i].v_max);
        ft_params[i].a_max = params_[i].a_max;
        ft_params[i].j_max = params_[i].j_max;
        ft_params[i].v0 = std::abs(ft_vel[i]);
        ft_params[i].v1 = params_[i].v_min;
        ft_params[i].a0 = 0;
        ft_params[i].a1 = 0;
    }
    ft_sync_.plan(ft_cur, ft_tgt, ft_params);

    max_time_ = ft_sync_.sync_time();
    return true;
}

void SCurvePlanner::evaluate(int idx, double t, double& pos, double& vel, double& acc) const
{
    auto pts = ft_sync_.evaluate(std::min(t, ft_sync_.sync_time()));
    auto& pt = pts[idx];
    pos = pt.position;
    vel = std::abs(pt.velocity);
    acc = std::abs(pt.acceleration);
}

double SCurvePlanner::axisTime(int idx) const {
    return ft_sync_.sync_time();
}
