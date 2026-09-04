#include "s_curve_planner/s_curve_trajectory.hpp"
#include "s_curve_planner/synced_trajectory.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace s_curve_planner;

void print_plan(const SCurveTrajectory &traj, double dt, double p0, double p1) {
  const auto &d = traj.durations();
  const auto &ts = traj.timestamps();

  std::cout << "\n========================================" << std::endl;
  std::cout << "7-Segment S-Curve Trajectory Plan" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Start: " << p0 << "  End: " << p1
            << "  Displacement: " << (p1 - p0) << std::endl;
  std::cout << "Total time: " << traj.total_time() << " s" << std::endl;
  std::cout << std::endl;
  std::cout << "Phase durations:" << std::endl;
  const char *names[] = {"T1(j+)", "T2(a+)", "T3(j-)", "T4(vc)",
                         "T5(j-)", "T6(a-)", "T7(j+)"};
  for (int i = 0; i < 7; ++i) {
    std::cout << "  " << names[i] << ": " << d[i] << " s"
              << "  [" << ts[i] << " -> " << ts[i + 1] << "]" << std::endl;
  }

  std::cout << std::endl;
  std::cout << std::setw(8) << "t(s)"
            << std::setw(14) << "pos"
            << std::setw(14) << "vel"
            << std::setw(14) << "acc"
            << std::setw(14) << "jerk" << std::endl;
  std::cout << std::string(64, '-') << std::endl;

  auto pts = traj.sample(dt);
  int phase_mark = 0;
  for (size_t i = 0; i < pts.size(); ++i) {
    double t = i * dt;
    if (phase_mark < 7 && t >= ts[phase_mark + 1] - 0.5 * dt) {
      std::cout << std::string(64, '-') << "  phase " << (phase_mark + 1) << std::endl;
      phase_mark++;
    }
    std::cout << std::fixed << std::setprecision(3)
              << std::setw(8) << t
              << std::setw(14) << std::setprecision(6) << pts[i].position
              << std::setw(14) << pts[i].velocity
              << std::setw(14) << pts[i].acceleration
              << std::setw(14) << pts[i].jerk << std::endl;
  }
}

int main() {
  SCurveParams params;
  params.v_max = 0.5;
  params.a_max = 0.3;
  params.j_max = 0.2;

  std::cout << "\n========== Case 1: Full 7-segment ==========" << std::endl;
  {
    SCurveTrajectory traj;
    if (traj.plan(0.0, 10.0, params)) {
      print_plan(traj, 0.5, 0.0, 10.0);
    } else {
      std::cout << "Plan failed!" << std::endl;
    }
  }

  std::cout << "\n\n========== Case 2: Short distance (no constant velocity) ==========" << std::endl;
  {
    SCurveTrajectory traj;
    if (traj.plan(0.0, 2.0, params)) {
      print_plan(traj, 0.2, 0.0, 2.0);
    } else {
      std::cout << "Plan failed!" << std::endl;
    }
  }

  std::cout << "\n\n========== Case 3: Very short distance (no constant accel) ==========" << std::endl;
  {
    SCurveTrajectory traj;
    if (traj.plan(0.0, 0.5, params)) {
      print_plan(traj, 0.1, 0.0, 0.5);
    } else {
      std::cout << "Plan failed!" << std::endl;
    }
  }

  std::cout << "\n\n========== Case 4: Negative displacement ==========" << std::endl;
  {
    SCurveTrajectory traj;
    if (traj.plan(5.0, -3.0, params)) {
      print_plan(traj, 0.5, 5.0, -3.0);
    } else {
      std::cout << "Plan failed!" << std::endl;
    }
  }

  std::cout << "\n\n========== Case 5: Synced multi-joint (prismatic + revolute) ==========" << std::endl;
  {
    const int NJ = 16;
    std::vector<double> p0(NJ, 0.0), p1(NJ, 0.0);
    // Simulate: different joints have different displacements + limits
    // Prismatic: larger displacement, lower speed
    // Revolute: smaller displacement, higher speed
    p1 = {0.3, 1.2, 0.08, 0.10, 0.12, -2.0, 1.0, -2.5,
          0.3, -1.0, 0.08, 0.10, 0.12, 2.0, -1.0, 2.5};

    std::vector<SCurveParams> all_params(NJ);
    for (int i = 0; i < NJ; i++) {
      bool is_pris = (i % 8 == 0 || i % 8 == 2 || i % 8 == 3 || i % 8 == 4);
      all_params[i].v_max = is_pris ? 0.12 : 1.0;
      all_params[i].a_max = is_pris ? 1.5  : 3.0;
      all_params[i].j_max = is_pris ? 15.0 : 25.0;
    }

    SyncedTrajectory sync;
    if (!sync.plan(p0, p1, all_params)) {
      std::cout << "Synced plan failed!" << std::endl;
      return 1;
    }

    std::cout << "Sync time: " << sync.sync_time() << " s" << std::endl;
    std::cout << "Individual joint times:" << std::endl;
    for (size_t i = 0; i < sync.num_joints(); i++) {
      double Ti = sync.joint_trajectory(i).total_time();
      std::cout << "  Joint " << i << " ("
                << ((i%8==0||i%8==2||i%8==3||i%8==4)?"pris":"rev ")
                << "): T=" << Ti << " s  scale=" << Ti/sync.sync_time()
                << "  disp=" << (p1[i]-p0[i]) << std::endl;
    }

    auto traj = sync.sample(0.1);
    std::cout << "\nSynced trajectory (" << traj.size() << " steps):" << std::endl;
    std::cout << std::setw(8) << "t(s)";
    for (int j = 0; j < 8; j++)
      std::cout << std::setw(12) << ("J" + std::to_string(j));
    std::cout << std::endl << std::string(8+8*12, '-') << std::endl;

    int step = 0;
    for (auto& row : traj) {
      std::cout << std::fixed << std::setprecision(2)
                << std::setw(8) << (step * 0.1);
      for (int j = 0; j < 8; j++)
        std::cout << std::setw(12) << std::setprecision(4) << row[j].position;
      std::cout << std::endl;
      step++;
    }
  }

  return 0;
}
