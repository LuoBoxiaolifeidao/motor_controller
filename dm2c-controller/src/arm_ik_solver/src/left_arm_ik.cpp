#include "arm_ik_solver/left_arm_ik.hpp"

#include <cmath>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace left_arm_ik {

using KDL::Vector;
using KDL::Rotation;
using KDL::Frame;

static constexpr double PI = 3.141592653589793;
static constexpr double HALF_PI = 1.5707963267948966;

static double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

// === URDF 20260623 左臂参数 ===
static const Frame T_O0(Rotation::Identity(), Vector(-0.1275, -0.1065, 0.34054));
static const Frame T_O1(Rotation::Identity(), Vector(0.0, 0.119, 0.0575));
static const Frame T_O2A(Rotation::RPY(-HALF_PI, 0, 0), Vector(0.0002, 0.254, 0.0328));
static const Frame T_O2B(Rotation::Identity(), Vector(0.0, 0.0, 0.015));
static const Frame T_O2C(Rotation::Identity(), Vector(0.0, 0.0, 0.011));
static const Frame T_O3(Rotation::RPY(-HALF_PI, 0, 0), Vector(-0.0002, 0.0368, 0.057));
static const Frame T_O4(Rotation::RPY(HALF_PI, 0, 0), Vector(0.0, 0.027, 0.0565));
static const Frame T_O5(Rotation::RPY(-HALF_PI, 0, 0), Vector(0.0, 0.042, 0.027));

static Frame joint_motion(int idx, double qi) {
  // idx: 0=post, 1=shoulder, 2/3/4=armA/B/C, 5=wrist-p, 6=wrist-y, 7=wrist-r
  if (idx == 0 || idx == 2 || idx == 3 || idx == 4) return Frame(Vector(0, 0, qi));
  return Frame(Rotation::RotZ(qi));
}

Frame fk(const std::array<double, 8>& q) {
  Frame T = Frame::Identity();
  T = T * T_O0 * joint_motion(0, q[0]);
  T = T * T_O1 * joint_motion(1, q[1]);
  T = T * T_O2A * joint_motion(2, q[2]);
  T = T * T_O2B * joint_motion(3, q[3]);
  T = T * T_O2C * joint_motion(4, q[4]);
  T = T * T_O3 * joint_motion(5, q[5]);
  T = T * T_O4 * joint_motion(6, q[6]);
  T = T * T_O5 * joint_motion(7, q[7]);
  return T;
}

Frame fk_arm(const std::array<double, 8>& q) {
  Frame T = Frame::Identity();
  T = T * T_O0 * joint_motion(0, q[0]);
  T = T * T_O1 * joint_motion(1, q[1]);
  T = T * T_O2A * joint_motion(2, q[2]);
  T = T * T_O2B * joint_motion(3, q[3]);
  T = T * T_O2C * joint_motion(4, q[4]);
  T = T * T_O3;
  return T;
}

Frame fk_wrist(const std::array<double, 3>& qw) {
  Frame T = Frame::Identity();
  T = T * joint_motion(5, qw[0]);
  T = T * T_O4 * joint_motion(6, qw[1]);
  T = T * T_O5 * joint_motion(7, qw[2]);
  return T;
}

static Frame wrist_base_from_target(const Vector& P_tcp, const Rotation& R_tcp,
                                     double q3, double q4, double q5) {
  Frame T_tcp(R_tcp, P_tcp);
  Frame M3 = joint_motion(5, q3);
  Frame M4 = joint_motion(6, q4);
  Frame M5 = joint_motion(7, q5);
  Frame T_inv = M5.Inverse() * T_O5.Inverse() * M4.Inverse() * T_O4.Inverse() * M3.Inverse();
  return T_tcp * T_inv;
}

// R_wrist = [[c3c4c5-s3s5, -c3c4s5-s3c5, -c3s4],
//            [s3c4c5+c3s5, -s3c4s5+c3c5, -s3s4],
//            [s4c5,        -s4s5,         c4]]
static std::array<double, 3> solve_wrist_from_R(const Rotation& R) {
  double c4 = clamp(R(2, 2), -1.0, 1.0);
  double q4 = std::acos(c4);
  double s4 = std::sin(q4);

  double q3, q5;
  if (std::abs(s4) > 1e-8) {
    q3 = std::atan2(-R(1, 2), -R(0, 2));
    q5 = std::atan2(-R(2, 1), R(2, 0));
  } else {
    q3 = 0.0;
    q5 = (c4 > 0) ? std::atan2(-R(1, 0), -R(0, 0))
                   : std::atan2(R(1, 0), R(0, 0));
  }
  return {clamp(q3, JOINT_LOWER[5], JOINT_UPPER[5]),
          clamp(q4, JOINT_LOWER[6], JOINT_UPPER[6]),
          clamp(q5, JOINT_LOWER[7], JOINT_UPPER[7])};
}

// R_arm_wp = [[c1, s1, 0], [s1, -c1, 0], [0, 0, -1]]
static Rotation arm_wp_rotation(double t1) {
  double c = std::cos(t1), s = std::sin(t1);
  return Rotation(c, s, 0, s, -c, 0, 0, 0, -1);
}

// P_wp_x = -0.1275 - (d_total + 0.337)*s1
// P_wp_y =  0.0125 + (d_total + 0.337)*c1
// P_wp_z =  0.39404 + d0
static void solve_arm_from_wp(const Vector& P_wp, double& d0, double& t1, double& d_total) {
  d0 = clamp(P_wp.z() - 0.39404, JOINT_LOWER[0], JOINT_UPPER[0]);
  double A = P_wp.x() + 0.1275;
  double B = P_wp.y() - 0.0125;
  double L = std::sqrt(A * A + B * B);
  d_total = clamp(L - 0.337, JOINT_LOWER[2], JOINT_UPPER[2] + JOINT_UPPER[3] + JOINT_UPPER[4]);
  t1 = clamp(std::atan2(-A, B), JOINT_LOWER[1], JOINT_UPPER[1]);
}

static void split_arm(double d_total, double& dA, double& dB, double& dC) {
  double maxA = JOINT_UPPER[2], maxB = JOINT_UPPER[3], maxC = JOINT_UPPER[4];
  dA = clamp(d_total, JOINT_LOWER[2], maxA);
  d_total -= dA;
  dB = clamp(d_total, JOINT_LOWER[3], maxB);
  d_total -= dB;
  dC = clamp(d_total, JOINT_LOWER[4], maxC);
}

IKResult solve_ik(const Vector& target_pos, const Rotation& target_rot,
                  const std::array<double, 8>& q_init) {
  IKResult result;
  double q3 = q_init[5], q4 = q_init[6], q5 = q_init[7];

  for (int iter = 0; iter < 10; ++iter) {
    Frame T_wp = wrist_base_from_target(target_pos, target_rot, q3, q4, q5);
    double d0, t1, d_total;
    solve_arm_from_wp(T_wp.p, d0, t1, d_total);

    Rotation R_arm = arm_wp_rotation(t1);
    Rotation R_wrist_target = R_arm.Inverse() * target_rot;
    auto wq = solve_wrist_from_R(R_wrist_target);
    q3 = wq[0]; q4 = wq[1]; q5 = wq[2];

    if (iter >= 1) {
      double dA, dB, dC; split_arm(d_total, dA, dB, dC);
      std::array<double, 8> q = {d0, t1, dA, dB, dC, q3, q4, q5};
      Frame T_fk = fk(q);
      double pe = (target_pos - T_fk.p).Norm();
      Rotation re = target_rot * T_fk.M.Inverse();
      double trace = re(0,0) + re(1,1) + re(2,2);
      double re_ang = std::acos(clamp((trace - 1.0) / 2.0, -1.0, 1.0));
      if (pe < 0.01 && re_ang < 0.05) {
        result.q8 = q; result.pos_err = pe; result.rot_err = re_ang;
        result.success = true; return result;
      }
    }
  }

  Frame T_wp = wrist_base_from_target(target_pos, target_rot, q3, q4, q5);
  double d0, t1, d_total;
  solve_arm_from_wp(T_wp.p, d0, t1, d_total);
  double dA, dB, dC; split_arm(d_total, dA, dB, dC);
  std::array<double, 8> q = {d0, t1, dA, dB, dC, q3, q4, q5};
  Frame T_fk = fk(q);
  result.pos_err = (target_pos - T_fk.p).Norm();
  Rotation re = target_rot * T_fk.M.Inverse();
  double trace = re(0,0) + re(1,1) + re(2,2);
  result.rot_err = std::acos(clamp((trace - 1.0) / 2.0, -1.0, 1.0));
  result.q8 = q;
  result.success = (result.pos_err < 0.01 && result.rot_err < 0.05);
  return result;
}

bool load_chain_from_urdf(const std::string& urdf_xml, KDL::Chain& chain) {
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) return false;
  return tree.getChain("base_link", "l_wrist_r_Link", chain);
}

}  // namespace left_arm_ik
