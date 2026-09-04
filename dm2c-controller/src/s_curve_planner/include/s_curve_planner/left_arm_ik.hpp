#pragma once

#include <array>
#include <string>
#include <vector>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>

namespace left_arm_ik {

constexpr std::array<const char*, 8> JOINT_NAMES = {
  "l_post_Joint", "l_shoulder_Joint",
  "l_arm_JointA", "l_arm_JointB", "l_arm_JointC",
  "l_wrist_p_Joint", "l_wrist_y_Joint", "l_wrist_r_Joint"
};

constexpr std::array<double, 8> JOINT_LOWER = {0.0, 0.0, 0.0, 0.0, 0.0, -3.14, -1.57, -3.14};
constexpr std::array<double, 8> JOINT_UPPER = {0.95, 1.57, 0.12, 0.135, 0.15, 3.14, 1.57, 3.14};

struct IKResult {
  std::array<double, 8> q8;
  double pos_err = 0.0;
  double rot_err = 0.0;
  bool success = false;
};

IKResult solve_ik(const KDL::Vector& target_pos, const KDL::Rotation& target_rot,
                  const std::array<double, 8>& q_init = {0.5, 0.5, 0.04, 0.04, 0.04, 0.0, 0.0, 0.0});

KDL::Frame fk(const std::array<double, 8>& q);
KDL::Frame fk_arm(const std::array<double, 8>& q);
KDL::Frame fk_wrist(const std::array<double, 3>& qw);

bool load_chain_from_urdf(const std::string& urdf_param, KDL::Chain& chain);

}  // namespace left_arm_ik
