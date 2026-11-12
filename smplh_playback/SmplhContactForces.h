#pragma once
#include <array>

namespace smplh_contact
{
inline constexpr const char * datastoreKey = "smplh_contact_forces";
inline constexpr std::array<const char *, 14> regions = {
    "torso", "head", "left_upper_arm", "left_forearm", "left_hand",
    "right_upper_arm", "right_forearm", "right_hand", "left_thigh",
    "left_lower_leg", "left_foot", "right_thigh", "right_lower_leg", "right_foot"};
struct Snapshot
{
  bool valid = false;
  double time = 0;
  std::array<double, 14> positiveNormalSumN{};
  std::array<double, 14> peakPositiveNormalSumN{};
  bool externalTorqueFeedback = false;
  bool highPassCompensation = false;
  double externalJointTorqueNormNm = 0;
};
}
