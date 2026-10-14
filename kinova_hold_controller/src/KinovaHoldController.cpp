#include "KinovaHoldController.h"

#include <mc_rtc/gui/Label.h>

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <vector>

namespace
{
const std::array<const char *, 14> regions = {{"head", "torso", "left_upper_arm", "left_forearm", "left_hand",
                                               "right_upper_arm", "right_forearm", "right_hand", "left_thigh",
                                               "left_lower_leg", "left_foot", "right_thigh", "right_lower_leg",
                                               "right_foot"}};
}

KinovaHoldController::KinovaHoldController(mc_rbdyn::RobotModulePtr rm,
                                           double dt,
                                           const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({});

  postureTask->target({{"joint_1", {0.0}},
                       {"joint_2", {0.2618}},
                       {"joint_3", {3.14}},
                       {"joint_4", {-2.269}},
                       {"joint_5", {0.0}},
                       {"joint_6", {0.959878729}},
                       {"joint_7", {1.57}}});
  postureTask->stiffness(1.0);

  logger().addLogEntry("SMPLH_distance_approximate", [this]() {
    const auto * snapshot = distanceSnapshot();
    return snapshot && snapshot->approximate;
  });
  for(const auto * region : regions)
  {
    const std::string name(region);
    gui()->addElement({"SMPL-H distances"}, mc_rtc::gui::Label(name, [this, name]() { return regionStatus(name); }));
    logger().addLogEntry("SMPLH_distance_" + name, [this, name]() { return regionDistance(name); });
  }

  mc_rtc::log::success("KinovaHoldController initialized");
}

const mc_mujoco::SmplhDistanceSnapshot * KinovaHoldController::distanceSnapshot() const
{
  static const std::string key = "SMPLH::DistanceSnapshot";
  if(!datastore().has(key)) return nullptr;
  const auto & snapshot = datastore().get<mc_mujoco::SmplhDistanceSnapshot>(key);
  return snapshot.ready && snapshot.error.empty() ? &snapshot : nullptr;
}

double KinovaHoldController::regionDistance(const std::string & region) const
{
  const auto * snapshot = distanceSnapshot();
  if(!snapshot) return std::numeric_limits<double>::quiet_NaN();
  for(const auto & value : snapshot->regions)
  {
    if(value.region == region) return value.distance;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

std::string KinovaHoldController::regionStatus(const std::string & region) const
{
  const auto * snapshot = distanceSnapshot();
  if(!snapshot) return "Waiting for distance sample";
  for(const auto & value : snapshot->regions)
  {
    if(value.region != region) continue;
    std::ostringstream status;
    status.precision(4);
    status << value.distance << " m";
    if(snapshot->approximate) status << " (approximate)";
    status << "  |  " << value.robot_geometry;
    if(value.intersecting) status << (snapshot->approximate ? "  |  hull overlap" : "  |  intersection");
    return status.str();
  }
  return "Region not present in cache";
}

CONTROLLER_CONSTRUCTOR("KinovaHoldController", KinovaHoldController)
