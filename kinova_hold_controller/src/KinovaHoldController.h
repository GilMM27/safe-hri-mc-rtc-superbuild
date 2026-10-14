#pragma once

#include <mc_control/mc_controller.h>
#include <mc_mujoco/SmplhDistanceSnapshot.h>

#include <string>

#include "api.h"

struct KinovaHoldController_DLLAPI KinovaHoldController : public mc_control::MCController
{
  KinovaHoldController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

private:
  const mc_mujoco::SmplhDistanceSnapshot * distanceSnapshot() const;
  double regionDistance(const std::string & region) const;
  std::string regionStatus(const std::string & region) const;
  Eigen::Vector3d closestPoint(const std::string & region, bool robotPoint) const;
};
