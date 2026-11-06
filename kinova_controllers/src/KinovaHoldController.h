#pragma once

#include <mc_control/mc_controller.h>
#include <human_robot_distance_api/DistanceSnapshot.h>

#include <string>

#include "api.h"

struct KinovaHoldController_DLLAPI KinovaHoldController : public mc_control::MCController
{
  KinovaHoldController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

private:
  const human_robot_distance::Snapshot * distanceSnapshot() const;
  double regionDistance(const std::string & region) const;
  std::string regionStatus(const std::string & region) const;
};
