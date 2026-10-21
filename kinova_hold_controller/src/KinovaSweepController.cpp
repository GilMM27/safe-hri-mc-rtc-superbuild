#include "KinovaSweepController.h"
#include "SSMPFLMath.h"

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/plot.h>
#include <mc_solver/TasksQPSolver.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyConfig.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <chrono>

namespace
{
const std::array<const char *, 14> regions = {{"head", "torso", "left_upper_arm", "left_forearm", "left_hand",
                                               "right_upper_arm", "right_forearm", "right_hand", "left_thigh",
                                               "left_lower_leg", "left_foot", "right_thigh", "right_lower_leg",
                                               "right_foot"}};
const std::array<const char *, 7> links = {{"joint_1", "joint_2", "joint_3", "joint_4",
                                            "joint_5", "joint_6", "joint_7"}};

std::string pairLabel(size_t index)
{
  return std::string(regions[index / links.size()]) + "/" + links[index % links.size()];
}

double smoothStep(double u)
{
  u = std::max(0.0, std::min(1.0, u));
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}
} // namespace

KinovaSweepController::KinovaSweepController(mc_rbdyn::RobotModulePtr rm,
                                             double dt,
                                             const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addTask(postureTask);
  solver().setContacts({});
  jointIndex_ = robot().jointIndexByName("joint_1");
  if(config.has("Sweep"))
  {
    const auto sweep = config("Sweep");
    sweep("AmplitudeRad", amplitude_);
    sweep("SegmentSeconds", segmentSeconds_);
    sweep("DwellSeconds", dwellSeconds_);
    sweep("MaxJointSpeedRadS", maxJointSpeed_);
    sweep("PostureStiffness", postureStiffness_);
    sweep("AutoStart", autoStart_);
  }
  if(!std::isfinite(amplitude_) || !std::isfinite(segmentSeconds_) || !std::isfinite(dwellSeconds_)
     || !std::isfinite(maxJointSpeed_) || !std::isfinite(postureStiffness_) || amplitude_ == 0.0
     || segmentSeconds_ <= 0.0 || dwellSeconds_ < 0.0 || maxJointSpeed_ <= 0.0 || postureStiffness_ <= 0.0
     || amplitude_ <= robot().ql().at(jointIndex_).at(0)
     || amplitude_ >= robot().qu().at(jointIndex_).at(0)
     || 0.0 <= robot().ql().at(jointIndex_).at(0))
  {
    mc_rtc::log::error_and_throw("KinovaSweepController: invalid Sweep configuration or joint_1 limits");
  }
  maxJointSpeed_ = std::min(maxJointSpeed_, robot().vu().at(jointIndex_).at(0));
  if(maxJointSpeed_ <= 0.0)
  {
    mc_rtc::log::error_and_throw("KinovaSweepController: joint_1 has no positive velocity limit");
  }
  postureTask->stiffness(postureStiffness_);

  pairLogs_.resize(regions.size());
  linkPairLogs_.resize(regions.size() * links.size());
  if(config.has("SSMPFL"))
  {
    const auto safety = config("SSMPFL");
    safety("Enabled", safetyEnabled_);
    if(safetyEnabled_)
    {
      safety("HoldOnConstraintUnavailable", holdOnConstraintUnavailable_);
      safety("ConstrainWhileHolding", constrainWhileHolding_);
      if(solver().backend() != mc_solver::QPSolver::Backend::Tasks)
        mc_rtc::log::error_and_throw("KinovaSweepController SSM+PFL requires the Tasks QP backend");
      std::string mode = "frozen";
      safety("HumanMode", mode);
      if(mode != "frozen" && mode != "moving")
        mc_rtc::log::error_and_throw("SSMPFL.HumanMode must be frozen or moving");
      movingHuman_ = mode == "moving";
      safety("StoppingTimeSeconds", stopTime_);
      safety("MaxSampleAgeSeconds", maxSampleAge_);
      safety("MaxModelWitnessErrorM", maxModelWitnessError_);
      safety("MaxActualResidualMps", maxActualResidual_);
      safety("MinDirectionalInverseMass", minMobility_);
      safety("MinMassEigenvalue", minMassEigenvalue_);
      if(!std::isfinite(stopTime_) || stopTime_ <= 0
         || !std::isfinite(maxSampleAge_) || maxSampleAge_ <= 0 || !std::isfinite(minMobility_)
         || minMobility_ < 0 || !std::isfinite(minMassEigenvalue_) || minMassEigenvalue_ <= 0
         || !std::isfinite(maxModelWitnessError_) || maxModelWitnessError_ <= 0
         || !std::isfinite(maxActualResidual_) || maxActualResidual_ < 0
         || !safety.has("Regions"))
        mc_rtc::log::error_and_throw("Invalid SSMPFL timing, tolerance, or missing Regions");
      const auto regionConfig = safety("Regions");
      for(const auto * region : regions)
      {
        if(!regionConfig.has(region)) mc_rtc::log::error_and_throw("Missing SSMPFL region {}", region);
        const auto entry = regionConfig(region);
        SafetyRegion params;
        if(!entry.has("ForceMaxN") || !entry.has("StiffnessNPerM"))
          mc_rtc::log::error_and_throw("SSMPFL region {} needs ForceMaxN and StiffnessNPerM", region);
        entry("ForceMaxN", params.force);
        entry("StiffnessNPerM", params.stiffness);
        entry("DistanceMarginM", params.margin);
        if(!std::isfinite(params.force) || params.force < 0 || !std::isfinite(params.stiffness)
           || params.stiffness <= 0 || !std::isfinite(params.margin) || params.margin < 0)
          mc_rtc::log::error_and_throw("Invalid SSMPFL parameters for {}", region);
        safetyRegions_[region] = params;
      }
      safetyConstraint_.reset(new SSMPFLConstraint(0, robot().mb().nrDof(), static_cast<int>(linkPairLogs_.size())));
      inertia_.reset(new rbd::ForwardDynamics(robot().mb()));
      mc_solver::TasksQPSolver::from_solver(solver()).addConstraint(safetyConstraint_.get());
    }
  }

  gui()->addElement({"Kinova sweep"}, mc_rtc::gui::Button("Start", [this]() { startRequested_ = true; }),
                    mc_rtc::gui::Button("Stop", [this]() { stopRequested_ = true; }));
  std::vector<std::string> plotPairs;
  plotPairs.reserve(linkPairLogs_.size());
  for(size_t i = 0; i < linkPairLogs_.size(); ++i) plotPairs.push_back(pairLabel(i));
  gui()->addElement({"SSM+PFL"},
                    mc_rtc::gui::ComboInput("Plot region/link", plotPairs,
                      [this]() { return pairLabel(plotPairIndex_); },
                      [this](const std::string & label) {
                        for(size_t i = 0; i < linkPairLogs_.size(); ++i)
                        {
                          if(label != pairLabel(i)) continue;
                          if(i == plotPairIndex_) return;
                          plotPairIndex_ = i;
                          speedPlot_ = SpeedPlotSample{};
                          gui()->removePlot("SSM+PFL robot approach speed (m/s)");
                          addSpeedPlot();
                          return;
                        }
                      }),
                    mc_rtc::gui::Label("Plot sample age", [this]() {
                      if(speedPlot_.sampleTime < 0) return std::string("no valid sample for ") + pairLabel(plotPairIndex_);
                      const double age = safetyTime_ - speedPlot_.sampleTime;
                      const std::string qpAge = speedPlot_.qpTime < 0 ? "unavailable"
                          : std::to_string(safetyTime_ - speedPlot_.qpTime) + " s old";
                      return pairLabel(plotPairIndex_) + ": sample " + std::to_string(age)
                          + " s old; QP next " + qpAge;
                    }),
                    mc_rtc::gui::Label("Mode", [this]() {
                      return !safetyEnabled_ ? std::string("disabled") : movingHuman_ ? std::string("moving human")
                                                                                : std::string("frozen human");
                    }),
                    mc_rtc::gui::Label("State", [this]() {
                      if(!solverSuccess_) return std::string("QP failed");
                      if(monitorOnly_) return std::string("monitoring while waiting for Start");
                      if(safetyFallback_)
                        return std::string(holdOnConstraintUnavailable_ ? "Stop requested: "
                                                                         : "Constraint unavailable: ") + fallbackReason_;
                      return std::string("QP running");
                    }),
                    mc_rtc::gui::Label("Limiting region", [this]() {
                      return limitingPair_ < 0 ? std::string("none") : pairLabel(limitingPair_);
                    }),
                    mc_rtc::gui::Label("At QP speed bound", [this]() { return bindingRegions(); }),
                    mc_rtc::gui::Label("Uncontrollable regions", [this]() { return uncontrollableRegions(); }),
                    mc_rtc::gui::Label("Limiting speed (m/s)", [this]() {
                      if(!solverSuccess_ || limitingPair_ < 0 || !linkPairLogs_[limitingPair_].active)
                        return std::string("unavailable");
                      const auto & pair = linkPairLogs_[limitingPair_];
                      return std::string("robot current ") + std::to_string(pair.robotSpeed)
                          + ", QP next " + std::to_string(pair.combinedLimit + pair.predictedResidual + pair.humanSpeed)
                          + ", sampled MuJoCo (delayed) "
                          + std::to_string(pair.combinedLimit + pair.actualResidual + pair.humanSpeed)
                          + ", robot limit " + std::to_string(pair.combinedLimit + pair.humanSpeed)
                          + ", human along normal " + std::to_string(pair.humanSpeed);
                    }),
                    mc_rtc::gui::Label("Observed relative closing > speed limit", [this]() {
                      return std::to_string(actualViolationPairs_);
                    }),
                    mc_rtc::gui::Label("Contact force (simulation)", [this]() {
                      const auto * snapshot = contactForceSnapshot();
                      if(!snapshot) return std::string("no contact proxy");
                      std::string result = "none";
                      double largestRatio = 0;
                      for(const auto & region : snapshot->regions)
                      {
                        const auto it = safetyRegions_.find(region.region);
                        const double ratio = it != safetyRegions_.end() && it->second.force > 0
                            ? region.peak_normal_force_n / it->second.force : 0;
                        if(region.peak_normal_force_n > 0 && (ratio > largestRatio || result == "none"))
                        {
                          largestRatio = ratio;
                          result = region.region + " peak " + std::to_string(region.peak_normal_force_n) + " N";
                          if(it != safetyRegions_.end())
                            result += " / " + std::to_string(it->second.force) + " N";
                        }
                      }
                      if(largestRatio > 1.0) result += " EXCEEDED";
                      return result;
                    }),
                    mc_rtc::gui::Label("Contact speed (sampled witness)", [this]() {
                      const auto * snapshot = contactForceSnapshot();
                      if(!snapshot) return std::string("no contact proxy");
                      const mc_mujoco::SmplhContactForceRegion * strongest = nullptr;
                      for(const auto & contact : snapshot->regions)
                        if(contact.normal_force_n > 0.0
                           && (!strongest || contact.normal_force_n > strongest->normal_force_n))
                          strongest = &contact;
                      if(!strongest) return std::string("no contact");
                      for(size_t i = 0; i < regions.size(); ++i)
                      {
                        if(strongest->region != regions[i]) continue;
                        const auto & pair = pairLogs_[i];
                        if(!pair.sampledVelocityValid) return strongest->region + ": velocity sample unavailable";
                        const double sampledRobotApproach = pair.combinedLimit + pair.actualResidual + pair.humanSpeed;
                        return strongest->region + ": robot approach " + std::to_string(sampledRobotApproach)
                            + ", relative closing " + std::to_string(pair.combinedLimit + pair.actualResidual)
                            + ", relative speed " + std::to_string(pair.actualRelativeSpeed)
                            + " m/s; QP row " + (pair.active ? "active" : "inactive")
                            + ", at bound " + (pair.binding ? "yes" : "no")
                            + ", sample age " + std::to_string(sampleAge_) + " s";
                      }
                      return strongest->region + ": untracked region";
                    }));
  addSpeedPlot();
  logger().addLogEntry("Kinova_sweep_phase", [this]() { return static_cast<int>(phase_); });
  for(int i = 1; i <= 7; ++i)
  {
    const auto name = "joint_" + std::to_string(i);
    const auto index = robot().jointIndexByName(name);
    logger().addLogEntry("Kinova_sweep_" + name + "_command", [this, index]() {
      return postureTask->posture().at(index).at(0);
    });
    logger().addLogEntry("Kinova_sweep_" + name + "_measured", [this, index]() { return robot().q().at(index).at(0); });
    logger().addLogEntry("Kinova_sweep_" + name + "_model", [this, index]() { return robot().q().at(index).at(0); });
    logger().addLogEntry("Kinova_sweep_" + name + "_simulated", [this, i]() {
      const auto * snapshot = distanceSnapshot();
      return snapshot && snapshot->robot_joints_valid ? snapshot->robot_joint_positions[i-1]
                                                      : std::numeric_limits<double>::quiet_NaN();
    });
    logger().addLogEntry("Kinova_sweep_" + name + "_simulated_velocity", [this, i]() {
      const auto * snapshot = distanceSnapshot();
      return snapshot && snapshot->robot_joints_valid ? snapshot->robot_joint_velocities[i-1]
                                                      : std::numeric_limits<double>::quiet_NaN();
    });
  }
  logger().addLogEntry("SMPLH_distance_approximate", [this]() {
    const auto * snapshot = distanceSnapshot();
    return snapshot && snapshot->approximate;
  });
  for(const auto * region : regions)
  {
    const std::string name(region);
    logger().addLogEntry("SMPLH_distance_" + name, [this, name]() { return regionDistance(name); });
    logger().addLogEntry("SMPLH_contact_" + name + "_normal_force_n", [this, name]() {
      return regionContactForce(name, false);
    });
    logger().addLogEntry("SMPLH_contact_" + name + "_peak_normal_force_n", [this, name]() {
      return regionContactForce(name, true);
    });
    logger().addLogEntry("SMPLH_contact_" + name + "_over_force_limit", [this, name]() {
      const auto it = safetyRegions_.find(name);
      const double peak = regionContactForce(name, true);
      return it != safetyRegions_.end() && std::isfinite(peak) && peak > it->second.force;
    });
    logger().addLogEntry("SMPLH_contact_" + name + "_sampled_relative_speed_mps", [this, name]() {
      const double force = regionContactForce(name, false);
      if(!(force > 0.0)) return std::numeric_limits<double>::quiet_NaN();
      for(size_t i = 0; i < regions.size(); ++i)
        if(name == regions[i] && pairLogs_[i].sampledVelocityValid) return pairLogs_[i].actualRelativeSpeed;
      return std::numeric_limits<double>::quiet_NaN();
    });
    logger().addLogEntry("SMPLH_contact_" + name + "_sampled_closing_mps", [this, name]() {
      const double force = regionContactForce(name, false);
      if(!(force > 0.0)) return std::numeric_limits<double>::quiet_NaN();
      for(size_t i = 0; i < regions.size(); ++i)
        if(name == regions[i] && pairLogs_[i].sampledVelocityValid)
          return pairLogs_[i].combinedLimit + pairLogs_[i].actualResidual;
      return std::numeric_limits<double>::quiet_NaN();
    });
  }
  logger().addLogEntry("SMPLH_contact_measurement_ready", [this]() { return contactForceSnapshot() != nullptr; });
  logger().addLogEntry("SSMPFL_enabled", [this]() { return safetyEnabled_; });
  logger().addLogEntry("SSMPFL_solver_success", [this]() { return solverSuccess_; });
  logger().addLogEntry("SSMPFL_fallback", [this]() { return safetyFallback_; });
  logger().addLogEntry("SSMPFL_fallback_reason", [this]() { return fallbackReason_; });
  logger().addLogEntry("SSMPFL_active_pairs", [this]() { return activePairs_; });
  logger().addLogEntry("SSMPFL_monitor_only", [this]() { return monitorOnly_; });
  logger().addLogEntry("SSMPFL_monitor_only_pairs", [this]() { return monitorOnlyPairs_; });
  logger().addLogEntry("SSMPFL_uncontrollable_pairs", [this]() { return uncontrollablePairs_; });
  logger().addLogEntry("SSMPFL_binding_pairs", [this]() { return bindingPairs_; });
  logger().addLogEntry("SSMPFL_at_speed_bound", [this]() { return solverSuccess_ && bindingPairs_ > 0; });
  logger().addLogEntry("SSMPFL_binding_regions", [this]() { return bindingRegions(); });
  logger().addLogEntry("SSMPFL_uncontrollable_regions", [this]() { return uncontrollableRegions(); });
  logger().addLogEntry("SSMPFL_limiting_closing_mps", [this]() {
    return limitingPair_ >= 0 && linkPairLogs_[limitingPair_].active
               ? linkPairLogs_[limitingPair_].closingSpeed : std::numeric_limits<double>::quiet_NaN();
  });
  logger().addLogEntry("SSMPFL_limiting_qp_next_closing_mps", [this]() {
    return solverSuccess_ && limitingPair_ >= 0 && linkPairLogs_[limitingPair_].active
               ? linkPairLogs_[limitingPair_].combinedLimit + linkPairLogs_[limitingPair_].predictedResidual
               : std::numeric_limits<double>::quiet_NaN();
  });
  logger().addLogEntry("SSMPFL_limiting_sampled_closing_mps", [this]() {
    return limitingPair_ >= 0 && linkPairLogs_[limitingPair_].active
               ? linkPairLogs_[limitingPair_].combinedLimit + linkPairLogs_[limitingPair_].actualResidual
               : std::numeric_limits<double>::quiet_NaN();
  });
  logger().addLogEntry("SSMPFL_limiting_speed_limit_mps", [this]() {
    return limitingPair_ >= 0 && linkPairLogs_[limitingPair_].active
               ? linkPairLogs_[limitingPair_].combinedLimit : std::numeric_limits<double>::quiet_NaN();
  });
  logger().addLogEntry("SSMPFL_actual_violation_pairs", [this]() { return actualViolationPairs_; });
  logger().addLogEntry("SSMPFL_limiting_pair", [this]() { return limitingPair_; });
  logger().addLogEntry("SSMPFL_compute_seconds", [this]() { return safetyComputationSeconds_; });
  logger().addLogEntry("SSMPFL_solver_seconds", [this]() { return solver().solveTime(); });
  logger().addLogEntry("SSMPFL_sample_age_seconds", [this]() { return sampleAge_; });
  for(size_t i = 0; i < regions.size(); ++i)
  {
    const std::string prefix = std::string("SSMPFL_") + regions[i] + "_";
    logger().addLogEntry(prefix + "link", [this, i]() { return pairLogs_[i].link; });
    logger().addLogEntry(prefix + "body", [this, i]() { return pairLogs_[i].body; });
    logger().addLogEntry(prefix + "geometry", [this, i]() { return pairLogs_[i].geometry; });
    logger().addLogEntry(prefix + "distance_m", [this, i]() { return pairLogs_[i].distance; });
    logger().addLogEntry(prefix + "intersecting", [this, i]() { return pairLogs_[i].intersecting; });
    logger().addLogEntry(prefix + "robot_point_x", [this, i]() { return pairLogs_[i].robotPointX; });
    logger().addLogEntry(prefix + "robot_point_y", [this, i]() { return pairLogs_[i].robotPointY; });
    logger().addLogEntry(prefix + "robot_point_z", [this, i]() { return pairLogs_[i].robotPointZ; });
    logger().addLogEntry(prefix + "human_point_x", [this, i]() { return pairLogs_[i].humanPointX; });
    logger().addLogEntry(prefix + "human_point_y", [this, i]() { return pairLogs_[i].humanPointY; });
    logger().addLogEntry(prefix + "human_point_z", [this, i]() { return pairLogs_[i].humanPointZ; });
    logger().addLogEntry(prefix + "model_witness_error_m", [this, i]() { return pairLogs_[i].modelWitnessError; });
    logger().addLogEntry(prefix + "normal_x", [this, i]() { return pairLogs_[i].nx; });
    logger().addLogEntry(prefix + "normal_y", [this, i]() { return pairLogs_[i].ny; });
    logger().addLogEntry(prefix + "normal_z", [this, i]() { return pairLogs_[i].nz; });
    logger().addLogEntry(prefix + "human_approach_mps", [this, i]() { return pairLogs_[i].humanSpeed; });
    logger().addLogEntry(prefix + "human_velocity_x", [this, i]() { return pairLogs_[i].humanVelocityX; });
    logger().addLogEntry(prefix + "human_velocity_y", [this, i]() { return pairLogs_[i].humanVelocityY; });
    logger().addLogEntry(prefix + "human_velocity_z", [this, i]() { return pairLogs_[i].humanVelocityZ; });
    logger().addLogEntry(prefix + "robot_approach_mps", [this, i]() { return pairLogs_[i].robotSpeed; });
    logger().addLogEntry(prefix + "closing_mps", [this, i]() { return pairLogs_[i].closingSpeed; });
    logger().addLogEntry(prefix + "actual_relative_speed_mps", [this, i]() { return pairLogs_[i].actualRelativeSpeed; });
    logger().addLogEntry(prefix + "actual_tangential_speed_mps", [this, i]() { return pairLogs_[i].actualTangentialSpeed; });
    logger().addLogEntry(prefix + "inverse_mass_kg_inv", [this, i]() { return pairLogs_[i].inverseMass; });
    logger().addLogEntry(prefix + "reflected_mass_kg", [this, i]() { return pairLogs_[i].reflectedMass; });
    logger().addLogEntry(prefix + "pfl_limit_mps", [this, i]() { return pairLogs_[i].pflLimit; });
    logger().addLogEntry(prefix + "combined_limit_mps", [this, i]() { return pairLogs_[i].combinedLimit; });
    logger().addLogEntry(prefix + "predicted_residual_mps", [this, i]() { return pairLogs_[i].predictedResidual; });
    logger().addLogEntry(prefix + "actual_residual_mps", [this, i]() { return pairLogs_[i].actualResidual; });
    logger().addLogEntry(prefix + "active", [this, i]() { return pairLogs_[i].active; });
    logger().addLogEntry(prefix + "uncontrollable", [this, i]() { return pairLogs_[i].uncontrollable; });
    logger().addLogEntry(prefix + "monitor_only", [this, i]() { return pairLogs_[i].monitorOnly; });
    logger().addLogEntry(prefix + "binding", [this, i]() { return pairLogs_[i].binding; });
    logger().addLogEntry(prefix + "actual_violation", [this, i]() { return pairLogs_[i].actualViolation; });
  }
  for(size_t i = 0; i < linkPairLogs_.size(); ++i)
  {
    const std::string prefix = std::string("SSMPFLPair_") + regions[i / links.size()] + "_"
                               + links[i % links.size()] + "_";
    logger().addLogEntry(prefix + "body", [this, i]() { return linkPairLogs_[i].body; });
    logger().addLogEntry(prefix + "distance_m", [this, i]() { return linkPairLogs_[i].distance; });
    logger().addLogEntry(prefix + "closing_mps", [this, i]() { return linkPairLogs_[i].closingSpeed; });
    logger().addLogEntry(prefix + "combined_limit_mps", [this, i]() { return linkPairLogs_[i].combinedLimit; });
    logger().addLogEntry(prefix + "predicted_residual_mps", [this, i]() { return linkPairLogs_[i].predictedResidual; });
    logger().addLogEntry(prefix + "actual_residual_mps", [this, i]() { return linkPairLogs_[i].actualResidual; });
    logger().addLogEntry(prefix + "active", [this, i]() { return linkPairLogs_[i].active; });
    logger().addLogEntry(prefix + "binding", [this, i]() { return linkPairLogs_[i].binding; });
    logger().addLogEntry(prefix + "uncontrollable", [this, i]() { return linkPairLogs_[i].uncontrollable; });
    logger().addLogEntry(prefix + "monitor_only", [this, i]() { return linkPairLogs_[i].monitorOnly; });
  }
  mc_rtc::log::success("KinovaSweepController initialized; waiting for Start");
}

KinovaSweepController::~KinovaSweepController()
{
  if(safetyConstraint_)
    mc_solver::TasksQPSolver::from_solver(solver()).removeConstraint(safetyConstraint_.get());
}

void KinovaSweepController::addSpeedPlot()
{
  const auto lastValid = [](double value, double time) {
    return time >= 0 ? value : std::numeric_limits<double>::quiet_NaN();
  };
  gui()->addPlot("SSM+PFL robot approach speed (m/s)",
                 mc_rtc::gui::plot::X("simulation time (s)", [this]() { return safetyTime_; }),
                 mc_rtc::gui::plot::Y("Current model robot approach", [this, lastValid]() {
                   return lastValid(speedPlot_.current, speedPlot_.sampleTime);
                 }, mc_rtc::gui::Color::Blue, mc_rtc::gui::plot::Style::Solid),
                 mc_rtc::gui::plot::Y("QP next robot approach", [this, lastValid]() {
                   return lastValid(speedPlot_.qpNext, speedPlot_.qpTime);
                 }, mc_rtc::gui::Color::Green, mc_rtc::gui::plot::Style::Solid),
                 mc_rtc::gui::plot::Y("Sampled MuJoCo robot approach (delayed)", [this, lastValid]() {
                   return lastValid(speedPlot_.sampled, speedPlot_.sampleTime);
                 }, mc_rtc::gui::Color::Magenta, mc_rtc::gui::plot::Style::Solid),
                 mc_rtc::gui::plot::Y("Speed limit", [this, lastValid]() {
                   return lastValid(speedPlot_.limit, speedPlot_.sampleTime);
                 }, mc_rtc::gui::Color::Red, mc_rtc::gui::plot::Style::Solid));
}

void KinovaSweepController::updateSpeedPlot()
{
  if(safetyFallback_) return;
  const auto & pair = linkPairLogs_[plotPairIndex_];
  if(!pair.sampledVelocityValid) return;
  speedPlot_.current = pair.robotSpeed;
  speedPlot_.sampled = pair.combinedLimit + pair.actualResidual + pair.humanSpeed;
  speedPlot_.limit = pair.combinedLimit + pair.humanSpeed;
  speedPlot_.sampleTime = safetyTime_;
  if(solverSuccess_ && pair.active)
  {
    speedPlot_.qpNext = pair.combinedLimit + pair.predictedResidual + pair.humanSpeed;
    speedPlot_.qpTime = safetyTime_;
  }
}

void KinovaSweepController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
  startRequested_ = autoStart_;
  autoStartPending_ = autoStart_;
  stopRequested_ = false;
  phase_ = Phase::Hold;
  elapsed_ = 0.0;
  commandedJoint1_ = sampledOrModelJoint1();
  safetyTime_ = 0;
  safetyFallback_ = false;
  fallbackReason_.clear();
  monitorOnly_ = false;
  resumeAfterMeasurement_ = false;
  speedPlot_ = SpeedPlotSample{};
}

double KinovaSweepController::sampledOrModelJoint1() const
{
  const auto * sample = distanceSnapshot();
  const double age = sample ? safetyTime_ - sample->sample_simulation_time : 0;
  if(sample && sample->robot_joints_valid && std::isfinite(age) && age >= -solver().dt() && age <= maxSampleAge_)
    return sample->robot_joint_positions[0];
  return robot().q().at(jointIndex_).at(0);
}

void KinovaSweepController::stopAtAvailablePosture()
{
  auto posture = robot().q();
  const auto * sample = distanceSnapshot();
  const double age = sample ? safetyTime_ - sample->sample_simulation_time : 0;
  if(sample && sample->robot_joints_valid && std::isfinite(age) && age >= -solver().dt() && age <= maxSampleAge_)
    for(int joint = 1; joint <= 7; ++joint)
      posture.at(robot().jointIndexByName("joint_" + std::to_string(joint))).at(0)
          = sample->robot_joint_positions[joint - 1];
  postureTask->posture(posture);
  commandedJoint1_ = sampledOrModelJoint1();
  phase_ = Phase::Hold;
  elapsed_ = 0.0;
}

void KinovaSweepController::stopSweepAtAvailableJoint1()
{
  if(phase_ == Phase::Hold) return;
  commandedJoint1_ = sampledOrModelJoint1();
  postureTask->target({{"joint_1", {commandedJoint1_}}});
  phase_ = Phase::Hold;
  elapsed_ = 0.0;
}

void KinovaSweepController::beginSegment(Phase phase, double goal)
{
  phase_ = phase;
  elapsed_ = 0.0;
  startPosition_ = commandedJoint1_;
  goalPosition_ = goal;
  // The quintic's maximum normalized slope is 1.875.
  activeSegmentSeconds_ = std::max(segmentSeconds_, 1.875 * std::abs(goal - startPosition_) / maxJointSpeed_);
}

bool KinovaSweepController::run()
{
  safetyTime_ += solver().dt();
  // Stop wins if both buttons were pressed since the previous controller tick.
  if(stopRequested_.exchange(false))
  {
    startRequested_ = false;
    resumeAfterMeasurement_ = false;
    autoStartPending_ = false;
    stopAtAvailablePosture();
  }
  else if(startRequested_.exchange(false) && phase_ == Phase::Hold)
  {
    commandedJoint1_ = sampledOrModelJoint1();
    beginSegment(Phase::ToWaypoint, amplitude_);
  }

  const double dt = solver().dt();
  if(phase_ == Phase::ToWaypoint || phase_ == Phase::ToHome)
  {
    elapsed_ = std::min(activeSegmentSeconds_, elapsed_ + dt);
    commandedJoint1_ = startPosition_ + (goalPosition_ - startPosition_) * smoothStep(elapsed_ / activeSegmentSeconds_);
    postureTask->target({{"joint_1", {commandedJoint1_}}});
    if(elapsed_ >= activeSegmentSeconds_)
    {
      phase_ = phase_ == Phase::ToWaypoint ? Phase::DwellWaypoint : Phase::DwellHome;
      elapsed_ = 0.0;
    }
  }
  else if(phase_ == Phase::DwellWaypoint || phase_ == Phase::DwellHome)
  {
    elapsed_ += dt;
    if(elapsed_ >= dwellSeconds_)
    {
      if(phase_ == Phase::DwellWaypoint) beginSegment(Phase::ToHome, 0.0);
      else beginSegment(Phase::ToWaypoint, amplitude_);
    }
  }
  monitorOnly_ = safetyEnabled_ && phase_ == Phase::Hold && !constrainWhileHolding_;
  if(safetyEnabled_ && !updateSafetyConstraint(!monitorOnly_) && holdOnConstraintUnavailable_)
    stopSweepAtAvailableJoint1();
  solverSuccess_ = mc_control::MCController::run();
  if(!solverSuccess_)
  {
    if(safetyEnabled_ && activePairs_ > 0 && limitingPair_ >= 0)
    {
      const auto & pair = linkPairLogs_[limitingPair_];
      mc_rtc::log::error("Kinova sweep QP failed with {} SSM+PFL rows; tightest pair {}: clearance {} m, "
                         "closing {} m/s, limit {} m/s, sample age {} s",
                         activePairs_, pairLabel(limitingPair_), pair.distance, pair.closingSpeed,
                         pair.combinedLimit, sampleAge_);
    }
    safetyFallback("QP solve failed, requested posture hold");
    stopSweepAtAvailableJoint1();
  }
  if(safetyEnabled_ && solverSuccess_)
  {
    const auto acceleration = rbd::dofToVector(robot().mb(), robot().alphaD());
    double closestResidual = -std::numeric_limits<double>::infinity();
    limitingPair_ = -1;
    bindingPairs_ = 0;
    for(size_t i = 0; i < linkPairLogs_.size(); ++i)
      if(linkPairLogs_[i].active)
      {
        linkPairLogs_[i].predictedResidual = safetyConstraint_->residual(static_cast<int>(i), acceleration);
        if(linkPairLogs_[i].inverseMass > minMobility_)
        {
          linkPairLogs_[i].binding = linkPairLogs_[i].predictedResidual >= -0.001;
          if(linkPairLogs_[i].binding) ++bindingPairs_;
          if(linkPairLogs_[i].predictedResidual > closestResidual)
          {
            closestResidual = linkPairLogs_[i].predictedResidual;
            limitingPair_ = static_cast<int>(i);
          }
        }
      }
    for(size_t region = 0; region < regions.size(); ++region)
    {
      const size_t first = region * links.size();
      size_t closest = first;
      for(size_t link = 1; link < links.size(); ++link)
        if(linkPairLogs_[first + link].distance < linkPairLogs_[closest].distance)
          closest = first + link;
      pairLogs_[region] = linkPairLogs_[closest];
    }
  }
  updateSpeedPlot();
  return solverSuccess_;
}

const mc_mujoco::SmplhDistanceSnapshot * KinovaSweepController::distanceSnapshot() const
{
  static const std::string key = "SMPLH::DistanceSnapshot";
  if(!datastore().has(key)) return nullptr;
  const auto & snapshot = datastore().get<mc_mujoco::SmplhDistanceSnapshot>(key);
  return snapshot.ready && snapshot.error.empty() ? &snapshot : nullptr;
}

const mc_mujoco::SmplhContactForceSnapshot * KinovaSweepController::contactForceSnapshot() const
{
  static const std::string key = "SMPLH::ContactForceSnapshot";
  if(!datastore().has(key)) return nullptr;
  const auto & snapshot = datastore().get<mc_mujoco::SmplhContactForceSnapshot>(key);
  return snapshot.ready ? &snapshot : nullptr;
}

double KinovaSweepController::regionContactForce(const std::string & region, bool peak) const
{
  const auto * snapshot = contactForceSnapshot();
  if(!snapshot) return std::numeric_limits<double>::quiet_NaN();
  for(const auto & value : snapshot->regions)
    if(value.region == region) return peak ? value.peak_normal_force_n : value.normal_force_n;
  return std::numeric_limits<double>::quiet_NaN();
}

double KinovaSweepController::regionDistance(const std::string & region) const
{
  const auto * snapshot = distanceSnapshot();
  if(!snapshot) return std::numeric_limits<double>::quiet_NaN();
  double distance = std::numeric_limits<double>::infinity();
  for(const auto & value : snapshot->regions)
    if(value.region == region) distance = std::min(distance, value.distance);
  return std::isfinite(distance) ? distance : std::numeric_limits<double>::quiet_NaN();
}

std::string KinovaSweepController::bindingRegions() const
{
  if(!solverSuccess_) return "QP failed";
  std::string result;
  for(size_t i = 0; i < linkPairLogs_.size(); ++i)
  {
    if(!linkPairLogs_[i].binding) continue;
    if(!result.empty()) result += ", ";
    result += pairLabel(i);
  }
  return result.empty() ? "none" : result;
}

std::string KinovaSweepController::uncontrollableRegions() const
{
  std::string result;
  for(size_t i = 0; i < linkPairLogs_.size(); ++i)
  {
    if(!linkPairLogs_[i].uncontrollable) continue;
    if(!result.empty()) result += ", ";
    result += pairLabel(i);
  }
  return result.empty() ? "none" : result;
}

void KinovaSweepController::safetyFallback(const std::string & reason)
{
  const bool transientMeasurement = reason == "distance snapshot unavailable or worker error"
      || reason == "distance sample is stale or timestamp invalid";
  if(!transientMeasurement) resumeAfterMeasurement_ = false;
  if(holdOnConstraintUnavailable_ && transientMeasurement
     && (phase_ == Phase::ToWaypoint || phase_ == Phase::ToHome))
  {
    resumeAfterMeasurement_ = true;
    resumePhase_ = phase_;
    resumeGoal_ = goalPosition_;
  }
  if(!safetyFallback_ || fallbackReason_ != reason)
  {
    if(holdOnConstraintUnavailable_) mc_rtc::log::error("SSM+PFL fallback: {}", reason);
    else if(reason == "QP solve failed, requested posture hold")
      mc_rtc::log::error("SSM+PFL QP failure: {}", reason);
    else mc_rtc::log::warning("SSM+PFL rows skipped for this sample: {}", reason);
  }
  safetyFallback_ = true;
  fallbackReason_ = reason;
  if(autoStartPending_) startRequested_ = true;
  if(safetyConstraint_) safetyConstraint_->clear();
}

bool KinovaSweepController::updateSafetyConstraint(bool applyConstraint)
{
  const auto started = std::chrono::steady_clock::now();
  safetyConstraint_->clear();
  activePairs_ = 0;
  uncontrollablePairs_ = 0;
  monitorOnlyPairs_ = 0;
  bindingPairs_ = 0;
  actualViolationPairs_ = 0;
  limitingPair_ = -1;
  for(auto & log : pairLogs_) log = PairLog{};
  for(auto & log : linkPairLogs_) log = PairLog{};
  auto fail = [this, &started](const std::string & reason) {
    safetyFallback(reason);
    safetyComputationSeconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return false;
  };
  const auto * snapshot = distanceSnapshot();
  if(!snapshot) return fail("distance snapshot unavailable or worker error");
  if(!movingHuman_ && !snapshot->playback_frozen)
    return fail("HumanMode frozen requires SMPLH_FREEZE_FRAME or held playback");
  const double age = safetyTime_ - snapshot->sample_simulation_time;
  sampleAge_ = age;
  if(!std::isfinite(age) || age < -solver().dt() || age > maxSampleAge_)
    return fail("distance sample is stale or timestamp invalid");
  if(snapshot->regions.size() != linkPairLogs_.size()) return fail("missing monitored region/link pairs");
  std::map<std::pair<std::string, std::string>, const mc_mujoco::SmplhRegionDistance *> samples;
  for(const auto & sample : snapshot->regions)
    if(!samples.emplace(std::make_pair(sample.region, sample.link), &sample).second)
      return fail("duplicate monitored region/link pair");
  const auto & mb = robot().mb();
  const auto & mbc = robot().mbc();
  const int dofs = mb.nrDof();
  const Eigen::VectorXd qdot = rbd::dofToVector(mb, robot().alpha());
  if(qdot.size() != dofs || !qdot.allFinite()) return fail("invalid model joint velocity");
  inertia_->computeH(mb, mbc);
  const Eigen::MatrixXd & mass = inertia_->H();
  if(mass.rows() != dofs || mass.cols() != dofs || !mass.allFinite()) return fail("invalid inertia matrix");
  const Eigen::LDLT<Eigen::MatrixXd> factor(mass);
  if(factor.info() != Eigen::Success || factor.vectorD().minCoeff() < minMassEigenvalue_)
    return fail("inertia matrix is not positive definite within tolerance");
  double tightest = std::numeric_limits<double>::infinity();
  for(size_t i = 0; i < linkPairLogs_.size(); ++i)
  {
    const std::string name(regions[i / links.size()]);
    const std::string pair = pairLabel(i);
    const auto found = samples.find({name, links[i % links.size()]});
    if(found == samples.end()) return fail("missing region/link pair " + pair);
    const auto * sample = found->second;
    auto & log = linkPairLogs_[i];
    log.link = sample->link;
    log.body = sample->robot_body;
    log.geometry = sample->robot_geometry;
    log.distance = sample->distance;
    log.intersecting = sample->intersecting;
    log.robotPointX = sample->robot_point[0];
    log.robotPointY = sample->robot_point[1];
    log.robotPointZ = sample->robot_point[2];
    log.humanPointX = sample->human_point[0];
    log.humanPointY = sample->human_point[1];
    log.humanPointZ = sample->human_point[2];
    if(!std::isfinite(sample->distance) || sample->distance < 0)
      return fail("invalid clearance at " + pair);
    if(!sample->closest_points_valid || !sample->direction_valid || sample->robot_body.empty())
      return fail("missing witness or normal at " + pair);
    if(movingHuman_ && !sample->human_velocity_valid)
      return fail("human material velocity unavailable at " + pair);
    Eigen::Vector3d n(-sample->direction[0], -sample->direction[1], -sample->direction[2]);
    const double length = n.norm();
    if(!n.allFinite() || !std::isfinite(length) || length < 1e-9)
      return fail("degenerate normal at " + pair);
    n /= length;
    const Eigen::Vector3d local(sample->robot_point_body_local[0], sample->robot_point_body_local[1],
                                sample->robot_point_body_local[2]);
    if(!local.allFinite()) return fail("invalid body-local witness at " + pair);
    try { robot().bodyIndexByName(sample->robot_body); }
    catch(const std::exception &) { return fail("MuJoCo body missing from mc_rtc model: " + sample->robot_body); }
    const auto & bodyPose = robot().bodyPosW(sample->robot_body);
    const Eigen::Vector3d modelWitness = bodyPose.rotation().transpose() * local + bodyPose.translation();
    const Eigen::Vector3d sampledWitness(sample->robot_point[0], sample->robot_point[1], sample->robot_point[2]);
    log.modelWitnessError = (modelWitness - sampledWitness).norm();
    if(!std::isfinite(log.modelWitnessError) || log.modelWitnessError > maxModelWitnessError_)
      return fail("robot model/witness disagreement at " + pair);
    rbd::Jacobian jacobian(mb, sample->robot_body, local);
    Eigen::MatrixXd full(6, dofs), fullDot(6, dofs);
    jacobian.fullJacobian(mb, jacobian.jacobian(mb, mbc), full);
    jacobian.fullJacobian(mb, jacobian.jacobianDot(mb, mbc), fullDot);
    if(!full.allFinite() || !fullDot.allFinite()) return fail("invalid point Jacobian at " + pair);
    const Eigen::RowVectorXd directional = n.transpose() * full.bottomRows(3);
    const auto & parameters = safetyRegions_.at(name);
    ssm_pfl::Limit limit;
    try { limit = ssm_pfl::limit(factor, directional, parameters.force, parameters.stiffness,
                               sample->distance, parameters.margin, stopTime_, minMobility_); }
    catch(const std::exception &) { return fail("invalid directional mass or speed limit at " + pair); }
    const Eigen::Vector3d humanVelocity = movingHuman_
      ? Eigen::Vector3d(sample->human_velocity[0], sample->human_velocity[1], sample->human_velocity[2])
      : Eigen::Vector3d::Zero();
    if(!humanVelocity.allFinite()) return fail("invalid human velocity at " + pair);
    log.humanVelocityX = humanVelocity.x();
    log.humanVelocityY = humanVelocity.y();
    log.humanVelocityZ = humanVelocity.z();
    const double humanApproach = n.dot(humanVelocity);
    const double robotApproach = directional.dot(qdot);
    if(!sample->robot_velocity_valid) return fail("MuJoCo point velocity unavailable at " + pair);
    const Eigen::Vector3d actualVelocity(sample->robot_velocity[0], sample->robot_velocity[1],
                                         sample->robot_velocity[2]);
    if(!actualVelocity.allFinite()) return fail("invalid simulated point speed at " + pair);
    const double actualRobotApproach = n.dot(actualVelocity);
    const Eigen::Vector3d relativeVelocity = actualVelocity - humanVelocity;
    log.actualRelativeSpeed = relativeVelocity.norm();
    log.actualTangentialSpeed = (relativeVelocity - n * n.dot(relativeVelocity)).norm();
    log.sampledVelocityValid = true;
    log.nx = n.x(); log.ny = n.y(); log.nz = n.z();
    log.humanSpeed = humanApproach;
    log.robotSpeed = robotApproach;
    log.closingSpeed = robotApproach - humanApproach;
    log.inverseMass = limit.inverseMass;
    log.reflectedMass = limit.reflectedMass;
    log.pflLimit = limit.pflSpeed;
    log.combinedLimit = limit.combinedSpeed;
    log.actualResidual = actualRobotApproach - humanApproach - limit.combinedSpeed;
    if(!std::isfinite(log.actualResidual)) return fail("invalid simulated point speed at " + pair);
    log.actualViolation = log.actualResidual > maxActualResidual_;
    if(log.actualViolation) ++actualViolationPairs_;
    // A selected point can have zero mobility along this direction. A negative
    // speed allowance would make its zero-acceleration row impossible.
    // Keep the measured violation visible, but do not add an unachievable row.
    if(limit.inverseMass <= minMobility_)
    {
      log.uncontrollable = true;
      ++uncontrollablePairs_;
      continue;
    }
    if(!applyConstraint)
    {
      log.monitorOnly = true;
      ++monitorOnlyPairs_;
      continue;
    }
    const double drift = (n.transpose() * fullDot.bottomRows(3) * qdot).value();
    double bound;
    try { bound = ssm_pfl::accelerationBound(limit.combinedSpeed, robotApproach,
                                            humanApproach, drift, solver().dt()); }
    catch(const std::exception &) { return fail("invalid QP bound at " + pair); }
    safetyConstraint_->set(static_cast<int>(i), solver().dt() * directional, bound);
    log.active = true;
    ++activePairs_;
    const double margin = limit.combinedSpeed - log.closingSpeed;
    if(margin < tightest) { tightest = margin; limitingPair_ = static_cast<int>(i); }
  }
  for(size_t region = 0; region < regions.size(); ++region)
  {
    const size_t first = region * links.size();
    size_t closest = first;
    for(size_t link = 1; link < links.size(); ++link)
      if(linkPairLogs_[first + link].distance < linkPairLogs_[closest].distance)
        closest = first + link;
    pairLogs_[region] = linkPairLogs_[closest];
  }
  safetyFallback_ = false;
  fallbackReason_.clear();
  if(resumeAfterMeasurement_)
  {
    commandedJoint1_ = sampledOrModelJoint1();
    beginSegment(resumePhase_, resumeGoal_);
    resumeAfterMeasurement_ = false;
  }
  autoStartPending_ = false;
  safetyComputationSeconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  return true;
}

CONTROLLER_CONSTRUCTOR("KinovaSweepController", KinovaSweepController)
