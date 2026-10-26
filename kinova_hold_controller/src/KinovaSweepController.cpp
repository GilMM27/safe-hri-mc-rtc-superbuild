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
  derivatives_.resize(linkPairLogs_.size());
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
      std::string constraintMode = "legacy";
      safety("ConstraintMode", constraintMode);
      if(constraintMode != "legacy" && constraintMode != "temporal_barrier" && constraintMode != "pointwise_speed")
        mc_rtc::log::error_and_throw("SSMPFL.ConstraintMode must be legacy, temporal_barrier or pointwise_speed");
      // Moving humans always use the continuous pointwise bound, including old configurations.
      temporalBarrier_ = !movingHuman_ && constraintMode == "temporal_barrier";
      safety("BarrierGammaPerSecond", barrierGamma_);
      safety("MaxDerivativeGapSeconds", maxDerivativeGap_);
      safety("MaxWitnessStepM", maxWitnessStep_);
      safety("MaxNormalStepRad", maxNormalStep_);
      if(temporalBarrier_ && (!std::isfinite(barrierGamma_) || barrierGamma_ <= 0
         || !std::isfinite(maxDerivativeGap_) || maxDerivativeGap_ <= 0
         || !std::isfinite(maxWitnessStep_) || maxWitnessStep_ <= 0
         || !std::isfinite(maxNormalStep_) || maxNormalStep_ <= 0))
        mc_rtc::log::error_and_throw("Invalid SSMPFL temporal derivative settings");
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
      if(safety.has("Relaxation"))
      {
        const auto recovery = safety("Relaxation");
        recovery("Enabled", relaxationEnabled_);
        if(relaxationEnabled_)
        {
          std::string mode = "two_stage_regularized";
          recovery("Mode", mode);
          if(mode != "two_stage_regularized" && mode != "two_stage_full_feasibility")
            mc_rtc::log::error_and_throw("SSMPFL.Relaxation.Mode must be two_stage_regularized or two_stage_full_feasibility");
          fullQPRecovery_ = mode == "two_stage_full_feasibility";
          recovery("ActivationTolerance", relaxationActivationTolerance_);
          recovery("RecoveryTolerance", relaxationRecoveryTolerance_);
          recovery("Stage2Tolerance", relaxationStage2Tolerance_);
          if(movingHuman_) recovery("MaxStage2SlackMps", relaxationMaxStage2Slack_);
          recovery("AccelerationRegularization", accelerationRegularization_);
          recovery("MaxAccelerationRadS2", relaxationMaxAcceleration_);
          double defaultScale = temporalBarrier_ ? 1.0 : 0.01;
          recovery("DefaultScale", defaultScale);
          if(!std::isfinite(defaultScale) || defaultScale <= 0
             || !std::isfinite(relaxationActivationTolerance_) || relaxationActivationTolerance_ <= 0
             || !std::isfinite(relaxationRecoveryTolerance_) || relaxationRecoveryTolerance_ < 0
             || relaxationRecoveryTolerance_ >= relaxationActivationTolerance_
             || !std::isfinite(relaxationStage2Tolerance_) || relaxationStage2Tolerance_ < 0
             || (movingHuman_ && (!std::isfinite(relaxationMaxStage2Slack_) || relaxationMaxStage2Slack_ <= 0))
             || !std::isfinite(accelerationRegularization_) || accelerationRegularization_ <= 0)
            mc_rtc::log::error_and_throw("Invalid SSMPFL relaxation settings");
          if(!std::isfinite(relaxationMaxAcceleration_) || relaxationMaxAcceleration_ <= 0)
            mc_rtc::log::error_and_throw("SSMPFL.Relaxation.MaxAccelerationRadS2 must state an explicit positive research acceleration bound");
          // The auxiliary QP can mirror model joint bounds, but mc_rtc does not
          // expose arbitrary loaded hard rows. Refuse unsupported configurations.
          if(!this->config()("constraints", std::vector<mc_rtc::Configuration>{}).empty()
             || !this->config()("collisions", std::vector<mc_rtc::Configuration>{}).empty()
             || !this->config()("contacts", std::vector<mc_rtc::Configuration>{}).empty())
            mc_rtc::log::error_and_throw("SSMPFL relaxation cannot mirror additional mc_rtc hard constraints");
          relaxationWeights_ = Eigen::VectorXd::Ones(linkPairLogs_.size());
          relaxationScales_ = Eigen::VectorXd::Constant(linkPairLogs_.size(), defaultScale);
          const auto overrides = recovery("Regions", mc_rtc::Configuration{});
          for(size_t i = 0; i < linkPairLogs_.size(); ++i)
          {
            const std::string region(regions[i / links.size()]);
            if(overrides.has(region))
            {
              const auto entry = overrides(region);
              entry("Weight", relaxationWeights_[i]);
              entry("Scale", relaxationScales_[i]);
            }
            if(!std::isfinite(relaxationWeights_[i]) || relaxationWeights_[i] <= 0
               || !std::isfinite(relaxationScales_[i]) || relaxationScales_[i] <= 0)
              mc_rtc::log::error_and_throw("Invalid SSMPFL relaxation weight/scale for {}", region);
          }
          relaxation_.reset(new SSMPFLRelaxation(robot().mb().nrDof(),
                                                static_cast<int>(linkPairLogs_.size())));
          accelerationBounds_.reset(new SSMPFLAccelerationBounds(0, robot().mb().nrDof()));
          mc_solver::TasksQPSolver::from_solver(solver()).addConstraint(accelerationBounds_.get());
        }
      }
      safetyConstraint_.reset(new SSMPFLConstraint(0, robot().mb().nrDof(),
                                                  static_cast<int>(linkPairLogs_.size()), temporalBarrier_,
                                                  relaxationEnabled_));
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
                      return !safetyEnabled_ ? std::string("disabled") : movingHuman_ ?
                          std::string("moving pointwise speed") : temporalBarrier_ ?
                          std::string(movingHuman_ ? "moving temporal barrier" : "frozen temporal barrier")
                          : movingHuman_ ? std::string("moving legacy") : std::string("frozen legacy");
                    }),
                    mc_rtc::gui::Label("State", [this]() {
                      if(!solverSuccess_) return std::string("QP failed");
                      if(monitorOnly_) return std::string("monitoring while waiting for Start");
                      if(safetyFallback_)
                        return std::string(resumeAfterMeasurement_ ? "Paused for valid measurement: "
                            : holdOnConstraintUnavailable_ ? "Stop requested: "
                                                           : "Constraint unavailable: ") + fallbackReason_;
                      if(recoveryMode_) return std::string("SSM+PFL relaxed recovery (safety unverified)");
                      return std::string("QP running");
                    }),
                    mc_rtc::gui::Label("Limiting region", [this]() {
                      return limitingPair_ < 0 ? std::string("none") : pairLabel(limitingPair_);
                    }),
                    mc_rtc::gui::Label("At QP bound", [this]() { return bindingRegions(); }),
                    mc_rtc::gui::Label("Uncontrollable regions", [this]() { return uncontrollableRegions(); }),
                    mc_rtc::gui::Label("Limiting speed (m/s)", [this]() {
                      if(!solverSuccess_ || limitingPair_ < 0 || !linkPairLogs_[limitingPair_].active)
                        return std::string("unavailable");
                      const auto & pair = linkPairLogs_[limitingPair_];
                      if(temporalBarrier_)
                        return std::string("barrier ") + std::to_string(pair.barrier) + " m/s, QP acceleration residual "
                            + std::to_string(pair.predictedResidual) + " m/s²";
                      return std::string("robot current ") + std::to_string(pair.robotSpeed)
                          + ", QP next " + std::to_string(pair.predictedRobotSpeed)
                          + ", sampled MuJoCo (delayed) "
                          + std::to_string(pair.combinedLimit + pair.actualResidual + pair.humanSpeed)
                          + ", robot limit " + std::to_string(pair.robotBudget)
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
               ? (temporalBarrier_
                    ? linkPairLogs_[limitingPair_].robotSpeed + solver().dt() *
                        (linkPairLogs_[limitingPair_].beta + linkPairLogs_[limitingPair_].qpBound
                         + linkPairLogs_[limitingPair_].originalResidual) - linkPairLogs_[limitingPair_].humanSpeed
                    : linkPairLogs_[limitingPair_].predictedRobotSpeed - linkPairLogs_[limitingPair_].humanSpeed)
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
  logger().addLogEntry("SSMPFL_constraint_mode", [this]() {
    return movingHuman_ ? std::string("pointwise_speed") : temporalBarrier_ ? std::string("temporal_barrier") : std::string("legacy");
  });
  logger().addLogEntry("SSMPFL_relaxation_enabled", [this]() { return relaxationEnabled_; });
  logger().addLogEntry("SSMPFL_recovery_mode", [this]() { return recoveryMode_; });
  logger().addLogEntry("SSMPFL_relaxed_pairs", [this]() { return relaxedPairs_; });
  logger().addLogEntry("SSMPFL_retreat_relaxed_pairs", [this]() { return retreatRelaxedPairs_; });
  logger().addLogEntry("SSMPFL_relaxation_objective", [this]() { return relaxationObjective_; });
  logger().addLogEntry("SSMPFL_stage1_status", [this]() { return stage1Status_; });
  logger().addLogEntry("SSMPFL_stage2_status", [this]() { return stage2Status_; });
  logger().addLogEntry("SSMPFL_stage1_seconds", [this]() { return stage1Seconds_; });
  logger().addLogEntry("SSMPFL_stage2_seconds", [this]() { return stage2Seconds_; });
  logger().addLogEntry("SSMPFL_stage2_retry_attempts", [this]() { return stage2RetryAttempts_; });
  logger().addLogEntry("SSMPFL_stage2_retry_cap_mps", [this]() { return stage2RetryCap_; });
  logger().addLogEntry("SSMPFL_full_qp_extra_slack", [this]() { return fullQPExtraSlack_; });
  logger().addLogEntry("SSMPFL_full_qp_exceeded_legacy_cap", [this]() { return fullQPExceededLegacyCap_; });
  logger().addLogEntry("SSMPFL_largest_normalized_relaxation", [this]() {
    double largest = 0;
    for(size_t i = 0; i < linkPairLogs_.size(); ++i)
      if(linkPairLogs_[i].active && relaxationEnabled_)
        largest = std::max(largest, std::max(linkPairLogs_[i].relaxation,
                                             linkPairLogs_[i].retreatRelaxation) / relaxationScales_[i]);
    return largest;
  });
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
    logger().addLogEntry(prefix + "robot_budget_mps", [this, i]() { return pairLogs_[i].robotBudget; });
    logger().addLogEntry(prefix + "qp_next_robot_speed_mps", [this, i]() { return pairLogs_[i].predictedRobotSpeed; });
    logger().addLogEntry(prefix + "predicted_residual_mps", [this, i]() {
      return temporalBarrier_ ? std::numeric_limits<double>::quiet_NaN() : pairLogs_[i].predictedResidual;
    });
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
    logger().addLogEntry(prefix + "robot_budget_mps", [this, i]() { return linkPairLogs_[i].robotBudget; });
    logger().addLogEntry(prefix + "qp_next_robot_speed_mps", [this, i]() { return linkPairLogs_[i].predictedRobotSpeed; });
    logger().addLogEntry(prefix + "predicted_residual_mps", [this, i]() {
      return temporalBarrier_ ? std::numeric_limits<double>::quiet_NaN() : linkPairLogs_[i].predictedResidual;
    });
    logger().addLogEntry(prefix + "acceleration_residual_mps2", [this, i]() {
      return temporalBarrier_ ? linkPairLogs_[i].predictedResidual : std::numeric_limits<double>::quiet_NaN();
    });
    logger().addLogEntry(prefix + "sample_sequence", [this, i]() { return linkPairLogs_[i].sequence; });
    logger().addLogEntry(prefix + "sample_age_seconds", [this, i]() { return linkPairLogs_[i].sampleAge; });
    logger().addLogEntry(prefix + "derivative_valid", [this, i]() { return linkPairLogs_[i].derivativeValid; });
    logger().addLogEntry(prefix + "derivative_fallback", [this, i]() { return linkPairLogs_[i].derivativeFallback; });
    logger().addLogEntry(prefix + "feature_switch", [this, i]() { return linkPairLogs_[i].featureSwitch; });
    logger().addLogEntry(prefix + "derivative_reason", [this, i]() { return linkPairLogs_[i].derivativeReason; });
    logger().addLogEntry(prefix + "distance_rate_mps", [this, i]() { return linkPairLogs_[i].distanceRate; });
    logger().addLogEntry(prefix + "human_approach_rate_mps2", [this, i]() { return linkPairLogs_[i].humanApproachRate; });
    logger().addLogEntry(prefix + "mobility_rate", [this, i]() { return linkPairLogs_[i].mobilityRate; });
    logger().addLogEntry(prefix + "pfl_rate_mps2", [this, i]() { return linkPairLogs_[i].pflRate; });
    logger().addLogEntry(prefix + "speed_limit_rate_mps2", [this, i]() { return linkPairLogs_[i].speedRate; });
    logger().addLogEntry(prefix + "barrier_mps", [this, i]() { return linkPairLogs_[i].barrier; });
    logger().addLogEntry(prefix + "original_envelope_violated", [this, i]() { return linkPairLogs_[i].barrier < 0; });
    logger().addLogEntry(prefix + "relaxation_physical", [this, i]() { return linkPairLogs_[i].relaxation; });
    logger().addLogEntry(prefix + "stage1_relaxation_physical", [this, i]() { return linkPairLogs_[i].stage1Relaxation; });
    logger().addLogEntry(prefix + "retreat_relaxation_mps", [this, i]() { return linkPairLogs_[i].retreatRelaxation; });
    logger().addLogEntry(prefix + "relaxation_normalized", [this, i]() {
      return relaxationEnabled_ ? linkPairLogs_[i].relaxation / relaxationScales_[i] : 0.0;
    });
    logger().addLogEntry(prefix + "relaxation_weight", [this, i]() {
      return relaxationEnabled_ ? relaxationWeights_[i] : 1.0;
    });
    logger().addLogEntry(prefix + "relaxation_scale", [this, i]() {
      return relaxationEnabled_ ? relaxationScales_[i] : 1.0;
    });
    logger().addLogEntry(prefix + "original_qp_residual", [this, i]() { return linkPairLogs_[i].originalResidual; });
    logger().addLogEntry(prefix + "relaxed_qp_residual", [this, i]() { return linkPairLogs_[i].relaxedResidual; });
    logger().addLogEntry(prefix + "beta_mps2", [this, i]() { return linkPairLogs_[i].beta; });
    logger().addLogEntry(prefix + "qp_bound_mps2", [this, i]() { return linkPairLogs_[i].qpBound; });
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
  if(accelerationBounds_)
    mc_solver::TasksQPSolver::from_solver(solver()).removeConstraint(accelerationBounds_.get());
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
  speedPlot_.limit = pair.robotBudget;
  speedPlot_.sampleTime = safetyTime_;
  if(solverSuccess_ && pair.active)
  {
    speedPlot_.qpNext = temporalBarrier_
        ? pair.robotSpeed + solver().dt() * (pair.beta + pair.qpBound + pair.originalResidual)
        : pair.predictedRobotSpeed;
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
  recoveryWarningTime_ = -std::numeric_limits<double>::infinity();
  safetyFallback_ = false;
  fallbackReason_.clear();
  recoveryMode_ = false;
  relaxedPairs_ = 0;
  retreatRelaxedPairs_ = 0;
  relaxationObjective_ = stage1Seconds_ = stage2Seconds_ = 0;
  stage2RetryCap_ = 0;
  fullQPExtraSlack_ = 0;
  fullQPExceededLegacyCap_ = false;
  stage2RetryAttempts_ = 0;
  stage2ToleranceApplied_ = 0;
  stage1Status_ = relaxationEnabled_ ? "not run" : "disabled";
  stage2Status_ = "not run";
  for(auto & state : derivatives_) state = ssm_pfl::TemporalDerivative{};
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
  monitorOnly_ = safetyEnabled_ && !movingHuman_ && phase_ == Phase::Hold && !constrainWhileHolding_;
  if(safetyEnabled_ && !updateSafetyConstraint(!monitorOnly_) && holdOnConstraintUnavailable_)
    stopSweepAtAvailableJoint1();
  relaxedPairs_ = 0;
  retreatRelaxedPairs_ = 0;
  relaxationObjective_ = stage1Seconds_ = stage2Seconds_ = 0;
  stage2RetryCap_ = 0;
  fullQPExtraSlack_ = 0;
  fullQPExceededLegacyCap_ = false;
  stage2RetryAttempts_ = 0;
  stage2ToleranceApplied_ = 0;
  stage1Status_ = relaxationEnabled_ ? "inactive" : "disabled";
  if(relaxationEnabled_ && !safetyFallback_ && activePairs_ > 0)
  {
    const auto & mb = robot().mb();
    Eigen::VectorXd lower = rbd::dofToVector(mb, robot().al());
    Eigen::VectorXd upper = rbd::dofToVector(mb, robot().au());
    if(lower.size() == mb.nrDof() && upper.size() == mb.nrDof())
      for(Eigen::Index j = 0; j < lower.size(); ++j)
      {
        lower[j] = std::max(lower[j], -relaxationMaxAcceleration_);
        upper[j] = std::min(upper[j], relaxationMaxAcceleration_);
      }
    bool boundsValid = lower.size() == mb.nrDof() && upper.size() == mb.nrDof()
        && lower.allFinite() && upper.allFinite();
    if(boundsValid)
      for(Eigen::Index j = 0; j < lower.size(); ++j) boundsValid = boundsValid && lower[j] <= upper[j];
    if(!boundsValid)
    {
      stage1Status_ = "invalid or infeasible hard acceleration bounds";
      solverSuccess_ = false;
      safetyFallback(stage1Status_);
      stopSweepAtAvailableJoint1();
      return false;
    }
    accelerationBounds_->set(lower, upper);
    std::vector<bool> active(linkPairLogs_.size(), false);
    for(size_t i = 0; i < active.size(); ++i) active[i] = linkPairLogs_[i].active;
    const auto stage1Start = std::chrono::steady_clock::now();
    // One slack per pair expands only the approach-side point-speed row.
    const bool stage1OK = relaxation_->solve(safetyConstraint_->rows(), safetyConstraint_->bounds(),
                                             active, relaxationWeights_, relaxationScales_, lower, upper,
                                             accelerationRegularization_);
    stage1Seconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - stage1Start).count();
    stage1Status_ = relaxation_->status();
    if(!stage1OK)
    {
      solverSuccess_ = false;
      safetyFallback("minimum-violation stage failed: " + stage1Status_);
      stopSweepAtAvailableJoint1();
      return false;
    }
    relaxationObjective_ = relaxation_->objective();
    stage2ToleranceApplied_ = stage1Status_ == "original rows feasible" ? 0 : relaxationStage2Tolerance_;
    for(size_t i = 0; i < active.size(); ++i)
    {
      if(!active[i]) continue;
      const double delta = relaxation_->delta()[i];
      linkPairLogs_[i].relaxation = delta;
      linkPairLogs_[i].stage1Relaxation = delta;
      safetyConstraint_->relax(static_cast<int>(i), delta + stage2ToleranceApplied_);
      if(movingHuman_)
      {
        linkPairLogs_[i].retreatRelaxation = 0;
      }
      if(delta > relaxationActivationTolerance_) ++relaxedPairs_;
    }
  }
  else if(accelerationBounds_)
  {
    const int dofs = robot().mb().nrDof();
    accelerationBounds_->set(Eigen::VectorXd::Constant(dofs, -1e12),
                             Eigen::VectorXd::Constant(dofs, 1e12));
  }
  const auto stage2Start = std::chrono::steady_clock::now();
  // A successful solver().run integrates state. Raw Tasks probes below do not.
  updateContacts();
  solverSuccess_ = solver().run(mc_solver::FeedbackType::None);
  stage2Seconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - stage2Start).count();
  stage2Status_ = solverSuccess_ ? "success" : "QP failure";
  if(!solverSuccess_ && fullQPRecovery_ && relaxationEnabled_ && activePairs_ > 0)
  {
    // The first solve included every hard constraint but did not integrate on
    // failure. A raw Tasks solve probes the same constraints without changing
    // the controller state or advancing its posture task.
    auto & tasksSolver = mc_solver::TasksQPSolver::from_solver(solver());
    auto probe = [this, &tasksSolver]() {
      return tasksSolver.solver().solveNoMbcUpdate(robots().mbs(), robots().mbcs());
    };
    safetyConstraint_->suspend(true);
    ++stage2RetryAttempts_;
    const bool hardOnlyOK = probe();
    safetyConstraint_->suspend(false);
    if(!hardOnlyOK)
    {
      stage2Status_ = "hard constraints infeasible without SSM+PFL";
    }
    else
    {
      const Eigen::VectorXd hardCandidate = tasksSolver.solver().alphaDVec(0);
      if(!hardCandidate.allFinite())
      {
        stage2Status_ = "nonfinite hard-only QP solution";
      }
      else
      {
        // This candidate satisfies all non-safety rows. Its largest safety
        // residual is therefore a known feasible upper bracket for one shared
        // allowance beyond the already-installed per-pair Stage 1 slack.
        double feasible = 0;
        for(size_t i = 0; i < linkPairLogs_.size(); ++i)
          if(linkPairLogs_[i].active)
            feasible = std::max(feasible,
                                safetyConstraint_->residual(static_cast<int>(i), hardCandidate));
        if(!std::isfinite(feasible))
        {
          stage2Status_ = "nonfinite full-QP safety residual";
          safetyFallback(stage2Status_);
          stopSweepAtAvailableJoint1();
          return false;
        }
        feasible = std::max(0.0, feasible) + 1e-6;
        for(size_t i = 0; i < linkPairLogs_.size(); ++i)
          if(linkPairLogs_[i].active)
            safetyConstraint_->setAllowance(static_cast<int>(i), linkPairLogs_[i].relaxation
                                              + stage2ToleranceApplied_ + feasible);
        ++stage2RetryAttempts_;
        if(!probe())
        {
          stage2Status_ = "full-QP upper bracket rejected by solver";
        }
        else
        {
          double infeasible = 0;
          const auto searchStart = std::chrono::steady_clock::now();
          for(int attempt = 0; attempt < 2; ++attempt)
          {
            if(std::chrono::duration<double>(std::chrono::steady_clock::now() - searchStart).count() >= 0.0005)
              break;
            const double candidate = 0.5 * (infeasible + feasible);
            for(size_t i = 0; i < linkPairLogs_.size(); ++i)
              if(linkPairLogs_[i].active)
                safetyConstraint_->setAllowance(static_cast<int>(i), linkPairLogs_[i].relaxation
                                                  + stage2ToleranceApplied_ + candidate);
            ++stage2RetryAttempts_;
            if(probe()) feasible = candidate;
            else infeasible = candidate;
          }
          fullQPExtraSlack_ = feasible;
          fullQPExceededLegacyCap_ = false;
          relaxedPairs_ = 0;
          relaxationObjective_ = 0;
          for(size_t i = 0; i < linkPairLogs_.size(); ++i)
            if(linkPairLogs_[i].active)
            {
              linkPairLogs_[i].relaxation += feasible;
              safetyConstraint_->setAllowance(static_cast<int>(i), linkPairLogs_[i].relaxation
                                                + stage2ToleranceApplied_);
              fullQPExceededLegacyCap_ = fullQPExceededLegacyCap_ ||
                  (movingHuman_ && linkPairLogs_[i].relaxation > relaxationMaxStage2Slack_);
              if(linkPairLogs_[i].relaxation > relaxationActivationTolerance_) ++relaxedPairs_;
              relaxationObjective_ += 0.5 * relaxationWeights_[i]
                  * std::pow(linkPairLogs_[i].relaxation / relaxationScales_[i], 2);
            }
          solverSuccess_ = solver().run(mc_solver::FeedbackType::None);
          stage2Status_ = solverSuccess_ ? "success after full-QP feasibility recovery"
                                         : "full-QP final solve failed after feasible probe";
        }
      }
    }
    stage2Seconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - stage2Start).count();
  }
  else if(!solverSuccess_ && movingHuman_ && relaxationEnabled_ && activePairs_ > 0)
  {
    // Stage 1 sees model acceleration bounds but not all mc_rtc dynamics and
    // collision rows. Retry the full QP with bounded approach-speed slack.
    // Each successful solve integrates the robot state, so stop at the first
    // success. Acceleration and other mc_rtc rows remain hard.
    const size_t count = linkPairLogs_.size();
    std::vector<double> allowance(count, 0.0);
    double initial = 0.01;
    for(size_t i = 0; i < count; ++i)
    {
      if(!linkPairLogs_[i].active) continue;
      allowance[i] = linkPairLogs_[i].relaxation + stage2ToleranceApplied_;
      initial = std::max(initial, allowance[i]);
      initial = std::max(initial, linkPairLogs_[i].robotSpeed
          + dt * linkPairLogs_[i].beta - linkPairLogs_[i].robotBudget + 0.005);
    }
    double cap = std::min(relaxationMaxStage2Slack_, initial);
    for(int attempt = 0; attempt < 6 && !solverSuccess_; ++attempt)
    {
      ++stage2RetryAttempts_;
      stage2RetryCap_ = cap;
      for(size_t i = 0; i < count; ++i)
      {
        if(!linkPairLogs_[i].active) continue;
        const double desired = std::max(allowance[i], cap);
        safetyConstraint_->relax(static_cast<int>(i), desired - allowance[i]);
        allowance[i] = desired;
      }
      const bool provisional = solver().run(mc_solver::FeedbackType::None);
      if(provisional)
      {
        solverSuccess_ = true;
        relaxedPairs_ = 0;
        retreatRelaxedPairs_ = 0;
        relaxationObjective_ = 0;
        for(size_t i = 0; i < count; ++i)
        {
          if(!linkPairLogs_[i].active) continue;
          const double slack = std::max(0.0, allowance[i] - stage2ToleranceApplied_);
          linkPairLogs_[i].relaxation = slack;
          linkPairLogs_[i].retreatRelaxation = 0;
          if(slack > relaxationActivationTolerance_)
          {
            ++relaxedPairs_;
          }
          relaxationObjective_ += 0.5 * relaxationWeights_[i]
              * std::pow(slack / relaxationScales_[i], 2);
        }
        stage2Status_ = "success after bounded shared-speed recovery";
      }
      if(cap >= relaxationMaxStage2Slack_) break;
      cap = std::min(relaxationMaxStage2Slack_, std::max(2 * cap, cap + 0.01));
    }
    stage2Seconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - stage2Start).count();
  }
  if(!solverSuccess_)
  {
    if(relaxationEnabled_)
    {
      int largestSlackPair = -1;
      for(size_t i = 0; i < linkPairLogs_.size(); ++i)
        if(linkPairLogs_[i].active && (largestSlackPair < 0
           || linkPairLogs_[i].relaxation > linkPairLogs_[largestSlackPair].relaxation))
          largestSlackPair = static_cast<int>(i);
      mc_rtc::log::error("SSM+PFL relaxation stages: Stage 1 {}, Stage 2 {}, {} positive slacks; "
                          "largest slack {} {} at {}; {} full-QP probes, extra shared allowance {} {}",
                          stage1Status_, stage2Status_, relaxedPairs_,
                          largestSlackPair >= 0 ? linkPairLogs_[largestSlackPair].relaxation : 0,
                          temporalBarrier_ ? "m/s²" : "m/s",
                          largestSlackPair >= 0 ? pairLabel(largestSlackPair) : "none",
                          stage2RetryAttempts_, fullQPRecovery_ ? fullQPExtraSlack_ : stage2RetryCap_,
                          temporalBarrier_ ? "m/s²" : "m/s");
    }
    if(relaxationEnabled_ && activePairs_ > 0 && stage1Status_ == "success")
    {
      const auto & candidate = relaxation_->acceleration();
      double largestRaw = -std::numeric_limits<double>::infinity();
      double largestSolver = -std::numeric_limits<double>::infinity();
      int largestPair = -1;
      for(size_t i = 0; i < linkPairLogs_.size(); ++i)
      {
        if(!linkPairLogs_[i].active) continue;
        const double raw = safetyConstraint_->residual(static_cast<int>(i), candidate);
        const double scaled = safetyConstraint_->A().row(static_cast<int>(i)).dot(candidate)
            - safetyConstraint_->bInEq()[static_cast<int>(i)];
        if(raw > largestRaw) { largestRaw = raw; largestPair = static_cast<int>(i); }
        largestSolver = std::max(largestSolver, scaled);
      }
      mc_rtc::log::error("Relaxed Stage 2 failure after Stage 1 success: {} positive slacks, objective {}, "
                          "Stage 1 candidate maximum raw row residual {} at {}, maximum scaled row residual {}, "
                          "Stage 2 tolerance {}",
                          relaxedPairs_, relaxationObjective_, largestRaw,
                          largestPair >= 0 ? pairLabel(largestPair) : "none", largestSolver,
                          stage2ToleranceApplied_);
    }
    if(safetyEnabled_ && activePairs_ > 0 && limitingPair_ >= 0)
    {
      const auto & pair = linkPairLogs_[limitingPair_];
      mc_rtc::log::error("Kinova sweep QP failed with {} SSM+PFL rows; smallest speed headroom at {}: clearance {} m, "
                         "closing {} m/s, limit {} m/s, sample age {} s",
                         activePairs_, pairLabel(limitingPair_), pair.distance, pair.closingSpeed,
                         pair.combinedLimit, sampleAge_);
      if(movingHuman_)
        mc_rtc::log::error("Pointwise robot speed QP infeasible with {} and other hard constraints; "
                           "budget {} m/s at {}",
                           fullQPRecovery_ ? "full-QP feasibility recovery" : relaxationEnabled_
                             ? "configured shared-slack cap" : "hard speed bounds",
                           pair.robotBudget, pairLabel(limitingPair_));
      else if(temporalBarrier_)
      {
        int smallestBoundPair = -1;
        for(size_t i = 0; i < linkPairLogs_.size(); ++i)
          if(linkPairLogs_[i].active && (smallestBoundPair < 0
             || linkPairLogs_[i].qpBound < linkPairLogs_[smallestBoundPair].qpBound))
            smallestBoundPair = static_cast<int>(i);
        if(smallestBoundPair >= 0)
        {
          const auto & boundPair = linkPairLogs_[smallestBoundPair];
          mc_rtc::log::error("Smallest SSM+PFL acceleration row bound: {} at {}; barrier {} m/s, "
                              "limit rate {} m/s2, beta {} m/s2, derivative fallback {}",
                              boundPair.qpBound, pairLabel(smallestBoundPair), boundPair.barrier,
                              boundPair.speedRate, boundPair.beta, boundPair.derivativeFallback);
        }
      }
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
        if(movingHuman_)
          linkPairLogs_[i].predictedRobotSpeed = linkPairLogs_[i].robotBudget
              + safetyConstraint_->upperResidual(static_cast<int>(i), acceleration)
              + linkPairLogs_[i].relaxation + stage2ToleranceApplied_;
        else if(!temporalBarrier_)
          linkPairLogs_[i].predictedRobotSpeed = linkPairLogs_[i].combinedLimit
              + linkPairLogs_[i].humanSpeed + linkPairLogs_[i].predictedResidual;
        if(movingHuman_)
        {
          const double originalUpper = safetyConstraint_->upperResidual(static_cast<int>(i), acceleration)
              + linkPairLogs_[i].relaxation + stage2ToleranceApplied_;
          linkPairLogs_[i].originalResidual = originalUpper;
          linkPairLogs_[i].relaxedResidual = linkPairLogs_[i].predictedResidual;
        }
        else
        {
          linkPairLogs_[i].originalResidual = linkPairLogs_[i].predictedResidual
              + linkPairLogs_[i].relaxation + stage2ToleranceApplied_;
          linkPairLogs_[i].relaxedResidual = linkPairLogs_[i].originalResidual - linkPairLogs_[i].relaxation;
        }
        if(linkPairLogs_[i].inverseMass > minMobility_)
        {
          linkPairLogs_[i].binding = linkPairLogs_[i].predictedResidual >= (temporalBarrier_ ? -0.01 : -0.001);
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
    if(relaxationEnabled_)
    {
      double largest = 0;
      for(size_t i = 0; i < linkPairLogs_.size(); ++i)
        if(linkPairLogs_[i].active)
          largest = std::max(largest, std::max(linkPairLogs_[i].relaxation,
                                               linkPairLogs_[i].retreatRelaxation) / relaxationScales_[i]);
      const bool wasRecovery = recoveryMode_;
      if(recoveryMode_) recoveryMode_ = largest > relaxationRecoveryTolerance_;
      else recoveryMode_ = largest > relaxationActivationTolerance_;
      if(movingHuman_ && recoveryMode_ && !wasRecovery
         && safetyTime_ - recoveryWarningTime_ >= 5.0)
      {
        mc_rtc::log::warning("Moving-human pointwise PFL recovery active: approach speed bound relaxed; "
                             "largest normalized slack {}", largest);
        recoveryWarningTime_ = safetyTime_;
      }
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
  recoveryMode_ = false;
  const bool transientMeasurement = reason == "distance snapshot unavailable or worker error"
      || reason == "distance sample is stale or timestamp invalid"
      || reason.rfind("human material velocity unavailable at ", 0) == 0;
  if(!transientMeasurement) resumeAfterMeasurement_ = false;
  if(holdOnConstraintUnavailable_ && transientMeasurement
     && phase_ != Phase::Hold)
  {
    resumeAfterMeasurement_ = true;
    resumePhase_ = phase_;
    resumeGoal_ = goalPosition_;
  }
  if(!safetyFallback_ || fallbackReason_ != reason)
  {
    if(reason.rfind("human material velocity unavailable at ", 0) == 0)
      mc_rtc::log::warning("SSM+PFL waiting for next valid human velocity sample: {}", reason);
    else if(holdOnConstraintUnavailable_) mc_rtc::log::error("SSM+PFL fallback: {}", reason);
    else if(reason == "QP solve failed, requested posture hold")
      mc_rtc::log::error("SSM+PFL QP failure: {}", reason);
    else mc_rtc::log::warning("SSM+PFL rows skipped for this sample: {}", reason);
  }
  safetyFallback_ = true;
  fallbackReason_ = reason;
  if(autoStartPending_) startRequested_ = true;
  if(safetyConstraint_) safetyConstraint_->clear();
  activePairs_ = 0;
  bindingPairs_ = 0;
  limitingPair_ = -1;
  for(auto & pair : linkPairLogs_) { pair.active = false; pair.binding = false; }
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
    log.sequence = snapshot->sequence;
    log.sampleTime = snapshot->sample_simulation_time;
    log.sampleAge = age;
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
    try { log.robotBudget = ssm_pfl::robotSpeedBudget(limit.combinedSpeed, humanApproach); }
    catch(const std::exception &) { return fail("invalid pointwise speed budget at " + pair); }
    log.barrier = limit.combinedSpeed - log.closingSpeed;
    log.distanceRate = humanApproach - robotApproach;
    log.actualResidual = actualRobotApproach - humanApproach - limit.combinedSpeed;
    if(!std::isfinite(log.actualResidual)) return fail("invalid simulated point speed at " + pair);
    log.actualViolation = log.actualResidual > maxActualResidual_;
    if(log.actualViolation) ++actualViolationPairs_;
    // A selected point can have zero mobility along this direction. A negative
    // speed allowance would make its zero-acceleration row impossible.
    // Keep the measured violation visible, but do not add an unachievable row.
    if(limit.inverseMass <= minMobility_ && !movingHuman_)
    {
      if(temporalBarrier_) derivatives_[i] = ssm_pfl::TemporalDerivative{};
      log.uncontrollable = true;
      ++uncontrollablePairs_;
      continue;
    }
    if(!applyConstraint && !temporalBarrier_)
    {
      log.monitorOnly = true;
      ++monitorOnlyPairs_;
      continue;
    }
    double bound;
    if(movingHuman_)
    {
      const double drift = (n.transpose() * fullDot.bottomRows(3) * qdot).value();
      try
      {
        const double upperBound = ssm_pfl::pointSpeedUpperBound(log.robotBudget, robotApproach,
                                                                drift, solver().dt());
        log.beta = drift;
        bound = upperBound;
        if(applyConstraint)
          safetyConstraint_->set(static_cast<int>(i), solver().dt() * directional,
                                 upperBound);
      }
      catch(const std::exception &) { return fail("invalid pointwise speed row at " + pair); }
    }
    else if(temporalBarrier_)
    {
      auto & state = derivatives_[i];
      state.observe(snapshot->sequence, snapshot->sample_simulation_time, directional, qdot,
                    limit.inverseMass, humanApproach, n, local, sample->robot_geometry, sample->intersecting,
                    maxDerivativeGap_, maxWitnessStep_, maxNormalStep_, minMobility_);
      log.derivativeValid = state.valid && ssm_pfl::mobilityExtrapolationValid(
          limit.inverseMass, state.mobilityRate, minMobility_, solver().dt());
      log.featureSwitch = state.switched;
      if(!log.derivativeValid)
      {
        log.derivativeReason = state.switched ? "geometry/witness/normal switch"
            : !state.valid ? "startup, sample gap, overlap or invalid derivative"
            : "mobility extrapolation reaches zero";
        log.derivativeFallback = true;
        const double drift = (n.transpose() * fullDot.bottomRows(3) * qdot).value();
        try
        {
          log.beta = drift;
          log.speedRate = ssm_pfl::combinedSpeedRate(sample->distance, parameters.margin,
                                                     log.distanceRate, 0.0, stopTime_);
          bound = ssm_pfl::barrierBound(log.speedRate, drift, barrierGamma_,
                                       limit.combinedSpeed, robotApproach, 0.0, 0.0);
        }
        catch(const std::exception &) { return fail("invalid derivative fallback bound at " + pair); }
      }
      else try
      {
        log.beta = state.beta;
        log.mobilityRate = state.mobilityRate;
        log.humanApproachRate = state.humanApproachRate;
        log.pflRate = ssm_pfl::pflSpeedRate(parameters.force, parameters.stiffness,
                                           limit.inverseMass, state.mobilityRate, minMobility_, solver().dt());
        log.speedRate = ssm_pfl::combinedSpeedRate(sample->distance, parameters.margin,
                                                   log.distanceRate, log.pflRate, stopTime_);
        bound = ssm_pfl::barrierBound(log.speedRate, log.beta, barrierGamma_,
                                     limit.combinedSpeed, robotApproach, humanApproach,
                                     log.humanApproachRate);
      }
      catch(const std::exception &) { return fail("invalid temporal barrier at " + pair); }
      if(applyConstraint) safetyConstraint_->set(static_cast<int>(i), directional, bound);
    }
    else
    {
      const double drift = (n.transpose() * fullDot.bottomRows(3) * qdot).value();
      try { bound = ssm_pfl::accelerationBound(limit.combinedSpeed, robotApproach,
                                              humanApproach, drift, solver().dt()); }
      catch(const std::exception &) { return fail("invalid QP bound at " + pair); }
      if(applyConstraint) safetyConstraint_->set(static_cast<int>(i), solver().dt() * directional, bound);
    }
    log.qpBound = bound;
    if(relaxationEnabled_)
    {
      log.relaxationWeight = relaxationWeights_[i];
      log.relaxationScale = relaxationScales_[i];
    }
    if(!applyConstraint)
    {
      log.monitorOnly = true;
      ++monitorOnlyPairs_;
      continue;
    }
    log.active = true;
    ++activePairs_;
    const double margin = movingHuman_ ? log.robotBudget - robotApproach
                                       : limit.combinedSpeed - log.closingSpeed;
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
    if(resumePhase_ == Phase::ToWaypoint || resumePhase_ == Phase::ToHome)
      beginSegment(resumePhase_, resumeGoal_);
    else
    {
      phase_ = resumePhase_;
      elapsed_ = 0.0;
    }
    resumeAfterMeasurement_ = false;
  }
  autoStartPending_ = false;
  safetyComputationSeconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  return true;
}

CONTROLLER_CONSTRUCTOR("KinovaSweepController", KinovaSweepController)
