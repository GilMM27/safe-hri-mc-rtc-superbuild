#include "KinovaPositionSSMPFLController.h"
#include "SSMPFLMath.h"

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/plot.h>
#include <mc_tvm/Robot.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBodyConfig.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

namespace
{
const std::array<const char *, 14> regions = {{"head", "torso", "left_upper_arm", "left_forearm", "left_hand",
                                               "right_upper_arm", "right_forearm", "right_hand", "left_thigh",
                                               "left_lower_leg", "left_foot", "right_thigh", "right_lower_leg",
                                               "right_foot"}};
const std::array<const char *, 7> links = {{"joint_1", "joint_2", "joint_3", "joint_4",
                                            "joint_5", "joint_6", "joint_7"}};

// This controller uses the FSM base for its solver setup but runs no FSM states.
mc_rtc::Configuration managedConfig(const mc_rtc::Configuration & config)
{
  mc_rtc::Configuration managed = config;
  if(managed.has("Managed") && !managed("Managed", false))
    mc_rtc::log::error_and_throw("KinovaPositionSSMPFLController requires Managed: true");
  if(!managed.has("Managed")) managed.add("Managed", true);
  return managed;
}
} // namespace

KinovaPositionSSMPFLController::KinovaPositionSSMPFLController(
    mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, managedConfig(config), Backend::TVM)
{
  // Match the sweep controller's TVM dynamics and collision setup. The FSM
  // idle posture would compete with manual XYZ motion, so remove its task.
  loadRobot(mc_rbdyn::RobotLoader::get_robot_module("env/ground"), "ground");
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.2, 100.0}, 0.95, true));
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  auto idlePosture = getPostureTask(robot().name());
  if(!idlePosture) mc_rtc::log::error_and_throw("Kinova position controller requires an actuated robot");
  solver().removeTask(idlePosture);
  solver().setContacts({});

  double positionWeight = 5.0;
  double positionStiffness = 2.0;
  if(config.has("PositionTask"))
  {
    const auto taskConfig = config("PositionTask");
    taskConfig("Weight", positionWeight);
    taskConfig("Stiffness", positionStiffness);
  }
  if(!std::isfinite(positionWeight) || positionWeight <= 0)
    mc_rtc::log::error_and_throw("PositionTask.Weight must be a positive finite value");
  if(!std::isfinite(positionStiffness) || positionStiffness <= 0)
    mc_rtc::log::error_and_throw("PositionTask.Stiffness must be a positive finite value");
  // PositionTask supplies its own draggable GUI target. Its constructor uses
  // critical damping for the configured stiffness.
  positionTask_ = std::make_shared<mc_tasks::PositionTask>("tool_frame", robots(), robot().robotIndex(),
                                                            positionStiffness, positionWeight);
  positionTask_->name("KinovaToolPosition");
  solver().addTask(positionTask_);

  if(!config.has("SSMPFL")) mc_rtc::log::error_and_throw("Missing SSMPFL configuration");
  const auto safety = config("SSMPFL");
  if(!safety.has("Weight") || !safety.has("Regions"))
    mc_rtc::log::error_and_throw("SSMPFL.Weight and SSMPFL.Regions are required");
  safety("Weight", safetyWeight_);
  std::string mode = "frozen";
  safety("HumanMode", mode);
  if(mode != "frozen" && mode != "moving")
    mc_rtc::log::error_and_throw("SSMPFL.HumanMode must be frozen or moving");
  movingHuman_ = mode == "moving";
  safety("StoppingTimeSeconds", stopTime_);
  safety("MaxSampleAgeSeconds", maxSampleAge_);
  safety("MaxModelWitnessErrorM", maxModelWitnessError_);
  safety("MinDirectionalInverseMass", minMobility_);
  safety("MinMassEigenvalue", minMassEigenvalue_);
  if(!std::isfinite(safetyWeight_) || safetyWeight_ <= 0 || !std::isfinite(stopTime_) || stopTime_ <= 0
     || !std::isfinite(maxSampleAge_) || maxSampleAge_ <= 0
     || !std::isfinite(maxModelWitnessError_) || maxModelWitnessError_ <= 0
     || !std::isfinite(minMobility_) || minMobility_ < 0
     || !std::isfinite(minMassEigenvalue_) || minMassEigenvalue_ <= 0)
    mc_rtc::log::error_and_throw("Invalid SSMPFL weight or tolerance");
  const auto configuredRegions = safety("Regions");
  for(const auto * name : regions)
  {
    if(!configuredRegions.has(name)) mc_rtc::log::error_and_throw("Missing SSMPFL region {}", name);
    const auto entry = configuredRegions(name);
    if(!entry.has("ForceMaxN") || !entry.has("StiffnessNPerM"))
      mc_rtc::log::error_and_throw("SSMPFL region {} needs ForceMaxN and StiffnessNPerM", name);
    RegionParameters parameters;
    entry("ForceMaxN", parameters.force);
    entry("StiffnessNPerM", parameters.stiffness);
    entry("DistanceMarginM", parameters.margin);
    if(!std::isfinite(parameters.force) || parameters.force < 0
       || !std::isfinite(parameters.stiffness) || parameters.stiffness <= 0
       || !std::isfinite(parameters.margin) || parameters.margin < 0)
      mc_rtc::log::error_and_throw("Invalid SSMPFL parameters for {}", name);
    regionParameters_[name] = parameters;
  }

  // Allocate one TVM row per region/link pair. Weight makes each row soft:
  // excess approach speed is penalized in the QP rather than forbidden.
  safetyConstraint_.reset(new SSMPFLConstraint(robot().tvmRobot().alphaD(), robot().mb().nrDof(),
                                                static_cast<int>(regions.size() * links.size()),
                                                true, safetyWeight_));
  safetyConstraint_->addToSolver(mc_solver::TVMQPSolver::from_solver(solver()));
  inertia_.reset(new rbd::ForwardDynamics(robot().mb()));

  gui()->addElement({"SSM+PFL"},
                    mc_rtc::gui::Label("State", [this]() {
                      return fallback_ ? std::string("Holding: ") + fallbackReason_ : std::string("QP running");
                    }),
                    mc_rtc::gui::Label("Human mode", [this]() {
                      return movingHuman_ ? std::string("moving pointwise") : std::string("frozen pointwise");
                    }),
                    mc_rtc::gui::Label("Active pairs", [this]() { return std::to_string(activePairs_); }),
                    mc_rtc::gui::Label("Limiting pair", [this]() { return limitingPair_; }),
                    mc_rtc::gui::Label("Closest pair in speed plot", [this]() { return speedPlot_.pair; }),
                    mc_rtc::gui::Label("Closest clearance (m)", [this]() {
                      return speedPlot_.valid ? std::to_string(speedPlot_.distance) : std::string("unavailable");
                    }),
                    mc_rtc::gui::Label("Closest pair soft slack (m/s)", [this]() {
                      return speedPlot_.valid ? std::to_string(pairSoftSlack_[speedPlot_.pairIndex])
                                              : std::string("unavailable");
                    }),
                    mc_rtc::gui::Label("Limiting robot speed allowance (m/s)", [this]() {
                      return limitingPair_ == "none" ? std::string("unavailable")
                                                      : std::to_string(limitingSpeed_);
                    }),
                    mc_rtc::gui::Label("Maximum soft slack (m/s)", [this]() {
                      return std::to_string(maxSoftSlack_);
                    }),
                    mc_rtc::gui::Label("Sample age (s)", [this]() { return std::to_string(sampleAge_); }));
  addSpeedPlot();
  logger().addLogEntry("SSMPFL_active_pairs", [this]() { return activePairs_; });
  logger().addLogEntry("SSMPFL_fallback", [this]() { return fallback_; });
  logger().addLogEntry("SSMPFL_solver_success", [this]() { return solverSuccess_; });
  logger().addLogEntry("SSMPFL_max_soft_slack_mps", [this]() { return maxSoftSlack_; });
  logger().addLogEntry("SSMPFL_sample_age_seconds", [this]() { return sampleAge_; });
  logger().addLogEntry("SSMPFL_limiting_pair", [this]() { return limitingPair_; });
  logger().addLogEntry("SSMPFL_fallback_reason", [this]() { return fallbackReason_; });
  logger().addLogEntry("SSMPFL_closest_pair", [this]() { return speedPlot_.pair; });
  logger().addLogEntry("SSMPFL_closest_distance_m", [this]() {
    return speedPlot_.valid ? speedPlot_.distance : std::numeric_limits<double>::quiet_NaN();
  });
  for(size_t i = 0; i < pairSpeedBudget_.size(); ++i)
  {
    const std::string prefix = std::string("SSMPFLPair_") + regions[i / links.size()] + "_"
                             + links[i % links.size()] + "_";
    logger().addLogEntry(prefix + "robot_speed_budget_mps", [this, i]() { return pairSpeedBudget_[i]; });
    logger().addLogEntry(prefix + "soft_slack_mps", [this, i]() { return pairSoftSlack_[i]; });
  }
}

KinovaPositionSSMPFLController::~KinovaPositionSSMPFLController()
{
  if(safetyConstraint_)
    safetyConstraint_->removeFromSolver(mc_solver::TVMQPSolver::from_solver(solver()));
}

void KinovaPositionSSMPFLController::addSpeedPlot()
{
  // A single plot follows the geometrically closest pair as snapshots change.
  const auto plotValue = [](bool valid, double value) {
    return valid ? value : std::numeric_limits<double>::quiet_NaN();
  };
  gui()->addPlot("SSM+PFL closest robot approach speed (m/s)",
                 mc_rtc::gui::plot::X("simulation time (s)", [this]() { return time_; }),
                 mc_rtc::gui::plot::Y("Current model robot approach", [this, plotValue]() {
                   return plotValue(speedPlot_.valid, speedPlot_.current);
                 }, mc_rtc::gui::Color::Blue, mc_rtc::gui::plot::Style::Solid),
                 mc_rtc::gui::plot::Y("QP next robot approach", [this, plotValue]() {
                   return plotValue(speedPlot_.valid, speedPlot_.predicted);
                 }, mc_rtc::gui::Color::Green, mc_rtc::gui::plot::Style::Solid),
                 mc_rtc::gui::plot::Y("Sampled MuJoCo robot approach (delayed)", [this, plotValue]() {
                   return plotValue(speedPlot_.valid, speedPlot_.sampled);
                 }, mc_rtc::gui::Color::Magenta, mc_rtc::gui::plot::Style::Solid),
                 mc_rtc::gui::plot::Y("Speed limit", [this, plotValue]() {
                   return plotValue(speedPlot_.valid, speedPlot_.limit);
                 }, mc_rtc::gui::Color::Red, mc_rtc::gui::plot::Style::Solid));
}

void KinovaPositionSSMPFLController::reset(const mc_control::ControllerResetData & data)
{
  mc_control::fsm::Controller::reset(data);
  // Start without a commanded jump, even after a MuJoCo reset.
  positionTask_->reset();
  safetyConstraint_->clear();
  time_ = 0;
  sampleAge_ = 0;
  fallback_ = false;
  solverSuccess_ = true;
  activePairs_ = 0;
  maxSoftSlack_ = 0;
  limitingSpeed_ = 0;
  limitingPair_ = "none";
  fallbackReason_.clear();
  pairSpeedBudget_.fill(std::numeric_limits<double>::quiet_NaN());
  pairSoftSlack_.fill(0);
  speedPlot_ = SpeedPlotSample{};
}

const mc_mujoco::SmplhDistanceSnapshot * KinovaPositionSSMPFLController::distanceSnapshot() const
{
  static const std::string key = "SMPLH::DistanceSnapshot";
  if(!datastore().has(key)) return nullptr;
  const auto & snapshot = datastore().get<mc_mujoco::SmplhDistanceSnapshot>(key);
  return snapshot.ready && snapshot.error.empty() ? &snapshot : nullptr;
}

void KinovaPositionSSMPFLController::holdTarget(const std::string & reason)
{
  if(!fallback_ || fallbackReason_ != reason) mc_rtc::log::warning("SSM+PFL holding tool: {}", reason);
  fallback_ = true;
  fallbackReason_ = reason;
  activePairs_ = 0;
  limitingPair_ = "none";
  limitingSpeed_ = 0;
  maxSoftSlack_ = 0;
  pairSpeedBudget_.fill(std::numeric_limits<double>::quiet_NaN());
  pairSoftSlack_.fill(0);
  speedPlot_ = SpeedPlotSample{};
  safetyConstraint_->clear();
  // Discard the previous GUI target. A fresh target is required after the
  // measurement recovers, so motion cannot resume unexpectedly.
  positionTask_->position(robot().frame("tool_frame").position().translation());
}

bool KinovaPositionSSMPFLController::updateSafetyRows()
{
  // The distance worker is asynchronous. Never reuse rows from a prior tick
  // when a snapshot is absent, stale, incomplete, or inconsistent.
  safetyConstraint_->clear();
  activePairs_ = 0;
  limitingPair_ = "none";
  limitingSpeed_ = 0;
  pairSpeedBudget_.fill(std::numeric_limits<double>::quiet_NaN());
  pairSoftSlack_.fill(0);
  speedPlot_ = SpeedPlotSample{};
  const auto * snapshot = distanceSnapshot();
  if(!snapshot) { holdTarget("distance snapshot unavailable or worker error"); return false; }
  if(!movingHuman_ && !snapshot->playback_frozen)
  { holdTarget("HumanMode frozen requires held playback"); return false; }
  sampleAge_ = time_ - snapshot->sample_simulation_time;
  if(!std::isfinite(sampleAge_) || sampleAge_ < -solver().dt() || sampleAge_ > maxSampleAge_)
  { holdTarget("distance sample stale or timestamp invalid"); return false; }
  if(snapshot->regions.size() != regions.size() * links.size())
  { holdTarget("missing monitored region/link pairs"); return false; }
  std::map<std::pair<std::string, std::string>, const mc_mujoco::SmplhRegionDistance *> samples;
  for(const auto & sample : snapshot->regions)
    if(!samples.emplace(std::make_pair(sample.region, sample.link), &sample).second)
    { holdTarget("duplicate region/link pair"); return false; }

  const auto & mb = robot().mb();
  const auto & mbc = robot().mbc();
  const int dofs = mb.nrDof();
  const Eigen::VectorXd qdot = rbd::dofToVector(mb, robot().alpha());
  if(qdot.size() != dofs || !qdot.allFinite())
  { holdTarget("invalid model joint velocity"); return false; }
  inertia_->computeH(mb, mbc);
  const Eigen::MatrixXd & mass = inertia_->H();
  if(mass.rows() != dofs || mass.cols() != dofs || !mass.allFinite())
  { holdTarget("invalid inertia matrix"); return false; }
  const Eigen::LDLT<Eigen::MatrixXd> factor(mass);
  if(factor.info() != Eigen::Success || factor.vectorD().minCoeff() < minMassEigenvalue_)
  { holdTarget("inertia matrix is not positive definite"); return false; }

  double smallestMargin = std::numeric_limits<double>::infinity();
  double closestDistance = std::numeric_limits<double>::infinity();
  // Every region/link pair contributes a row; choosing only the nearest pair
  // would miss another pair with a tighter approach-speed allowance.
  for(size_t i = 0; i < regions.size() * links.size(); ++i)
  {
    const std::string region(regions[i / links.size()]);
    const std::string label = region + "/" + links[i % links.size()];
    const auto found = samples.find({region, links[i % links.size()]});
    if(found == samples.end()) { holdTarget("missing pair " + label); return false; }
    const auto & sample = *found->second;
    if(!std::isfinite(sample.distance) || sample.distance < 0
       || !sample.closest_points_valid || !sample.direction_valid || sample.robot_body.empty())
    { holdTarget("invalid distance or witness at " + label); return false; }
    if(movingHuman_ && !sample.human_velocity_valid)
    { holdTarget("human velocity unavailable at " + label); return false; }
    Eigen::Vector3d n(-sample.direction[0], -sample.direction[1], -sample.direction[2]);
    const double length = n.norm();
    if(!n.allFinite() || !std::isfinite(length) || length < 1e-9)
    { holdTarget("invalid normal at " + label); return false; }
    n /= length;
    const Eigen::Vector3d local(sample.robot_point_body_local[0], sample.robot_point_body_local[1],
                                sample.robot_point_body_local[2]);
    if(!local.allFinite()) { holdTarget("invalid local witness at " + label); return false; }
    try { robot().bodyIndexByName(sample.robot_body); }
    catch(const std::exception &) { holdTarget("unknown robot body at " + label); return false; }
    const auto & bodyPose = robot().bodyPosW(sample.robot_body);
    // The MuJoCo witness and mc_rtc kinematic model must refer to the same
    // physical point before its Jacobian can be used in a QP row.
    const Eigen::Vector3d modelWitness = bodyPose.rotation().transpose() * local + bodyPose.translation();
    const Eigen::Vector3d sampledWitness(sample.robot_point[0], sample.robot_point[1], sample.robot_point[2]);
    if(!sampledWitness.allFinite() || !std::isfinite((modelWitness - sampledWitness).norm())
       || (modelWitness - sampledWitness).norm() > maxModelWitnessError_)
    { holdTarget("robot model/witness disagreement at " + label); return false; }
    rbd::Jacobian jacobian(mb, sample.robot_body, local);
    Eigen::MatrixXd full(6, dofs), fullDot(6, dofs);
    jacobian.fullJacobian(mb, jacobian.jacobian(mb, mbc), full);
    jacobian.fullJacobian(mb, jacobian.jacobianDot(mb, mbc), fullDot);
    if(!full.allFinite() || !fullDot.allFinite())
    { holdTarget("invalid point Jacobian at " + label); return false; }
    // Project point velocity and acceleration onto the robot-to-human normal.
    const Eigen::RowVectorXd directional = n.transpose() * full.bottomRows(3);
    const auto & parameters = regionParameters_.at(region);
    ssm_pfl::Limit limit;
    try
    {
      limit = ssm_pfl::limit(factor, directional, parameters.force, parameters.stiffness,
                             sample.distance, parameters.margin, stopTime_, minMobility_);
    }
    catch(const std::exception &) { holdTarget("invalid speed limit at " + label); return false; }
    const Eigen::Vector3d humanVelocity = movingHuman_
        ? Eigen::Vector3d(sample.human_velocity[0], sample.human_velocity[1], sample.human_velocity[2])
        : Eigen::Vector3d::Zero();
    if(!humanVelocity.allFinite()) { holdTarget("invalid human velocity at " + label); return false; }
    const double humanApproach = n.dot(humanVelocity);
    const double robotApproach = directional.dot(qdot);
    const double drift = (n.transpose() * fullDot.bottomRows(3) * qdot).value();
    try
    {
      // v_next = J*qdot + dt*(J*qddot + Jdot*qdot). The soft row bounds
      // its approach component by the SSM+PFL robot-speed budget.
      const double budget = ssm_pfl::robotSpeedBudget(limit.combinedSpeed, humanApproach);
      const double bound = ssm_pfl::pointSpeedUpperBound(budget, robotApproach, drift, solver().dt());
      safetyConstraint_->set(static_cast<int>(i), solver().dt() * directional, bound);
      // The plot follows clearance; the separate limiting label follows the
      // smallest budget-minus-current-speed margin.
      if(sample.distance < closestDistance)
      {
        closestDistance = sample.distance;
        speedPlot_.pairIndex = i;
        speedPlot_.pair = label;
        speedPlot_.distance = sample.distance;
        speedPlot_.current = robotApproach;
        speedPlot_.limit = budget;
        speedPlot_.drift = drift;
        speedPlot_.directional = directional;
        speedPlot_.sampled = std::numeric_limits<double>::quiet_NaN();
        if(sample.robot_velocity_valid)
        {
          const Eigen::Vector3d sampledVelocity(sample.robot_velocity[0], sample.robot_velocity[1],
                                                sample.robot_velocity[2]);
          if(sampledVelocity.allFinite()) speedPlot_.sampled = n.dot(sampledVelocity);
        }
      }
      pairSpeedBudget_[i] = budget;
      ++activePairs_;
      const double margin = budget - robotApproach;
      if(margin < smallestMargin) { smallestMargin = margin; limitingPair_ = label; limitingSpeed_ = budget; }
    }
    catch(const std::exception &) { holdTarget("invalid pointwise row at " + label); return false; }
  }
  fallback_ = false;
  fallbackReason_.clear();
  speedPlot_.valid = true;
  return true;
}

bool KinovaPositionSSMPFLController::run()
{
  // Run the FSM lifecycle without solving, install this tick's distance rows,
  // then solve with the current GUI position target.
  mc_control::fsm::Controller::run(mc_solver::FeedbackType::SkipQP);
  time_ += solver().dt();
  updateSafetyRows();
  updateContacts();
  solverSuccess_ = solver().run(mc_solver::FeedbackType::None);
  if(!solverSuccess_)
  {
    holdTarget("QP solve failed");
    updateContacts();
    // A failed QP does not integrate the robot. Retry once with a stationary
    // target and the unavailable safety rows removed.
    if(!solver().run(mc_solver::FeedbackType::None)) return false;
  }
  maxSoftSlack_ = 0;
  if(!fallback_)
  {
    for(int i = 0; i < static_cast<int>(regions.size() * links.size()); ++i)
    {
      pairSoftSlack_[i] = safetyConstraint_->softSlack(i);
      maxSoftSlack_ = std::max(maxSoftSlack_, pairSoftSlack_[i]);
    }
    // Predict the plotted point's next approach speed from the QP acceleration.
    const Eigen::VectorXd acceleration = rbd::dofToVector(robot().mb(), robot().alphaD());
    speedPlot_.predicted = speedPlot_.current + solver().dt()
        * (speedPlot_.directional.dot(acceleration) + speedPlot_.drift);
  }
  return true;
}

CONTROLLER_CONSTRUCTOR("KinovaPositionSSMPFLController", KinovaPositionSSMPFLController)
