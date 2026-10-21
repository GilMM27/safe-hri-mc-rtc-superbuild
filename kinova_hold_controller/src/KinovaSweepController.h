#pragma once

#include <mc_control/mc_controller.h>
#include <mc_mujoco/SmplhDistanceSnapshot.h>
#include "SSMPFLConstraint.h"
#include <RBDyn/FD.h>

#include <atomic>
#include <string>
#include <map>
#include <memory>
#include <limits>
#include <vector>

struct KinovaSweepController : public mc_control::MCController
{
  KinovaSweepController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  ~KinovaSweepController() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
  bool run() override;

private:
  enum class Phase { Hold, ToWaypoint, DwellWaypoint, ToHome, DwellHome };

  void beginSegment(Phase phase, double goal);
  void stopAtAvailablePosture();
  void stopSweepAtAvailableJoint1();
  double sampledOrModelJoint1() const;
  const mc_mujoco::SmplhDistanceSnapshot * distanceSnapshot() const;
  const mc_mujoco::SmplhContactForceSnapshot * contactForceSnapshot() const;
  double regionContactForce(const std::string & region, bool peak) const;
  double regionDistance(const std::string & region) const;
  std::string bindingRegions() const;
  std::string uncontrollableRegions() const;
  bool updateSafetyConstraint(bool applyConstraint);
  void safetyFallback(const std::string & reason);
  void addSpeedPlot();
  void updateSpeedPlot();

  struct SafetyRegion
  {
    double force = 0.0;
    double stiffness = 0.0;
    double margin = 0.0;
  };
  struct PairLog
  {
    std::string link, body, geometry;
    double distance = std::numeric_limits<double>::quiet_NaN();
    double robotPointX = 0, robotPointY = 0, robotPointZ = 0;
    double humanPointX = 0, humanPointY = 0, humanPointZ = 0;
    double modelWitnessError = 0;
    double nx = 0, ny = 0, nz = 0;
    double humanVelocityX = 0, humanVelocityY = 0, humanVelocityZ = 0;
    double humanSpeed = 0, robotSpeed = 0, closingSpeed = 0;
    double actualRelativeSpeed = 0, actualTangentialSpeed = 0;
    double inverseMass = 0, reflectedMass = 0, pflLimit = 0, combinedLimit = 0;
    double predictedResidual = 0, actualResidual = 0;
    bool active = false;
    bool uncontrollable = false;
    bool monitorOnly = false;
    bool binding = false;
    bool intersecting = false;
    bool sampledVelocityValid = false;
    bool actualViolation = false;
  };
  bool safetyEnabled_ = false;
  bool constrainWhileHolding_ = true;
  bool monitorOnly_ = false;
  bool holdOnConstraintUnavailable_ = true;
  bool movingHuman_ = false;
  bool safetyFallback_ = false;
  bool solverSuccess_ = true;
  double stopTime_ = 0.5, maxSampleAge_ = 0.1;
  double maxModelWitnessError_ = 0.1, sampleAge_ = std::numeric_limits<double>::quiet_NaN();
  double maxActualResidual_ = 0.02;
  double minMobility_ = 1e-12, minMassEigenvalue_ = 1e-9;
  double safetyTime_ = 0, safetyComputationSeconds_ = 0;
  int limitingPair_ = -1, activePairs_ = 0, bindingPairs_ = 0;
  int uncontrollablePairs_ = 0;
  int monitorOnlyPairs_ = 0;
  int actualViolationPairs_ = 0;
  std::string fallbackReason_;
  std::map<std::string, SafetyRegion> safetyRegions_;
  std::vector<PairLog> pairLogs_; // closest link per human region, for existing GUI/log keys
  std::vector<PairLog> linkPairLogs_; // one entry per region/link pair
  struct SpeedPlotSample
  {
    double current = 0, qpNext = 0, sampled = 0, limit = 0;
    double sampleTime = -1, qpTime = -1;
  };
  SpeedPlotSample speedPlot_;
  size_t plotPairIndex_ = 6; // head/joint_7
  std::unique_ptr<SSMPFLConstraint> safetyConstraint_;
  std::unique_ptr<rbd::ForwardDynamics> inertia_;

  std::atomic<bool> startRequested_{false};
  std::atomic<bool> stopRequested_{false};
  Phase phase_ = Phase::Hold;
  unsigned int jointIndex_ = 0;
  double amplitude_ = 0.2;
  double segmentSeconds_ = 1.0;
  double dwellSeconds_ = 0.25;
  double maxJointSpeed_ = 0.6;
  double postureStiffness_ = 25.0;
  bool autoStart_ = false;
  bool autoStartPending_ = false;
  bool resumeAfterMeasurement_ = false;
  Phase resumePhase_ = Phase::Hold;
  double resumeGoal_ = 0.0;
  double elapsed_ = 0.0;
  double activeSegmentSeconds_ = 1.0;
  double startPosition_ = 0.0;
  double goalPosition_ = 0.0;
  double commandedJoint1_ = 0.0;
};
