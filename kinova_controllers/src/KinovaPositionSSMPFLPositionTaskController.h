#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_mujoco/SmplhDistanceSnapshot.h>
#include <mc_tasks/PositionTask.h>
#include <mc_tasks/PostureTask.h>
#include <RBDyn/FD.h>

#include <array>
#include <map>
#include <memory>
#include <string>

/** GUI-controlled tool position with one soft SSM+PFL position task per robot link. */
struct KinovaPositionSSMPFLPositionTaskController : public mc_control::fsm::Controller
{
  KinovaPositionSSMPFLPositionTaskController(mc_rbdyn::RobotModulePtr rm, double dt,
                                 const mc_rtc::Configuration & config);
  ~KinovaPositionSSMPFLPositionTaskController() override;

  void reset(const mc_control::ControllerResetData & data) override;
  bool run() override;

private:
  struct RegionParameters
  {
    double force = 0;
    double stiffness = 0;
    double margin = 0;
  };

  const mc_mujoco::SmplhDistanceSnapshot * distanceSnapshot() const;
  bool updateSafetyTasks();
  void holdTarget(const std::string & reason);
  void setSafetyTaskActive(size_t index, bool active);
  void addSpeedPlot();

  std::shared_ptr<mc_tasks::PositionTask> positionTask_;
  std::shared_ptr<mc_tasks::PostureTask> postureTask_;
  std::array<std::shared_ptr<mc_tasks::PositionTask>, 7> safetyTasks_;
  std::array<bool, 7> safetyTaskActive_{};
  std::unique_ptr<rbd::ForwardDynamics> inertia_;
  std::map<std::string, RegionParameters> regionParameters_;
  bool movingHuman_ = false;
  bool fallback_ = false;
  bool solverSuccess_ = true;
  double time_ = 0;
  double sampleAge_ = 0;
  double stopTime_ = 0.5;
  double maxSampleAge_ = 0.1;
  double maxModelWitnessError_ = 0.1;
  double minMobility_ = 1e-12;
  double minMassEigenvalue_ = 1e-9;
  double safetyWeight_ = 5.0;
  double safetyStiffness_ = 2.0;
  int activePairs_ = 0;
  double limitingSpeed_ = 0;
  std::array<double, 98> pairSpeedBudget_{};
  // The limiting pair has the smallest speed margin. The plotted pair has
  // the smallest geometric clearance; they need not be the same pair.
  std::string limitingPair_ = "none";
  std::string fallbackReason_;
  struct SpeedPlotSample
  {
    // Invalid samples plot as gaps instead of retaining an old pair's values.
    bool valid = false;
    size_t pairIndex = 0;
    std::string pair = "none";
    double distance = 0;
    double current = 0;
    double predicted = 0;
    double sampled = 0;
    double limit = 0;
    double drift = 0;
    Eigen::RowVectorXd directional;
  } speedPlot_;
};
