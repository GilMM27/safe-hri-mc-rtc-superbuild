#include "../src/KinovaPositionSSMPFLController.h"
#include <mc_rbdyn/RobotLoader.h>
#include <cmath>
#include <stdexcept>
#include <array>

int main(int argc, char ** argv)
{
  if(argc != 2) throw std::runtime_error("expected SSMPFL configuration path");
  mc_rtc::Configuration config;
  config.load(std::string(argv[1]));
  config.add("PositionTask", mc_rtc::Configuration::fromYAMLData("Weight: 5.0\nStiffness: 2.0"));
  config("SSMPFL").add("HumanMode", "moving");
  if(std::abs(config("PositionTask")("Weight", 0.0) - 5.0) > 1e-12)
    throw std::runtime_error("example position weight is not 5");
  if(std::abs(config("PositionTask")("Stiffness", 0.0) - 2.0) > 1e-12)
    throw std::runtime_error("example position stiffness is not 2");
  config("PositionTask").remove("Weight"); // Exercise the controller default separately.
  config("PositionTask").remove("Stiffness");
  auto module = mc_rbdyn::RobotLoader::get_robot_module("Kinova");
  KinovaPositionSSMPFLController controller(module, 0.001, config);
  if(controller.solver().backend() != mc_solver::QPSolver::Backend::TVM)
    throw std::runtime_error("position controller did not select TVM");
  if(!controller.robots().hasRobot("ground"))
    throw std::runtime_error("position controller did not load ground");
  const auto & tasks = controller.solver().tasks();
  auto * task = tasks.size() == 2 ? dynamic_cast<mc_tasks::PositionTask *>(tasks.front()) : nullptr;
  auto * posture = tasks.size() == 2 ? dynamic_cast<mc_tasks::PostureTask *>(tasks.back()) : nullptr;
  if(!task || task->name() != "KinovaToolPosition")
    throw std::runtime_error("unexpected position task setup");
  if(!posture || posture->name() != "KinovaPreferredPosture")
    throw std::runtime_error("preferred posture task is missing");
  if(std::abs(task->weight() - 5.0) > 1e-12)
    throw std::runtime_error("position task default weight is not 5");
  if(std::abs(task->stiffness() - 2.0) > 1e-12)
    throw std::runtime_error("position task default stiffness is not 2");
  if(std::abs(posture->weight() - 0.1) > 1e-12 || std::abs(posture->stiffness() - 1.0) > 1e-12)
    throw std::runtime_error("preferred posture task default gains are incorrect");
  auto displacedPosture = controller.robot().q();
  displacedPosture.at(controller.robot().jointIndexByName("joint_1")).at(0) += 0.1;
  posture->posture(displacedPosture);
  controller.reset({controller.robot().q()});
  if(posture->posture() != controller.robot().q())
    throw std::runtime_error("preferred posture was not anchored at reset");
  const Eigen::Vector3d initial = controller.robot().frame("tool_frame").position().translation();
  if((task->position() - initial).norm() > 1e-10)
    throw std::runtime_error("position target was not initialized to tool position");
  task->position(initial + Eigen::Vector3d(0.1, 0.0, 0.0));
  if(!controller.run()) throw std::runtime_error("controller failed without a distance sample");
  if((task->position() - controller.robot().frame("tool_frame").position().translation()).norm() > 1e-3)
    throw std::runtime_error("missing distance sample did not cancel target motion");

  human_robot_distance::Snapshot snapshot;
  snapshot.ready = true;
  snapshot.sample_simulation_time = 0.001;
  snapshot.sequence = 1;
  const std::array<const char *, 14> regions = {{"head", "torso", "left_upper_arm", "left_forearm", "left_hand",
                                                 "right_upper_arm", "right_forearm", "right_hand", "left_thigh",
                                                 "left_lower_leg", "left_foot", "right_thigh", "right_lower_leg",
                                                 "right_foot"}};
  const Eigen::Vector3d witness = controller.robot().bodyPosW("bracelet_link").translation();
  for(const char * region : regions)
    for(int joint = 1; joint <= 7; ++joint)
    {
      human_robot_distance::RegionDistance sample;
      sample.region = region;
      sample.link = "joint_" + std::to_string(joint);
      sample.distance = 10.0;
      sample.robot_body = "bracelet_link";
      sample.robot_point = {{witness.x(), witness.y(), witness.z()}};
      sample.direction = {{1.0, 0.0, 0.0}};
      sample.closest_points_valid = true;
      sample.direction_valid = true;
      sample.human_velocity_valid = true;
      snapshot.regions.push_back(sample);
    }
  controller.datastore().make<human_robot_distance::Snapshot>(human_robot_distance::datastoreKey, snapshot);
  const Eigen::Vector3d target = task->position() + Eigen::Vector3d(0.02, 0.0, 0.0);
  task->position(target);
  if(!controller.run()) throw std::runtime_error("controller failed with valid moving-human rows");
  if((task->position() - target).norm() > 1e-10)
    throw std::runtime_error("valid sample unexpectedly cancelled GUI target");

  snapshot.sample_simulation_time = -1.0;
  controller.datastore().assign(human_robot_distance::datastoreKey, snapshot);
  if(!controller.run()) throw std::runtime_error("controller failed while holding on stale sample");
  if((task->position() - target).norm() < 1e-5)
    throw std::runtime_error("stale sample did not cancel target motion");

  snapshot.sample_simulation_time = 0.003;
  snapshot.regions.front().human_velocity_valid = false;
  controller.datastore().assign(human_robot_distance::datastoreKey, snapshot);
  task->position(target);
  if(!controller.run()) throw std::runtime_error("controller failed while holding on invalid human velocity");
  if((task->position() - target).norm() < 1e-5)
    throw std::runtime_error("invalid moving-human velocity did not cancel target motion");

  config("SSMPFL").add("HumanMode", "frozen");
  config("SSMPFL").remove("Weight"); // Exercise hard speed rows without slack.
  config("PositionTask").add("Weight", 7.5);
  config("PositionTask").add("Stiffness", 5.0);
  KinovaPositionSSMPFLController frozenController(module, 0.001, config);
  frozenController.reset({frozenController.robot().q()});
  snapshot.playback_frozen = true;
  snapshot.sample_simulation_time = 0.0;
  const Eigen::Vector3d frozenWitness = frozenController.robot().bodyPosW("bracelet_link").translation();
  for(auto & sample : snapshot.regions)
    sample.robot_point = {{frozenWitness.x(), frozenWitness.y(), frozenWitness.z()}};
  frozenController.datastore().make<human_robot_distance::Snapshot>(human_robot_distance::datastoreKey, snapshot);
  auto * frozenTask = dynamic_cast<mc_tasks::PositionTask *>(frozenController.solver().tasks().front());
  if(!frozenTask || std::abs(frozenTask->weight() - 7.5) > 1e-12)
    throw std::runtime_error("configured position task weight was not applied");
  if(std::abs(frozenTask->stiffness() - 5.0) > 1e-12)
    throw std::runtime_error("configured position task stiffness was not applied");
  const Eigen::Vector3d frozenTarget = frozenTask->position() + Eigen::Vector3d(0.02, 0.0, 0.0);
  frozenTask->position(frozenTarget);
  if(!frozenController.run()) throw std::runtime_error("frozen controller failed");
  if((frozenTask->position() - frozenTarget).norm() > 1e-10)
    throw std::runtime_error("frozen mode incorrectly required human velocity");
}
