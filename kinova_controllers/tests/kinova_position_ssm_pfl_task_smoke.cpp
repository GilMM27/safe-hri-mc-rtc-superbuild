#include "../src/KinovaPositionSSMPFLPositionTaskController.h"

#include <mc_rbdyn/RobotLoader.h>
#include <RBDyn/MultiBodyConfig.h>

#include <cmath>
#include <array>
#include <stdexcept>

int main(int argc, char ** argv)
{
  if(argc != 2) throw std::runtime_error("expected SSMPFL configuration path");
  mc_rtc::Configuration config;
  config.load(std::string(argv[1]));
  config("SSMPFL").remove("Weight");
  config("SSMPFL").add("Weight", 7.5);
  config("SSMPFL").add("Stiffness", 3.0);
  config("SSMPFL").add("HumanMode", "moving");
  auto module = mc_rbdyn::RobotLoader::get_robot_module("Kinova");
  KinovaPositionSSMPFLPositionTaskController controller(module, 0.001, config);
  if(controller.solver().backend() != mc_solver::QPSolver::Backend::TVM)
    throw std::runtime_error("task controller did not select TVM");
  const auto & tasks = controller.solver().tasks();
  if(tasks.size() != 9) throw std::runtime_error("expected tool, posture, and seven safety tasks");
  std::array<mc_tasks::PositionTask *, 7> safetyTasks{};
  for(int joint = 1; joint <= 7; ++joint)
  {
    auto * task = dynamic_cast<mc_tasks::PositionTask *>(tasks[joint + 1]);
    safetyTasks[joint - 1] = task;
    if(!task || task->name() != "SSMPFLPosition_joint_" + std::to_string(joint))
      throw std::runtime_error("missing link position task");
    if(std::abs(task->weight() - 7.5) > 1e-12 || std::abs(task->stiffness() - 3.0) > 1e-12)
      throw std::runtime_error("link task gains were not configured");
  }
  controller.reset({controller.robot().q()});
  auto * tool = dynamic_cast<mc_tasks::PositionTask *>(tasks.front());
  if(!tool) throw std::runtime_error("missing tool position task");
  tool->position(tool->position() + Eigen::Vector3d(0.02, 0, 0));
  if(!controller.run()) throw std::runtime_error("controller failed without a snapshot");
  if((tool->position() - controller.robot().frame("tool_frame").position().translation()).norm() > 1e-3)
    throw std::runtime_error("missing snapshot did not cancel the tool target");
  if(controller.solver().tasks().size() != 2)
    throw std::runtime_error("fallback left safety tasks opposing tool motion");

  mc_mujoco::SmplhDistanceSnapshot snapshot;
  snapshot.ready = true;
  snapshot.sample_simulation_time = 0.002;
  const std::array<const char *, 14> regions = {{"head", "torso", "left_upper_arm", "left_forearm",
                                                 "left_hand", "right_upper_arm", "right_forearm", "right_hand",
                                                 "left_thigh", "left_lower_leg", "left_foot", "right_thigh",
                                                 "right_lower_leg", "right_foot"}};
  const Eigen::Vector3d witness = controller.robot().bodyPosW("bracelet_link").translation();
  for(const auto * region : regions)
    for(int joint = 1; joint <= 7; ++joint)
    {
      mc_mujoco::SmplhRegionDistance sample;
      sample.region = region;
      sample.link = "joint_" + std::to_string(joint);
      sample.distance = 0.01;
      sample.robot_body = "bracelet_link";
      sample.robot_point = {{witness.x(), witness.y(), witness.z()}};
      sample.direction = {{-1.0, 0.0, 0.0}};
      sample.human_velocity = {{0.01, 0.0, 0.0}};
      sample.closest_points_valid = true;
      sample.direction_valid = true;
      sample.human_velocity_valid = true;
      snapshot.regions.push_back(sample);
    }
  controller.datastore().make<mc_mujoco::SmplhDistanceSnapshot>("SMPLH::DistanceSnapshot", snapshot);
  const Eigen::Vector3d target = tool->position() + Eigen::Vector3d(0.01, 0, 0);
  tool->position(target);
  if(!controller.run()) throw std::runtime_error("controller failed with a valid snapshot");
  if((tool->position() - target).norm() > 1e-10)
    throw std::runtime_error("valid snapshot unexpectedly cancelled tool target");
  if(controller.solver().tasks().size() != 2)
    throw std::runtime_error("unneeded safety tasks remained active");
  if(rbd::dofToVector(controller.robot().mb(), controller.robot().alphaD()).norm() < 1e-8)
    throw std::runtime_error("tool goal did not produce robot acceleration");
  for(auto * task : safetyTasks)
  {
    if(!task->refAccel().allFinite())
      throw std::runtime_error("link task acceleration is invalid");
  }
}
