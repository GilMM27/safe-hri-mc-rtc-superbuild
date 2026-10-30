#include "../src/KinovaSweepController.h"
#include <mc_rbdyn/RobotLoader.h>
#include <cmath>
#include <stdexcept>

static void runController(const mc_rtc::Configuration & config, int ticks = 5, bool expectMotion = false,
                          double expectedWeight = 10.0)
{
  auto module = mc_rbdyn::RobotLoader::get_robot_module("Kinova");
  KinovaSweepController controller(module, 0.001, config);
  if(controller.solver().backend() != mc_solver::QPSolver::Backend::TVM)
    throw std::runtime_error("sweep controller did not select TVM");
  if(!controller.robots().hasRobot("ground"))
    throw std::runtime_error("sweep controller did not load the ground environment");
  const auto & tasks = controller.solver().tasks();
  auto * sweepTask = tasks.size() == 1 ? dynamic_cast<mc_tasks::PostureTask *>(tasks.front()) : nullptr;
  if(!sweepTask || sweepTask->name() != "KinovaSweepPosture")
    throw std::runtime_error("sweep controller has an unexpected posture task");
  auto checkWeight = [&]() {
    if(std::abs(sweepTask->weight() - expectedWeight) > 1e-12)
      throw std::runtime_error("sweep posture weight does not match Sweep.PostureWeight");
  };
  checkWeight();
  controller.reset({controller.robot().q()});
  checkWeight();
  const auto joint = controller.robot().jointIndexByName("joint_1");
  const double initial = controller.robot().q().at(joint).at(0);
  for(int tick = 0; tick < ticks; ++tick)
    if(!controller.run()) throw std::runtime_error("TVM controller solve failed");
  checkWeight();
  if(expectMotion && controller.robot().q().at(joint).at(0) - initial < 1e-4)
    throw std::runtime_error("FSM idle posture prevented the sweep");
}

int main(int argc, char ** argv)
{
  if(argc != 2) throw std::runtime_error("expected SSM+PFL example path");
  runController(mc_rtc::Configuration{});
  mc_rtc::Configuration weighted;
  weighted.add("Sweep", mc_rtc::Configuration::fromYAMLData("PostureWeight: 7.5"));
  runController(weighted, 5, false, 7.5);
  mc_rtc::Configuration moving;
  moving.add("Sweep", mc_rtc::Configuration::fromYAMLData("AutoStart: true"));
  runController(moving, 300, true);
  mc_rtc::Configuration safety;
  safety.load(std::string(argv[1]));
  runController(safety);
  auto safetyConfig = safety("SSMPFL");
  safetyConfig.remove("Weight");
  auto recovery = safety("SSMPFL")("Relaxation");
  recovery.add("Enabled", true);
  runController(safety);
  recovery.add("Mode", "two_stage_full_feasibility");
  runController(safety);
}
