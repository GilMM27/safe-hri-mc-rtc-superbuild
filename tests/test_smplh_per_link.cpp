#include "../smplh_playback/SmplhDistanceWorker.h"

#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>

int main()
{
  std::ostringstream xml;
  xml << "<mujoco><worldbody><geom name='human' type='sphere' size='.001'/>"
      << "<body name='kinova_base_link'><geom name='kinova_base_geom' type='sphere' size='.03' group='2'/>";
  for(int joint = 1; joint <= 7; ++joint)
  {
    xml << "<body name='kinova_link_" << joint << "' pos='.2 0 0'>"
        << "<joint name='kinova_joint_" << joint << "' type='hinge' axis='0 0 1'/>"
        << "<geom name='kinova_geom_" << joint << "' type='sphere' size='.03' group='2'/>";
    if(joint == 6)
      xml << "<body name='kinova_fixed_six' pos='.05 0 0'>"
          << "<geom name='kinova_fixed_geom' type='sphere' size='.03' group='2'/></body>";
  }
  for(int joint = 1; joint <= 7; ++joint) xml << "</body>";
  xml << "</body></worldbody></mujoco>";
  char error[1024]{};
  auto * spec = mj_parseXMLString(xml.str().c_str(), nullptr, error, sizeof(error));
  if(!spec) throw std::runtime_error(error);
  auto * model = mj_compile(spec, nullptr);
  if(!model) throw std::runtime_error(mjs_getError(spec));
  auto * data = mj_makeData(model);
  mj_forward(model, data);

  const uint32_t faces[]{0, 1, 2};
  const float vertices[]{2, -1, -1, 2, 1, -1, 2, 0, 1};
  for(const char * mode : {"fcl", "sch_approx"})
  {
    setenv("SMPLH_DISTANCE_MODE", mode, 1);
    mc_mujoco::SmplhDistanceWorker worker;
    worker.configure(model, {{0}, {0}}, {"torso", "head"}, faces, "kinova_", 2, true);
    worker.submit(1, 0, 0, vertices, 3, nullptr, 1.0, true, data, model, 0);
    mc_mujoco::SmplhDistanceSnapshot result;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do
    {
      result = worker.latest();
      if(result.sequence == 1) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while(std::chrono::steady_clock::now() < deadline);
    if(!result.ready || !result.error.empty() || result.regions.size() != 14)
      throw std::runtime_error("per-link distance query failed: " + result.error);
    if(result.total_pairs != 16) throw std::runtime_error("base was included or fixed child was omitted");
    for(size_t region = 0; region < 2; ++region)
    {
      for(size_t link = 0; link < 7; ++link)
      {
        const auto & pair = result.regions[region * 7 + link];
        const std::string expected = "joint_" + std::to_string(link + 1);
        if(pair.region != (region ? "head" : "torso") || pair.link != expected || !pair.closest_points_valid
           || !pair.direction_valid || !pair.robot_velocity_valid || !(pair.distance > 0))
          throw std::runtime_error("invalid region/link witness " + expected);
        if(link && !(pair.distance < result.regions[region * 7 + link - 1].distance))
          throw std::runtime_error("link distances are out of order");
      }
      if(result.regions[region * 7 + 5].robot_body != "fixed_six")
        throw std::runtime_error("fixed child was not assigned to joint_6");
      if(result.regions[region * 7 + 6].robot_body != "link_7")
        throw std::runtime_error("joint_7 was assigned to joint_6");
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}
