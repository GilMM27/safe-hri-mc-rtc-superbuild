#include "../smplh_playback/SmplhDistanceWorker.h"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

int main()
{
  char error[1024]{};
  auto * spec = mj_parseXMLString(R"(
    <mujoco><worldbody>
      <geom name="human" type="sphere" size=".001" group="0"/>
      <body name="kinova_test" pos="0 0 .1">
        <geom type="sphere" size=".05" group="2"/>
      </body>
    </worldbody></mujoco>)", nullptr, error, sizeof(error));
  if(!spec) throw std::runtime_error(error);
  auto * model = mj_compile(spec, nullptr);
  if(!model) throw std::runtime_error(mjs_getError(spec));
  auto * data = mj_makeData(model);
  mj_forward(model, data);

  // Two triangles form an inverted V. Their shared convex hull fills the gap
  // beneath the roof, where the robot sphere sits.
  const uint32_t faces[]{0, 1, 2, 0, 1, 3};
  const float vertices[]{0, -1, 1, 0, 1, 1, -1, 0, 0, 1, 0, 0};
  auto measure = [&](const char * mode, const char * patchFaces) {
    setenv("SMPLH_DISTANCE_MODE", mode, 1);
    setenv("SMPLH_SCH_PATCH_FACES", patchFaces, 1);
    mc_mujoco::SmplhDistanceWorker worker;
    worker.configure(model, {{0, 1}}, {"roof"}, faces);
    worker.submit(1, 1, 0, vertices, 4, data, model, 0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    mc_mujoco::SmplhDistanceSnapshot result;
    do
    {
      result = worker.latest();
      if(result.sequence == 1) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while(std::chrono::steady_clock::now() < deadline);
    if(!result.ready || result.regions.size() != 1)
      throw std::runtime_error("distance worker failed: " + result.error);
    return result;
  };
  const auto exact = measure("fcl", "2");
  const auto hull = measure("sch_approx", "2");
  const auto triangles = measure("sch_approx", "1");
  if(exact.approximate || exact.regions[0].distance < .4 || exact.regions[0].intersecting)
    throw std::runtime_error("exact roof distance is invalid");
  if(!hull.approximate || hull.regions[0].distance != 0 || !hull.regions[0].intersecting)
    throw std::runtime_error("shared convex hull did not expose approximation error");
  if(!triangles.approximate || std::abs(triangles.regions[0].distance - exact.regions[0].distance) > 1e-4
     || !triangles.regions[0].closest_points_valid)
    throw std::runtime_error("single-triangle SCH patches disagree with FCL");
  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}
