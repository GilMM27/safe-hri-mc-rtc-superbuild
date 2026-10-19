#include "../smplh_playback/SmplhDistanceWorker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <thread>

int main(int argc, char ** argv)
{
  const int side = argc > 1 ? std::stoi(argv[1]) : 60;
  const int samples = argc > 2 ? std::stoi(argv[2]) : 30;
  std::ostringstream xml;
  xml << "<mujoco><worldbody><geom name='human' type='sphere' size='.001' group='0'/>";
  xml << "<body name='kinova_nearest' pos='.1 .1 .1'><geom type='sphere' size='.05' group='2'/></body>";
  for(int i = 0; i < 20; ++i)
  {
    const double x = .65 + .012 * i, y = .72 - .006 * i;
    xml << "<body name='kinova_" << i << "' pos='" << x << ' ' << y
        << " .1'><geom type='sphere' size='.05' group='2'/></body>";
  }
  xml << "</worldbody></mujoco>";
  char error[1024]{};
  auto * spec = mj_parseXMLString(xml.str().c_str(), nullptr, error, sizeof(error));
  if(!spec) throw std::runtime_error(error);
  auto * model = mj_compile(spec, nullptr);
  if(!model) throw std::runtime_error(mjs_getError(spec));
  auto * data = mj_makeData(model);
  mj_forward(model, data);

  std::vector<float> vertices;
  std::vector<uint32_t> faces;
  std::vector<uint32_t> faceIds;
  auto index = [side](int row, int col) {
    return static_cast<uint32_t>(row * (side + 1) - row * (row - 1) / 2 + col);
  };
  for(int row = 0; row <= side; ++row)
    for(int col = 0; col <= side - row; ++col)
    {
      vertices.push_back(static_cast<float>(col) / side);
      vertices.push_back(static_cast<float>(row) / side);
      vertices.push_back(0);
    }
  auto triangle = [&](uint32_t a, uint32_t b, uint32_t c) {
    faces.insert(faces.end(), {a, b, c});
    faceIds.push_back(static_cast<uint32_t>(faceIds.size()));
  };
  for(int row = 0; row < side; ++row)
    for(int col = 0; col < side - row; ++col)
    {
      triangle(index(row, col), index(row, col+1), index(row+1, col));
      if(col + row + 1 < side)
        triangle(index(row, col+1), index(row+1, col+1), index(row+1, col));
    }

  std::vector<double> baselineDistances;
  for(int mode : {0, 1, 2})
  {
    setenv("SMPLH_DISTANCE_USE_SCH", mode == 1 ? "1" : "0", 1);
    setenv("SMPLH_DISTANCE_MODE", mode == 2 ? "sch_approx" : "fcl", 1);
    mc_mujoco::SmplhDistanceWorker worker;
    worker.configure(model, {faceIds}, {"test"}, faces.data());
    std::vector<double> durations, distances;
    double absoluteError = 0, maximumError = 0;
    uint64_t queried = 0, total = 0, pruned = 0;
    double schTime = 0;
    double prepTime = 0, collisionTime = 0, distanceTime = 0;
    for(int sample = 0; sample <= samples; ++sample)
    {
      // Force BVH refitting at each sample, as in animated playback.
      for(size_t v = 0; v < vertices.size()/3; ++v)
        vertices[3*v+2] = static_cast<float>(.001 * std::sin(.1*sample + .03*v));
      const uint64_t sequence = static_cast<uint64_t>(sample + 1);
      worker.submit(sequence, static_cast<uint32_t>(sequence), sample*.025,
                    vertices.data(), static_cast<uint32_t>(vertices.size()/3), data, model, 0);
      mc_mujoco::SmplhDistanceSnapshot result;
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
      do
      {
        result = worker.latest();
        if(result.sequence == sequence) break;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      } while(std::chrono::steady_clock::now() < deadline);
      if(result.sequence != sequence || !result.ready || result.regions.size() != 1)
        throw std::runtime_error("worker failed: " + result.error);
      if(sample == 0) continue;
      durations.push_back(1000*result.worker_duration_seconds);
      distances.push_back(result.regions[0].distance);
      if(result.approximate != (mode == 2)) throw std::runtime_error("unexpected distance mode");
      if(mode == 1 && std::abs(distances.back() - baselineDistances[distances.size()-1]) > 1e-8)
        throw std::runtime_error("SCH pruning changed an exact surface distance");
      if(mode == 2)
      {
        const auto & region = result.regions[0];
        if(region.distance > 0)
        {
          if(!region.closest_points_valid || !region.direction_valid)
            throw std::runtime_error("approximate closest points are missing");
          double squared = 0;
          for(int axis = 0; axis < 3; ++axis)
            squared += std::pow(region.robot_point[axis] - region.human_point[axis], 2);
          if(std::abs(std::sqrt(squared) - region.distance) > 1e-4)
            throw std::runtime_error("approximate witnesses disagree with distance");
        }
        const double error = std::abs(distances.back() - baselineDistances[distances.size()-1]);
        absoluteError += error;
        maximumError = std::max(maximumError, error);
      }
      queried += result.queried_pairs;
      total += result.total_pairs;
      pruned += result.sch_pruned_pairs;
      schTime += result.sch_duration_seconds;
      prepTime += result.preparation_duration_seconds;
      collisionTime += result.collision_duration_seconds;
      distanceTime += result.distance_duration_seconds;
    }
    std::sort(durations.begin(), durations.end());
    const double mean = std::accumulate(durations.begin(), durations.end(), 0.0) / samples;
    if(mode == 0) baselineDistances = distances;
    else if(mode == 1 && pruned == 0) throw std::runtime_error("benchmark did not exercise SCH pruning");
    std::cout << (mode == 0 ? "FCL" : mode == 1 ? "SCH-prune" : "SCH-approx")
              << " triangles=" << faces.size()/3
              << " samples=" << samples << " mean_ms=" << mean
              << " median_ms=" << durations[samples/2]
              << " p95_ms=" << durations[static_cast<size_t>(.95*(samples-1))]
              << " pairs_per_sample=" << static_cast<double>(queried)/samples
              << '/' << static_cast<double>(total)/samples
              << " sch_pruned_per_sample=" << static_cast<double>(pruned)/samples
              << " sch_ms_per_sample=" << 1000*schTime/samples
              << " prep_ms=" << 1000*prepTime/samples
              << " collision_ms=" << 1000*collisionTime/samples
              << " distance_ms=" << 1000*distanceTime/samples
              << " mean_distance=" << std::accumulate(distances.begin(), distances.end(), 0.0)/samples
              << " mean_absolute_error=" << absoluteError/samples
              << " max_absolute_error=" << maximumError
              << '\n';
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}
