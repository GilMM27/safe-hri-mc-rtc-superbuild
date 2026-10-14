#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace mc_mujoco
{

/** Immutable-by-convention result of one asynchronous SMPL-H clearance sample. */
struct SmplhRegionDistance
{
  std::string region;
  double distance = 0.0;
  std::array<double, 3> human_point{};
  std::array<double, 3> robot_point{};
  std::array<double, 3> direction{};
  std::array<double, 3> human_normal{};
  bool closest_points_valid = false;
  bool direction_valid = false;
  bool human_normal_valid = false;
  bool intersecting = false;
  std::string robot_geometry;
};

struct SmplhDistanceSnapshot
{
  bool ready = false;
  std::uint64_t sequence = 0;
  std::uint32_t motion_frame = 0;
  double sample_simulation_time = 0.0;
  double worker_duration_seconds = 0.0;
  std::string error;
  std::vector<SmplhRegionDistance> regions;
};

} // namespace mc_mujoco
