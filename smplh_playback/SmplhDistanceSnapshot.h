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
  std::string link; // joint_1 ... joint_7 in per-link mode
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
  std::string robot_body;
  std::array<double, 3> robot_point_body_local{};
  std::array<double, 3> human_velocity{};
  bool human_velocity_valid = false;
  std::array<double, 3> robot_velocity{};
  bool robot_velocity_valid = false;
};

struct SmplhDistanceSnapshot
{
  bool ready = false;
  bool approximate = false;
  bool playback_frozen = false;
  std::uint64_t sequence = 0;
  std::array<double, 7> robot_joint_positions{};
  std::array<double, 7> robot_joint_velocities{};
  bool robot_joints_valid = false;
  std::uint32_t motion_frame = 0;
  double sample_simulation_time = 0.0;
  double worker_duration_seconds = 0.0;
  std::uint64_t queried_pairs = 0;
  std::uint64_t total_pairs = 0;
  std::uint64_t sch_pruned_pairs = 0;
  double preparation_duration_seconds = 0.0;
  double sch_duration_seconds = 0.0;
  double collision_duration_seconds = 0.0;
  double distance_duration_seconds = 0.0;
  std::string error;
  std::vector<SmplhRegionDistance> regions;
};

/** Forces from one completed MuJoCo physics step, before mj_forward resets them. */
struct SmplhContactForceRegion
{
  std::string region;
  double normal_force_n = 0.0; // Sum of positive normal forces on this region's capsules
  double peak_normal_force_n = 0.0; // Largest sampled sum since reset
  unsigned int contacts = 0;
};

struct SmplhContactForceSnapshot
{
  bool ready = false;
  double simulation_time = 0.0;
  std::uint64_t steps = 0;
  std::vector<SmplhContactForceRegion> regions;
};

} // namespace mc_mujoco
