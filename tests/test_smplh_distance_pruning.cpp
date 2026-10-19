#include "../smplh_playback/SmplhDistanceWorker.h"
#include <chrono>
#include <thread>
#include <stdexcept>
#include <iostream>

int main()
{
  setenv("SMPLH_DISTANCE_MODE", "fcl", 1);
  char error[1024]{};
  auto * spec = mj_parseXMLString(R"(
    <mujoco><worldbody>
      <geom name="human" type="sphere" size=".01" group="0"/>
      <body name="kinova_far" pos="20 0 0"><geom type="sphere" size=".25" group="2"/></body>
      <body name="kinova_near" pos="2 0 0"><geom type="sphere" size=".25" group="2"/></body>
      <body name="kinova_other" pos="-7 0 0"><geom type="sphere" size=".25" group="2"/></body>
      <body name="kinova_rotated" pos="8 0 0" euler="0 0 45">
        <geom type="box" size=".5 .2 .3" group="2"/>
      </body>
    </worldbody></mujoco>)", nullptr, error, sizeof(error));
  if(!spec) throw std::runtime_error(error);
  auto * model = mj_compile(spec, nullptr);
  if(!model) throw std::runtime_error(mjs_getError(spec));
  auto * data = mj_makeData(model);
  mj_forward(model, data);
  {
    mc_mujoco::SmplhDistanceWorker worker;
    // Noncontiguous global IDs, with unused vertices far outside the region.
    const uint32_t faces[]{3, 5, 7};
    worker.configure(model, {{0}}, {"test"}, faces);
    uint64_t sequence = 0;
    for(float x : {0.f, 7.5f, 8.6f, 8.f, 2.f, -7.f, 0.f})
    {
      float vertices[]{1000, 0, 0, 1000, 1, 0, 1000, 0, 1,
                       x, -1, -1, 1000, 0, 0, x, 1, -1, 1000, 0, 0, x, 0, 1};
      ++sequence;
      worker.submit(sequence, sequence, 0, vertices, 8, data, model, 0);
      mc_mujoco::SmplhDistanceSnapshot snapshot;
      auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      do
      {
        snapshot = worker.latest();
        if(snapshot.sequence == sequence) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } while(std::chrono::steady_clock::now() < deadline);
      if(!snapshot.ready || snapshot.sequence != sequence || snapshot.regions.size() != 1)
        throw std::runtime_error("worker sample failed: " + snapshot.error);

      auto humanMesh = std::make_shared<mc_mujoco::SmplhBVH>();
      std::vector<fcl::Vector3d> points;
      for(auto i : faces) points.emplace_back(vertices[3*i], vertices[3*i+1], vertices[3*i+2]);
      humanMesh->beginModel();
      humanMesh->addSubModel(points, std::vector<fcl::Triangle>{{0, 1, 2}});
      humanMesh->endModel();
      fcl::CollisionObjectd human(humanMesh);
      double expected = std::numeric_limits<double>::infinity();
      fcl::Vector3d expectedHuman, expectedRobot;
      bool intersects = false;
      for(int i = 1; i < model->ngeom; ++i)
      {
        std::shared_ptr<fcl::CollisionGeometryd> geometry;
        const auto * size = model->geom_size + 3*i;
        if(model->geom_type[i] == mjGEOM_SPHERE) geometry = std::make_shared<fcl::Sphered>(size[0]);
        else geometry = std::make_shared<fcl::Boxd>(2*size[0], 2*size[1], 2*size[2]);
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(data->geom_xmat+9*i);
        pose.translation() = Eigen::Map<const Eigen::Vector3d>(data->geom_xpos+3*i);
        fcl::CollisionObjectd robot(geometry, pose);
        fcl::CollisionResultd collision;
        if(fcl::collide(&human, &robot, fcl::CollisionRequestd(), collision))
        { expected = 0; intersects = true; }
        else
        {
          fcl::DistanceResultd distance;
          const double value = fcl::distance(&human, &robot, fcl::DistanceRequestd(true), distance);
          if(value < expected)
          {
            expected = value;
            expectedHuman = distance.nearest_points[0];
            expectedRobot = distance.nearest_points[1];
          }
        }
      }
      const auto & result = snapshot.regions.front();
      if(std::abs(result.distance-expected) > 1e-9 || result.intersecting != intersects)
        throw std::runtime_error("pruned result differs from exhaustive query");
      if(!intersects)
      {
        if(!result.closest_points_valid) throw std::runtime_error("missing nearest points");
        for(int i=0; i<3; ++i)
          if(std::abs(result.human_point[i]-expectedHuman[i])>1e-9
             || std::abs(result.robot_point[i]-expectedRobot[i])>1e-9)
            throw std::runtime_error("pruned nearest points differ from exhaustive query");
        if(!result.human_normal_valid || std::abs(result.human_normal[0]-1)>1e-9)
          throw std::runtime_error("remapped face normal changed");
      }
      if(snapshot.total_pairs != 4 || snapshot.queried_pairs >= snapshot.total_pairs)
        throw std::runtime_error("expected distant candidates to be pruned");
      const double stages = snapshot.preparation_duration_seconds + snapshot.collision_duration_seconds
                            + snapshot.distance_duration_seconds;
      if(!std::isfinite(stages) || stages <= 0 || stages > snapshot.worker_duration_seconds)
        throw std::runtime_error("invalid stage timings");
      std::cout << "x=" << x << " distance=" << result.distance
                << " pairs=" << snapshot.queried_pairs << '/' << snapshot.total_pairs << '\n';
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  mj_deleteSpec(spec);
}
