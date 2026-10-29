#include "../smplh_playback/NativeSmplhPlayback.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

int main(int argc, char ** argv)
{
  if(argc != 2) return 2;
  setenv("SMPLH_DISTANCE_MODE", "fcl", 1);
  char error[1024] = {};
  mjModel * model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if(!model) { std::cerr << error << '\n'; return 3; }
  mjData * data = mj_makeData(model);
  try
  {
    SmplhBridge bridge;
    data->time = 0.0; bridge.update(model, data);
    const double initial_x = data->qpos[0];
    const double initial_y = data->qpos[1];
    int mesh = mj_name2id(model, mjOBJ_MESH, "smplh_surface");
    if(mesh < 0) throw std::runtime_error("Missing test surface mesh");
    int surfaceGeom=mj_name2id(model,mjOBJ_GEOM,"smplh_surface");
    const double initial_surface_x = data->geom_xpos[3*surfaceGeom];
    const double initial_surface_y = data->geom_xpos[3*surfaceGeom+1];
    if(std::abs(model->geom_pos[3*surfaceGeom])+std::abs(model->geom_pos[3*surfaceGeom+1])
       +std::abs(model->geom_pos[3*surfaceGeom+2])+std::abs(model->mesh_pos[3*mesh])
       +std::abs(model->mesh_pos[3*mesh+1])+std::abs(model->mesh_pos[3*mesh+2])>1e-9)
      throw std::runtime_error("surface received an extra MuJoCo asset transform");
    float initial_vertex = model->mesh_vert[3 * model->mesh_vertadr[mesh]];
    data->time = 1.0 / 30.0; bridge.update(model, data);
    if(std::abs(data->qpos[0] - initial_x - .004) > 1e-6) throw std::runtime_error("frame did not advance");
    float advanced_vertex = model->mesh_vert[3 * model->mesh_vertadr[mesh]];
    if(std::abs(advanced_vertex - initial_vertex - .004f) > 1e-6) throw std::runtime_error("surface did not advance");
    if(std::getenv("SMPLH_LOOP"))
    {
      data->time = .1; bridge.update(model, data);
      if(std::abs(data->qpos[0] - initial_x) > 1e-6) throw std::runtime_error("loop did not wrap to frame zero");
    }
    else
    {
      data->time = 100.; bridge.update(model, data);
      if(std::abs(data->qpos[0] - initial_x - .008) > 1e-6) throw std::runtime_error("playback did not hold final frame");
    }
    data->time = 0.0; bridge.update(model, data);
    if(std::abs(data->qpos[0] - initial_x) > 1e-6) throw std::runtime_error("reset did not return to frame zero");
    if(!std::getenv("SMPLH_DISTANCE_DISABLE"))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto result=bridge.snapshot();
      if(!result.ready || result.regions.size()!=14) throw std::runtime_error("asynchronous region query did not complete");
      bool sawIntersection=false;
      for(const auto & region : result.regions)
      {
        if(region.intersecting)
        {
          sawIntersection=true;
          if(region.distance!=0 || region.closest_points_valid || region.direction_valid)
            throw std::runtime_error("intersection should report zero clearance without nearest points");
          continue;
        }
        if(!(region.distance>0) || !region.closest_points_valid || !region.direction_valid)
          throw std::runtime_error("invalid native FCL closest-point result");
        double norm=0;
        for(int i=0;i<3;++i)
        {
          double delta=region.robot_point[i]-region.human_point[i];
          norm+=delta*delta;
          if(std::abs(delta / region.distance - region.direction[i])>1e-5)
            throw std::runtime_error("native FCL direction does not match closest points");
        }
        if(std::abs(std::sqrt(norm)-region.distance)>1e-5)
          throw std::runtime_error("native FCL points do not match distance");
      }
      if(std::getenv("SMPLH_TEST_INTERSECT") && !sawIntersection)
        throw std::runtime_error("expected native FCL intersection was not detected");
    }
    // mc_mujoco writes init_pos to the free joint before playback starts.
    // Playback must transfer that placement to the fixed visual body.
    const double quarterTurn = std::sqrt(.5);
    mjData * placedData = mj_makeData(model);
    const int rootJoint = mj_name2id(model, mjOBJ_JOINT, "smplh_root");
    auto * placedRoot = placedData->qpos + model->jnt_qposadr[rootJoint];
    placedRoot[0] = .5;
    placedRoot[1] = -.25;
    placedRoot[2] = .1;
    placedRoot[3] = quarterTurn;
    placedRoot[4] = 0;
    placedRoot[5] = 0;
    placedRoot[6] = quarterTurn;
    {
      SmplhBridge placedBridge;
      placedBridge.update(model, placedData);
      if(std::abs(placedData->qpos[0] - (.5 - initial_y)) > 1e-6
         || std::abs(placedData->qpos[1] - (-.25 + initial_x)) > 1e-6
         || std::abs(placedData->qpos[3] - quarterTurn) > 1e-6
         || std::abs(placedData->qpos[6] - quarterTurn) > 1e-6)
        throw std::runtime_error("scene placement did not transform the cached root pose");
      if(std::abs(placedData->geom_xpos[3*surfaceGeom] - (.5 - initial_surface_y)) > 1e-6
         || std::abs(placedData->geom_xpos[3*surfaceGeom+1] - (-.25 + initial_surface_x)) > 1e-6)
        throw std::runtime_error("scene placement did not transform the surface");
    }
    mj_deleteData(placedData);
  }
  catch(const std::exception & e)
  {
    std::cerr << e.what() << '\n'; mj_deleteData(data); mj_deleteModel(model); return 1;
  }
  mj_deleteData(data); mj_deleteModel(model);
  return 0;
}
