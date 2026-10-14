#pragma once
#include "SmplhDistanceSnapshot.h"

#include <fcl/fcl.h>
#include <sch/CD/CD_Pair.h>
#include <sch/S_Object/S_Box.h>
#include <sch/S_Object/S_Capsule.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>
#include <mujoco/mujoco.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <condition_variable>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <limits>
#include <thread>
#include <unordered_map>
#include <utility>

namespace mc_mujoco
{
using SmplhBVH = fcl::BVHModel<fcl::RSSd>;
using SmplhCollisionGeometry = fcl::CollisionGeometryd;

// The support set of these points is their convex hull. Its distance to a
// convex robot geom is a lower bound on distance to the original mesh surface.
class SmplhSchVertexCloud : public sch::S_ObjectNonNormalized
{
  const std::vector<fcl::Vector3d> * vertices_;
  const std::vector<uint32_t> * subset_ = nullptr;
  std::shared_ptr<const std::vector<fcl::Vector3d>> ownedVertices_;

public:
  explicit SmplhSchVertexCloud(const std::vector<fcl::Vector3d> * vertices,
                               const std::vector<uint32_t> * subset = nullptr)
  : vertices_(vertices), subset_(subset) {}
  explicit SmplhSchVertexCloud(std::shared_ptr<const std::vector<fcl::Vector3d>> vertices)
  : vertices_(vertices.get()), ownedVertices_(std::move(vertices)) {}
  SmplhSchVertexCloud * clone() const override { return new SmplhSchVertexCloud(*this); }
  sch::Point3 l_Support(const sch::Vector3 & direction, int & lastFeature) const override
  {
    size_t best = 0;
    double height = -std::numeric_limits<double>::infinity();
    const size_t count = subset_ ? subset_->size() : vertices_->size();
    for(size_t i = 0; i < count; ++i)
    {
      const auto & v = (*vertices_)[subset_ ? (*subset_)[i] : i];
      const double candidate = direction.m_x * v.x() + direction.m_y * v.y() + direction.m_z * v.z();
      if(candidate > height) { height = candidate; best = i; }
    }
    lastFeature = static_cast<int>(best);
    const auto & v = (*vertices_)[subset_ ? (*subset_)[best] : best];
    return sch::Point3(v.x(), v.y(), v.z());
  }
};

struct SmplhRobotGeom
{
  std::string name;
  std::shared_ptr<SmplhCollisionGeometry> geometry;
  std::unique_ptr<sch::S_Object> sch_geometry;
};

struct SmplhRobotPose
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

struct SmplhQueryInput
{
  std::uint64_t sequence = 0;
  std::uint32_t frame = 0;
  double time = 0;
  std::vector<Eigen::Vector3d> vertices;
  std::vector<SmplhRobotPose> robot_poses;
};

class SmplhDistanceWorker
{
  struct Region
  {
    struct Patch
    {
      std::vector<uint32_t> vertexIds;
      fcl::AABBd bounds;
    };
    std::string name;
    std::vector<uint32_t> vertexIds;
    std::vector<fcl::Triangle> triangles;
    std::vector<fcl::Vector3d> vertices;
    fcl::AABBd bounds;
    std::vector<Patch> patches;
  };
  std::vector<Region> regions_;
  std::vector<std::shared_ptr<SmplhBVH>> humanModels_;
  std::uint32_t humanModelFrame_ = UINT32_MAX;
  std::vector<SmplhRobotGeom> robots_;
  std::vector<int> robotIds_;
  bool useSch_ = false;
  bool approximateSch_ = false;
  size_t patchFaces_ = 64;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::unique_ptr<SmplhQueryInput> pending_;
  SmplhDistanceSnapshot completed_;
  bool stopping_ = false;
  std::thread thread_;

  static std::shared_ptr<SmplhCollisionGeometry> robotGeometry(const mjModel * m, int id)
  {
    const auto * s = m->geom_size + 3 * id;
    switch(m->geom_type[id])
    {
      case mjGEOM_SPHERE: return std::make_shared<fcl::Sphered>(s[0]);
      case mjGEOM_CAPSULE: return std::make_shared<fcl::Capsuled>(s[0], 2 * s[1]);
      case mjGEOM_CYLINDER: return std::make_shared<fcl::Cylinderd>(s[0], 2 * s[1]);
      case mjGEOM_BOX: return std::make_shared<fcl::Boxd>(2 * s[0], 2 * s[1], 2 * s[2]);
      case mjGEOM_MESH:
      {
        int mesh = m->geom_dataid[id];
        int va = m->mesh_vertadr[mesh], nv = m->mesh_vertnum[mesh];
        int fa = m->mesh_faceadr[mesh], nf = m->mesh_facenum[mesh];
        std::vector<fcl::Vector3d> vertices; vertices.reserve(nv);
        for(int i = 0; i < nv; ++i) vertices.emplace_back(m->mesh_vert[3*(va+i)],m->mesh_vert[3*(va+i)+1],m->mesh_vert[3*(va+i)+2]);
        std::vector<fcl::Triangle> faces; faces.reserve(nf);
        for(int i = 0; i < nf; ++i) faces.emplace_back(m->mesh_face[3*(fa+i)],m->mesh_face[3*(fa+i)+1],m->mesh_face[3*(fa+i)+2]);
        auto bvh = std::make_shared<SmplhBVH>();
        bvh->beginModel(nf,nv); bvh->addSubModel(vertices,faces); bvh->endModel(); return bvh;
      }
      default: throw std::runtime_error("Unsupported robot distance geom type");
    }
  }

  static std::unique_ptr<sch::S_Object> schRobotGeometry(const mjModel * m, int id)
  {
    const auto * s = m->geom_size + 3 * id;
    const sch::Point3 lower(0, 0, -s[1]), upper(0, 0, s[1]);
    switch(m->geom_type[id])
    {
      case mjGEOM_SPHERE: return std::make_unique<sch::S_Sphere>(s[0]);
      case mjGEOM_CAPSULE: return std::make_unique<sch::S_Capsule>(lower, upper, s[0]);
      case mjGEOM_CYLINDER: return std::make_unique<sch::S_Cylinder>(lower, upper, s[0]);
      case mjGEOM_BOX: return std::make_unique<sch::S_Box>(2*s[0], 2*s[1], 2*s[2]);
      case mjGEOM_MESH:
      {
        const int mesh = m->geom_dataid[id];
        const int va = m->mesh_vertadr[mesh], nv = m->mesh_vertnum[mesh];
        auto vertices = std::make_shared<std::vector<fcl::Vector3d>>();
        vertices->reserve(nv);
        for(int i = 0; i < nv; ++i)
          vertices->emplace_back(m->mesh_vert[3*(va+i)], m->mesh_vert[3*(va+i)+1],
                                 m->mesh_vert[3*(va+i)+2]);
        return std::make_unique<SmplhSchVertexCloud>(vertices);
      }
      default: return nullptr;
    }
  }

  void buildPatches(Region & region)
  {
    std::unordered_map<uint64_t, std::vector<uint32_t>> edgeFaces;
    for(uint32_t i = 0; i < region.triangles.size(); ++i)
    {
      const auto & face = region.triangles[i];
      for(int edge = 0; edge < 3; ++edge)
      {
        const auto a = std::min(face[edge], face[(edge+1)%3]);
        const auto b = std::max(face[edge], face[(edge+1)%3]);
        edgeFaces[(static_cast<uint64_t>(a) << 32) | b].push_back(i);
      }
    }
    std::vector<std::vector<uint32_t>> neighbors(region.triangles.size());
    for(const auto & entry : edgeFaces)
      for(uint32_t a : entry.second)
        for(uint32_t b : entry.second)
          if(a != b) neighbors[a].push_back(b);
    std::vector<bool> assigned(region.triangles.size(), false);
    std::vector<uint32_t> queued(region.triangles.size(), 0);
    std::vector<uint32_t> seenVertex(region.vertexIds.size(), 0);
    uint32_t epoch = 0;
    for(uint32_t seed = 0; seed < region.triangles.size(); ++seed)
    {
      if(assigned[seed]) continue;
      ++epoch;
      std::deque<uint32_t> queue{seed};
      queued[seed] = epoch;
      Region::Patch patch;
      size_t faces = 0;
      while(!queue.empty() && faces < patchFaces_)
      {
        const uint32_t faceId = queue.front(); queue.pop_front();
        if(assigned[faceId]) continue;
        assigned[faceId] = true;
        ++faces;
        const auto & face = region.triangles[faceId];
        for(int corner = 0; corner < 3; ++corner)
        {
          const uint32_t vertex = face[corner];
          if(seenVertex[vertex] != epoch)
          { seenVertex[vertex] = epoch; patch.vertexIds.push_back(vertex); }
        }
        for(uint32_t adjacent : neighbors[faceId])
          if(!assigned[adjacent] && queued[adjacent] != epoch)
          { queued[adjacent] = epoch; queue.push_back(adjacent); }
      }
      region.patches.push_back(std::move(patch));
    }
  }

  void queryApproximate(SmplhDistanceSnapshot & snapshot,
                        const std::vector<std::unique_ptr<fcl::CollisionObjectd>> & robotObjects) const
  {
    snapshot.approximate = true;
    for(const auto & region : regions_)
    {
      struct Candidate { double bound; size_t patch; size_t robot; };
      std::vector<Candidate> candidates;
      candidates.reserve(region.patches.size() * robots_.size());
      for(size_t patch = 0; patch < region.patches.size(); ++patch)
        for(size_t robot = 0; robot < robots_.size(); ++robot)
          candidates.push_back({region.patches[patch].bounds.distance(robotObjects[robot]->getAABB()),
                                patch, robot});
      std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
        if(a.bound != b.bound) return a.bound < b.bound;
        if(a.robot != b.robot) return a.robot < b.robot;
        return a.patch < b.patch;
      });
      bool have = false;
      double bestDistance = std::numeric_limits<double>::infinity();
      SmplhRegionDistance best;
      for(const auto & entry : candidates)
      {
        if(have && entry.bound > bestDistance + 1e-9) break;
        const auto & robot = robots_[entry.robot];
        SmplhSchVertexCloud hull(&region.vertices, &region.patches[entry.patch].vertexIds);
        sch::CD_Pair pair(&hull, robot.sch_geometry.get());
        pair.setRelativePrecision(1e-6);
        const auto start = std::chrono::steady_clock::now();
        const double squared = pair.getDistanceWithoutPenetrationDepth();
        ++snapshot.queried_pairs;
        if(!std::isfinite(squared)) throw std::runtime_error("SCH approximate distance failed");
        SmplhRegionDistance candidate;
        candidate.region = region.name;
        candidate.robot_geometry = robot.name;
        if(squared <= 0)
        { candidate.distance = 0; candidate.intersecting = true; }
        else
        {
          candidate.distance = std::sqrt(squared);
          if(candidate.distance <= bestDistance)
          {
            sch::Point3 humanPoint, robotPoint;
            const double witnessSquared = pair.getClosestPoints(humanPoint, robotPoint);
            if(!std::isfinite(witnessSquared) || witnessSquared <= 0)
              throw std::runtime_error("SCH approximate witness query failed");
            candidate.distance = std::sqrt(witnessSquared);
            candidate.human_point = {humanPoint.m_x, humanPoint.m_y, humanPoint.m_z};
            candidate.robot_point = {robotPoint.m_x, robotPoint.m_y, robotPoint.m_z};
            candidate.closest_points_valid = true;
            Eigen::Vector3d delta(robotPoint.m_x-humanPoint.m_x,
                                  robotPoint.m_y-humanPoint.m_y,
                                  robotPoint.m_z-humanPoint.m_z);
            if(delta.norm() > 1e-12)
            { delta.normalize(); candidate.direction = {delta.x(),delta.y(),delta.z()}; candidate.direction_valid = true; }
          }
        }
        snapshot.sch_duration_seconds +=
            std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        if(!have || candidate.distance < bestDistance)
        { have = true; bestDistance = candidate.distance; best = std::move(candidate); }
        if(bestDistance == 0) break;
      }
      if(have) snapshot.regions.push_back(std::move(best));
    }
  }

  SmplhDistanceSnapshot query(const SmplhQueryInput & input)
  {
    const auto preparationStart = std::chrono::steady_clock::now();
    SmplhDistanceSnapshot snapshot;
    if(input.frame != humanModelFrame_)
    {
      for(size_t i=0;i<regions_.size();++i)
      {
        auto & region = regions_[i];
        for(size_t v=0; v<region.vertexIds.size(); ++v)
          region.vertices[v] = input.vertices.at(region.vertexIds[v]);
        region.bounds = fcl::AABBd(region.vertices.front());
        for(const auto & vertex : region.vertices) region.bounds += vertex;
        if(approximateSch_)
        {
          for(auto & patch : region.patches)
          {
            patch.bounds = fcl::AABBd(region.vertices[patch.vertexIds.front()]);
            for(uint32_t vertex : patch.vertexIds) patch.bounds += region.vertices[vertex];
          }
        }
        else if(i == humanModels_.size())
        {
          auto model=std::make_shared<SmplhBVH>();
          model->beginModel(static_cast<int>(region.triangles.size()),static_cast<int>(region.vertices.size()));
          model->addSubModel(region.vertices,region.triangles); model->endModel(); humanModels_.push_back(std::move(model));
        }
        else
        {
          // These are instantaneous queries, not continuous collision checks.
          // Replacement refits bounds to this pose only; update would also
          // include the previous pose and weaken pruning across motion frames.
          auto & model=*humanModels_[i];
          model.beginReplaceModel();
          model.replaceSubModel(region.vertices);
          // Fit each node directly from its current primitives. FCL 0.7.0's
          // bottom-up merging disagreed with fresh BVHs on deforming
          // meshes and caused seconds-long distance traversals in replay tests.
          model.endReplaceModel(/* refit = */ true, /* bottomup = */ false);
        }
      }
      humanModelFrame_=input.frame;
    }
    snapshot.ready = true; snapshot.sequence = input.sequence; snapshot.motion_frame = input.frame;
    snapshot.sample_simulation_time = input.time;
    // Build current-pose robot objects once, rather than once per region.
    std::vector<std::unique_ptr<fcl::CollisionObjectd>> robotObjects;
    for(size_t i = 0; i < robots_.size(); ++i)
    {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.linear() = input.robot_poses[i].rotation;
      transform.translation() = input.robot_poses[i].translation;
      robotObjects.emplace_back(std::make_unique<fcl::CollisionObjectd>(robots_[i].geometry, transform));
      if(auto * schObject = robots_[i].sch_geometry.get())
      {
        const auto & r = input.robot_poses[i].rotation;
        const auto & t = input.robot_poses[i].translation;
        schObject->setOrientation(r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2));
        schObject->setPosition(t.x(), t.y(), t.z());
      }
    }
    snapshot.total_pairs = 0;
    for(const auto & region : regions_)
      snapshot.total_pairs += (approximateSch_ ? region.patches.size() : 1) * robots_.size();
    snapshot.preparation_duration_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now()-preparationStart).count();
    if(approximateSch_)
    {
      queryApproximate(snapshot, robotObjects);
      return snapshot;
    }
    for(size_t regionIndex=0; regionIndex<regions_.size(); ++regionIndex)
    {
      const auto & region=regions_[regionIndex];
      fcl::CollisionObjectd human(humanModels_[regionIndex]);
      SmplhSchVertexCloud schHuman(&region.vertices);
      std::vector<std::pair<double, size_t>> candidates;
      for(size_t i = 0; i < robots_.size(); ++i)
        candidates.emplace_back(region.bounds.distance(robotObjects[i]->getAABB()), i);
      std::sort(candidates.begin(), candidates.end());
      bool have = false; SmplhRegionDistance best; double bestDistance = std::numeric_limits<double>::infinity();
      size_t bestIndex = robots_.size();
      for(const auto & entry : candidates)
      {
        // Small tolerance keeps borderline floating-point comparisons conservative.
        if(have && entry.first > bestDistance + 1e-9) break;
        const size_t i = entry.second;
        const auto & robot = robots_[i];
        auto & object = *robotObjects[i];
        if(useSch_ && have && robot.sch_geometry)
        {
          const auto schStart = std::chrono::steady_clock::now();
          sch::CD_Pair pair(&schHuman, robot.sch_geometry.get());
          pair.setRelativePrecision(1e-6);
          const double squaredLowerBound = pair.getDistanceWithoutPenetrationDepth();
          snapshot.sch_duration_seconds +=
              std::chrono::duration<double>(std::chrono::steady_clock::now()-schStart).count();
          // SCH's iterative tolerance needs a margin before rejecting an exact query.
          if(std::isfinite(squaredLowerBound) && squaredLowerBound > 0
             && std::sqrt(squaredLowerBound) > bestDistance + 1e-4)
          { ++snapshot.sch_pruned_pairs; continue; }
        }
        ++snapshot.queried_pairs;
        fcl::CollisionRequestd collisionRequest; fcl::CollisionResultd collisionResult;
        const auto collisionStart = std::chrono::steady_clock::now();
        bool intersecting = fcl::collide(&human, &object, collisionRequest, collisionResult) > 0;
        snapshot.collision_duration_seconds +=
            std::chrono::duration<double>(std::chrono::steady_clock::now()-collisionStart).count();
        SmplhRegionDistance candidate; candidate.region = region.name; candidate.robot_geometry = robot.name;
        if(intersecting) { candidate.distance = 0; candidate.intersecting = true; }
        else
        {
          fcl::DistanceRequestd request(true); fcl::DistanceResultd result;
          const auto distanceStart = std::chrono::steady_clock::now();
          candidate.distance = fcl::distance(&human, &object, request, result);
          snapshot.distance_duration_seconds +=
              std::chrono::duration<double>(std::chrono::steady_clock::now()-distanceStart).count();
          if(!std::isfinite(candidate.distance) || candidate.distance < 0) continue;
          auto ph = result.nearest_points[0], pr = result.nearest_points[1];
          candidate.human_point = {ph.x(),ph.y(),ph.z()}; candidate.robot_point = {pr.x(),pr.y(),pr.z()};
          candidate.closest_points_valid = true;
          Eigen::Vector3d delta = pr - ph;
          if(delta.norm() > 1e-12) { delta.normalize(); candidate.direction = {delta.x(),delta.y(),delta.z()}; candidate.direction_valid = true; }
          if(result.b1 >= 0 && static_cast<size_t>(result.b1) < region.triangles.size())
          {
            const auto & face = region.triangles[result.b1];
            Eigen::Vector3d n = (region.vertices[face[1]] - region.vertices[face[0]]).cross(region.vertices[face[2]] - region.vertices[face[0]]);
            if(n.norm() > 1e-12) { n.normalize(); candidate.human_normal = {n.x(),n.y(),n.z()}; candidate.human_normal_valid = true; }
          }
        }
        if(!have || candidate.distance < bestDistance || (candidate.distance == bestDistance && i < bestIndex))
        { have=true; bestDistance=candidate.distance; bestIndex=i; best=std::move(candidate); }
        if(bestDistance == 0) break;
      }
      if(have) snapshot.regions.push_back(std::move(best));
    }
    return snapshot;
  }

  void run()
  {
    while(true)
    {
      std::unique_ptr<SmplhQueryInput> input;
      { std::unique_lock<std::mutex> lock(mutex_); cv_.wait(lock,[&]{return stopping_ || pending_;});
        if(stopping_) return; input=std::move(pending_); }
      SmplhDistanceSnapshot result;
      const auto start = std::chrono::steady_clock::now();
      try { result=query(*input); }
      catch(const std::exception & e)
      {
        result.sequence=input->sequence; result.motion_frame=input->frame;
        result.sample_simulation_time=input->time; result.error=e.what();
      }
      result.worker_duration_seconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      { std::lock_guard<std::mutex> lock(mutex_); if(result.sequence >= completed_.sequence) completed_=std::move(result); }
    }
  }

public:
  SmplhDistanceWorker() = default;
  SmplhDistanceWorker(const SmplhDistanceWorker &) = delete;
  ~SmplhDistanceWorker() { {std::lock_guard<std::mutex> lock(mutex_); stopping_=true;} cv_.notify_one(); if(thread_.joinable()) thread_.join(); }
  void configure(const mjModel * m, const std::vector<std::vector<uint32_t>> & regionFaces,
                 const std::vector<std::string> & regionNames, const uint32_t * faces,
                 const char * prefix="kinova_", int group=2)
  {
    if(regionFaces.size()!=regionNames.size()) throw std::runtime_error("SMPL-H region metadata mismatch");
    const char * schSetting = std::getenv("SMPLH_DISTANCE_USE_SCH");
    useSch_ = schSetting && std::string(schSetting) == "1";
    const char * mode = std::getenv("SMPLH_DISTANCE_MODE");
    if(mode && std::string(mode) != "fcl" && std::string(mode) != "sch_approx")
      throw std::runtime_error("SMPLH_DISTANCE_MODE must be fcl or sch_approx");
    approximateSch_ = !mode || std::string(mode) == "sch_approx";
    if(approximateSch_)
    {
      useSch_ = false;
      const char * patchSetting = std::getenv("SMPLH_SCH_PATCH_FACES");
      if(patchSetting)
      {
        char * end = nullptr;
        const long value = std::strtol(patchSetting, &end, 10);
        if(end == patchSetting || *end || value < 1 || value > 100000)
          throw std::runtime_error("Invalid SMPLH_SCH_PATCH_FACES");
        patchFaces_ = static_cast<size_t>(value);
      }
    }
    for(size_t r=0;r<regionNames.size();++r)
    {
      Region region; region.name=regionNames[r];
      std::unordered_map<uint32_t, uint32_t> localIds;
      for(uint32_t faceId:regionFaces[r])
      {
        std::array<uint32_t,3> local;
        for(size_t corner=0; corner<3; ++corner)
        {
          const uint32_t global = faces[3*faceId+corner];
          auto entry = localIds.emplace(global, static_cast<uint32_t>(region.vertexIds.size()));
          if(entry.second) region.vertexIds.push_back(global);
          local[corner] = entry.first->second;
        }
        region.triangles.emplace_back(local[0],local[1],local[2]);
      }
      if(region.triangles.empty()) throw std::runtime_error("Empty SMPL-H region: "+region.name);
      region.vertices.resize(region.vertexIds.size());
      if(approximateSch_) buildPatches(region);
      regions_.push_back(std::move(region));
    }
    std::string wanted=prefix?prefix:"";
    for(int i=0;i<m->ngeom;++i)
    {
      const char * body=mj_id2name(m,mjOBJ_BODY,m->geom_bodyid[i]);
      if(!body || m->geom_group[i]!=group || (!wanted.empty() && std::string(body).compare(0,wanted.size(),wanted)!=0)) continue;
      const char * geom=mj_id2name(m,mjOBJ_GEOM,i);
      robots_.push_back({geom?geom:std::to_string(i),robotGeometry(m,i),
                         (useSch_ || approximateSch_) ? schRobotGeometry(m,i) : nullptr});
      if(approximateSch_ && !robots_.back().sch_geometry)
        throw std::runtime_error("SCH approximate mode does not support selected robot geom");
      robotIds_.push_back(i);
    }
    if(robots_.empty()) throw std::runtime_error("No robot geometries match SMPLH_ROBOT_PREFIX/GROUP");
    thread_=std::thread(&SmplhDistanceWorker::run,this);
  }
  void submit(std::uint64_t seq,uint32_t frame,double time,const float * vertices,uint32_t nvertices,
              const mjData * data,const mjModel * model,int humanSurfaceGeom)
  {
    auto input=std::make_unique<SmplhQueryInput>(); input->sequence=seq; input->frame=frame; input->time=time;
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> humanRotation(data->geom_xmat+9*humanSurfaceGeom);
    Eigen::Map<const Eigen::Vector3d> humanTranslation(data->geom_xpos+3*humanSurfaceGeom);
    input->vertices.reserve(nvertices);
    for(uint32_t i=0;i<nvertices;++i)
    {
      Eigen::Vector3d local(vertices[3*i],vertices[3*i+1],vertices[3*i+2]);
      input->vertices.emplace_back(humanRotation * local + humanTranslation);
    }
    input->robot_poses.reserve(robots_.size());
    for(size_t j=0;j<robots_.size();++j)
    {
      int id=robotIds_[j]; SmplhRobotPose pose;
      Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> rotation(data->geom_xmat+9*id);
      pose.rotation=rotation; pose.translation=Eigen::Map<const Eigen::Vector3d>(data->geom_xpos+3*id);
      input->robot_poses.push_back(pose);
    }
    {std::lock_guard<std::mutex> lock(mutex_); pending_=std::move(input);} cv_.notify_one();
  }
  SmplhDistanceSnapshot latest() const
  {
    auto * self=const_cast<SmplhDistanceWorker *>(this);
    std::lock_guard<std::mutex> lock(self->mutex_); return self->completed_;
  }
};
} // namespace mc_mujoco
