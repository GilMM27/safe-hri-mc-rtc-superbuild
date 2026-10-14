#pragma once
#include <mujoco/mujoco.h>
#include "SmplhDistanceWorker.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

// Native deterministic playback of smplh_cache.bin. The cache mapping is read-only;
// MuJoCo receives copied qpos and display vertices on the simulation thread.
class SmplhBridge
{
#pragma pack(push, 1)
  struct Header { char magic[8]; uint32_t frames, vertices, faces, regions, nq; double fps; };
#pragma pack(pop)
  int fd_ = -1, mesh_ = -1, surfaceGeom_ = -1;
  void * mapping_ = MAP_FAILED;
  size_t bytes_ = 0;
  const float * qpos_ = nullptr;
  const float * vertices_ = nullptr;
  const uint32_t * faces_ = nullptr;
  uint32_t frameCount_ = 0, vertexCount_ = 0, faceCount_ = 0, frame_ = UINT32_MAX;
  double fps_ = 0;
  bool loop_ = false, dirty_ = false, initialized_ = false;
  std::vector<int> joints_;
  std::vector<std::string> regionNames_;
  std::vector<std::vector<uint32_t>> regionFaceIds_;
  mc_mujoco::SmplhDistanceWorker distanceWorker_;
  std::uint64_t sequence_ = 0;
  unsigned int steps_ = 0, distanceEvery_ = 25;
  bool distanceEnabled_ = true;

  static int find(const mjModel * m, mjtObj type, const std::string & name)
  {
    int count = type == mjOBJ_JOINT ? m->njnt : m->ngeom;
    for(int i = 0; i < count; ++i)
    {
      const char * raw = mj_id2name(m, type, i);
      if(!raw) continue;
      const std::string candidate(raw);
      if(candidate == name || (candidate.size() > name.size()
          && candidate.compare(candidate.size() - name.size(), name.size(), name) == 0
          && candidate[candidate.size() - name.size() - 1] == '_')) return i;
    }
    throw std::runtime_error("Missing SMPL-H model element: " + name);
  }
  void initialize(mjModel * m)
  {
    if(initialized_) return;
    initialized_ = true;
    const char * path = std::getenv("SMPLH_CACHE");
    if(!path) return;
    struct stat pathStat{};
    std::string cachePath(path);
    if(!stat(path, &pathStat) && S_ISDIR(pathStat.st_mode)) cachePath += "/smplh_cache.bin";
    fd_ = ::open(cachePath.c_str(), O_RDONLY);
    if(fd_ < 0) throw std::runtime_error("Cannot open SMPLH_CACHE");
    struct stat st{};
    if(fstat(fd_, &st) || st.st_size < static_cast<off_t>(sizeof(Header))) throw std::runtime_error("Invalid SMPL-H cache");
    bytes_ = static_cast<size_t>(st.st_size);
    mapping_ = mmap(nullptr, bytes_, PROT_READ, MAP_PRIVATE, fd_, 0);
    if(mapping_ == MAP_FAILED) throw std::runtime_error("Cannot map SMPL-H cache");
    const auto * h = static_cast<const Header *>(mapping_);
    if(std::memcmp(h->magic, "SMPLHC1\0", 8) || h->nq != 211 || !h->frames || !h->vertices || !h->faces
       || !(h->fps > 0) || !std::isfinite(h->fps)) throw std::runtime_error("Unsupported or invalid SMPL-H cache header");
    frameCount_ = h->frames; vertexCount_ = h->vertices; faceCount_ = h->faces; fps_ = h->fps;
    const char * cursor = static_cast<const char *>(mapping_) + sizeof(Header);
    const char * end = static_cast<const char *>(mapping_) + bytes_;
    for(uint32_t r = 0; r < h->regions; ++r)
    {
      if(end - cursor < 68) throw std::runtime_error("Truncated SMPL-H region table");
      char name[65] = {}; std::memcpy(name, cursor, 64);
      uint32_t count; std::memcpy(&count, cursor + 64, sizeof(count)); cursor += 68;
      if(count > static_cast<uint64_t>(end - cursor) / 4) throw std::runtime_error("Truncated SMPL-H face region");
      regionNames_.emplace_back(name);
      std::vector<uint32_t> ids(count); std::memcpy(ids.data(), cursor, 4ull * count); cursor += 4 * count;
      for(auto id : ids) if(id >= faceCount_) throw std::runtime_error("SMPL-H region face index out of range");
      regionFaceIds_.push_back(std::move(ids));
    }
    const size_t required = static_cast<size_t>(frameCount_) * (211 + 3ull * vertexCount_) * 4
                            + static_cast<size_t>(faceCount_) * 3 * 4;
    if(required != static_cast<size_t>(end - cursor)) throw std::runtime_error("SMPL-H cache size mismatch");
    qpos_ = reinterpret_cast<const float *>(cursor);
    vertices_ = qpos_ + static_cast<size_t>(frameCount_) * 211;
    faces_ = reinterpret_cast<const uint32_t *>(vertices_ + static_cast<size_t>(frameCount_) * vertexCount_ * 3);
    const char * every = std::getenv("SMPLH_DISTANCE_EVERY");
    if(every) distanceEvery_ = std::max(1, std::atoi(every));
    int geom = find(m, mjOBJ_GEOM, "smplh_surface");
    surfaceGeom_ = geom;
    mesh_ = m->geom_dataid[geom];
    if(mesh_ < 0 || static_cast<uint32_t>(m->mesh_vertnum[mesh_]) != vertexCount_
       || static_cast<uint32_t>(m->mesh_normalnum[mesh_]) != vertexCount_)
      throw std::runtime_error("SMPL-H mesh topology does not match cache");
    // Cache vertices are already expressed in world coordinates. MuJoCo's OBJ
    // compiler recenters mesh assets and stores compensating geom/mesh poses;
    // clear those transforms so the cache coordinates are not shifted again.
    mju_zero(m->geom_pos + 3 * geom, 3);
    mju_zero(m->geom_quat + 4 * geom, 4);
    m->geom_quat[4 * geom] = 1;
    mju_zero(m->mesh_pos + 3 * mesh_, 3);
    mju_zero(m->mesh_quat + 4 * mesh_, 4);
    m->mesh_quat[4 * mesh_] = 1;
    for(int i = 0; i < 52; ++i) joints_.push_back(find(m, mjOBJ_JOINT, i ? "smplh_joint_" + std::to_string(i) : "smplh_root"));
    loop_ = std::getenv("SMPLH_LOOP") && std::string(std::getenv("SMPLH_LOOP")) == "1";
    const char * prefix = std::getenv("SMPLH_ROBOT_PREFIX");
    const char * groupText = std::getenv("SMPLH_ROBOT_GROUP");
    int group = groupText ? std::atoi(groupText) : 2;
    distanceEnabled_ = !(std::getenv("SMPLH_DISTANCE_DISABLE")
                         && std::string(std::getenv("SMPLH_DISTANCE_DISABLE")) == "1");
    if(distanceEnabled_) distanceWorker_.configure(m, regionFaceIds_, regionNames_, faces_, prefix ? prefix : "kinova_", group);
  }

public:
  SmplhBridge() = default;
  SmplhBridge(const SmplhBridge &) = delete;
  ~SmplhBridge() { if(mapping_ != MAP_FAILED) munmap(mapping_, bytes_); if(fd_ >= 0) close(fd_); }
  void update(mjModel * m, mjData * d)
  {
    initialize(m);
    if(mapping_ == MAP_FAILED) return;
    if(!std::isfinite(d->time) || d->time < 0) return;
    uint64_t index = static_cast<uint64_t>(std::floor(d->time * fps_ + 1e-9));
    uint32_t next = loop_ ? index % frameCount_ : static_cast<uint32_t>(std::min<uint64_t>(index, frameCount_ - 1));
    const float * v = vertices_ + static_cast<size_t>(next) * vertexCount_ * 3;
    if(next != frame_)
    {
    const float * pose = qpos_ + static_cast<size_t>(next) * 211;
    for(size_t j = 0, offset = 0; j < joints_.size(); ++j)
    {
      int id = joints_[j], nq = j ? 4 : 7, nv = j ? 3 : 6;
      for(int k = 0; k < nq; ++k) d->qpos[m->jnt_qposadr[id] + k] = pose[offset + k];
      mju_zero(d->qvel + m->jnt_dofadr[id], nv);
      offset += nq;
    }
    int va = m->mesh_vertadr[mesh_], na = m->mesh_normaladr[mesh_];
    std::copy(v, v + 3 * vertexCount_, m->mesh_vert + 3 * va);
    std::vector<mjtNum> normals(3ull * vertexCount_, 0);
    for(uint32_t f = 0; f < faceCount_; ++f)
    {
      uint32_t a=faces_[3*f], b=faces_[3*f+1], c=faces_[3*f+2];
      if(a>=vertexCount_ || b>=vertexCount_ || c>=vertexCount_) throw std::runtime_error("SMPL-H face index out of range");
      mjtNum e1[3], e2[3], n[3];
      for(int k=0;k<3;++k) { e1[k]=v[3*b+k]-v[3*a+k]; e2[k]=v[3*c+k]-v[3*a+k]; }
      mju_cross(n,e1,e2);
      for(int k=0;k<3;++k) { normals[3*a+k]+=n[k]; normals[3*b+k]+=n[k]; normals[3*c+k]+=n[k]; }
    }
    for(uint32_t i=0;i<vertexCount_;++i) mju_normalize3(normals.data()+3*i);
    std::copy(normals.begin(), normals.end(), m->mesh_normal + 3 * na);
    mj_forward(m,d); frame_=next; dirty_=true;
    }
    if(distanceEnabled_ && (++steps_ % distanceEvery_ == 0 || sequence_ == 0))
      distanceWorker_.submit(++sequence_, next, d->time, v, vertexCount_, d, m, surfaceGeom_);
  }
  mc_mujoco::SmplhDistanceSnapshot snapshot() const { return distanceWorker_.latest(); }
  void renderDistanceOverlay(mjvScene * scene) const
  {
    if(!distanceEnabled_ || !scene) return;
    const auto distances = distanceWorker_.latest();
    if(!distances.ready) return;
    const mjtNum geomSize[3] = {0.012, 0.024, 0.03};
    const mjtNum identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const mjtNum origin[3] = {0, 0, 0};
    const float humanColor[4] = {0.1f, 0.9f, 1.0f, 1.0f};
    const float robotColor[4] = {1.0f, 0.55f, 0.1f, 1.0f};
    const float arrowColor[4] = {0.25f, 1.0f, 0.35f, 1.0f};
    for(const auto & region : distances.regions)
    {
      if(!region.closest_points_valid || scene->ngeom + 3 > scene->maxgeom) continue;
      mjtNum human[3], robot[3];
      for(int axis = 0; axis < 3; ++axis)
      {
        human[axis] = region.human_point[axis];
        robot[axis] = region.robot_point[axis];
      }
      const int humanId = scene->ngeom++;
      mjv_initGeom(&scene->geoms[humanId], mjGEOM_SPHERE, geomSize, human, identity, humanColor);
      scene->geoms[humanId].category = mjCAT_DECOR;
      scene->geomorder[humanId] = humanId;
      const int robotId = scene->ngeom++;
      mjv_initGeom(&scene->geoms[robotId], mjGEOM_SPHERE, geomSize, robot, identity, robotColor);
      scene->geoms[robotId].category = mjCAT_DECOR;
      scene->geomorder[robotId] = robotId;
      const int arrowId = scene->ngeom++;
      mjv_initGeom(&scene->geoms[arrowId], mjGEOM_ARROW, geomSize, origin, identity, arrowColor);
      mjv_connector(&scene->geoms[arrowId], mjGEOM_ARROW, 0.006, human, robot);
      scene->geoms[arrowId].category = mjCAT_DECOR;
      scene->geomorder[arrowId] = arrowId;
    }
  }
  void upload(const mjModel * m, mjrContext * context)
  {
    if(dirty_) { mjr_uploadMesh(m, context, mesh_); dirty_=false; }
  }
};
