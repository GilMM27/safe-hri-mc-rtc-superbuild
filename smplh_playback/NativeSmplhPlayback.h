#pragma once
#include <mujoco/mujoco.h>
#include "SmplhDistanceWorker.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
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
  int surfaceBody_ = -1;
  void * mapping_ = MAP_FAILED;
  size_t bytes_ = 0;
  const float * qpos_ = nullptr;
  const float * vertices_ = nullptr;
  const uint32_t * faces_ = nullptr;
  uint32_t frameCount_ = 0, vertexCount_ = 0, faceCount_ = 0, frame_ = UINT32_MAX;
  double fps_ = 0;
  bool loop_ = false, dirty_ = false, initialized_ = false;
  int freezeFrame_ = -1;
  std::vector<int> joints_;
  std::vector<std::string> regionNames_;
  std::vector<std::vector<uint32_t>> regionFaceIds_;
  mc_mujoco::SmplhDistanceWorker distanceWorker_;
  std::uint64_t sequence_ = 0;
  unsigned int steps_ = 0, distanceEvery_ = 25;
  bool distanceEnabled_ = true;
  bool contactsEnabled_ = true;
  mjData * contactData_ = nullptr;
  std::vector<std::pair<int, int>> contactBodies_; // mocap id, skeleton body id
  std::map<std::string, std::array<double, 2>> contactSizeScales_;
  std::map<std::string, std::array<double, 3>> contactOffsetSettings_;
  std::map<int, std::array<double, 3>> contactWorldOffsetsByBody_;
  std::vector<std::array<mjtNum, 7>> previousMocap_;
  bool havePreviousMocap_ = false;
  int lastContactCount_ = 0;
  double lastContactDepth_ = 0;

  void configureContacts(mjModel * m)
  {
    const char * setting = std::getenv("SMPLH_CONTACT_MODE");
    const std::string mode = setting ? setting : "kinematic";
    if(mode != "kinematic" && mode != "off")
      throw std::runtime_error("SMPLH_CONTACT_MODE must be kinematic or off");
    contactsEnabled_ = mode == "kinematic";
    if(!contactsEnabled_) return;
    // Reserve a bit unused anywhere in this scene, retaining existing robot contacts.
    int used = 0;
    for(int g = 0; g < m->ngeom; ++g) used |= m->geom_contype[g] | m->geom_conaffinity[g];
    int bit = 0;
    for(int b = 29; b >= 0; --b) if(!(used & (1 << b))) { bit = 1 << b; break; }
    if(!bit) throw std::runtime_error("No free collision mask bit for SMPL-H contacts");
    std::vector<int> geoms;
    try
    {
      for(int joint : {0, 15, 16, 18, 20, 17, 19, 21, 1, 4, 7, 2, 5, 8})
      {
        int geom = find(m, mjOBJ_GEOM, "smplh_contact_geom_" + std::to_string(joint));
        int body = m->geom_bodyid[geom];
        int mocap = m->body_mocapid[body];
        if(mocap < 0) throw std::runtime_error("Contact proxy is not a mocap body");
        if(m->geom_type[geom] != mjGEOM_CAPSULE)
          throw std::runtime_error("Contact proxy is not a capsule: run smplh_playback.refit_contacts on this cache");
        contactBodies_.emplace_back(mocap, m->jnt_bodyid[joints_[joint]]);
        geoms.push_back(geom);
      }
    }
    catch(const std::exception & e)
    {
      throw std::runtime_error(std::string("SMPL-H contact proxies missing or invalid: ") + e.what()
                               + "; regenerate the cache with smplh_playback.prepare, refit it with "
                                 "smplh_playback.refit_contacts, or set SMPLH_CONTACT_MODE=off");
    }
    const std::map<std::string, int> regionJoints = {
      {"torso", 0}, {"head", 15},
      {"left_upper_arm", 16}, {"left_forearm", 18}, {"left_hand", 20},
      {"right_upper_arm", 17}, {"right_forearm", 19}, {"right_hand", 21},
      {"left_thigh", 1}, {"left_lower_leg", 4}, {"left_foot", 7},
      {"right_thigh", 2}, {"right_lower_leg", 5}, {"right_foot", 8}};
    for(const auto & [name, offset] : contactOffsetSettings_)
      contactWorldOffsetsByBody_[m->jnt_bodyid[joints_[regionJoints.at(name)]]] = offset;
    for(const auto & [name, scales] : contactSizeScales_)
    {
      int geom = find(m, mjOBJ_GEOM, "smplh_contact_geom_" + std::to_string(regionJoints.at(name)));
      if(m->geom_type[geom] != mjGEOM_CAPSULE)
        throw std::runtime_error("SMPL-H contact region " + name + " is not a capsule; refit the cache");
      m->geom_size[3 * geom] *= scales[0];
      m->geom_size[3 * geom + 1] *= scales[1];
      // Broad-phase collision detection uses these compiled bounds separately
      // from geom_size. Recompute them when a controller enlarges a proxy.
      const mjtNum * size = m->geom_size + 3 * geom;
      mjtNum * bounds = m->geom_aabb + 6 * geom;
      bounds[3] = size[0];
      bounds[4] = size[0];
      bounds[5] = size[0] + size[1];
      m->geom_rbound[geom] = size[0] + size[1];
    }
    const char * prefixSetting = std::getenv("SMPLH_ROBOT_PREFIX");
    const std::string prefix = prefixSetting ? prefixSetting : "kinova_";
    int robots = 0;
    for(int g = 0; g < m->ngeom; ++g)
    {
      const char * body = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g]);
      if(body && std::string(body).compare(0, prefix.size(), prefix) == 0 && (m->geom_contype[g] || m->geom_conaffinity[g]))
      {
        m->geom_contype[g] |= bit;
        m->body_contype[m->geom_bodyid[g]] |= bit;
        ++robots;
      }
    }
    if(!robots) throw std::runtime_error("No collision-enabled robot geometries match SMPLH_ROBOT_PREFIX for contacts");
    const char * debug = std::getenv("SMPLH_CONTACT_DEBUG");
    for(int geom : geoms)
    {
      m->geom_conaffinity[geom] = bit;
      m->body_conaffinity[m->geom_bodyid[geom]] |= bit;
      if(debug && std::string(debug) == "1") m->geom_group[geom] = 2;
    }
    contactData_ = mj_makeData(m);
    if(!contactData_) throw std::runtime_error("Cannot allocate SMPL-H contact kinematics data");
  }

  static void interpolateQuat(mjtNum * out, const float * a, const float * b, double t)
  {
    double dot = 0;
    for(int k = 0; k < 4; ++k) dot += a[k]*b[k];
    const double sign = dot < 0 ? -1 : 1;
    dot = std::min(1.0, std::abs(dot));
    double wa = 1-t, wb = t;
    if(dot < .9995)
    {
      const double angle = std::acos(dot), denom = std::sin(angle);
      wa = std::sin((1-t)*angle)/denom; wb = std::sin(t*angle)/denom;
    }
    for(int k = 0; k < 4; ++k) out[k] = wa*a[k] + wb*sign*b[k];
    mju_normalize4(out);
  }

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
  void initialize(mjModel * m, const mjData * d)
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
    surfaceBody_ = m->geom_bodyid[geom];
    mesh_ = m->geom_dataid[geom];
    if(mesh_ < 0 || static_cast<uint32_t>(m->mesh_vertnum[mesh_]) != vertexCount_
       || static_cast<uint32_t>(m->mesh_normalnum[mesh_]) != vertexCount_)
      throw std::runtime_error("SMPL-H mesh topology does not match cache");
    // MuJoCo's OBJ compiler recenters mesh assets and stores compensating
    // geom/mesh poses. Clear those asset transforms while retaining the pose
    // of the merged body that places the human in the mc_mujoco scene.
    mju_zero(m->geom_pos + 3 * geom, 3);
    mju_zero(m->geom_quat + 4 * geom, 4);
    m->geom_quat[4 * geom] = 1;
    mju_zero(m->mesh_pos + 3 * mesh_, 3);
    mju_zero(m->mesh_quat + 4 * mesh_, 4);
    m->mesh_quat[4 * mesh_] = 1;
    for(int i = 0; i < 52; ++i) joints_.push_back(find(m, mjOBJ_JOINT, i ? "smplh_joint_" + std::to_string(i) : "smplh_root"));
    // mc_mujoco assigns objects.human.init_pos to the first free joint,
    // whereas the display mesh belongs to a separate fixed body. Capture
    // that placement before the cached free-joint pose overwrites it.
    const auto * initialRoot = d->qpos + m->jnt_qposadr[joints_[0]];
    mju_copy3(m->body_pos + 3 * surfaceBody_, initialRoot);
    mju_copy4(m->body_quat + 4 * surfaceBody_, initialRoot + 3);
    loop_ = std::getenv("SMPLH_LOOP") && std::string(std::getenv("SMPLH_LOOP")) == "1";
    if(const char * frozen = std::getenv("SMPLH_FREEZE_FRAME"))
    {
      char * end = nullptr;
      const long value = std::strtol(frozen, &end, 10);
      if(end == frozen || *end || value < 0 || static_cast<unsigned long>(value) >= frameCount_)
        throw std::runtime_error("SMPLH_FREEZE_FRAME must be a valid nonnegative cache frame");
      freezeFrame_ = static_cast<int>(value);
    }
    const char * prefix = std::getenv("SMPLH_ROBOT_PREFIX");
    const char * groupText = std::getenv("SMPLH_ROBOT_GROUP");
    int group = groupText ? std::atoi(groupText) : 2;
    distanceEnabled_ = !(std::getenv("SMPLH_DISTANCE_DISABLE")
                         && std::string(std::getenv("SMPLH_DISTANCE_DISABLE")) == "1");
    configureContacts(m);
    if(distanceEnabled_)
      distanceWorker_.configure(m, regionFaceIds_, regionNames_, faces_, prefix ? prefix : "kinova_", group,
                                true);
  }

public:
  SmplhBridge() = default;
  SmplhBridge(const SmplhBridge &) = delete;
  template<typename Config>
  void configureContactSizes(const Config & controllerConfig)
  {
    if(initialized_) throw std::runtime_error("Configure SMPL-H contact sizes before playback initialization");
    contactSizeScales_.clear();
    contactOffsetSettings_.clear();
    if(!controllerConfig.has("SMPLHContactSizes")) return;
    const auto settings = controllerConfig("SMPLHContactSizes");
    if(!settings.isObject()) throw std::runtime_error("SMPLHContactSizes must map region names to scales");
    const std::vector<std::string> regions = {
      "torso", "head", "left_upper_arm", "left_forearm", "left_hand",
      "right_upper_arm", "right_forearm", "right_hand", "left_thigh",
      "left_lower_leg", "left_foot", "right_thigh", "right_lower_leg", "right_foot"};
    auto positive = [](double value, const std::string & label) {
      if(!std::isfinite(value) || value <= 0) throw std::runtime_error(label + " must be positive and finite");
      return value;
    };
    for(const auto & name : settings.keys())
    {
      if(std::find(regions.begin(), regions.end(), name) == regions.end())
        throw std::runtime_error("Unknown SMPL-H contact size region: " + name);
      const auto value = settings(name);
      std::array<double, 2> scales = {1, 1};
      if(value.isNumeric())
      {
        double scale = positive(static_cast<double>(value), name);
        scales = {scale, scale};
      }
      else if(value.isObject())
      {
        for(const auto & key : value.keys())
          if(key != "scale" && key != "radius_scale" && key != "length_scale" && key != "offset_world_m")
            throw std::runtime_error("Invalid SMPL-H contact size key: " + name + "." + key);
        double base = positive(value.has("scale") ? static_cast<double>(value("scale")) : 1, name + ".scale");
        scales = {base, base};
        if(value.has("radius_scale")) scales[0] *= positive(static_cast<double>(value("radius_scale")), name + ".radius_scale");
        if(value.has("length_scale")) scales[1] *= positive(static_cast<double>(value("length_scale")), name + ".length_scale");
        if(value.has("offset_world_m"))
        {
          const auto offset = value("offset_world_m");
          if(!offset.isArray() || offset.size() != 3)
            throw std::runtime_error(name + ".offset_world_m must have three values");
          std::array<double, 3> displacement;
          for(int axis = 0; axis < 3; ++axis)
          {
            displacement[axis] = static_cast<double>(offset[axis]);
            if(!std::isfinite(displacement[axis]))
              throw std::runtime_error(name + ".offset_world_m must be finite");
          }
          contactOffsetSettings_[name] = displacement;
        }
      }
      else throw std::runtime_error("Invalid SMPL-H contact size for region: " + name);
      contactSizeScales_[name] = scales;
    }
  }
  ~SmplhBridge()
  {
    if(contactData_) mj_deleteData(contactData_);
    if(mapping_ != MAP_FAILED) munmap(mapping_, bytes_);
    if(fd_ >= 0) close(fd_);
  }
  // Called immediately before mj_step. Scratch kinematics never alters the
  // dynamic robot state or the discrete mesh frames used by distance queries.
  void beforeStep(mjModel * m, mjData * d)
  {
    initialize(m, d);
    if(mapping_ == MAP_FAILED || !contactsEnabled_ || !std::isfinite(d->time) || d->time < 0) return;
    double sample = d->time * fps_;
    if(freezeFrame_ >= 0) sample = freezeFrame_;
    else if(loop_) sample = std::fmod(sample, static_cast<double>(frameCount_));
    else sample = std::min(sample, static_cast<double>(frameCount_ - 1));
    const uint32_t first = static_cast<uint32_t>(std::floor(sample));
    // Hold the final pose until wrapping; never interpolate across a loop teleport.
    const uint32_t second = std::min(first + 1, frameCount_ - 1);
    const double alpha = sample - first;
    const float * a = qpos_ + 211ull*first;
    const float * b = qpos_ + 211ull*second;
    for(size_t j = 0, offset = 0; j < joints_.size(); ++j)
    {
      mjtNum * target = contactData_->qpos + m->jnt_qposadr[joints_[j]];
      if(j == 0)
      {
        mjtNum local[3];
        for(int k = 0; k < 3; ++k) local[k] = (1-alpha)*a[k] + alpha*b[k];
        mju_rotVecQuat(target, local, m->body_quat + 4*surfaceBody_);
        mju_addTo3(target, m->body_pos + 3*surfaceBody_);
        mjtNum quat[4];
        interpolateQuat(quat, a+3, b+3, alpha);
        mju_mulQuat(target+3, m->body_quat + 4*surfaceBody_, quat);
      }
      else interpolateQuat(target, a+offset, b+offset, alpha);
      offset += j ? 4 : 7;
    }
    mj_kinematics(m, contactData_);
    for(const auto & body : contactBodies_)
    {
      mju_copy3(d->mocap_pos + 3*body.first, contactData_->xpos + 3*body.second);
      if(const auto offset = contactWorldOffsetsByBody_.find(body.second); offset != contactWorldOffsetsByBody_.end())
        mju_addTo3(d->mocap_pos + 3*body.first, offset->second.data());
      mju_copy4(d->mocap_quat + 4*body.first, contactData_->xquat + 4*body.second);
    }
  }
  // Project position-servo targets onto the non-penetrating contact halfspaces.
  // The human still follows mocap; MuJoCo's solver supplies the actual force.
  void guardPositionControls(const mjModel * m, mjData * d)
  {
    if(!contactsEnabled_ || mapping_ == MAP_FAILED) return;
    mj_forward(m, d); // Include the human pose just set by beforeStep.
    const char * setting = std::getenv("SMPLH_ROBOT_PREFIX");
    const std::string prefix = setting ? setting : "kinova_";
    std::vector<int> actuators, dofs;
    std::vector<mjtNum> delta;
    for(int a = 0; a < m->nu; ++a)
    {
      if(m->actuator_trntype[a] != mjTRN_JOINT || m->actuator_biastype[a] != mjBIAS_AFFINE
         || m->actuator_biasprm[mjNBIAS*a + 1] >= 0) continue;
      int joint = m->actuator_trnid[2*a];
      if(joint < 0 || (m->jnt_type[joint] != mjJNT_HINGE && m->jnt_type[joint] != mjJNT_SLIDE)) continue;
      const char * body = mj_id2name(m, mjOBJ_BODY, m->jnt_bodyid[joint]);
      if(!body || std::string(body).compare(0, prefix.size(), prefix) != 0) continue;
      int q = m->jnt_qposadr[joint];
      actuators.push_back(a);
      dofs.push_back(m->jnt_dofadr[joint]);
      delta.push_back(d->ctrl[a] - d->qpos[q]);
    }
    if(previousMocap_.size() != static_cast<size_t>(m->nmocap))
      previousMocap_.resize(m->nmocap);
    if(actuators.empty()) return;
    std::vector<mjtNum> jac(3ull*m->nv);
    for(int pass = 0; pass < 4; ++pass)
      for(int c = 0; c < d->ncon; ++c)
      {
        const auto & contact = d->contact[c];
        int humanSide = -1;
        for(int side = 0; side < 2; ++side)
        {
          const char * name = mj_id2name(m, mjOBJ_GEOM, contact.geom[side]);
          if(name && std::string(name).find("smplh_contact_geom_") != std::string::npos)
            humanSide = side;
        }
        if(humanSide < 0) continue;
        const int robotGeom = contact.geom[1-humanSide];
        const int robotBody = m->geom_bodyid[robotGeom];
        const char * name = mj_id2name(m, mjOBJ_BODY, robotBody);
        if(!name || std::string(name).compare(0, prefix.size(), prefix) != 0) continue;
        mj_jac(m, d, jac.data(), nullptr, contact.pos, robotBody);
        std::vector<mjtNum> row(delta.size());
        mjtNum inward = 0, norm2 = 0;
        // MuJoCo's normal points from geom[0] toward geom[1]. The robot
        // must move in the opposite direction when it is geom[0].
        const mjtNum sign = humanSide == 0 ? 1 : -1;
        for(size_t i = 0; i < delta.size(); ++i)
        {
          int dof = dofs[i];
          row[i] = sign*(contact.frame[0]*jac[dof] + contact.frame[1]*jac[m->nv+dof]
                         + contact.frame[2]*jac[2*m->nv+dof]);
          inward += row[i]*delta[i];
          norm2 += row[i]*row[i];
        }
        mjtNum required = 0;
        const int humanBody = m->geom_bodyid[contact.geom[humanSide]];
        const int mocap = m->body_mocapid[humanBody];
        if(havePreviousMocap_ && mocap >= 0)
        {
          const auto & previous = previousMocap_[mocap];
          mjtNum local[3], previousOffset[3], previousPoint[3];
          mju_sub3(local, contact.pos, d->mocap_pos + 3*mocap);
          mjtNum conjugate[4];
          mju_negQuat(conjugate, d->mocap_quat + 4*mocap);
          mju_rotVecQuat(local, local, conjugate);
          mju_rotVecQuat(previousOffset, local, previous.data()+3);
          mju_add3(previousPoint, previous.data(), previousOffset);
          for(int k = 0; k < 3; ++k)
            required += sign*contact.frame[k]*(contact.pos[k]-previousPoint[k]);
          required = std::clamp(required, mjtNum(-.005), mjtNum(.005));
        }
        if(inward < required && norm2 > 1e-12)
          for(size_t i = 0; i < delta.size(); ++i) delta[i] += (required-inward)*row[i]/norm2;
      }
    for(size_t i = 0; i < actuators.size(); ++i)
    {
      int a = actuators[i], joint = m->actuator_trnid[2*a];
      mjtNum target = d->qpos[m->jnt_qposadr[joint]] + delta[i];
      if(m->actuator_ctrllimited[a])
        target = std::clamp(target, m->actuator_ctrlrange[2*a], m->actuator_ctrlrange[2*a+1]);
      d->ctrl[a] = target;
    }
    for(int k = 0; k < m->nmocap; ++k)
    {
      mju_copy3(previousMocap_[k].data(), d->mocap_pos + 3*k);
      mju_copy4(previousMocap_[k].data()+3, d->mocap_quat + 4*k);
    }
    havePreviousMocap_ = true;
  }

  void captureContactStats(const mjModel * m, const mjData * d)
  {
    contactStats(m, d, lastContactCount_, lastContactDepth_);
  }
  void latestContactStats(int & count, double & deepest) const
  {
    count = lastContactCount_;
    deepest = lastContactDepth_;
  }

  // Counts physical contacts involving a fitted human proxy after mj_step.
  void contactStats(const mjModel * m, const mjData * d, int & count, double & deepest) const
  {
    count = 0;
    deepest = 0;
    for(int i = 0; i < d->ncon; ++i)
    {
      const auto & contact = d->contact[i];
      bool human = false;
      for(int side = 0; side < 2; ++side)
      {
        const char * name = mj_id2name(m, mjOBJ_GEOM, contact.geom[side]);
        if(name && std::string(name).find("smplh_contact_geom_") != std::string::npos) human = true;
      }
      if(human)
      {
        ++count;
        deepest = std::max(deepest, -static_cast<double>(contact.dist));
      }
    }
  }
  void update(mjModel * m, mjData * d)
  {
    initialize(m, d);
    if(mapping_ == MAP_FAILED) return;
    if(!std::isfinite(d->time) || d->time < 0) return;
    uint64_t index = static_cast<uint64_t>(std::floor(d->time * fps_ + 1e-9));
    uint32_t next = freezeFrame_ >= 0 ? static_cast<uint32_t>(freezeFrame_)
                  : loop_ ? index % frameCount_ : static_cast<uint32_t>(std::min<uint64_t>(index, frameCount_ - 1));
    const float * v = vertices_ + static_cast<size_t>(next) * vertexCount_ * 3;
    const float * pose = qpos_ + static_cast<size_t>(next) * 211;
    for(size_t j = 0, offset = 0; j < joints_.size(); ++j)
    {
      int id = joints_[j], nq = j ? 4 : 7, nv = j ? 3 : 6;
      for(int k = 0; k < nq; ++k) d->qpos[m->jnt_qposadr[id] + k] = pose[offset + k];
      if(j == 0)
      {
        // The visual mesh is placed by the fixed scene body. Apply the same
        // placement to the animated free root so debug geoms stay aligned.
        auto * root = d->qpos + m->jnt_qposadr[id];
        const auto * rotation = m->body_quat + 4 * surfaceBody_;
        mjtNum position[3], orientation[4];
        mju_rotVecQuat(position, root, rotation);
        mju_add3(root, position, m->body_pos + 3 * surfaceBody_);
        mju_mulQuat(orientation, rotation, root + 3);
        mju_copy4(root + 3, orientation);
      }
      mju_zero(d->qvel + m->jnt_dofadr[id], nv);
      offset += nq;
    }
    // The skeleton has a free joint and MuJoCo advances it during mj_step.
    // Restore its cached pose even while a frame is held (including freeze
    // and the final frame), or its geoms fall below the fixed display mesh.
    mj_forward(m, d);
    if(next != frame_)
    {
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
      frame_=next; dirty_=true;
    }
    beforeStep(m, d);
    if(contactsEnabled_) mj_forward(m, d);
    // MuJoCo precomputes the pose of this fixed geom when loading the OBJ.
    // Clearing geom_pos above does not clear that compiled world pose, so the
    // displayed mesh otherwise retains the OBJ center offset above the proxies.
    mju_copy3(d->geom_xpos + 3 * surfaceGeom_, d->xpos + 3 * surfaceBody_);
    mju_copy(d->geom_xmat + 9 * surfaceGeom_, d->xmat + 9 * surfaceBody_, 9);
    if(distanceEnabled_ && (++steps_ % distanceEvery_ == 0 || sequence_ == 0))
    {
      const bool held = freezeFrame_ >= 0 || (!loop_ && index >= frameCount_ - 1);
      // Backward material-vertex difference at the cache frame interval. No
      // future frame is accessed; the first/loop-wrap frame has no estimate.
      const float * previous = !held && next > 0
          ? vertices_ + static_cast<size_t>(next - 1) * vertexCount_ * 3 : nullptr;
      distanceWorker_.submit(++sequence_, next, d->time, v, vertexCount_, previous,
                             1.0 / fps_, held, d, m, surfaceGeom_);
    }
  }
  human_robot_distance::Snapshot snapshot() const { return distanceWorker_.latest(); }
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
    // Keep one visible arrow per human region; the worker still publishes and
    // constrains all region/link pairs.
    std::map<std::string, const human_robot_distance::RegionDistance *> closest;
    for(const auto & region : distances.regions)
    {
      auto & chosen = closest[region.region];
      if(!chosen || region.distance < chosen->distance) chosen = &region;
    }
    for(const auto & entry : closest)
    {
      const auto & region = *entry.second;
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
      // The MuJoCo arrow mesh reaches half of the connector's stored length.
      scene->geoms[arrowId].size[2] *= 2.0f;
      scene->geoms[arrowId].category = mjCAT_DECOR;
      scene->geomorder[arrowId] = arrowId;
    }
  }
  void upload(const mjModel * m, mjrContext * context)
  {
    if(dirty_) { mjr_uploadMesh(m, context, mesh_); dirty_=false; }
  }
};
