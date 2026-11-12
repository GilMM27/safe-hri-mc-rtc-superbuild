#include "SmplhBridge.h"
#include <mc_rtc/Configuration.h>
#include <iostream>
#include <tuple>
#include <stdexcept>

void require(bool condition, const char * message)
{
  if(!condition) throw std::runtime_error(message);
}

int main(int argc, char ** argv)
{
  if(argc != 2 && argc != 3) return 2;
  if(argc == 3 && std::string(argv[2]) == "impact")
  {
    setenv("SMPLH_DISTANCE_DISABLE", "1", 1);
    setenv("SMPLH_FREEZE_FRAME", "0", 1);
    try
    {
      for(double timestep : {.0005, .001, .002})
        for(int iterations : {50, 100})
        for(double speed : {.25, .5, 1.0, 2.0})
        {
          char error[1024] = {};
          mjModel * model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
          if(!model) throw std::runtime_error(error);
          model->opt.timestep = timestep;
          model->opt.iterations = iterations;
          mjData * data = mj_makeData(model);
          double peak = 0;
          {
            SmplhBridge bridge;
            bridge.update(model, data);
            const int proxy = mj_name2id(model, mjOBJ_GEOM, "smplh_contact_geom_0");
            for(int geom = 0; geom < model->ngeom; ++geom)
            {
              const char * name = mj_id2name(model, mjOBJ_GEOM, geom);
              if(name && std::string(name).find("smplh_contact_geom_") == 0 && geom != proxy)
                model->geom_contype[geom] = model->geom_conaffinity[geom] = 0;
            }
            const int probe = mj_name2id(model, mjOBJ_JOINT, "probe_root");
            const int q = model->jnt_qposadr[probe], v = model->jnt_dofadr[probe];
            for(int axis = 0; axis < 3; ++axis)
            {
              const double outward = data->geom_xmat[9*proxy + 3*axis];
              data->qpos[q+axis] = data->geom_xpos[3*proxy+axis]
                                   + outward*(model->geom_size[3*proxy] + .04);
              data->qvel[v+axis] = -outward*speed;
            }
            for(int step = 0; step < 400; ++step)
            {
              bridge.beforeStep(model, data);
              mj_step(model, data);
              bridge.captureContactStats(model, data);
              const auto sample = bridge.contactForces();
              for(double value : sample.positiveNormalSumN)
                peak = std::max(peak, value);
              bridge.update(model, data);
            }
          }
          mj_deleteData(data); mj_deleteModel(model);
          require(std::isfinite(peak) && peak > 0, "Impact force was missing at a tested speed or timestep");
        }
    }
    catch(const std::exception & e) { std::cerr << e.what() << '\n'; return 1; }
    return 0;
  }
  if(argc == 3 && std::string(argv[2]) == "guard")
  {
    setenv("SMPLH_DISTANCE_DISABLE", "1", 1);
    setenv("SMPLH_FREEZE_FRAME", "0", 1);
    auto run = [&](bool guarded) {
      char error[1024] = {};
      mjModel * model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
      if(!model) throw std::runtime_error(error);
      mjData * data = mj_makeData(model);
      double finalJoint = 0;
      int contacts = 0;
      bool advancedTarget = false;
      {
        SmplhBridge bridge;
        bridge.update(model, data);
        int joint = mj_name2id(model, mjOBJ_JOINT, "arm");
        int proxy = mj_name2id(model, mjOBJ_GEOM, "smplh_contact_geom_0");
        int target = mj_name2id(model, mjOBJ_BODY, "smplh_contact_target_0");
        int mocap = model->body_mocapid[target];
        for(int i = 0; i < 2000; ++i)
        {
          bridge.beforeStep(model, data);
          data->mocap_pos[3*mocap+1] = -.3 + .0003*i;
          if(guarded) bridge.guardPositionControls(model, data);
          advancedTarget |= data->ctrl[0] > .1;
          mj_step(model, data);
          for(int c = 0; c < data->ncon; ++c)
            contacts += data->contact[c].geom[0] == proxy || data->contact[c].geom[1] == proxy;
          bridge.update(model, data);
        }
        finalJoint = data->qpos[model->jnt_qposadr[joint]];
      }
      mj_deleteData(data); mj_deleteModel(model);
      return std::make_tuple(finalJoint, contacts, advancedTarget);
    };
    try
    {
      auto guarded = run(true), unguarded = run(false);
      require(std::get<1>(guarded) > 100 && std::get<2>(guarded),
              "Moving human did not produce an outward position command");
      require(std::get<0>(guarded) > std::get<0>(unguarded) + .02,
              "Contact guard did not preserve robot separation after the human passed");
    }
    catch(const std::exception & e) { std::cerr << e.what() << '\n'; return 1; }
    return 0;
  }
  setenv("SMPLH_DISTANCE_DISABLE", "1", 1);
  char error[1024] = {};
  mjModel * m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if(!m) { std::cerr << error << '\n'; return 2; }
  mjData * d = mj_makeData(m);
  try
  {
    const double fps = std::getenv("SMPLH_TEST_FPS") ? std::atof(std::getenv("SMPLH_TEST_FPS")) : 30;
    const bool configured = std::getenv("SMPLH_TEST_CONTACT_CONFIG");
    const bool kinematic = std::getenv("SMPLH_CONTACT_MODE") && std::string(std::getenv("SMPLH_CONTACT_MODE")) == "kinematic";
    const int headGeom = mj_name2id(m, mjOBJ_GEOM, kinematic ? "smplh_contact_kinematic_geom_15" : "smplh_contact_geom_15");
    const double headRadius = m->geom_size[3*headGeom];
    const double headHalfLength = m->geom_size[3*headGeom+1];
    SmplhBridge bridge;
    if(configured)
    {
      auto config = mc_rtc::Configuration::fromYAMLData(
          "SMPLHContactSizes:\n  head: {radius_scale: 5.0, length_scale: 0.5, "
          "offset_world_m: [0, 0, 0.03]}\n");
      bridge.configureContactSizes(config);
    }
    bridge.update(m, d);
    require(m->geom_margin[headGeom] == 0,
            "New SMPL-H contact proxy unexpectedly has a margin");
    if(configured)
    {
      require(std::abs(m->geom_size[3*headGeom] - 5.0*headRadius) < 1e-9
              && std::abs(m->geom_size[3*headGeom+1] - .5*headHalfLength) < 1e-9,
              "Controller YAML did not resize the head capsule");
      const int headMocap = m->body_mocapid[mj_name2id(m, mjOBJ_BODY, "smplh_contact_target_15")];
      const int headJoint = mj_name2id(m, mjOBJ_JOINT, "smplh_joint_15");
      const int headBody = m->jnt_bodyid[headJoint];
      require(std::abs(d->mocap_pos[3*headMocap+2] - d->xpos[3*headBody+2] - .03) < 1e-9,
              "Controller YAML did not raise the head capsule");
      const int probeJoint = mj_name2id(m, mjOBJ_JOINT, "probe_root");
      const int probeQ = m->jnt_qposadr[probeJoint];
      for(int axis = 0; axis < 3; ++axis)
        d->qpos[probeQ+axis] = d->geom_xpos[3*headGeom+axis]
            + d->geom_xmat[9*headGeom+3*axis] * (m->geom_size[3*headGeom] + .025);
      mj_forward(m, d);
      bool headContact = false;
      for(int i = 0; i < d->ncon; ++i)
        headContact |= d->contact[i].geom[0] == headGeom || d->contact[i].geom[1] == headGeom;
      require(headContact, "Enlarged head capsule was missed by collision broad phase");
      bridge.update(m, d);
    }
    const int surface = mj_name2id(m, mjOBJ_GEOM, "smplh_surface");
    const int surfaceBody = m->geom_bodyid[surface];
    for(int axis = 0; axis < 3; ++axis)
      require(std::abs(d->geom_xpos[3*surface+axis] - d->xpos[3*surfaceBody+axis]) < 1e-6,
              "Display mesh retains MuJoCo's baked OBJ center offset");
    const int visual = mj_name2id(m, mjOBJ_GEOM, "robot_visual");
    if(visual >= 0) require(m->geom_contype[visual] == 0 && m->geom_conaffinity[visual] == 0,
                            "Robot visual geometry was enabled for contacts");
    const bool off = std::getenv("SMPLH_CONTACT_MODE") && std::string(std::getenv("SMPLH_CONTACT_MODE")) == "off";
    int proxy = mj_name2id(m, mjOBJ_GEOM, kinematic ? "smplh_contact_kinematic_geom_0" : "smplh_contact_geom_0");
    int root = mj_name2id(m, mjOBJ_JOINT, "probe_root");
    const int q = m->jnt_qposadr[root], v = m->jnt_dofadr[root];
    // Place a dynamic robot sphere just inside the side of the torso capsule.
    mjtNum outward[3];
    for(int axis = 0; axis < 3; ++axis)
    {
      outward[axis] = d->geom_xmat[9*proxy+3*axis];
      d->qpos[q+axis] = d->geom_xpos[3*proxy+axis]
                       + outward[axis] * (m->geom_size[3*proxy] + .025);
    }
    bridge.beforeStep(m, d);
    mj_forward(m, d);
    for(int i = 0; i < d->ncon; ++i)
    {
      const auto & c = d->contact[i];
      const int g = c.geom[0] == proxy ? c.geom[1] : c.geom[1] == proxy ? c.geom[0] : -1;
      if(g >= 0)
      {
        const char * body = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g]);
        require(body && std::string(body) == "kinova_probe", "Human contacted an environment geom or itself");
      }
    }
    bool found = false;
    for(int i = 0; i < d->ncon; ++i)
      if(d->contact[i].geom[0] == proxy || d->contact[i].geom[1] == proxy)
        found = true;
    int reported = 0;
    double deepest = 0;
    bridge.contactStats(m, d, reported, deepest);
    require(off ? !found && reported == 0 : found && reported > 0 && deepest > 0,
            "Unexpected or missing contact");
    mj_step(m, d);
    bridge.captureContactStats(m, d);
    smplh_contact::Selection selection;
    selection.valid = true;
    selection.time = d->time;
    selection.region = 0;
    selection.robotBody = "kinova_probe";
    int selectedContact = -1;
    for(int i = 0; i < d->ncon; ++i)
    {
      const auto & c = d->contact[i];
      if(c.efc_address < 0 || (c.geom[0] != proxy && c.geom[1] != proxy)) continue;
      const int robotSide = c.geom[0] == proxy ? 1 : 0;
      if(m->geom_bodyid[c.geom[robotSide]] != mj_name2id(m, mjOBJ_BODY, "kinova_probe")) continue;
      selectedContact = i;
      for(int axis = 0; axis < 3; ++axis)
      {
        selection.position[axis] = c.pos[axis];
        selection.normal[axis] = (robotSide == 0 ? 1.0 : -1.0) * c.frame[axis];
      }
      break;
    }
    require(off || selectedContact >= 0, "Could not select the solved torso contact point");
    if(selectedContact >= 0)
    {
      bridge.setContactSelection(selection);
      bridge.captureSelectedPointComparison(m, d, {v, v+1, v+2, v+3, v+4, v+5}, "kinova_");
    }
    const auto force = bridge.contactForces();
    require(off || force.selectedPointValid, "Selected-point comparison missed the solved contact");
    if(force.selectedPointValid)
    {
      require(force.selectedRegion == 0 && force.selectedPointTime == d->time,
              "Selected-point comparison metadata does not match the selected contact");
      require(std::isfinite(force.selectedTorqueForceN) && force.selectedTorqueForceN >= 0
              && std::isfinite(force.selectedMujocoForceN) && force.selectedMujocoForceN >= 0
              && std::isfinite(force.selectedForceDifferenceN),
              "Selected-point comparison returned invalid force values");
      require(force.selectedMujocoForceN > 0,
              "Selected-point comparison did not recover the solved normal force");
    }
    require(force.valid == !off, "Contact force sample validity does not match contact mode");
    double torso = 0;
    for(int i = 0; i < d->ncon; ++i)
    {
      const auto & contact = d->contact[i];
      if(contact.efc_address < 0 || (contact.geom[0] != proxy && contact.geom[1] != proxy)) continue;
      mjtNum wrench[6] = {};
      mj_contactForce(m, d, i, wrench);
      torso += std::max(0.0, static_cast<double>(wrench[0]));
    }
    require(std::abs(torso - force.positiveNormalSumN[0]) < 1e-8,
            "Per-region normal force did not sum contacts");
    double summed = 0;
    for(double value : force.positiveNormalSumN)
    {
      require(value >= 0 && std::isfinite(value), "Invalid normal force in newtons");
      summed += value;
    }
    if(!off && !kinematic)
    {
      require(std::count_if(force.positiveNormalSumN.begin(), force.positiveNormalSumN.end(),
                            [](double value) { return value > 0; }) > 1,
              "Synthetic simultaneous contact did not reach multiple regions");
    }
    require(off ? summed == 0 : summed > 0, "Missing post-step contact force");
    const double outwardSpeed = outward[0]*d->qvel[v] + outward[1]*d->qvel[v+1] + outward[2]*d->qvel[v+2];
    require(off ? std::abs(outwardSpeed) < 1e-9 : outwardSpeed > 0,
            "Robot did not respond to human contact");
    // Once playback reaches its final frame, the display mesh is held. The
    // free skeleton must also be restored after every physics step.
    const int humanRoot = mj_name2id(m, mjOBJ_JOINT, "smplh_root");
    const int humanQ = m->jnt_qposadr[humanRoot];
    d->time = 100;
    bridge.update(m, d);
    const double heldHeight = d->qpos[humanQ+2];
    for(int i = 0; i < 100; ++i)
    {
      mj_step(m, d);
      bridge.update(m, d);
    }
    require(std::abs(d->qpos[humanQ+2] - heldHeight) < 1e-6,
            "Held human skeleton fell below the display mesh");
    if(!off)
    {
      const int mocap = m->body_mocapid[mj_name2id(m, mjOBJ_BODY, "smplh_contact_target_0")];
      d->time = 0; bridge.beforeStep(m, d);
      const double x = d->mocap_pos[3*mocap];
      const bool frozen = std::getenv("SMPLH_FREEZE_FRAME");
      const bool loop = std::getenv("SMPLH_LOOP") && std::string(std::getenv("SMPLH_LOOP")) == "1";
      d->time = .5/fps; bridge.beforeStep(m, d);
      require(std::abs(d->mocap_pos[3*mocap] - x - (frozen ? 0 : .002)) < 1e-6, "Contact pose was not interpolated");
      d->time = 100; bridge.beforeStep(m, d);
      require(std::abs(d->mocap_pos[3*mocap] - x - (frozen || loop ? 0 : .008)) < 1e-6, "Final contact pose was not held");
      // Scene placement is captured from the free root at initialization.
      mjData * placed = mj_makeData(m);
      int human = mj_name2id(m, mjOBJ_JOINT, "smplh_root");
      auto * pose = placed->qpos + m->jnt_qposadr[human];
      pose[0] = .5; pose[1] = -.25;
      pose[3] = std::sqrt(.5); pose[6] = std::sqrt(.5);
      {
        SmplhBridge other;
        other.beforeStep(m, placed);
        placed->time = .5/fps; other.beforeStep(m, placed);
        require(std::abs(placed->mocap_pos[3*mocap] - .5) < 1e-6
                && std::abs(placed->mocap_pos[3*mocap+1] + (frozen ? .246 : .248)) < 1e-6,
                "Scene placement did not rotate interpolated contact motion");
      }
      mj_deleteData(placed);
    }
  }
  catch(const std::exception & e)
  {
    std::cerr << e.what() << '\n'; mj_deleteData(d); mj_deleteModel(m); return 1;
  }
  mj_deleteData(d); mj_deleteModel(m); return 0;
}
