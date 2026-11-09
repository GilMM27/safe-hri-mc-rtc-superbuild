# SMPL-H playback in MuJoCo

This guide continues the [superbuild setup](../README.md): build the Kinova
robot and `mc_mujoco` first, then add an AMASS motion and a licensed SMPL-H
body model. `mc_mujoco` shows the moving human beside Kinova and draws the
distance arrows. `mc_rtc_gui` shows controller data but does not contain the
MuJoCo scene.

The bridge and controllers use the
[human_robot_distance_api](https://github.com/GilMM27/human_robot_distance_api) header for
the `HumanRobotDistance::Snapshot` datastore type. The superbuild installs this
header package before building the bridge.

Run these commands from the superbuild repository root unless noted.

## 1. Download the licensed data

Create `datasets/` in the repository root. It is ignored by Git; the models,
motions, and generated caches stay on your machine.

1. Register and accept the terms at the [AMASS download site](https://amass.is.tue.mpg.de/). Download a SMPL-H motion archive such as CMU and extract it under `datasets/`, so a sequence has a path like `datasets/CMU/02/02_02_poses.npz`.
2. Register and accept the model license at the [MANO / SMPL+H site](https://mano.is.tue.mpg.de/). Extract the SMPL-H models into `datasets/smplh/`. For the CMU example below, use `SMPLH_MALE.pkl`.

Download through your accounts and extract the archives locally. The example
sequence needs a 156-value `poses` vector for each frame. If your downloaded
paths differ, adjust the commands below.

```text
datasets/
├── CMU/02/02_02_poses.npz
├── smplh/SMPLH_MALE.pkl
└── smplh_cache/                 # created in step 2
```

## 2. Prepare a playback cache

Install the Python preprocessing dependencies:

```sh
python3 -m venv venv
venv/bin/pip install -r smplh_playback/requirements.txt
```

Set paths for one sequence and prepare it once. This creates a large cache.
Use `--translation X Y Z` if you need to place the human beside Kinova.

```sh
export SMPLH_MODEL="$PWD/datasets/smplh/<model>.pkl" # try SMPLH_MALE
export AMASS_SEQUENCE="$PWD/datasets/CMU/<subject>/<trial>.npz" # try 01/01_01_poses
export SMPLH_CACHE="$PWD/datasets/smplh_cache/<name>" # try cmu_01_01

venv/bin/python -m smplh_playback.prepare
```

The cache contains `human.xml`, `surface.obj`, `smplh.yaml`, and playback
arrays. `--rotation RX RY RZ` (degrees), `--translation X Y Z` (meters), and
`--fps` let you align or retime another sequence. Preparation writes absolute
paths into `human.xml` and `smplh.yaml`; if you move a prepared cache, prepare
it again or update those paths. `--output` defaults to `SMPLH_CACHE` when set;
otherwise it is required. `--translation` defaults to `0 0 0`.

## 3. Build and select the controller

Enable the native playback bridge and rebuild. The root README covers the
superbuild prerequisites and Kinova installation.

```sh
# aka mc_superbuild_config & mc_setup
cmake --preset relwithdebinfo \
  -DWITH_ROS_SUPPORT=ON \
  -DWITH_SMPLH_PLAYBACK=ON \
  -DWITH_Kinova=ON

# aka mc_build
cmake --build --preset relwithdebinfo
source ~/workspace/install/setup_mc_rtc.sh
```

Edit `~/.config/mc_rtc/mc_rtc.yaml` aka mc_config:

```yaml
MainRobot: Kinova
Enabled: KinovaHoldController # or choose from kinova_controllers/
Timestep: 0.001
LogPolicy: threaded
GUIServer:
  Enable: true
```

`KinovaHoldController` holds the arm still for a first playback check. If you
already have a compatible Kinova controller, use its name under `Enabled`.

## 4. Add the human to the MuJoCo scene

Create `~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml` aka mc_mujoco_config with the
human object, or add `human` to your existing `objects` map:

```yaml
objects:
  human:
    module: smplh
    init_pos:
      translation: [0.0, 0.0, 0.0]
      rotation: [0.0, 0.0, 0.0]
```

Edit `init_pos` here and restart `mc_mujoco` to change the human's placement.
The `-f` option loads an mc_rtc configuration, where `objects` is ignored.

When `SMPLH_CACHE` is set, the bridge loads `human.xml` from that cache.
The bridge applies the merged human body's pose to the rendered surface,
and distance samples. The configured `init_pos` therefore affects the human's
scene placement. Keep the cache and its generated XML together because they must
share the same mesh topology.

## 5. Run playback and check distances

In the terminal where you launch MuJoCo, source the installed environment and
export `SMPLH_CACHE` again if this is a new terminal. Set looping if you want
the motion to repeat.

```sh
source ~/workspace/install/setup_mc_rtc.sh
export SMPLH_CACHE="$PWD/datasets/smplh_cache/<name>"
export SMPLH_LOOP=1 # Optional
mc_mujoco --sync
```

Without `SMPLH_CACHE`, native motion and distance updates do not start.
Keep the `mc_mujoco` window open to see the
human, endpoint dots, and arrows. The first distance sample arrives
asynchronously; wait briefly if no lines appear at startup. Open `mc_rtc_gui`
separately for the **SMPL-H distances** values when its server is enabled.

The default distance mode uses fast approximate convex patches (SCH). For
exact triangle-surface distances, export `SMPLH_DISTANCE_MODE=fcl` before
launch. The worker requests a distance sample every 25 physics steps by
default; `SMPLH_DISTANCE_EVERY=1` requests one every step at higher compute
cost. Distances are logged as `SMPLH_distance_<region>` in meters.
The worker keeps a closest point for each human-region/Kinova-link pair on the
seven movable joint links. The fixed base is excluded. The regional display
and `SMPLH_distance_<region>` log show the minimum across links.

For region definitions, timing details, cache conversion, and SCH/FCL
tradeoffs, see [distance and cache notes](../doc/smplh-distance.md).

For the opt-in directional SSM+PFL simulation baseline, example regional
configuration, equations, and measured residuals, see
[Kinova SSM+PFL](../doc/ssm-pfl.md). The baseline requires the updated native
bridge and a prepared cache.

## Physical human contacts

Newly prepared caches include motion-fitted capsule proxies for all 14 body regions. Their
shapes enclose each region’s cached surface vertices throughout playback.
A 5 cm contact margin gives the solver time to react before visible overlap.
`SMPLH_CONTACT_MODE=kinematic` is the default: the human follows the
recording without yielding, and contact forces act on Kinova. The simulator
projects position-actuator targets away from active human contacts, including
the proxy’s measured movement at the contact point. Pose interpolation
runs before each physics step. The visible surface and existing distance queries
remain separate; their cached frames are not interpolated.

Re-run `python -m smplh_playback.prepare` with your original model and motion to
regenerate an older cache, then rebuild `mc_mujoco` to refresh the native bridge.
Cache-format conversion alone does not add contact proxies. Alternatively, set
`SMPLH_CONTACT_MODE=off` for legacy visual-only playback. Invalid modes and missing
proxies produce an error rather than silently disabling contacts.

The MuJoCo status panel shows `SMPL-H contacts` and deepest penetration in mm.
Set `SMPLH_CONTACT_DEBUG=1` to display the translucent collision proxies, or enable
geom group 4 in the MuJoCo viewer. Contacts select collision-enabled robot geoms on bodies matching
`SMPLH_ROBOT_PREFIX` (default `kinova_`), independently of the distance-query
`SMPLH_ROBOT_GROUP`. Visual-only robot geoms remain non-colliding.
The bridge allocates an unused collision-mask bit, preserving existing robot
contacts while excluding proxy self-contact and proxy–environment contact.

The final pose is held at the end of playback; looping holds it until the wrap
and then jumps to frame zero. Frozen mode holds the selected pose. Avoid loop
boundaries during contact experiments when the recording is not cyclic.

The shapes approximate the skin surface. MuJoCo contacts allow some penetration.
Mocap motion is used in the position-command guard, but MuJoCo’s contact solver
still does not include that motion as a physical body velocity. Use this mode
for solid obstacles, not calibrated impact-force or friction measurements. Existing controller avoidance constraints still apply.

### Size contact shapes by region

Add `SMPLHContactSizes` to the active controller YAML, such as
`~/.config/mc_rtc/controllers/KinovaPositionSSMPFLController.yaml`:

```yaml
SMPLHContactSizes:
  torso: {radius_scale: 0.85}
  left_forearm: {radius_scale: 0.8, length_scale: 1.0}
  head: {radius_scale: 0.9, offset_world_m: [0.0, 0.0, 0.03]}
```

These scales apply when `mc_mujoco` starts, to whichever `SMPLH_CACHE` is
selected. A scalar such as `left_forearm: 0.85` scales the whole shape.
`radius_scale` changes thickness and `length_scale` changes the straight
segment between the rounded ends. `scale` changes both.
`offset_world_m` moves a capsule in world coordinates; a positive third value
raises it in Z. For example, the head setting above raises only the head
collision capsule by 3 cm. It does not move the visible SMPL-H surface or
the distance-query mesh.
Omitted regions keep their motion-fitted sizes. Restart `mc_mujoco` after
editing the YAML; no cache regeneration is needed. Values below `1` can
expose the visual surface outside the collision shape.
For an older cache that still has head, hand, or foot boxes, run
`venv/bin/python -m smplh_playback.refit_contacts "$SMPLH_CACHE"` once and
restart `mc_mujoco`.

### Contact regression tests

Compile the native test against the superbuild's installed dependencies, then run
its Python harness (the harness creates synthetic data without licensed models):

```bash
export MUJOCO_ROOT=/path/to/build/superbuild/extensions/mujoco/mujoco-3.5.0
export SUPERBUILD_INSTALL=/path/to/install
g++ -O0 tests/test_smplh_contacts.cpp -o /tmp/test-smplh-contacts \
  -I"$MUJOCO_ROOT/include" -I"$SUPERBUILD_INSTALL/include" \
  $(pkg-config --cflags fcl sch-core) -std=c++17 \
  -L"$MUJOCO_ROOT/lib" -Wl,-rpath,"$MUJOCO_ROOT/lib" \
  -Wl,-rpath,"$SUPERBUILD_INSTALL/lib" \
  -lmujoco $(pkg-config --libs fcl sch-core) -lmc_rtc_utils -lfmt -pthread
SMPLH_CONTACT_TEST_BINARY=/tmp/test-smplh-contacts \
  venv/bin/python -m unittest discover -s tests -p test_smplh_contacts.py
```

To refit an existing prepared cache without recomputing SMPL-H vertices, run
`venv/bin/python -m smplh_playback.refit_contacts datasets/smplh_cache/<name>`
and restart `mc_mujoco`.
