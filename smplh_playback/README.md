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

Newly prepared caches contain 14 fitted dynamic capsules, each connected by a
MuJoCo weld to a mocap target following the recorded pose. This is the default
`SMPLH_CONTACT_MODE=dynamic`. The weld uses `solref=".01 1"` and
`solimp=".95 .99 .001"`; capsule density is `1000 kg/m³`, contact friction is
`1 .005 .0001`, and contact `solref`/`solimp` are `.02 1`/`.9 .95 .001`.
The controller also projects
position servo targets away from contact. The visible surface remains separate
from these collision shapes. The SSM+PFL distance query uses the contact
capsules by default; set `SSMPFL.SeparationGeometry: mesh` to use the surface.
Regenerate older caches
with `smplh_playback.prepare`; the bridge does not rewrite them.

`SMPLH_CONTACT_MODE=kinematic` uses duplicate mocap collision capsules, and
`SMPLH_CONTACT_MODE=off` disables human contacts. To inspect the capsules,
set `SMPLH_CONTACT_DEBUG=1` or enable geom group 4 in the MuJoCo viewer.

The bridge calls `mj_contactForce` immediately after each `mj_step`, before
`mj_forward` can replace the contact solution. Set `SMPLH_CONTACT_CSV` to a
writable path for one CSV row per solved contact, with simulation time, contact
index, region, robot geom, nonnegative normal force in newtons, and penetration
depth in metres. With `KinovaPositionSSMPFLController` active, the mc_rtc log has each region's
`SMPLH_<region>_positive_normal_sum_N`, `positive_normal_peak_N`, `ForceMaxN`,
`exceeded`, and `peak_exceeded` fields. The sum adds the positive normal component of every
contact involving that region. A sample exceeds its configured `ForceMaxN` only
when the sum is greater than the limit. Both transient and quasi-static contacts
use the same limit in this first model. The peak captures every physics step,
including steps between controller ticks, and resets when simulation time rewinds.
`SMPLH_force_sample_available` identifies runs where the bridge has published a
force snapshot. The PositionTask controller does not register these force logs.

The selected-point torque estimate projects the constraint torque onto the
controller's closest robot point and normal. Other contacts and forces can
contribute to the joint torque, so this remains a diagnostic.
`SMPLH_external_joint_torque_norm_Nm` is also logged as a separate joint-space
diagnostic; external-torque feedback and high-pass compensation states are
logged independently.

In `mc_log_ui`, open the current `mc-control-KinovaPositionSSMPFLController`
log and plot `SMPLH_selected_point_torque_force_N`,
`SMPLH_selected_point_mujoco_force_N`, and
`SMPLH_selected_point_ForceMaxN`. Use
`SMPLH_selected_point_force_valid` to identify samples with a matching solved
contact. `SMPLH_<region>_ForceMaxN` and exceedance fields compare against the
solved `mj_contactForce` regional sum.

For baseline force experiments, leave external-torque feedback and high-pass
compensation disabled in mc_mujoco. These simulated forces are not calibrated
physical measurements.
Check timestep and solver sensitivity, and compare matched static loads and
impacts against calibrated force and pressure measurements before making an ISO
compliance claim. [ISO/PAS 5672:2023](https://www.iso.org/standard/82488.html)
covers physical force and pressure measurement methods; pressure is outside
this first implementation. The example `ForceMaxN` values are simulation
placeholders, not limits established for a particular application.

### Size contact shapes by region

Add `SMPLHContactSizes` to the active controller YAML, such as
`~/.config/mc_rtc/controllers/KinovaPositionSSMPFLController.yaml`:

```yaml
SMPLHContactSizes:
  torso: {radius_scale: 0.85}
  left_forearm: {radius_scale: 0.8, length_scale: 1.0}
  head: {radius_scale: 0.9, offset_world_m: [0.0, 0.0, 0.03]}
```

`radius_scale` changes thickness; `length_scale` changes the straight segment
between the rounded ends; `scale` changes both. A single number, such as
`left_forearm: 0.85`, scales the whole capsule. `offset_world_m` shifts a
capsule in world coordinates, so the example raises the head capsule by 3 cm.
Unlisted regions keep their fitted sizes. These settings affect collision
shapes only, not the visible surface or distance-query mesh. Values below `1`
may leave visible skin outside a capsule. Restart `mc_mujoco` after editing the
YAML; you do not need to regenerate the cache.
