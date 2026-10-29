# SMPL-H playback in MuJoCo

This guide continues the [superbuild setup](../README.md): build the Kinova
robot and `mc_mujoco` first, then add an AMASS motion and a licensed SMPL-H
body model. `mc_mujoco` shows the moving human beside Kinova and draws the
distance arrows. `mc_rtc_gui` shows controller data but does not contain the
MuJoCo scene.

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
python3 -m venv ~/.venvs/smplh-playback
~/.venvs/smplh-playback/bin/pip install -r smplh_playback/requirements.txt
```

Set paths for one sequence and prepare it once. The example translation places
the human beside Kinova; adjust it for your scene. This creates a large cache.

```sh
export SMPLH_MODEL="$PWD/datasets/smplh/<model>.pkl" # try SMPLH_MALE
export AMASS_SEQUENCE="$PWD/datasets/CMU/<subject>/<trial>.npz" # try 01/01_01_poses
export SMPLH_CACHE="$PWD/datasets/smplh_cache/<name>" # try cmu_01_01

~/.venvs/smplh-playback/bin/python -m smplh_playback.prepare \
  --output "$SMPLH_CACHE" --translation 1.2 0 0
```

The cache contains `human.xml`, `surface.obj`, `smplh.yaml`, and playback
arrays. `--rotation RX RY RZ` (degrees), `--translation X Y Z` (meters), and
`--fps` let you align or retime another sequence. Preparation writes absolute
paths into `human.xml` and `smplh.yaml`; if you move a prepared cache, prepare
it again or update those paths.

## 3. Build and select the controller

Enable the native playback bridge and rebuild. The root README covers the
superbuild prerequisites and Kinova installation.

```sh
cmake --preset relwithdebinfo -DWITH_SMPLH_PLAYBACK=ON -DWITH_Kinova=ON # aka mc_superbuild_config & mc_setup
cmake --build --preset relwithdebinfo # aka mc_build
source ~/workspace/install/setup_mc_rtc.sh
```

Create or edit `~/.config/mc_rtc/mc_rtc.yaml` aka mc_config:

```yaml
MainRobot: Kinova
Enabled: KinovaHoldController
Timestep: 0.001
LogPolicy: threaded
GUIServer:
  Enable: true
```

`KinovaHoldController` holds the arm still for a first playback check. If you
already have a compatible Kinova controller, use its name under `Enabled`.
For a repeatable robot-motion check, set `Enabled: KinovaSweepController`,
open `mc_rtc_gui`, and press **Kinova sweep → Start**. Keep the human clear of
the arm for this first test. **Stop** holds the measured posture, and a MuJoCo
reset returns the sweep to its waiting state.

## 4. Add the human to the MuJoCo scene

Create `~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml` aka mc_mujoco_config with the human object, or add
`human` to your existing `objects` map:

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

Copy the generated module configuration beside it:

```sh
mkdir -p ~/.config/mc_rtc/mc_mujoco
cp "$SMPLH_CACHE/smplh.yaml" ~/.config/mc_rtc/mc_mujoco/smplh.yaml
```

`smplh.yaml` points to the cache's `human.xml`. Keep it paired with the same
`SMPLH_CACHE`: a different cache may have incompatible mesh topology.
The bridge applies the merged human body's pose to the rendered surface,
and distance samples. The configured
`init_pos` therefore affects the human's scene placement. Keep the cache and
its generated XML together because they must share the same mesh topology.

## 5. Run playback and check distances

In the terminal where you launch MuJoCo, source the installed environment and
export `SMPLH_CACHE` again if this is a new terminal. Set looping if you want
the motion to repeat.

```sh
source ~/workspace/install/setup_mc_rtc.sh
export SMPLH_CACHE="$PWD/datasets/smplh_cache/<name>"
export SMPLH_LOOP=1
mc_mujoco --sync
```

Without `SMPLH_CACHE`, the human object can load but native motion and
distance updates do not start. Keep the `mc_mujoco` window open to see the
human, endpoint dots, and arrows. The first distance sample arrives
asynchronously; wait briefly if no lines appear at startup. Open `mc_rtc_gui`
separately for the **SMPL-H distances** values when its server is enabled.

The default distance mode uses fast approximate convex patches (SCH). For
exact triangle-surface distances, export `SMPLH_DISTANCE_MODE=fcl` before
launch. The worker requests a distance sample every 25 physics steps by
default; `SMPLH_DISTANCE_EVERY=1` requests one every step at higher compute
cost. Distances are logged as `SMPLH_distance_<region>` in meters.

For region definitions, timing details, cache conversion, and SCH/FCL
tradeoffs, see [distance and cache notes](../doc/smplh-distance.md).

For the opt-in directional SSM+PFL simulation baseline, example regional
configuration, equations, and measured residuals, see
[Kinova SSM+PFL](../doc/ssm-pfl.md). The baseline requires the updated native
bridge and a prepared cache.

### Visible SSM+PFL slowdown example

The paired [sweep](kinova_slowdown_mc_rtc.yaml) and
[playback scene](kinova_slowdown_mc_mujoco.yaml) configurations use the `drink`
cache frozen at frame 1359. The sweep goes to joint 1 = -3 rad. Its deliberately
restrictive 20 N, 10,000 N/m, 0.4 m regional envelope makes the QP speed rows
bind during the outbound motion. These values are **simulation demonstration
parameters**, not calibrated force limits or a safe deployment configuration.
Use the ordinary non-contact `smplh.yaml`, not `smplh_contact.yaml`, so contact
forces do not obscure the QP slowdown.

From the repository root, after building and sourcing the playback-enabled
installation:

```sh
export SMPLH_CACHE="$PWD/datasets/smplh_cache/drink"
export SMPLH_FREEZE_FRAME=1359
export SMPLH_DISTANCE_EVERY=25
mkdir -p ~/.config/mc_rtc/controllers ~/.config/mc_rtc/mc_mujoco
cp smplh_playback/kinova_slowdown_mc_rtc.yaml \
  ~/.config/mc_rtc/controllers/KinovaSweepController.yaml
cp smplh_playback/kinova_slowdown_mc_mujoco.yaml \
  ~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml
cp "$SMPLH_CACHE/smplh.yaml" ~/.config/mc_rtc/mc_mujoco/smplh.yaml
printf 'xmlModelPath: "%s"\n' \
  "$(realpath ../install/share/mc_mujoco/kinova_gen3/kinova_table.xml)" \
  > ~/.config/mc_rtc/mc_mujoco/ground.yaml
mc_mujoco --sync -f smplh_playback/kinova_slowdown_mc_rtc.yaml
```

Press **Kinova sweep → Start** in `mc_rtc_gui`.
The **SSM+PFL robot approach speed** plot and **At QP speed bound** label show
the active limit; the sweep joint position and velocity in the log show the
resulting motion. For a direct comparison, rerun with only `SSMPFL.Enabled:
false` in the copied controller file. The `-f` global configuration loads
before that controller file, so edit the copied controller file for both runs.
Set `Sweep.AutoStart: true` there for repeatable headless runs.

On the local Kinova table installation, synchronized headless runs with that
single setting changed placed simulated joint 1 at -1.04 rad with SSM+PFL and
-2.15 rad without it at simulation time 4 s. The enabled run had a binding
regional row in 3,972 of 5,183 logged samples and no QP failure. The first 13
samples used the startup hold fallback while waiting for a distance snapshot.
The comparison demonstrates a slower sweep in this scene; it does not establish
a physical impact-force limit. Check `SSMPFL_fallback`, `SSMPFL_solver_success`,
`SSMPFL_binding_pairs`, and `Kinova_sweep_joint_1_simulated` when reproducing it.
