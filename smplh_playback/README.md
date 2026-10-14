# SMPL-H / AMASS human playback

This directory adds a playback path with three separate representations:

```
AMASS sequence -> SMPL-H cache -> kinematic MuJoCo human
                              -> SMPL-H triangle regions -> FCL distance log
                              -> optional small MuJoCo spheres for display/debug
```

`mc_mujoco` remains the only simulator. Python prepares the licensed AMASS and
SMPL-H data offline. Native `mc_mujoco` playback reads the generated binary
cache directly and updates the kinematic human and its display surface.

## External data and Python environment

Obtain an AMASS sequence and the SMPL-H neutral/male/female model from their
respective licensors. Do not add either to this repository. Create a separate
virtual environment and install the preprocessing dependencies:

```sh
python3 -m venv ~/.venvs/smplh-playback
~/.venvs/smplh-playback/bin/pip install -r smplh_playback/requirements.txt
```

For SMPL-H model files, use the path to the corresponding `.pkl` file (for
example `SMPLH_MALE.pkl`) or its containing model directory. AMASS input must
be an `.npz` sequence containing `poses` with 156 values per frame, `trans`,
`betas`, `gender`, and `mocap_framerate`.

If the licensed files are not yet available, create a synthetic fixture for
native playback development. It is deliberately not a human surface and must
not be used for distance research:

```sh
python -m smplh_playback.make_smoke_fixture --output /tmp/smplh-smoke-cache
```

Prepare a cache outside the repository. This resolves SMPL-H once and keeps
the high-resolution vertices and faces for every frame:

```sh
SMPLH_MODEL=/secure/models/SMPLH_MALE.pkl \
AMASS_SEQUENCE=/secure/amass/sequence_01.npz \
~/.venvs/smplh-playback/bin/python -m smplh_playback.prepare \
  --output /tmp/amass-cache --translation 1.2 0 0
```

New caches include `cache_format: 1` and explicit frame, vertex, face, and
region counts in `metadata.json`. To validate and upgrade a cache made by an
older version without rerunning SMPL-H preprocessing, use:

```sh
python -m smplh_playback.convert_cache /path/to/old-cache
```

Use `--output /path/to/new-cache` to copy the arrays and upgrade metadata into
a separate directory. The large `.npy` arrays remain memory-mappable.

Set `--fps` to change playback rate, `--rotation RX RY RZ` and
`--translation X Y Z` to align AMASS coordinates with the Kinova scene, and
`--regions regions.json` to replace the joint-influence body-region mapping.
The generated `smplh.yaml` exposes `human.xml` to `mc_mujoco` as an object.

## Build and run

Configure the superbuild with `-DWITH_SMPLH_PLAYBACK=ON` and rebuild. This
applies the native playback bridge to the external `mc_mujoco` source.

Create `~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml` (or use your local
mc_mujoco configuration directory) with the generated object module:

```yaml
objects:
  human:
    module: smplh
    init_pos: [0, 0, 0, 1, 0, 0, 0]
```

Copy `/tmp/amass-cache/smplh.yaml` to the same configuration directory as
`smplh.yaml`. Set the cache path and start `mc_mujoco`:

```sh
export SMPLH_CACHE=/tmp/amass-cache
export SMPLH_LOOP=1 # optional; omit to hold the final frame
mc_mujoco --sync
```

`kinova_smoke_mc_mujoco.yaml` and `kinova_smoke_mc_rtc.yaml` are minimal
Kinova settings for an isolated smoke run. They are provided so an existing
mc_mujoco object configuration need not be replaced.

`SMPLH_CACHE` may name the cache directory or its `smplh_cache.bin` file.

## Playback and measurements

The native bridge selects `floor(time * fps)`, so simulation reset
deterministically restores frame zero. `SMPLH_LOOP=1` wraps frame indices;
without it playback holds the final frame. It applies the free-root pose and all
51 ball-joint poses and updates the visual surface when the frame changes. The
human remains kinematic and its display/debug geoms have no contact bits.

The native worker evaluates the latest immutable pose and robot-transform
snapshot in the background. Completed samples are assigned to
`SMPLH::DistanceSnapshot` in the controller datastore immediately before each
controller run. Results include readiness, sequence, motion frame, simulation
sample time, and per-region closest points, direction, normal validity,
intersection, distance, and closest robot geometry.

Regions are assigned by the largest average SMPL-H linear-blend-skinning
influence over each triangle. The defaults provide head, torso, upper arm,
forearm, hand, thigh, lower leg, and foot on both sides. `regions.npz` stores
the exact face IDs, and a custom JSON object mapping region names to SMPL-H
joint indexes changes the policy without changing code.

The native worker reports each region's minimum surface distance,
closest points, unit direction `(robot_point - human_point) / distance`,
selected robot geom, and nearest triangle normal when available. Intersections
have zero clearance and no reliable closest points or separation direction.
The Python worker remains as a reference implementation for tests; install
`requirements-test.txt` only when running that reference suite.

The MuJoCo viewer draws a green arrow between each region's closest human and
Kinova points, with cyan and orange endpoint markers. These markers are only
available when a non-intersecting distance sample has valid closest points.
The MuJoCo status panel reports the latest sample age and FCL query duration.
At a 1 ms timestep, the default `SMPLH_DISTANCE_EVERY=25` requests a sample
every 25 ms; lower this value (for example, `SMPLH_DISTANCE_EVERY=5`) when the
query duration is comfortably below the interval. If query duration exceeds the
interval, the worker is the refresh-rate limit.
With `KinovaHoldController`, the separate mc_rtc GUI also shows per-region
distances and closest robot geometry under **SMPL-H distances**, and draws the
closest-point arrows under **SMPL-H closest points**. Distances are recorded in
the mc_rtc log as `SMPLH_distance_<region>` in meters (for example,
`SMPLH_distance_left_hand`) and can be plotted in mc_rtc log tools. The initial
value is unavailable until the first asynchronous distance sample completes.

The SMPL-H surface is a reconstructed body estimate, with clothing, soft tissue,
and model-fitting errors absent. Treat its distances as a research measurement,
then validate any ISO/TS 15066 region policy and margins for the intended
system.
