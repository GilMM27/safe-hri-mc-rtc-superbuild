# SMPL-H distance and cache notes

The [playback guide](../smplh_playback/README.md) covers installation and launch.
These notes describe cache behavior and distance measurement choices.

## Cache and playback

Python resolves the licensed AMASS motion and SMPL-H model once. The resulting
cache stores poses, vertices, faces, regions, an MJCF human, and a binary native
cache. The native bridge reads it directly; `mc_mujoco` remains the simulator.
The human is kinematic, and its display and debug geoms have no contact bits.

Playback selects `floor(time * fps)`, so a simulation reset restores frame
zero. `SMPLH_LOOP=1` wraps frame indices; otherwise the final frame is held.
The bridge applies a free-root pose and 51 ball-joint poses. It refreshes the
visual surface when the motion frame changes.

New caches include `cache_format: 1` and frame, vertex, face, and region counts
in `metadata.json`. To validate and upgrade an older cache without rerunning
SMPL-H preprocessing:

```sh
python -m smplh_playback.convert_cache datasets/smplh_cache/old-cache
```

`--output datasets/smplh_cache/new-cache` copies arrays and upgrades metadata
into a separate directory. Large `.npy` arrays remain memory-mappable.
`SMPLH_CACHE` accepts a cache directory or its `smplh_cache.bin` file.

If licensed data are unavailable, a synthetic fixture can test the scene,
bridge, worker, and logging path:

```sh
python -m smplh_playback.make_smoke_fixture --output /tmp/smplh-smoke-cache
```

It is not a human surface and must not be used for distance research. The
`smplh_playback/kinova_smoke_mc_mujoco.yaml` and
`smplh_playback/kinova_smoke_mc_rtc.yaml` files provide isolated Kinova
settings for a smoke run.

## Regions and measurements

Each triangle belongs to the region with the largest average SMPL-H skinning
influence. Defaults include head, torso, upper arm, forearm, hand, thigh,
lower leg, and foot on both sides. `regions.npz` stores face IDs. Supply
`--regions regions.json` to `smplh_playback.prepare` to use a JSON object
mapping region names to SMPL-H joint indexes.

The native worker processes the latest immutable human-pose and robot-transform
snapshot in the background. Before each controller run, completed results are
published as `SMPLH::DistanceSnapshot` in the datastore. Results include sample
age, motion frame, readiness, per-region and per-link closest points, selected
robot geom, and clearance. The playback bridge compares each human region with
MuJoCo group 2 geometry on the seven movable Kinova joint links. The fixed
base is excluded. Fixed descendants belong to their nearest joint link. Each
pair retains the closest geom on that link. For a valid non-intersecting result,
the unit direction is
`(robot_point - human_point) / distance`. Intersections report zero clearance
and have no reliable separation direction. The first result is unavailable
until the asynchronous query finishes.

The MuJoCo viewer draws the closest pair per human region as an arrow with
cyan/orange endpoints. The standalone `mc_rtc_gui` lists each region's closest
link and robot geometry under **SMPL-H distances**; it does not display the
MuJoCo scene. The mc_rtc
log records the minimum across links as `SMPLH_distance_<region>` in meters and
`SMPLH_distance_approximate`.

The native worker requests a sample every 25 physics steps by default
(`SMPLH_DISTANCE_EVERY=25`): 40 Hz at the configured 1 ms physics step.
`SMPLH_DISTANCE_EVERY=1` requests every step. Worker throughput can limit the
actual update rate. The articulated pose still follows simulation time, while
vertices and normals are transferred only when the motion frame changes. The
separate Python reference server uses `--distance-every` and `--log-every` for
JSONL output; install `requirements-test.txt` for that reference suite.

## SCH and FCL

The default `SMPLH_DISTANCE_MODE=sch_approx` measures each region using
connected convex patches with SCH. `SMPLH_SCH_PATCH_FACES` sets the maximum
triangle count per patch (default 64). Patch membership stays fixed while
vertices update for each motion frame. Robot mesh geoms are treated as convex
hulls in this mode. The snapshot, status panel, and mc_rtc GUI label the result
as approximate. Closest points can lie on a patch hull rather than the human
surface, and triangle normals are unavailable. A concave surface can have
positive clearance while its convex patch reports intersection and zero
clearance. Smaller patches may reduce this error at the cost of more work.

Set `SMPLH_DISTANCE_MODE=fcl` to measure exact triangle-surface distance. The
worker orders robot geom pairs by current-pose bounding-box distance and skips
pairs that cannot improve a region's minimum. Regional RSS BVHs contain only
referenced vertices and are refitted top-down for each pose. The tested FCL
0.7.0 installation produced disagreements with fresh builds when refitting
bottom-up on deforming meshes, so bottom-up refitting is unused.

In FCL mode, `SMPLH_DISTANCE_USE_SCH=1` enables an additional SCH convex-hull
bound that can skip more FCL pairs. The snapshot reports `sch_pruned_pairs` and
`sch_duration_seconds`. This bound is off by default because it was slower in
the synthetic benchmark. The MuJoCo status panel shows sample age, query
duration, and queried/total pairs. `prep`, `collide`, and `distance` split
geometry preparation from FCL calls; total time also includes sorting and
result processing.

`benchmarks/bench_smplh_sch.cpp` compares paths on a synthetic animated surface
with 14,400 triangles and 21 robot spheres. Across 30 samples on the tested
machine, original OBBRSS/FCL averaged 27.1 ms, RSS/FCL 20.7 ms, RSS with SCH
pruning 22.2 ms, and approximate SCH patches 0.65 ms. The approximate distance
differed by 0.21 mm on average and 0.35 mm at worst in that scene. SCH pruning
skipped 20 of 21 FCL pairs but cost 2.7 ms per sample; RSS refitting cost about
19.4 ms. Real SMPL-H motion and different hardware will produce other timings.

SMPL-H is a reconstructed body estimate. Clothing, soft tissue, and fitting
errors are absent. Validate region policy and margins for the intended system
before using these distances beyond research measurements.
