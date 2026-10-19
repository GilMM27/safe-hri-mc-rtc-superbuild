# Kinova SSM+PFL simulation baseline

This controller is a sampled, pre-contact research baseline inspired by Lucci
et al., *Combining Speed and Separation Monitoring With Power and Force Limiting
for Safe Collaborative Robotics Applications*, IEEE RA-L 2020,
[DOI:10.1109/LRA.2020.3010211](https://doi.org/10.1109/LRA.2020.3010211).
It does not establish ISO 15066 compliance or guaranteed collision-force
limits. The example force and stiffness values are simulation placeholders,
not verified biomechanical parameters.

## Architecture and coordinates

`KinovaSweepController` is the nominal posture-task controller at 2 ms. Its
Tasks QP retains the contact, dynamics (including model joint/torque limits),
and Kinova minimal self-collision constraint sets. SSM+PFL adds a fixed-size
general inequality with one row for each of the 14 SMPL-H body regions in the
existing distance worker. A different Kinova geometry can win for each region
and at each sample. The selected geometry's MuJoCo body name is mapped to the
mc_rtc link name, and its witness point is stored in body-local coordinates.
The RBDyn point Jacobian and its derivative are evaluated at that point using
the current controller model. `RBDyn::ForwardDynamics::computeH` supplies the
full fixed-base joint inertia for the same model and generalized-coordinate
ordering. A positive-definite LDLT solve supplies directional mobility.

The worker's witness positions and velocities are in the MuJoCo world frame.
The controller negates the worker's human-to-robot separation vector to get
`n`, from robot toward human. RBDyn's spatial Jacobian has angular rows first
and world translational rows last. With `J` denoting those last three rows:

```text
w = nᵀ J M(q)⁻¹ Jᵀ n              [1/kg]
mR = 1/w                         [kg]
vPFL = (Fmax / sqrt(k)) sqrt(w)   [m/s]
dEffective = max(d - margin, 0)   [m]
vLimit = dEffective/Tstop + vPFL  [m/s]
closing = nᵀ(J qdot - vH)         [m/s]
dt nᵀ J qddot <= vLimit - nᵀJ qdot - dt nᵀJdot qdot
```

The last inequality predicts the next sampled point velocity with current
`n`, witness, and robot speed limit held constant for one controller step.
Human approach is an external input and cannot make this robot-acceleration
row demand retreat. Human motion is logged for contact analysis but does not
stop the sweep based on a separate protective-distance threshold.
Every selected region row constrains the whole joint-acceleration vector.
Region retreat does not disable any other region's row. It can redistribute
joint motion and change the nominal path. No slack is added to these rows.
The Tasks backend is required and checked at controller construction.
`SSMPFL.ConstrainWhileHolding: false` keeps distance and speed monitoring active
while the sweep waits for Start or after Stop, but adds no SSM+PFL QP rows in
that phase. Rows become active on Start. The default is `true` for existing
configurations. Monitoring alone does not enforce a speed limit.

Near-zero nonnegative mobility yields zero PFL speed and infinite logged
reflected mass. The controller omits a QP row when the selected robot body has
no useful directional mobility, as with the fixed `base_link`. Such a row
cannot change the robot's speed. The measured relative closing residual and the region's
`SSMPFL_<region>_uncontrollable` flag remain logged; the GUI lists these
regions. An omitted row does not enforce the speed limit. At zero reported
clearance a controllable row uses the PFL speed. In approximate SCH mode,
overlapping convex patches use SCH's penetration witnesses and human-to-robot
penetration vector; the witness-point difference has the opposite sign after
overlap. In FCL mode, an intersection uses FCL's contact position and human-to-robot
contact normal. These contact witnesses support continued sampled speed
limiting, but a convex-hull overlap can precede mesh contact and neither mode
measures contact force. Degenerate contact normals remain invalid inputs.
This is still a directional speed branch, **not** the paper's near-contact
velocity-norm branch or post-contact force control.

For a graze, the velocity component along the surface can be large while
`nᵀ(vR-vH)` remains small or negative. The present row therefore does not cap
the full point speed at the arm. The logged
`SSMPFL_<region>_actual_relative_speed_mps` and
`SSMPFL_<region>_actual_tangential_speed_mps` expose this gap. A test fixture
at reported torso overlap measured 0.216 m/s relative point speed, including
0.197 m/s tangential speed, while normal closing speed was -0.073 m/s and the
normal-direction PFL limit was 0.156 m/s. Those two PFL/speed numbers use
different directions and should not be compared as a force guarantee; the
sample shows why the normal row did not slow a graze. Experimental Cartesian
and travel-direction QP rows slowed it but later made the hard QP infeasible,
so they were not retained. A feasible near-contact speed branch and earlier
trajectory slowdown remain required before this behavior can be called a
complete SSM+PFL controller.

Invalid or non-positive-definite inertia, invalid inputs, missing witnesses,
degenerate directions, and stale samples request the controller's Stop/hold
fallback by default. Set `SSMPFL.HoldOnConstraintUnavailable: false` to keep the
sweep running when constraint inputs are unavailable or invalid; the controller
clears all SSM+PFL rows for that tick and logs the reason in
`SSMPFL_fallback_reason`. Safety limits are not enforced on those ticks. QP
failure is logged and requests hold, but `mc_mujoco` then stops the run.
Neither holding a posture task nor requesting a stop proves the robot
physically stopped, especially if a human keeps approaching.
An automatic fallback stops the joint-1 sweep at the most recent valid sampled
MuJoCo angle when available, or at the controller model angle otherwise. It
keeps the existing targets for joints 2–7. The explicit **Stop** button captures
the measured posture of all seven joints.

## Human motion and actual-state check

Set `SMPLH_FREEZE_FRAME=0` to hold one pose, and choose `HumanMode: frozen`.
The controller rejects a nonfrozen snapshot in this mode.
For moving playback, omit that environment variable and choose
`HumanMode: moving`. The visual SMPL-H surface is a deforming vertex cache;
the MuJoCo skeleton joint velocities are set to zero by playback and do not
describe surface material-point motion. The bridge therefore calculates a
backward finite difference for each **identified mesh vertex** from the
current and preceding cached frame, without reading future frames. It assigns
the velocity of the nearest material vertex in the selected region to the
current human witness. It never differences successive closest-point
coordinates. There is no filtering. The estimate is quantized to the cache
frame period, then delayed by asynchronous distance processing and sampled
every `SMPLH_DISTANCE_EVERY` physics steps (25 by default, 50 ms at 2 ms).
It is invalid on the initial and wrap frames. A nearest vertex need not have
the velocity of an interior triangle or convex-hull witness; this is a major
moving-mode approximation.

`robot().mbc()` supplies the model state and QP prediction. The distance
snapshot also contains MuJoCo's world body velocity at the sampled robot
witness, calculated with `mj_objectVelocity` and the body-point angular offset.
The controller logs its **sampled actual residual** against the current pair's
limit. `MaxActualResidualMps` marks a diagnostic violation in
`SSMPFL_<region>_actual_violation` and `SSMPFL_actual_violation_pairs`; it
does not stop the sweep. `SSMPFL_<region>_intersecting` marks reported geometric
overlap. `SSMPFL_sample_age_seconds` shows the sample age. This check is not
the next-step physical residual and cannot remove latency or tracking errors.
Sampled MuJoCo joint positions and velocities are logged separately as
`Kinova_sweep_joint_*_simulated`. The legacy `*_measured` key is an alias for
the mc_rtc model state, also available as `*_model`.
`MaxModelWitnessErrorM` detects a large disagreement between the sampled
MuJoCo witness and the current mc_rtc model point; it also catches frame/name
mismatches, although ordinary motion between sample and control tick consumes
part of this tolerance.

The residual monitor is diagnostic. A positive sampled residual means the
simulated point moved faster toward that region than the configured envelope
allowed at the sample. The direct QP row constrains the controller model;
simulation tracking, sample age, and changing closest points can violate the
envelope in MuJoCo. The controller does not claim a physical PFL guarantee.

## Geometry and model limits

The native worker considers Kinova collision geoms in MuJoCo group 2 whose
body names start with `kinova_` by default. It returns **one closest robot
geom per human region**, not every geom-region pair. The 14 human regions are
mesh-triangle partitions from the SMPL-H skinning weights. The default
`sch_approx` path uses convex patches and convex robot meshes. Its clearance
may be smaller than exact surface clearance, and its witness may lie on a
convex hull. `SMPLH_DISTANCE_MODE=fcl` uses the triangle surface instead.
Neither mode selects the pair with the worst directional velocity margin, so
the QP does not cover every possible pre-contact witness. Attached tools or
payloads must be represented consistently in both the robot inertia model and
monitored MuJoCo geometry; this has not been validated for a payload.

`Tstop` is a configured braking-time assumption, not an online braking
estimate. The distance margin is the only explicit allowance for latency,
sensor uncertainty, and geometry error; its adequacy is unverified. Adding
instantaneous `vH` extends the stationary-human envelope but gives no safety
proof over the braking horizon if human motion changes. An approaching human
can make the row infeasible even for a stationary robot. Sustained clamping,
effective-mass variation, and post-contact force are outside this baseline.

## Build and run

Start with the [SMPL-H playback guide](../smplh_playback/README.md) and a
prepared licensed cache. The bridge installer is idempotent for the current
external `mc_mujoco` source:

```sh
python3 smplh_playback/install_bridge.py --source ~/workspace/src/mc_mujoco
cmake --build ~/workspace/build/mc_mujoco -j2
cmake --install ~/workspace/build/mc_mujoco --prefix ~/workspace/install
cmake -S kinova_hold_controller -B /tmp/kinova-ssm-build \
  -DCMAKE_PREFIX_PATH=~/workspace/install
cmake --build /tmp/kinova-ssm-build -j2
cmake --install /tmp/kinova-ssm-build --prefix ~/workspace/install
```

Merge the `SSMPFL` block from
[the example configuration](../smplh_playback/KinovaSweepController.ssm_pfl.example.yaml)
into `~/.config/mc_rtc/controllers/KinovaSweepController.yaml`, preserving
any existing Sweep settings. Create that file from the example if it does not
exist. Set
`Enabled: KinovaSweepController`, `MainRobot: Kinova`, and `Timestep: 0.002`
in `mc_rtc.yaml`. Enable the human object in `mc_mujoco.yaml` as in the
playback guide. Start `mc_mujoco --sync`, then press **Kinova sweep → Start**
in `mc_rtc_gui`. `Sweep.AutoStart: true` supports a scripted headless run.

```sh
export SMPLH_CACHE="$PWD/datasets/smplh_cache/<prepared-sequence>"
export SMPLH_DISTANCE_EVERY=25
export SMPLH_FREEZE_FRAME=0
mc_mujoco --sync --without-visualization --without-mc-rtc-gui -f /tmp/ssm-pfl-frozen.yaml
# For moving playback: unset SMPLH_FREEZE_FRAME and set HumanMode: moving.
```

The `-f` file can contain the global mc_rtc entries followed by the example
controller configuration. A user controller override loads after that file,
so inspect its Sweep values when reproducing a run. For isolated headless
validation, `ClearGlobalPluginPath: true` prevents the optional ROS autoload.
Use `mc_bin_utils show <log.bin>` to list signals and
`python3 smplh_playback/analyze_ssm_pfl.py <log.bin>` to summarize sampled
predicted robot-speed and observed relative-closing residuals. A positive
predicted residual exceeds the robot-speed QP bound; a positive observed
residual means human and robot closed faster than that robot-speed bound,
which may be caused entirely by human motion. The sample age and per-region witness coordinates,
velocities, inverse mass, limits, and residuals are logged in the bin file.
`SSMPFL_binding_pairs` counts nonzero-mobility rows whose predicted residual
is within 0.001 m/s of the bound; `SSMPFL_limiting_pair` identifies the
closest such row even when none is binding.
`SSMPFL_uncontrollable_pairs` counts selected pairs with no useful directional
mobility. These pairs are monitored but have no QP row.
`SSMPFL_monitor_only` and `SSMPFL_monitor_only_pairs` identify waiting-state
samples when `ConstrainWhileHolding` is disabled. The per-region
`SSMPFL_<region>_monitor_only` keys distinguish these rows from active and
uncontrollable ones.
The **SSM+PFL** GUI panel lists the regions at the QP speed bound and the
limiting row's current model robot approach speed, solved next-step robot
approach speed, sampled MuJoCo robot approach speed, and allowed robot speed.
The **SSM+PFL robot approach speed (m/s)** live plot shows these four values.
The MuJoCo value belongs to the distance sample and can lag the current
control tick. Positive robot approach is toward the human; negative values
are away. Human approach remains visible in the panel and in the relative
closing log. The MuJoCo trace is magenta for visibility. The log records
the relative closing values as
`SSMPFL_limiting_closing_mps`, `SSMPFL_limiting_qp_next_closing_mps`, and
`SSMPFL_limiting_sampled_closing_mps`, and `SSMPFL_limiting_speed_limit_mps`,
plus `SSMPFL_at_speed_bound` and
`SSMPFL_binding_regions`. A row at its bound is evidence that the solved QP
used the full allowance; it does not by itself prove that the row changed the
robot's motion compared with an unconstrained solve.

Focused mathematical checks:

```sh
g++ -std=c++17 -O2 -I/usr/include/eigen3 tests/ssm_pfl_math.cpp -o /tmp/ssm-pfl-math
/tmp/ssm-pfl-math
```

## Validation on 2026-10-15

The installed `mc_mujoco` and controller were rebuilt and run with the local
prepared `drink` cache (1603 frames at 120 Hz), 2 ms physics and controller
steps, 25-step distance requests, and one joint-1 sweep. The local controller
override used a -1 rad sweep. All times below are simulation samples from
bounded, synchronized headless runs; durations varied from about 2.4 to 7.4 s. The
example F/k values are unverified placeholders. These runs used the installed
plain-floor environment: its `ground.yaml` had not yet been updated to select
the repository's Kinova table scene. Revalidate the motion and residuals with
the restored table environment before comparing table-scene behavior.

| Scenario | Complete 14-region samples | Max predicted residual | Max sampled MuJoCo residual | Result |
| --- | ---: | ---: | ---: | --- |
| Frozen frame, example envelope | 3677/3693 | -0.902 m/s | -0.785 m/s | MuJoCo joint 1 reached -0.970 rad; no solver failure |
| Moving playback, example envelope | 3658/3693 | -0.859 m/s | -0.782 m/s | MuJoCo joint 1 reached -0.970 rad; first-frame velocity invalid until next frame |
| Frozen, deliberately restrictive 10 N / 2 m margin | 2052/2191 | 0 m/s | +0.0363 m/s | Stop/hold fallback triggered; model joint 1 reached -0.471 rad, sampled MuJoCo joint 1 reached -0.489 rad |
| Frozen mode with unfrozen playback | 0/1193 | n/a | n/a | Mode mismatch requested Stop/hold; joint 1 stayed at 0 rad |

The near-contact threshold test from an earlier version is omitted because
that automatic hold has been removed. The restrictive run above used the
earlier residual-triggered hold; the current version only reports those
sampled violations.

Additional checks: 14 distinct region rows were present whenever samples were
valid; selected Kinova links included base, shoulder, half-arm, forearm,
spherical wrist, and bracelet. The selected link switched as the arm moved.
The left-upper-arm region's reflected mass varied from about 1.70 to 52.7 kg
in one sweep. With the robot stationary and moving mocap, `nᵀvH` ranged from
-0.315 to +0.696 m/s, and relative closing speed reached +0.315 m/s while
robot approach speed remained zero. In the moving sweep, 1852 valid samples
contained at least one pair with robot and human approaching each other.
The mathematical checks cover the directional mass solve, force/stiffness
scaling, margins, human-approach sign, two opposing regions, and invalid
stiffness.

Still unverified: a physical scenario with an intermediate link approaching
while the end effector retreats from the same region; independent numeric
checks of model torque and self-collision margins; calibrated stopping bounds,
payload inertia, and robust velocity estimates for deforming skin. Next steps
are braking-bound measurement, better human-motion uncertainty handling,
path-preserving trajectory scaling, and effective-mass variation constraints.
