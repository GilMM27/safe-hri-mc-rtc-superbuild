# Kinova SSM+PFL simulation baseline

This controller is a sampled, pre-contact research baseline inspired by Lucci
et al., *Combining Speed and Separation Monitoring With Power and Force Limiting
for Safe Collaborative Robotics Applications*, IEEE RA-L 2020,
[DOI:10.1109/LRA.2020.3010211](https://doi.org/10.1109/LRA.2020.3010211).
It does not establish ISO 15066 compliance or guaranteed collision-force
limits. The example force and stiffness values are simulation placeholders,
not verified biomechanical parameters.

## Architecture and coordinates

`KinovaSweepController` is the nominal posture-task controller at 1 ms. Its
Tasks QP has a posture task, SSM+PFL rows, and the controller's contact,
dynamics, and self-collision constraint sets. The dynamics constraint also
includes joint limits. mc_rtc may also load global constraint configuration.
Enabling relaxation additionally installs hard acceleration bounds from an
explicit research cap intersected with any model acceleration limits. The
relaxation mode does not remove the existing joint, dynamics, contact, or
self-collision constraints. SSM+PFL adds a fixed-size
one-sided inequality with one row slot for each of the 14 SMPL-H body regions paired
with each of the seven movable Kinova links (joints 1–7). The distance worker
returns the closest geometry on each link for each region. The selected
geometry's MuJoCo body name is mapped to the
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
dt nᵀ J qddot <= vLimit - nᵀ(J qdot - vH) - dt nᵀJdot qdot
```

For frozen humans, the last inequality predicts next-step relative closing speed with current
`n`, witness, sampled human velocity, and speed limit held constant for one
controller step. Human motion toward the robot tightens the bound and can
require robot retreat. It is not a separate protective-distance threshold.

For `HumanMode: moving`, every region/link pair constrains only the robot
point's next-step approach speed. Human approach is negative along
`n`; the row budget is `b = max(0, vLimit + nᵀvH)`. With
`vNext = nᵀJ qdot + dt(nᵀJ qddot + nᵀJdot qdot)`, the QP enforces
`vNext <= b`. At contact, `vLimit` equals the pair's PFL limit.
An approaching human faster than that limit makes `b = 0`, preventing
positive robot approach at that point while allowing retreat. A slower or
receding human restores the budget continuously. Other points and tangential
motion remain free within their own constraints. Human motion can still make
relative contact speed exceed PFL even while the robot point is stationary;
the sampled relative-speed and force measurements retain that exceedance.
If hard acceleration or collision limits prevent the next-step bound, the QP
reports infeasibility unless explicit relaxation is enabled. Relaxed speed
allowances and original-bound exceedance are logged separately.

For frozen humans, set `SSMPFL.ConstraintMode: temporal_barrier` to anticipate a changing speed
limit. For each region/link pair, the controller uses consecutive fresh distance
samples to estimate the rate of directional mobility and of `nᵀJ`. With
`h = vLimit - nᵀ(J qdot - vH)`, it adds a QP row

```text
nᵀJ qddot <= vLimitRate - beta + gamma h  [m/s²]
beta = d(nᵀJ)/dt · qdot                   [m/s²]
vLimitRate = dEffectiveRate/Tstop + vPFLRate
```

`dEffectiveRate` uses `nᵀ(vH - J qdot)` above the distance margin and zero
below it. At the margin it uses the smaller one-sided rate.
`vPFLRate` uses the sampled change in directional mobility to predict the
change in the nonnegative PFL allowance over one controller step. This finite
step difference avoids the singular derivative of `sqrt(w)` as mobility
approaches zero. The
derivative history is kept separately for every region/link pair, so a switch
of the closest geometry does not mix the histories of different links.
Startup, stale or repeated samples, overlap, and large witness or normal
changes can invalidate a derivative. The controller also rejects a mobility
derivative whose one-step extrapolation reaches zero, where the PFL-rate
linearization is unreliable. When a pair's derivative is unavailable, frozen
mode retains an acceleration barrier using the model-estimated distance-rate
term and zero estimated PFL rate. Moving mode uses the pointwise speed rows
above and does not require a human-approach derivative. The
`derivative_fallback` log key applies to frozen temporal mode.
The frozen fallback requests gradual recovery if speed is already over the
limit; neither fallback guarantees a contact-force bound. Invalid distance
snapshots or QP inputs still clear all rows and follow
`HoldOnConstraintUnavailable`. The default `legacy` mode uses next-step speed
rows for every pair.
`BarrierGammaPerSecond`, `MaxDerivativeGapSeconds`, `MaxWitnessStepM`, and
`MaxNormalStepRad` configure the estimate and are shown in the example YAML.
The rate estimate and barrier are experimental and do not guarantee sampled
speed-envelope satisfaction.

## Opt-in minimum-violation recovery

`Relaxation.Mode: two_stage_full_feasibility` retains the weighted per-pair
Stage 1 slack and checks it against the complete Tasks QP. On a failed full
solve, it temporarily disables only the SSM+PFL rows and solves without
integrating the robot state. If the remaining hard constraints are feasible,
that solution supplies a finite upper bound on the extra shared allowance
needed for every active safety row. Up to two non-integrating probes
reduce the bound, subject to a 0.5 ms refinement budget. The final solve uses
the smallest verified feasible bound and is the only successful solve that
advances the state. The allowance is approximate and may be larger than the
minimum, especially when the time budget prevents refinement.

This mode applies to moving pointwise speed, frozen legacy speed, and frozen
temporal barrier rows. The extra allowance is in m/s for speed rows and m/s²
for temporal rows. It is not capped: moving-mode `MaxStage2SlackMps` is only
an exceedance threshold in this mode, logged by
`SSMPFL_full_qp_exceeded_legacy_cap`. `SSMPFL_full_qp_extra_slack` records the
shared amount beyond the per-pair `stage1_relaxation_physical` values; each
pair's `relaxation_physical` records its final permitted allowance. If the hard-only QP fails, extra safety
slack cannot fix it and the controller requests a hold. A solver failure on
the final solve also requests a hold. This mode does not establish a 1 ms
worst-case execution time or a physical safety guarantee.

This experimental recovery mode is opt-in for both frozen and moving humans.
For moving humans, Stage 1 uses one nonnegative slack per pair on the
approach-side point-speed row: `vNext <= b + delta`. There is no PFL
retreat-side speed row, so retreat remains subject to the other QP limits. If
the full mc_rtc QP still fails because Stage 1 cannot see all dynamics and
collision rows, Stage 2 retries with a shared allowance up to
`MaxStage2SlackMps` (default 0.2 m/s). The first feasible full QP command is used once; successful QP
solves advance the robot state, so the controller does not tighten and resolve
again in the same tick. This recovery is bounded feasibility search, not a
minimum-violation solution. It logs the full bound expansion and original
point-speed residual, even when the solved command uses less than the allowed
slack. Failed intermediate probes do not produce QP error messages; a final
failure remains an error and requests the configured posture hold.

The per-pair approach slack, original residual, predicted robot speed, and QP
status make that exception visible. The `retreat_relaxation_mps` and
`retreat_relaxed_pairs` log fields remain zero because no retreat row is present.
If the approach row still conflicts with hard acceleration, dynamics, or collision
limits after recovery, the QP fails and the controller reports it.

Set `SSMPFL.Relaxation.Enabled: true` with `Mode: two_stage_regularized` to
retain all active pair rows when their acceleration demands conflict. The
default remains hard enforcement. The Tasks backend fixes its decision vector
to robot acceleration and contact forces, so the controller runs a separate
eigen-qld QP before the ordinary mc_rtc solve. Its fixed pair index is
`7 * region_index + link_index` and its variables are the seven joint
accelerations and 98 normalized pair slacks. For an active pair, the first QP
uses

```text
A_i qddot - scale_i z_i <= b_i,   z_i >= 0,   delta_i = scale_i z_i
minimize 1/2 sum_i weight_i z_i² + epsilon/2 ||qddot||²
```

The inactive pair's slack is fixed to zero and its row is disabled. A first
feasibility solve applies the original rows and fixes every slack at zero when
that problem is feasible. Otherwise the relaxation QP minimizes the displayed
objective under the configured acceleration bounds. The ordinary mc_rtc QP
then optimizes its existing posture task with
each approach row capped by `A_i qddot <= b_i + delta_i + Stage2Tolerance` when Stage 1
needed relaxation. A feasible original-row solve uses `A_i qddot <= b_i`
without a tolerance expansion. Thus the
posture task cannot add approach-side violation beyond that explicit tolerance on each pair.
Both stages use one robot state and one set of pair rows; only the second solve
updates the robot command.
The Tasks row is multiplied by `1/max(0.001, ||A_i||)` in relaxation mode to
improve QLD conditioning. Stage 1 and all slack/residual logs retain the
original physical units; both sides of the Stage 2 inequality receive the
same multiplier.

The Kinova model in the tested installation reports infinite acceleration
limits. `Relaxation.MaxAccelerationRadS2` is therefore required when enabling
this mode; it intersects the model acceleration limits and is applied in both
QPs. The example's 10 rad/s² is a research setting, not a measured actuator
capability. Calibrate it before interpreting the command as physically
admissible. The auxiliary Stage 1 QP currently includes the explicit
acceleration box and SSM+PFL rows, but does not mirror the controller's
dynamics, joint, contact, or self-collision rows. Stage 1 success therefore
does not prove that the ordinary Tasks QP in Stage 2 is feasible. A Stage 2
failure remains possible even with zero or positive pair slack.

This is **two-stage regularized priority**, not an exact lexicographic
minimum of slack alone: eigen-qld requires a positive-definite QP Hessian, so
the small configurable acceleration tie breaker can alter the optimum when
zero slack is impossible. The first feasibility solve preserves exact hard
enforcement when the original rows are feasible. `Stage2Tolerance` also allows
a small additional physical-unit row violation. These two approximations are
reported rather than described as strict priority. The fallback remains
active for invalid or stale samples, infeasible model bounds, and solver
failure. This mode rejects additional mc_rtc global contacts, collision, and
constraint sets at construction because their hard rows cannot be mirrored in
the auxiliary QP. It does not turn a posture hold into a verified emergency
stop.

For `temporal_barrier`, `delta` and `Stage2Tolerance` are in m/s² and the
relaxed barrier condition is `hdot + gamma*h >= -delta`. For `legacy`, the
actual row is multiplied by `dt`, so they are in m/s. Moving pointwise
approach slack and the Stage 2 retry cap are also in m/s. `DefaultScale` has the
same respective units, and `delta/scale` is dimensionless. Default weights
are equal for all pairs; optional `Relaxation.Regions.<region>.Weight` and
`.Scale` need measured justification. No finite upper bound is placed on an
active slack. An upper bound could make the recovery QP infeasible again.
`ActivationTolerance` and `RecoveryTolerance` apply only to reporting
hysteresis, not constraints. A negative logged `barrier_mps` means the current
speed exceeds the chosen envelope; positive `relaxation_physical` means the
requested derivative condition was relaxed. Either can occur without the
other. Per-pair logs include original and relaxed QP residuals, weight, scale,
slack, derivative status, and sample age. Global logs expose relaxed pair
count, largest normalized slack, objective, solver statuses, timings, and
recovery status. Positive slack is not evidence of safety or a guarantee of
eventual recovery or minimum collision severity.

## Pair diagnostics and monitoring

For `temporal_barrier`, `SSMPFLPair_<region>_<link>_acceleration_residual_mps2`
reports the solved QP row residual. The same prefix exposes derivative
validity, feature switches, the limit rate, and the barrier value. The
`predicted_residual_mps` key is NaN in temporal mode because its units would
otherwise be misleading. `analyze_ssm_pfl.py` reports acceleration residuals
separately from sampled speed residuals.
Every selected region row constrains the whole joint-acceleration vector.
Region retreat does not disable any other region's row. It can redistribute
joint motion and change the nominal path. In default hard mode, no slack is
added to these rows.
The Tasks backend is required and checked at controller construction.
`SSMPFL.ConstrainWhileHolding: false` keeps distance and speed monitoring active
while the sweep waits for Start or after Stop, but adds no SSM+PFL QP rows in
that phase. Rows become active on Start. The default is `true` for existing
configurations. Monitoring alone does not enforce a speed limit.

Near-zero nonnegative mobility yields zero PFL speed and infinite logged
reflected mass. The controller omits a QP row when a selected point has
no useful directional mobility. Such a row cannot change the robot's speed.
The measured relative closing residual and the region's
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
`SSMPFL_<region>_actual_tangential_speed_mps` expose this gap. A feasible near-contact speed branch and earlier
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
`HumanMode: moving`. Moving mode uses pointwise speed bounds regardless of an
older `ConstraintMode` setting; `pointwise_speed` is the explicit setting in
the example. The visual SMPL-H surface is a deforming vertex cache;
the MuJoCo skeleton joint velocities are set to zero by playback and do not
describe surface material-point motion. The bridge therefore calculates a
backward finite difference for each **identified mesh vertex** from the
current and preceding cached frame, without reading future frames. It assigns
the velocity of the nearest material vertex in the selected region to the
current human witness. It never differences successive closest-point
coordinates. There is no filtering. The estimate is quantized to the cache
frame period, then delayed by asynchronous distance processing and sampled
every `SMPLH_DISTANCE_EVERY` physics steps (25 by default, 25 ms at 1 ms).
It is invalid on the initial and wrap frames. With
`HoldOnConstraintUnavailable: true`, the controller temporarily pauses the
sweep for those frames and resumes it when a valid velocity sample arrives.
A nearest vertex need not have
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

The native worker considers MuJoCo group 2 geoms on the seven movable Kinova
links, with body names starting `kinova_`. The fixed base is excluded. Fixed
descendants belong to the nearest joint link. It returns **one closest geom
per human-region/link pair**, for 98 reported pairs and up to 98 active QP
rows. The 14 human regions are mesh-triangle partitions from the SMPL-H
skinning weights. The default
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
`Enabled: KinovaSweepController`, `MainRobot: Kinova`, and `Timestep: 0.001`
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
predicted point-speed and observed relative-closing residuals. In moving mode,
a positive predicted residual exceeds the robot approach-speed bound;
a positive observed residual
means human and robot closed faster than that bound. The sample age and per-region witness coordinates,
velocities, inverse mass, limits, and residuals are logged in the bin file.
Each region/link pair also logs distance, closing speed, limit, residuals, and
row status under `SSMPFLPair_<region>_<link>_*`. The existing
`SSMPFL_<region>_*` keys report the closest link for that region.
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
The **SSM+PFL robot approach speed (m/s)** live plot shows these four values
for the pair selected in **Plot region/link** (initially `head/joint_7`).
Selection resets the plot history, so traces never splice values from different
pairs together. Its red speed limit is the allowed robot approach speed:
relative closing limit plus the human velocity along the contact normal.
During an unavailable distance sample or QP solve, traces hold their last
valid values; **Plot sample age** reports how old the distance and QP values
are. Held values are for display only and do not affect the constraints.
The MuJoCo value belongs to the distance sample and can lag the current
control tick. Positive robot approach is toward the human; negative values
are away. Human velocity remains visible in the panel and in the relative
closing log. The MuJoCo trace is magenta for visibility. The log records the
relative closing values as
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

The measurements in this section used the earlier single-closest-link row per
human region. They are historical results and do not validate the current
per-link QP.

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

Additional checks from earlier single-winner runs over all links: 14 distinct region rows were present whenever samples were
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
