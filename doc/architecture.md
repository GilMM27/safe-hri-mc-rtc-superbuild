# Project architecture and QP guide

This repository is a **superbuild plus a Kinova simulation experiment**. The
superbuild downloads and builds `mc_rtc` and its dependencies. The code in this
repository adds a Kinova simulation scene, two controllers, and optional
SMPL-H human playback and distance measurement. The SSM+PFL experiment adds
speed constraints to the sweep controller's quadratic program (QP).

Start with [the root README](../README.md) for installation and
[the playback guide](../smplh_playback/README.md) for a runnable scene. This
page explains how the pieces fit together and what the QP actually enforces.

## The system in one picture

```text
AMASS motion + SMPL-H model
          │  offline preparation
          ▼
  local playback cache ──► patched mc_mujoco / MuJoCo
                                  │  human and robot geometry, sampled velocities
                                  ▼
                           distance worker
                                  │  latest region/link snapshot
                                  ▼
                        mc_rtc datastore
                                  │
                                  ▼
                     KinovaSweepController
                        │ posture target
                        │ SSM+PFL inequality rows
                        ▼
                       TVM QP ──► robot command ──► MuJoCo
```

The simulator, distance worker, and controller operate at different rates. In
the example configuration MuJoCo and the controller use 1 ms steps, while a
distance query is requested every 25 physics steps by default. The controller
uses the latest completed, timestamped query; it does not wait for a new query
on every tick.

## What each module does

| Module | Responsibility | Where to look |
| --- | --- | --- |
| Superbuild | CMake project graph that fetches, builds, and installs `mc_rtc`, Tasks, RBDyn, geometry libraries, and optional integrations. Its presets choose source, build, and install locations. | [`CMakeLists.txt`](../CMakeLists.txt), [`mc_rtc.cmake`](../mc_rtc.cmake), [`cmake/`](../cmake/), [`CMakePresets.json`](../CMakePresets.json) |
| Robot selection | `WITH_<robot>` options add robot packages. `WITH_Kinova` adds ROS 2 Kortex, the Kinova `mc_rtc` robot module, and the hardware interface; it requires ROS 2 support. | [`robots/`](../robots/), especially [`Kinova.cmake`](../robots/Kinova.cmake) |
| Extensions | Adds `mc_mujoco`, the Kinova Menagerie model and table scene, plus joystick and external-force packages. `WITH_SMPLH_PLAYBACK` patches the separately checked out `mc_mujoco` source during its build. | [`extensions/`](../extensions/) |
| Kinova hold controller | Targets a fixed default posture for the seven movable Kinova joints. Useful as a stationary simulation and playback check. | [`KinovaHoldController.cpp`](../kinova_hold_controller/src/KinovaHoldController.cpp) |
| Kinova sweep controller | Creates a smooth `joint_1` back-and-forth posture target, Start/Stop GUI controls, safety-row assembly, QP recovery, and diagnostics. With SSM+PFL disabled, it is a repeatable motion baseline. | [`KinovaSweepController.cpp`](../kinova_hold_controller/src/KinovaSweepController.cpp), [`KinovaSweepController.h`](../kinova_hold_controller/src/KinovaSweepController.h) |
| Human cache preparation | Converts licensed AMASS poses and an SMPL-H model into per-frame surface vertices, joint poses, region labels, MJCF, and a compact native cache. Source data and generated caches stay local under `datasets/`. | [`prepare.py`](../smplh_playback/prepare.py), [`native_cache.py`](../smplh_playback/native_cache.py), [`convert_cache.py`](../smplh_playback/convert_cache.py) |
| Native playback bridge | Reads the cache, poses the kinematic human in MuJoCo, updates the visible surface, requests distance queries, and publishes results to the controller datastore. The installation script applies this bridge to the external `mc_mujoco` checkout. | [`NativeSmplhPlayback.h`](../smplh_playback/NativeSmplhPlayback.h), [`install_bridge.py`](../smplh_playback/install_bridge.py) |
| Distance worker and snapshot | In a background thread, finds the closest human/robot geometry for each region and movable link. The snapshot carries clearance, witness points, selected robot body, point velocities, timing, and validity flags. | [`SmplhDistanceWorker.h`](../smplh_playback/SmplhDistanceWorker.h), [`SmplhDistanceSnapshot.h`](../smplh_playback/SmplhDistanceSnapshot.h) |
| Safety math and QP rows | Converts each measured pair into a directional speed limit and a linear acceleration inequality. Fixed row storage lets the QP retain the same shape as closest geometry changes. | [`SSMPFLMath.h`](../kinova_hold_controller/src/SSMPFLMath.h), [`SSMPFLConstraint.h`](../kinova_hold_controller/src/SSMPFLConstraint.h) |
| Optional relaxation | Solves an auxiliary QP for the smallest weighted *per-pair* allowances under model acceleration bounds, then passes those allowances to the full controller QP. An opt-in full-feasibility mode can search for further shared allowance. | [`SSMPFLRelaxation.h`](../kinova_hold_controller/src/SSMPFLRelaxation.h), sweep controller `run()` |
| Analysis and validation | Synthetic fixtures, distance benchmarks, log analysis, and focused C++/Python checks of geometry, cache, and QP algebra. | [`smplh_playback/analyze_ssm_pfl.py`](../smplh_playback/analyze_ssm_pfl.py), [`benchmarks/`](../benchmarks/), [`tests/`](../tests/) |
| Development setup and examples | Bootstrap scripts install build prerequisites; the devcontainer supplies a repeatable development environment. Example YAML files show controller and MuJoCo configurations for smoke tests and slowdown experiments. | [`utils/`](../utils/), [`.devcontainer/`](../.devcontainer/), [`smplh_playback/`](../smplh_playback/) |

The older [`SmplhBridge.h`](../smplh_playback/SmplhBridge.h) contains a
separate bridge implementation; the opt-in build copies
`NativeSmplhPlayback.h` into `mc_mujoco` as `SmplhBridge.h`.

### Human geometry and measurements

`prepare.py` assigns each SMPL-H triangle to one of 14 body regions using
skinning weights. The worker compares each region with the robot geometry on
each of seven movable Kinova links: **14 × 7 = 98 pairs**. For each pair it
keeps the closest robot geometry and its two closest points ("witnesses"). The
fixed base is excluded. The visible SMPL-H mesh and simplified skeleton do not
create physical contacts by default.

The default `sch_approx` distance mode uses convex patches and reports an
approximate clearance. `SMPLH_DISTANCE_MODE=fcl` uses triangle-surface
distance. This choice affects the witnesses and thus the QP rows. See
[distance and cache notes](smplh-distance.md) for the tradeoffs and query
timing.

## How the controller's QP works

A QP chooses a set of robot joint accelerations that best follows the posture
target while satisfying linear constraints. The sweep controller uses
`mc_rtc`'s **TVM** backend. Its ordinary QP already contains the posture
task and contact, dynamics, self-collision, and joint-limit constraints (plus
any configured global constraints). SSM+PFL contributes **one upper-bound
inequality for each active region/link pair**. It changes what motion the QP
may choose; it does not directly set the commanded joint angle or replace the
posture task.

The QP cycle is:

1. Update the smooth `joint_1` posture target. Start begins the sweep; Stop
   holds the available posture.
2. Read the latest distance snapshot. Check its age, pair count, witness and
   direction validity, human velocity when needed, and agreement between the
   MuJoCo witness and `mc_rtc`'s robot model.
3. Compute the full fixed-base joint inertia matrix and a point Jacobian for
   each robot witness. Use the row corresponding to motion **toward the human**.
4. Compute a regional speed limit and install up to 98 linear inequality rows.
   Unused fixed slots are harmless `0 ≤ 0` rows. Depending on configuration,
   optionally compute allowed row relaxation.
5. Solve the full TVM QP. Log its status, the limiting or binding pairs,
   predicted row residuals, sampled speed discrepancies, and any relaxation.

### From distance to a speed limit

For a region/link pair, let `d` be clearance, `m` its configured distance
margin, `T` the assumed stopping time, `n` a unit vector from the robot witness
toward the human, `J` the **translational** point Jacobian, and `M(q)` the full
joint inertia matrix. `Fmax` and `k` are that region's configured force and
stiffness parameters. The code computes:

```text
w        = nᵀ J M(q)⁻¹ Jᵀ n                directional inverse mass [1/kg]
mR       = 1 / w                            reflected mass [kg]
vPFL     = Fmax / sqrt(k) × sqrt(w)          PFL speed allowance [m/s]
vSSM     = max(d - m, 0) / T                 distance-based allowance [m/s]
vLimit   = vSSM + vPFL                       combined speed allowance [m/s]
```

The inertia is solved with a positive-definite factorization; the code does
not invert `M` explicitly. A lighter effective direction permits a larger PFL
speed for the same example force/stiffness values. Clearance above the margin
adds an SSM allowance; at or inside the margin, only the PFL term remains.
These are **model-based simulation parameters**, not measured stopping or
human biomechanical limits.

### Turning that limit into a QP row

Let `qdot` and `qddot` be joint velocity and the QP's acceleration variables;
`vH` is sampled human witness velocity; `dt` is the controller step. The
robot's current directional speed is `r = nᵀJ qdot`, and the local Jacobian
drift is `aDrift = nᵀJdot qdot`. Over one step, the predicted robot approach
speed is

```text
rNext ≈ r + dt × (nᵀJ qddot + aDrift).
```

The direction, witness, and sampled human velocity are held fixed for this
one-step prediction. Thus each row is linear in `qddot`, which the TVM QP
can accept. The repository has three named `ConstraintMode` values, but the
actual row choice also depends on `HumanMode`:

| Human mode and setting | Row installed in the QP | Meaning |
| --- | --- | --- |
| `frozen` + `legacy` (also `frozen` + `pointwise_speed`) | `dt nᵀJ qddot ≤ vLimit - (r - nᵀvH) - dt aDrift` | Limit **next-step relative closing speed**. Frozen mode requires frozen playback, so `vH = 0`. `legacy` is the default setting. |
| `moving` + `legacy` or `pointwise_speed` | `dt nᵀJ qddot ≤ max(0, vLimit + nᵀvH) - r - dt aDrift` | Limit **robot point approach speed**. A human approaching faster than `vLimit` reduces the robot's positive approach budget to zero, but this row does not force retreat. |
| `frozen` or `moving` + `temporal_barrier` | `nᵀJ qddot ≤ vLimitRate - beta + humanApproachRate + gamma × (vLimit - r + nᵀvH)` | Anticipate a changing **relative** speed limit. `beta` estimates the rate of `nᵀJ` times `qdot`; `humanApproachRate` estimates the rate of `nᵀvH`; `gamma` sets recovery rate. Both human terms are zero in frozen mode. |

The temporal barrier estimates changes from consecutive *fresh* snapshots for
each pair. Its `vLimitRate` combines the estimated clearance-rate and PFL-rate
terms. A closest-feature switch, overlapping geometry, stale/reused sample,
large witness movement, or unreliable mobility estimate disables that
derivative. The controller then uses a model-based drift estimate and zero
estimated PFL and human-approach rates for the barrier row, while retaining
the current sampled human velocity. In moving temporal mode, an approaching
human can make the relative barrier demand robot retreat; the pointwise row's
zero floor does not apply. The human-approach derivative comes from discrete,
asynchronous mesh samples and can change abruptly.

**Example.** At a pair with `d = 0.15 m`, `m = 0.05 m`, `T = 0.5 s`, and
`vPFL = 0.10 m/s`, the limit is `0.30 m/s`. A moving human approaching along
`n` at `0.20 m/s` has `nᵀvH = -0.20 m/s`; the pointwise row allows at most
`0.10 m/s` of robot approach on the next step. If the human approaches at
`0.40 m/s`, the robot budget is zero. The QP can still move tangentially or
retreat, subject to its other constraints.

### What happens when rows conflict

Without relaxation, an infeasible safety row can make the full QP fail.
Enabling `SSMPFL.Relaxation` adds explicit **hard joint-acceleration bounds**
from the model intersected with `MaxAccelerationRadS2`. Its auxiliary Stage 1
QP chooses accelerations and one nonnegative slack per active pair. It
minimizes weighted squared slack (with a small acceleration tie breaker),
subject to the safety rows and those acceleration bounds. The slack expands
only the corresponding safety upper bound. It does not relax joint limits,
dynamics, contact, collision, or the acceleration bounds. The full `mc_rtc` QP
then solves with the expanded rows and the posture task. Stage 1 sees fewer
hard constraints than that final QP, so its feasible answer might still fail
in the full solve.

The optional recovery modes handle that situation differently:

- `two_stage_regularized` can retry a **moving-human pointwise-speed** full QP with increasing
  shared approach-speed allowance, up to `MaxStage2SlackMps`. It stops at the
  first successful solve. This is a bounded feasibility retry, not a proof of
  minimum violation.
- `two_stage_full_feasibility` works with all row types. After a failed full
  solve, it probes the full TVM QP with SSM+PFL rows suspended, without
  integrating robot state. If the remaining hard constraints are feasible, it
  derives a feasible shared allowance and tries up to two refinements before
  the final solve. The allowance is approximate and has no hard cap; the
  moving pointwise mode's `MaxStage2SlackMps` is reported as an exceedance
  threshold here; temporal slack is measured in m/s².

Every relaxed row and its original-bound exceedance are logged. A failed
hard-only QP cannot be repaired by safety-row slack. A failed final solve
requests a posture hold. On invalid or stale distance data, the controller
clears the safety rows and follows `HoldOnConstraintUnavailable`; by default
it requests a hold. These are response paths, **not guarantees of a safe
stop**.

## Configuration and interpretation

The installed [sweep defaults](../kinova_hold_controller/etc/KinovaSweepController.in.yaml)
set the motion only. The
[SSM+PFL example](../smplh_playback/KinovaSweepController.ssm_pfl.example.yaml)
shows opt-in safety parameters. The key choices are `SSMPFL.Enabled`,
`HumanMode`, `ConstraintMode`, per-region `ForceMaxN` / `StiffnessNPerM` /
`DistanceMarginM`, `StoppingTimeSeconds`, and optional `Relaxation` settings.
Use `SMPLH_FREEZE_FRAME` with frozen mode. The example force, stiffness,
acceleration, and stopping-time numbers are placeholders for simulation.

The GUI and logs distinguish the QP's **predicted next-step** row residual
from the **sampled MuJoCo** relative speed. Differences can come from
asynchronous sample age, model/simulator tracking, and geometry approximation.
The controller checks one closest geometry per region/link, not every possible
surface interaction. A satisfied QP row does not establish physical safety or
ISO 15066 compliance. For equations, recovery details,
and logging keys, see [the SSM+PFL technical notes](ssm-pfl.md).
