# Kinova controllers

This directory builds three `mc_rtc` controllers for the Kinova Gen3. The
`KinovaPositionSSMPFLController` lives in its
[own repository](https://github.com/GilMM27/kinova_position_SSM_PFL_controller)
and is registered by `extensions/kinova_position_SSM_PFL_controller.cmake`.

| Controller | Behavior |
| --- | --- |
| `KinovaHoldController` | Holds the seven arm joints at a fixed posture. Useful for a stationary simulation check. |
| `KinovaSweepController` | Sweeps `joint_1` between home and a configured waypoint. It can add SSM+PFL constraints to its QP. |
| `KinovaPositionSSMPFLPositionTaskController` | Uses the same tool target and SSM+PFL speed budgets, with one weighted position task per monitored robot link. |

Build these controllers through the [superbuild](../README.md). For SSM+PFL,
enable `WITH_SMPLH_PLAYBACK=ON`, prepare a cache, and add the human object to
MuJoCo as described in the [playback guide](../smplh_playback/README.md).
The safety controllers need fresh human-to-robot distance samples from that
bridge; the position controller holds its current target when samples are
unavailable.

To use one set of physical human collision sizes across SMPL-H caches, add
`SMPLHContactSizes` to the active controller YAML, including
`~/.config/mc_rtc/controllers/KinovaPositionSSMPFLController.yaml` when using
that controller. The MuJoCo bridge applies the scales when it starts; see the
[contact shape configuration](../smplh_playback/README.md#size-contact-shapes-by-region).

## Run the sweep controller

Set `MainRobot: Kinova`, `Enabled: KinovaSweepController`, and `Timestep: 0.001`
in `~/.config/mc_rtc/mc_rtc.yaml`. Enable `GUIServer` there to use the Start
and Stop buttons. The installed defaults in
[`etc/KinovaSweepController.in.yaml`](etc/KinovaSweepController.in.yaml) define
the sweep trajectory.

The [complete SSM+PFL option example](examples/ssm_pfl_all_options.yaml)
contains one `SSMPFL` block with every option supported by the sweep
controller. Copy it to
`~/.config/mc_rtc/controllers/KinovaSweepController.yaml` to override the
installed controller configuration, then set its values for your experiment.

```sh
mkdir -p ~/.config/mc_rtc/controllers
cp kinova_controllers/examples/ssm_pfl_all_options.yaml \
  ~/.config/mc_rtc/controllers/KinovaSweepController.yaml
```

It is a configuration fragment, so it does not replace `mc_rtc.yaml` or the
MuJoCo human-object configuration. `SSMPFL.Weight` selects soft QP rows;
to enable `SSMPFL.Relaxation`, remove `Weight` and set
`Relaxation.Enabled: true`. The regional force and stiffness values are
unverified simulation examples. Set `SSMPFL.HumanMode: moving` for moving
playback, or freeze a playback frame when using the example's `frozen` mode.

After exporting `SMPLH_CACHE`, run `mc_mujoco --sync` and press
**Kinova sweep → Start** in `mc_rtc_gui`. See the
[SSM+PFL guide](../doc/ssm-pfl.md) for the constraint modes, diagnostics, and
limitations.

## Run the task-based position controller

Set `Enabled: KinovaPositionSSMPFLPositionTaskController`
and copy the same `SSMPFL.Regions` configuration into its controller YAML.
`SSMPFL.Weight` (default `5.0`) sets the weight of each of the seven link
position tasks, while `SSMPFL.Stiffness` (default `2.0`) sets their stiffness.
The controller projects each region's speed budget into a desired link
acceleration. These weighted tasks approximate the speed rows; they do not
provide a hard speed limit, especially when task objectives conflict.
Link tasks enter the solver only when the current speed budget calls for
deceleration, so they do not oppose normal tool movement. Set
`SSMPFL.HumanMode: moving` for moving playback; the controller holds the tool
target if playback moves while this setting is `frozen`.
