# Kinova controllers

This directory builds three `mc_rtc` controllers for the Kinova Gen3:

| Controller | Behavior |
| --- | --- |
| `KinovaHoldController` | Holds the seven arm joints at a fixed posture. Useful for a stationary simulation check. |
| `KinovaSweepController` | Sweeps `joint_1` between home and a configured waypoint. It can add SSM+PFL constraints to its QP. |
| `KinovaPositionSSMPFLController` | Moves the tool toward a position target set in `mc_rtc_gui`, with hard SSM+PFL speed rows by default or soft rows when `SSMPFL.Weight` is set. |

Build these controllers through the [superbuild](../README.md). For SSM+PFL,
enable `WITH_SMPLH_PLAYBACK=ON`, prepare a cache, and add the human object to
MuJoCo as described in the [playback guide](../smplh_playback/README.md).
The safety controllers need fresh human-to-robot distance samples from that
bridge; the position controller holds its current target when samples are
unavailable.

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

## Run the position controller

Set `Enabled: KinovaPositionSSMPFLController` in `mc_rtc.yaml`. Its controller
configuration needs all 14 `SSMPFL.Regions` entries. Omit `SSMPFL.Weight` for
hard speed rows or set a positive weight for soft rows. The complete option
example above supplies those values, but this controller
uses only its shared subset of keys. Copy the file to
`~/.config/mc_rtc/controllers/KinovaPositionSSMPFLController.yaml` and set
`SSMPFL.HumanMode` to match playback. `PositionTask.Weight` and
`PositionTask.Stiffness` set the tool task's startup gains. The controller also
anchors a weak posture task to the measured joints on reset to prevent arm
swivel at a fixed tool position. `PostureTask.Weight` (default `0.1`) and
`PostureTask.Stiffness` (default `1.0`) tune that preference; keep its weight
below the position task's weight. In `mc_rtc_gui`,
move **Tasks → KinovaToolPosition → pos_target** to set a world-frame XYZ
target. This controller requires fresh distance samples to move; it holds the
tool when samples are missing or stale.
