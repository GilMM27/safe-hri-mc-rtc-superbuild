# safe-hri-mc-rtc-superbuild

## Contents

- [Overview](#overview)
- [Installation](#installation)
  - [Fork the superbuild](#fork-the-superbuild)
  - [Installing the requirements (bootstrapping)](#installing-the-requirements-bootstrapping)
  - [Git setup](#git-setup)
  - [Build](#build)
  - [Config superbuild (adding robots and options)](#config-superbuild-adding-robots-and-options)
  - [Bashrc](#bashrc)
  - [Running the Kinova Gen3 in MuJoCo](#running-the-kinova-gen3-in-mujoco)
- [SMPL-H / AMASS human playback](#smpl-h--amass-human-playback)
- [Project architecture and QP guide](#project-architecture-and-qp-guide)
- [mc_rtc robots](#mc_rtc-robots)
- [Contributing and issues](#contributing-and-issues)

## Overview

This project is based on [mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild): it will clone, update, build, and install all of the dependencies.

Only tested on Ubuntu 24.04. We do not officially support other operating systems at the moment.

## Installation

### Fork the superbuild

We recommend having your own superbuild by forking this one. As you add your own controllers and extensions, it's much easier to manage with your own fork rather than working directly off of ours.

Create a workspace folder in your home directory and clone your fork into it:

```bash
mkdir ~/workspace
cd ~/workspace
git clone git@github.com:{GIT_USERNAME}/safe-hri-mc-rtc-superbuild.git
```

### Installing the requirements (bootstrapping)

You can install the requirements by running our bootstrap script:

```bash
cd ~/workspace/
./safe-hri-mc-rtc-superbuild/utils/bootstrap-linux.sh
```

### Git setup

Make sure you've configured `git` first:

```sh
git config --global user.name "Full Name"
git config --global user.email "your.email@provider.com"
```

### Build

Run the superbuild from the terminal, or use VS Code's "CMake Tools" extension to select your desired build preset.

By default, the presets will:

- clone all projects into `~/workspace/src`
- build all projects into `~/workspace/build/`
- install all projects into `~/workspace/install`

```bash
cd ~/workspace/safe-hri-mc-rtc-superbuild
# Build all projects
cmake --preset relwithdebinfo
cmake --build --preset relwithdebinfo
```

### Config superbuild (adding robots and options)

To add robots or modify the superbuild options, after your first build you can run:

```bash
cd ~/workspace/build/superbuild
# Configure the superbuild
ccmake .
```

### Bashrc

At the end of the build, the superbuild will ask you to add the following line to your `.bashrc`:

```bash
source /home/$USERNAME/workspace/install/setup_mc_rtc.sh
```

This file is generated after the first configuration of the superbuild (the step above).

You can also add the following aliases to your `.bashrc` to simplify using the framework:

```bash
# Setup the superbuild
alias mc_setup='cd ~/workspace/safe-hri-mc-rtc-superbuild; cmake --preset relwithdebinfo'

# Run the superbuild
alias mc_build='cd ~/workspace/safe-hri-mc-rtc-superbuild; cmake --build --preset relwithdebinfo'

# Config the superbuild
alias mc_superbuild_config="cd ~/workspace/build/superbuild; ccmake ."

# Automatically update the superbuild and all associated projects via git pull
alias mc_update='cd ~/workspace/build/superbuild; cmake --build . --config RelWithDebInfo --target update'

# Open the mc_rtc rviz interface
alias mc_rviz="ros2 launch mc_rtc_ticker display.launch"

# Configure mc_rtc: robot and controller selection (you may replace with your preferred text editor)
alias mc_config="gnome-text-editor ~/.config/mc_rtc/mc_rtc.yaml &"

# Configure mc_mujoco: adding objects
alias mc_mujoco_config="gnome-text-editor ~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml &"

# Source automatically for new terminals
source ~/workspace/install/setup_mc_rtc.sh
```

When you're done editing your `.bashrc`, don't forget to source it or open a fresh terminal.

### Running the Kinova Gen3 in MuJoCo

Kinova support includes the ROS 2/Kortex interfaces by default only when enabled. The
superbuild can also install the Kinova Gen3 MuJoCo model from the
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie). Enable ROS 2
and Kinova support during configuration:

```bash
cmake --preset relwithdebinfo \
  -DWITH_ROS_SUPPORT=ON \
  -DWITH_Kinova=ON
cmake --build --preset relwithdebinfo
```

The build installs the Kinova Gen3 model as `kinova` for `mc_mujoco`. Modify
`~/.config/mc_rtc/mc_rtc.yaml` and select a controller that is compatible with a 7-DoF arm
and use a 1 ms controller timestep to match the Kinova MuJoCo model's physics step:

```yaml
MainRobot: Kinova
Enabled: YourController
Timestep: 0.001
LogPolicy: threaded
```

After sourcing the installed environment, start `mc_mujoco` from the directory
containing the controller's assets:

```bash
source ~/workspace/install/setup_mc_rtc.sh
mc_mujoco --sync
```

The Kinova module and MuJoCo model are separate pieces: `WITH_Kinova` builds the
`mc_rtc` robot model and Kortex support, while the extension downloads and installs
the Menagerie MJCF and its meshes. This provides simulation; `mc_kortex` remains the
separate interface for a physical Kinova Gen3.


## SMPL-H / AMASS human playback

An optional AMASS + SMPL-H pipeline drives a kinematic articulated human in
the same MuJoCo scene as Kinova and measures each body region against the
reconstructed SMPL-H triangle surface. The human surface and the MuJoCo
collision/debug geoms are separate, so a later experiment can compare mesh
surface distances with simplified collision elements. See
[the playback guide](smplh_playback/README.md) for model/data setup, build
option, coordinate alignment, launch commands, region definitions, distance
records, and limitations.

## Project architecture and QP guide

See [the project architecture and QP guide](doc/architecture.md) for a map of
the superbuild, controllers, human playback, distance worker, and the SSM+PFL
quadratic program.

## mc_rtc robots

By default, mc_rtc is compatible with many robots, as you can see when configuring the superbuild via `mc_superbuild_config`. If you want to add a custom robot that isn't already in the list, follow this [tutorial](https://jrl.cnrs.fr/mc_rtc/tutorials/advanced/new-robot.html).

In short, each robot in mc_rtc has four modules:

- **Robot module** `mc_{robot name}` — e.g. [mc_h1](https://github.com/isri-aist/mc_h1). Contains the constraint definitions (joint limits, self-collision), joint order, and sensor declarations.
- **Robot URDF** `{robot name}_description` — e.g. [h1_description](https://github.com/isri-aist/h1_description). Installed in `src/catkin_data_ws`; contains the full robot description in the format used by ROS. This is what mc_rtc uses for QP constraints and the dynamic model, and what mc_rviz uses for display.
- **Robot Mujoco description** `{robot name}_mj_description` — e.g. [h1_mj_description](https://github.com/isri-aist/h1_mj_description). Used by Mujoco to display and simulate the robot's physics. Unless intentional, it's important to keep the same robot model in the URDF and the Mujoco XML description.
- **Robot driver interface** `mc_{robot driver name}` — e.g. [mc_unitree2](https://github.com/isri-aist/mc_unitree2). Installs the driver needed to run your mc_rtc controller on the real robot. The exact command differs per robot driver — for H1, running your controller on the real robot requires `MCControlUnitree`. Always check the README of the relevant driver repo, since most require additional information in your mc_rtc config file (`mc_config`), such as the robot's IP address.

The `WITH_H1` option in `mc_superbuild_config` adds all four modules at once — this isn't true for every robot. At minimum you'll need the robot module and the URDF; if the Mujoco description or robot driver are missing, check the [isri-aist](https://github.com/isri-aist) GitHub account and add the missing modules manually in the superbuild's extension folder.

## Contributing and issues

Since this repository is a fork of [mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild), if you run into installation issues we recommend addressing them directly on the original repo. Likewise, since the superbuild is just a collection of CMake files, issues with a specific project are best reported directly on that project's repo.

The main useful way to contribute to this repo specifically is by improving this installation guide (this README) if you spot any issues.
