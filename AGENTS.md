# Repository Guidelines

## Project Structure & Module Organization

This repository is a CMake superbuild for `mc_rtc` and related robotics modules. The root `CMakeLists.txt`, `CMakePresets.json`, and `cmake/` define dependency builds and installation. Add robot selections in `robots/` and optional modules in `extensions/`. The local Kinova controllers live in `kinova_hold_controller/src/`, with installed YAML templates in `kinova_hold_controller/etc/`. SMPL-H preparation and playback tools are in `smplh_playback/`; C++ and Python checks are in `tests/`, benchmarks in `benchmarks/`, and longer guides in `doc/`. Keep downloaded model and motion data in `datasets/`, outside commits.

## Build, Test, and Development Commands

Run commands from the repository root. On a supported Linux system, `utils/bootstrap-linux.sh` installs build prerequisites. `cmake --preset relwithdebinfo` configures an out-of-tree build in `../build/superbuild`; `cmake --build --preset relwithdebinfo` builds and installs into `../install`. Use `cmake --preset relwithdebinfo -DWITH_Kinova=ON -DWITH_SMPLH_PLAYBACK=ON` for the local Kinova playback integration. Source `../install/setup_mc_rtc.sh` before running installed controllers or MuJoCo examples. Presets ending in `-notests` disable subproject tests.

## Coding Style & Naming Conventions

Follow nearby C++ and Python code when editing. Use four spaces in Python and preserve the existing C++ style in each file. Name Python test files `test_*.py` and C++ test files `test_*.cpp`. Format CMake files with the repository's `cmake-format` configuration (`.cmake-format.py`). Run `pre-commit run --all-files` before submitting to check CMake formatting, YAML, whitespace, merge markers, and large files. The separate commit-msg hook checks commit subjects.

## Testing Guidelines

Python tests use `unittest`: install optional playback test dependencies with `python -m pip install -r smplh_playback/requirements-test.txt`, then run `python -m unittest discover -s tests -p 'test_*.py'` from the root. Run `ctest --test-dir ../build/superbuild --output-on-failure` for any tests registered by the configured CMake build. Some C++ checks in `tests/` need the installed `mc_rtc` and are not wired into the root CTest suite; build and run those with the relevant integration setup. Add focused regression coverage for behavior changes. No repository-wide coverage target is specified.

## Commits & Pull Requests

Recent commits use short imperative subjects such as `Add Kinova Gen3 MuJoCo support`; the commit-msg hook also enforces Conventional Commits, so use subjects such as `feat: add distance check` or `fix: correct cache parsing`. Keep each pull request focused, explain the affected build options and test commands, link related issues, and include screenshots or recordings for viewer or GUI changes. Do not commit licensed SMPL-H models or motion datasets.
