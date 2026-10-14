set(MUJOCO_VERSION 3.5.0)

if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
  set(MUJOCO_URL "https://github.com/deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    set(MUJOCO_URL "${MUJOCO_URL}-aarch64.tar.gz")
  else()
    set(MUJOCO_URL "${MUJOCO_URL}-x86_64.tar.gz")
  endif()
  DownloadFile("${MUJOCO_URL}" "${CMAKE_CURRENT_BINARY_DIR}/mujoco/mujoco.tar.gz" "")
  file(ARCHIVE_EXTRACT
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/mujoco/mujoco.tar.gz"
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/mujoco"
  )
  set(MUJOCO_ROOT_DIR "${CMAKE_CURRENT_BINARY_DIR}/mujoco/mujoco-${MUJOCO_VERSION}")
else()
  if(NOT DEFINED MUJOCO_ROOT_DIR)
    message(FATAL_ERROR "MuJoCo must be installed manually on your platform, you must then defined MUJOCO_ROOT_DIR")
  endif()
endif()

AptInstall(libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libglew-dev)

option(WITH_SMPLH_PLAYBACK "Enable native SMPL-H cache playback in mc_mujoco (Linux)" OFF)
set(SMPLH_PATCH_ARGS)
if(WITH_SMPLH_PLAYBACK)
  if(NOT UNIX OR APPLE)
    message(FATAL_ERROR "The native SMPL-H bridge currently requires Linux")
  endif()
  set(SMPLH_PATCH_ARGS PATCH_COMMAND
    "${CMAKE_COMMAND}"
    "-DSOURCE_DIR=${SOURCE_DESTINATION}/mc_mujoco"
    "-DBRIDGE_SCRIPT=${CMAKE_CURRENT_LIST_DIR}/../smplh_playback/install_bridge.py"
    -P "${CMAKE_CURRENT_LIST_DIR}/../cmake/scripts/apply-smplh-bridge.cmake")
endif()

AddProject(mc_mujoco
  GITHUB bastien-muraccioli/mc_mujoco
  GIT_TAG origin/main
  CMAKE_ARGS -DMUJOCO_ROOT_DIR=${MUJOCO_ROOT_DIR}
  DEPENDS mc_rtc
  ${SMPLH_PATCH_ARGS}
)

if(WITH_SMPLH_PLAYBACK)
  ExternalProject_Add_Step(mc_mujoco smplh_bridge_refresh
    COMMAND "${CMAKE_COMMAND}"
      "-DSOURCE_DIR=${SOURCE_DESTINATION}/mc_mujoco"
      "-DBRIDGE_SCRIPT=${CMAKE_CURRENT_LIST_DIR}/../smplh_playback/install_bridge.py"
      -P "${CMAKE_CURRENT_LIST_DIR}/../cmake/scripts/apply-smplh-bridge.cmake"
    DEPENDEES patch
    DEPENDERS configure
    DEPENDS
      "${CMAKE_CURRENT_LIST_DIR}/../smplh_playback/install_bridge.py"
      "${CMAKE_CURRENT_LIST_DIR}/../smplh_playback/NativeSmplhPlayback.h"
      "${CMAKE_CURRENT_LIST_DIR}/../smplh_playback/SmplhDistanceWorker.h"
      "${CMAKE_CURRENT_LIST_DIR}/../smplh_playback/SmplhDistanceSnapshot.h"
  )
endif()
