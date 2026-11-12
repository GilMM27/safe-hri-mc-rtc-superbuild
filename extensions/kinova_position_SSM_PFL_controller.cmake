include(${CMAKE_CURRENT_LIST_DIR}/human_robot_distance_api.cmake)

set(SMPLH_FORCE_LOG_PATCH_ARGS)
if(WITH_SMPLH_PLAYBACK)
  set(SMPLH_FORCE_LOG_PATCH_ARGS PATCH_COMMAND
    "${CMAKE_COMMAND}"
    "-DSOURCE_DIR=${SOURCE_DESTINATION}/kinova_position_SSM_PFL_controller"
    "-DBRIDGE_SCRIPT=${CMAKE_CURRENT_LIST_DIR}/../smplh_playback/install_force_logging.py"
    -P "${CMAKE_CURRENT_LIST_DIR}/../cmake/scripts/apply-smplh-bridge.cmake")
endif()

AddProject(kinova_position_SSM_PFL_controller
  GITHUB GilMM27/kinova_position_SSM_PFL_controller
  GIT_TAG origin/main
  DEPENDS mc_rtc human_robot_distance_api
  ${SMPLH_FORCE_LOG_PATCH_ARGS}
)

if(WITH_SMPLH_PLAYBACK)
  ExternalProject_Add_Step(kinova_position_SSM_PFL_controller smplh_force_logging_refresh
    COMMAND "${CMAKE_COMMAND}"
      "-DSOURCE_DIR=${SOURCE_DESTINATION}/kinova_position_SSM_PFL_controller"
      "-DBRIDGE_SCRIPT=${CMAKE_CURRENT_LIST_DIR}/../smplh_playback/install_force_logging.py"
      -P "${CMAKE_CURRENT_LIST_DIR}/../cmake/scripts/apply-smplh-bridge.cmake"
    DEPENDEES patch
    DEPENDERS configure
    DEPENDS
      "${CMAKE_CURRENT_LIST_DIR}/../smplh_playback/install_force_logging.py"
      "${CMAKE_CURRENT_LIST_DIR}/../smplh_playback/SmplhContactForces.h"
  )
endif()
